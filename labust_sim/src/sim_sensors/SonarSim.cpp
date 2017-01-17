/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the LABUST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Dula Nad
 *  Created: 01.02.2013.
 *********************************************************************/
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <navcon_msgs/RelativePosition.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <labust/simulation/NoiseModel.hpp>
#include <labust/tools/conversions.hpp>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using labust::simulation::NoiseGenerators;

/// TODO: Add roll-off on the edges of the cone (increase detection noise
/// continuously)
struct SonarSim
{
  SonarSim()
    : last_pub(ros::Time::now())
    , rate(10)
    , listener(buffer)
    , max_elevation(10)
    , max_bearing(45)
    , max_range(50)
    , min_range(1)
    , tf_prefix("")
    , sigma_bearing(0)
    , sigma_range(0)
    , frame_id("sonar_frame")
  {
    ros::NodeHandle nh, ph("~");

    ph.param("sonar_rate", rate, rate);

    std::vector<double> offset(3, 0), orot(3, 0);
    ph.param("offset", offset, offset);
    ph.param("orot", orot, orot);
    ph.param("max_elevation_deg", max_elevation, max_elevation);
    ph.param("max_bearing_deg", max_bearing, max_bearing);
    ph.param("max_range", max_range, max_range);
    ph.param("min_range", min_range, min_range);
    ph.param("sigma_bearing", sigma_bearing, sigma_bearing);
    ph.param("sigma_range", sigma_range, sigma_range);
    ph.param("frame_id", frame_id, frame_id);

    gen.addNew(0, sigma_range);
    gen.addNew(0, sigma_bearing);

    // Convert from deg to radians
    max_elevation *= M_PI / 180;
    max_bearing *= M_PI / 180;
    this->offset << offset[0], offset[1], offset[2];
    labust::tools::quaternionFromEulerZYX(M_PI * orot[0] / 180,
                                          M_PI * orot[1] / 180,
                                          M_PI * orot[2] / 180, this->orot);

    std::string key;
    if (nh.searchParam("tf_prefix", key))
      nh.getParam(key, tf_prefix);

    odom = nh.subscribe<nav_msgs::Odometry>("target_odom", 1,
                                            &SonarSim::onOdom, this);
    sonar_pub = nh.advertise<navcon_msgs::RelativePosition>("sonar_fix", 1);
  }

  void onOdom(const typename nav_msgs::Odometry::ConstPtr& msg)
  {
    navcon_msgs::RelativePosition::Ptr fix(
        new navcon_msgs::RelativePosition());
    fix->header.stamp = msg->header.stamp;
    fix->header.frame_id = tf_prefix + "sonar_frame";

    geometry_msgs::TransformStamped transformLocal, transformGps, transformDeg;
    try
    {
      // Get the simulated vehicle position in the frame of the target
      // measurements
      transformLocal = buffer.lookupTransform(
          msg->header.frame_id, tf_prefix + "base_link_sim", ros::Time(0));

      Eigen::Vector3d vehiclepos(transformLocal.transform.translation.x,
                                 transformLocal.transform.translation.y,
                                 transformLocal.transform.translation.z);
      Eigen::Vector3d targetpos(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
      // Rotation from base_link <-> msg->header.frame_id
      Eigen::Quaternion<double> qrot(transformLocal.transform.rotation.w,
                                     transformLocal.transform.rotation.x,
                                     transformLocal.transform.rotation.y,
                                     transformLocal.transform.rotation.z);
      // Relative target position in body frame
      Eigen::Vector3d rel_b =
          qrot.matrix().transpose() * (targetpos - vehiclepos) - offset;
      // Relative target position in sensor frame
      Eigen::Vector3d rel = this->orot.matrix() * rel_b;

      // Switch to spherical coordinates
      bool target_in_fov(false);
      double range = rel.norm();
      double bearing = atan2(rel(1), rel(0));

      if ((range > min_range) && (range < max_range))
      {
        double elevation = asin(rel(2) / range);

        target_in_fov =
            (fabs(bearing) < max_bearing) && (fabs(elevation) < max_elevation);

        ROS_INFO("Target (%f, %f) in FOV ? Flag = %d", bearing, elevation,
                 target_in_fov);
        ROS_INFO("Relative pos (%f, %f, %f). Distance = %f", rel(0), rel(1),
                 rel(2), range);

        ROS_INFO("Relative pos body (%f, %f, %f). Distance = %f", rel_b(0),
                 rel_b(1), rel_b(2), range);
      }

      double dT = (ros::Time::now() - last_pub).toSec();
      if (target_in_fov && (dT >= 1 / rate))
      {
        last_pub = ros::Time::now();
        navcon_msgs::RelativePosition::Ptr fixout(
            new navcon_msgs::RelativePosition());
        fixout->header.stamp = last_pub;
        fixout->header.frame_id = frame_id;
        fixout->range = range + gen(0);
        fixout->bearing = bearing + gen(1);
        sonar_pub.publish(fixout);
      }
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
  }

private:
  ros::Subscriber odom;
  ros::Publisher sonar_pub;
  ros::Time last_pub;
  double rate;
  Eigen::Vector3d offset;
  Eigen::Quaternion<double> orot;
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;
  tf2_ros::TransformBroadcaster broadcaster;
  double max_elevation;
  double max_bearing;
  double max_range;
  double min_range;
  std::string tf_prefix;
  std::string frame_id;
  NoiseGenerators gen;
  double sigma_bearing;
  double sigma_range;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sonar_sim");
  ros::NodeHandle nh;
  SonarSim sonar;
  ros::spin();
  return 0;
}
