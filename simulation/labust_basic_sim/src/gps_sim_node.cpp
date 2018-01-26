/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2018, LABUST, UNIZG-FER
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
#include <labust/simulation/NoiseModel.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <labust/tools/conversions.hpp>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <boost/nondet_random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

using labust::simulation::NoiseGenerators;

struct GPSSim
{
  typedef boost::variate_generator<boost::random_device&,
                                   boost::normal_distribution<> >
      NoiseGenerator;
  typedef boost::shared_ptr<NoiseGenerator> NoiseGeneratorPtr;

  GPSSim()
    : last_gps(ros::Time::now())
    , rate(10)
    , listener(buffer)
    , gps_z(0)
    , tf_prefix("")
    , sigma(0)
    , lambda(0)
    , Ts(0.1)
    , max_walk(0)
    , noisex(0)
    , noisey(0)
  {
    ros::NodeHandle nh, ph("~");
    ph.param("gps_pub", rate, rate);
    std::vector<double> offset(3, 0), orot(3, 0);
    ph.param("offset", offset, offset);
    ph.param("orot", orot, orot);
    ph.param("sigma", sigma, sigma);
    ph.param("max_walk", max_walk, max_walk);
    ph.param("lambda", lambda, lambda);
    ph.param("sampling_time", Ts, Ts);
    gen.addNew(0, sigma);
    gen.addNew(0, sigma);

    this->offset << offset[0], offset[1], offset[2];
    labust::tools::quaternionFromEulerZYX(M_PI * orot[0] / 180,
                                          M_PI * orot[1] / 180,
                                          M_PI * orot[2] / 180, this->orot);

    std::string key;
    if (nh.searchParam("tf_prefix", key))
      nh.getParam(key, tf_prefix);

    odom = nh.subscribe<nav_msgs::Odometry>("meas_odom", 1, &GPSSim::onOdom,
                                            this);
    gps_pub = nh.advertise<sensor_msgs::NavSatFix>("fix", 1);
    //gps_odom_pub = nh.advertise<nav_msgs::Odometry>("odometry/gps", 1);
    
  }

  void onOdom(const typename nav_msgs::Odometry::ConstPtr& msg)
  {
    sensor_msgs::NavSatFix::Ptr fix(new sensor_msgs::NavSatFix());
    fix->header.stamp = msg->header.stamp;
    fix->header.frame_id = tf_prefix + "geodetic";

    geometry_msgs::TransformStamped transformLocal, transformGps, transformDeg;

    try
    {


      // Get the simulated vehicle position
      transformLocal = buffer.lookupTransform(
          tf_prefix + "map", tf_prefix + "sim/base_link_frd", ros::Time(0));
      // Add the sensor offset
      Eigen::Quaternion<double> qrot(transformLocal.transform.rotation.w,
                                     transformLocal.transform.rotation.x,
                                     transformLocal.transform.rotation.y,
                                     transformLocal.transform.rotation.z);
      Eigen::Vector3d gps_w = qrot.toRotationMatrix() * offset;
      // Update the simulated position of GPS with offset
      // Eigen::Vector3d ned;
      // ned<<msg->pose.pose.position.x,
      //		msg->pose.pose.position.y,
      //		msg->pose.pose.position.z;
      // labust::tools::quaternionFromEulerZYX(M_PI,0,M_PI/2,qrot);
      // Eigen::Vector3d enu = qrot.matrix().transpose()*ned + gps_w;
      Eigen::Vector3d enu;

      enu << transformLocal.transform.translation.x + noisex,
          transformLocal.transform.translation.y + noisey,
          transformLocal.transform.translation.z;
      enu += gps_w;

      // Make some noise.
      updateNoise();

      transformLocal.transform.translation.x = enu(0);
      transformLocal.transform.translation.y = enu(1);
      transformLocal.transform.translation.z = enu(2);
      // In case the origin changes
      transformDeg = buffer.lookupTransform("earth", tf_prefix + "map",
                                            ros::Time(0), ros::Duration(5.0));
      // Set the projection origin
      double lat0, lon0, h0;
      GeographicLib::Geocentric::WGS84().Reverse(
          transformDeg.transform.translation.x,
          transformDeg.transform.translation.y,
          transformDeg.transform.translation.z, lat0, lon0, h0);
      proj.Reset(lat0, lon0, h0);
      proj.Reverse(transformLocal.transform.translation.x,
                   transformLocal.transform.translation.y,
                   transformLocal.transform.translation.z, fix->latitude,
                   fix->longitude, fix->altitude);
                   
      // Set covariance
      fix->position_covariance[0] = 16;
      fix->position_covariance[4] = 16;
      fix->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;                 
                      
      double dT = (ros::Time::now() - last_gps).toSec();
      if ((fix->altitude >= -0.1) && (dT >= 1 / rate))
      {
        last_gps = ros::Time::now();
        gps_pub.publish(fix);
        // nav_msgs::Odometry gps_odom;
        // gps_odom.header = fix->header;
        // gps_odom.header.frame_id = tf_prefix + "map";        
        // gps_odom.child_frame_id = "";
        // gps_odom.pose.pose.position.x = enu(0);
        // gps_odom.pose.pose.position.y = enu(1);
        // gps_odom.pose.pose.position.z = enu(2);
        // gps_odom.pose.covariance[0] = 16.0;
        // gps_odom.pose.covariance[7] = 16.0;   
        //     
        // gps_odom_pub.publish(gps_odom);
   
      }
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
  }

  void updateNoise()
  {
    noisex = (1 - lambda * Ts) * noisex + Ts * gen(0);
    noisey = (1 - lambda * Ts) * noisey + Ts * gen(1);
  }

private:
  ros::Subscriber odom;
  ros::Publisher gps_pub, gps_odom_pub;
  ros::Time last_gps;
  double rate;
  Eigen::Vector3d offset;
  Eigen::Quaternion<double> orot;
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;
  tf2_ros::TransformBroadcaster broadcaster;
  double gps_z;
  // The ENU frame
  GeographicLib::LocalCartesian proj;
  std::string tf_prefix;
  double noisex;
  double noisey;
  NoiseGenerators gen;
  double sigma;
  double max_walk;
  double lambda;
  double Ts;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gps_sim");
  ros::NodeHandle nh;
  GPSSim gps;
  ros::spin();
  return 0;
}
