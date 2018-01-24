/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, LABUST, UNIZG-FER
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
 *********************************************************************/
#include <auv_msgs/NavigationStatus.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/conversions.hpp>

class OdomToNavSts
{
public:
  OdomToNavSts();
  virtual ~OdomToNavSts();

private:
  void onOdom(const nav_msgs::Odometry::ConstPtr& in);

  void onOrigin(const geographic_msgs::GeoPointStamped::ConstPtr& in);

  ros::Publisher pub_navsts;
  ros::Subscriber sub_odom;
  ros::Subscriber sub_origin;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  std::string tf_prefix;
  geographic_msgs::GeoPointStamped origin;
  // ENU projection
  GeographicLib::LocalCartesian proj;
};

OdomToNavSts::OdomToNavSts() : tfListener(tfBuffer), tf_prefix("")
{
  ros::NodeHandle nh, ph("~");

  std::string key;
  if (nh.searchParam("tf_prefix", key))
    nh.getParam(key, tf_prefix);

  // // Get rotation between the two
  // std::vector<double> rpy(3, 0);
  // ph.param("rpy", rpy, rpy);
  //
  // // Setup the LTP to Odom frame
  // Eigen::Quaternion<double> q;
  // labust::tools::quaternionFromEulerZYX(rpy[0], rpy[1], rpy[2], q);
  // Eigen::Matrix3d rot = q.toRotationMatrix().transpose();

  // TODO subscribe to altitude using message filter.

  pub_navsts = nh.advertise<auv_msgs::NavigationStatus>("navsts", 1);
  sub_odom =
      nh.subscribe<nav_msgs::Odometry>("odom", 1, &OdomToNavSts::onOdom, this);
  sub_origin = nh.subscribe<geographic_msgs::GeoPointStamped>(
      "map_origin", 1, &OdomToNavSts::onOrigin, this);
}

OdomToNavSts::~OdomToNavSts()
{
}

void OdomToNavSts::onOdom(const nav_msgs::Odometry::ConstPtr& in)
{
  try
  {
    auv_msgs::NavigationStatus::Ptr out(new auv_msgs::NavigationStatus());
    geometry_msgs::TransformStamped transform;
    transform = tfBuffer.lookupTransform(tf_prefix + "base_link_frd",
                                         tf_prefix + "base_link", ros::Time(0),
                                         ros::Duration(0));

    proj.Reverse(in->pose.pose.position.x, in->pose.pose.position.y, 0,
                 out->global_position.latitude, out->global_position.longitude,
                 out->global_position.altitude);
    out->origin = origin.position;

    // Transform twist.
    // Transform linear velocity.
    geometry_msgs::Vector3Stamped l_in, l_out;
    l_in.header = in->header;
    l_in.vector = in->twist.twist.linear;
    tf2::doTransform(l_in, l_out, transform);

    // Transform angular velocity.
    geometry_msgs::Vector3Stamped a_in, a_out;
    a_in.header = in->header;
    a_in.vector = in->twist.twist.angular;
    tf2::doTransform(a_in, a_out, transform);

    double roll, pitch, yaw;
    labust::tools::eulerZYXFromQuaternion(transform.transform.rotation, roll,
                                          pitch, yaw);
    yaw += M_PI / 2;  // Set North as zero. Due to ROS convention East equals 0.
    labust::tools::quaternionFromEulerZYX(roll, pitch, yaw,
                                          transform.transform.rotation);

    // Transform pose.
    geometry_msgs::PoseStamped p_in, p_out;
    p_in.header = in->header;
    p_in.pose = in->pose.pose;
    tf2::doTransform(p_in, p_out, transform);

    out->position.north = p_out.pose.position.x;
    out->position.east = p_out.pose.position.y;
    out->position.depth = p_out.pose.position.z;

    labust::tools::eulerZYXFromQuaternion(p_out.pose.orientation, roll, pitch,
                                          yaw);

    out->orientation.x =
        labust::math::wrapRad(roll + M_PI);  // Due to convention roll is zero
                                             // when depth vector points towards
                                             // the seafloor.
    out->orientation.y = labust::math::wrapRad(pitch);
    out->orientation.z = labust::math::wrapRad(yaw);

    out->body_velocity.x = l_out.vector.x;
    out->body_velocity.y = l_out.vector.y;
    out->body_velocity.z = l_out.vector.z;

    out->seafloor_velocity.x = l_out.vector.x;
    out->seafloor_velocity.y = l_out.vector.y;
    out->seafloor_velocity.z = l_out.vector.z;

    out->orientation_rate = a_out.vector;

    out->header = in->header;
    out->header.frame_id = tf_prefix + "map_ned";

    pub_navsts.publish(out);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void OdomToNavSts::onOrigin(
    const geographic_msgs::GeoPointStamped::ConstPtr& in)
{
  origin = *in;
  proj.Reset(origin.position.latitude, origin.position.longitude,
             origin.position.altitude);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "odom_to_navsts_node");
  OdomToNavSts odom2navsts;
  ros::spin();
  return 0;
}
