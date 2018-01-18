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
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <labust/tools/conversions.hpp>

void onOdom(ros::Publisher& NavigationStatus, const Eigen::Matrix3d& rot,
            const nav_msgs::Odometry::ConstPtr& in)
{
  auv_msgs::NavigationStatus::Ptr out(new auv_msgs::NavigationStatus());

  out->body_velocity.x = in->twist.twist.linear.x;
  out->body_velocity.y = in->twist.twist.linear.y;
  out->body_velocity.z = in->twist.twist.linear.z;
  out->orientation_rate.x = in->twist.twist.angular.x;
  out->orientation_rate.y = in->twist.twist.angular.y;
  out->orientation_rate.z = in->twist.twist.angular.z;

  // Eigen::Vector3d ned;
  // ned<<in->position.north, in->position.east, in->position.depth;
  // Eigen::Vector3d xyz = rot*ned;

  out->position.north = in->pose.pose.position.x;
  out->position.east = in->pose.pose.position.y;
  out->position.depth = in->pose.pose.position.z;

  double roll, pitch, yaw;
  labust::tools::eulerZYXFromQuaternion(in->pose.pose.orientation, roll, pitch,
                                        yaw);
  out->orientation.x = roll;
  out->orientation.y = pitch;
  out->orientation.z = yaw;

  NavigationStatus.publish(out);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "odom_to_NavigationStatus_node");
  ros::NodeHandle nh, ph("~");

  // Get rotation between the two
  std::vector<double> rpy(3, 0);
  ph.param("rpy", rpy, rpy);

  // Setup the LTP to Odom frame
  Eigen::Quaternion<double> q;
  labust::tools::quaternionFromEulerZYX(rpy[0], rpy[1], rpy[2], q);
  Eigen::Matrix3d rot = q.toRotationMatrix().transpose();

  // TODO subscribe to altitude using message filter.
  // TODO get origin position.
  // TODO get lat/lon position.
  
  
  
  ros::Publisher NavigationStatus = nh.advertise<auv_msgs::NavigationStatus>("NavigationStatus", 1);
  ros::Subscriber odom = nh.subscribe<nav_msgs::Odometry>(
      "odom", 1, boost::bind(&onOdom, boost::ref(NavigationStatus), boost::ref(rot), _1));
  ros::spin();
  return 0;
}
