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
 *********************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <auv_msgs/NavSts.h>
#include <tf2_ros/transform_broadcaster.h>
#include <labust/tools/conversions.hpp>

volatile double radius(2);
ros::Publisher marker_pub;
tf2_ros::TransformBroadcaster* broadcaster;

volatile void onRadius(const std_msgs::Float32::ConstPtr& data)
{
	//ROS_INFO("Sub radius.");
	radius = data->data;
}

void onDiverPos(const auv_msgs::NavSts::ConstPtr& diver_pos)
{
	//Publish transform
	geometry_msgs::TransformStamped tfs;
	tfs.transform.translation.x = diver_pos->position.north;
	tfs.transform.translation.y = diver_pos->position.east;
	tfs.transform.translation.z = diver_pos->position.depth;
	labust::tools::quaternionFromEulerZYX(
			diver_pos->orientation.roll,
			diver_pos->orientation.pitch,
			diver_pos->orientation.yaw,
			tfs.transform.rotation);
	tfs.child_frame_id = "diver_frame";
	tfs.header.frame_id = "local";
	tfs.header.stamp = ros::Time::now();
	broadcaster->sendTransform(tfs);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "diver_marker");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber radius_sub = nh.subscribe<std_msgs::Float32>("radius", 1, &onRadius);
  ros::Subscriber diver_sub = nh.subscribe<auv_msgs::NavSts>("position", 1, &onDiverPos);
  broadcaster = new tf2_ros::TransformBroadcaster();

  ros::Rate rate(10);
  while (ros::ok())
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "diver_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "safety_radius";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 2*radius;
    marker.scale.y = 2*radius;
    marker.scale.z = 2*radius;

    //ROS_INFO("Radius: %f", radius);

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.3;
    marker.lifetime = ros::Duration();
    // Publish the marker
    marker_pub.publish(marker);

    rate.sleep();
    ros::spinOnce();
  }

  delete broadcaster;
}
