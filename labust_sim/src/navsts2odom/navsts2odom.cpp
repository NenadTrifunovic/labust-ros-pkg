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
#include <labust/tools/conversions.hpp>

#include <auv_msgs/NavSts.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>


void onNavSts(ros::Publisher& odom, const Eigen::Matrix3d& rot,
    const Eigen::Quaternion<double>& rotc, const auv_msgs::NavSts::ConstPtr& msg)
{
    nav_msgs::Odometry::Ptr out(new nav_msgs::Odometry());

    out->twist.twist.linear.x = msg->body_velocity.x;
    out->twist.twist.linear.y = msg->body_velocity.y;
    out->twist.twist.linear.z = msg->body_velocity.z;
    out->twist.twist.angular.x = msg->orientation_rate.roll;
    out->twist.twist.angular.y = msg->orientation_rate.pitch;
    out->twist.twist.angular.z = msg->orientation_rate.yaw;

    Eigen::Vector3d ned;
    ned<<msg->position.north, msg->position.east, msg->position.depth;
    Eigen::Vector3d xyz = rot*ned;

    out->pose.pose.position.x = xyz(0);
    out->pose.pose.position.y = xyz(1);
    out->pose.pose.position.z = xyz(2);

    Eigen::Quaternion<double> q;
    labust::tools::quaternionFromEulerZYX(msg->orientation.roll,
    		msg->orientation.pitch,
			msg->orientation.yaw,
			q);

    Eigen::Quaternion<double> q2 = rotc*q;
    out->pose.pose.orientation.w = q2.w();
    out->pose.pose.orientation.x = q2.x();
    out->pose.pose.orientation.y = q2.y();
    out->pose.pose.orientation.z = q2.z();

    odom.publish(out);
}


int main(int argc, char* argv[])
{
	ros::init(argc,argv,"navsts2odom");
	ros::NodeHandle nh, ph("~");

	//Get rotation between the two
	std::vector<double> rpy_pos(3,0), rpy_ang(3,0);
	ph.param("rpy_position",rpy_pos,rpy_pos);
	ph.param("rpy_orientation",rpy_ang,rpy_ang);

	//Setup the LTP to Odom frame
	Eigen::Quaternion<double> q, rot_ang;
	labust::tools::quaternionFromEulerZYX(rpy_pos[0], rpy_pos[1], rpy_pos[2], q);
	labust::tools::quaternionFromEulerZYX(rpy_ang[0], rpy_ang[1], rpy_ang[2], rot_ang);
	Eigen::Matrix3d rot_pos = q.toRotationMatrix().transpose();

	ros::Publisher odom = nh.advertise<nav_msgs::Odometry>("uwsim_hook", 1);
	ros::Subscriber navsts = nh.subscribe<auv_msgs::NavSts>("navsts",1,
			boost::bind(&onNavSts, boost::ref(odom), boost::ref(rot_pos), boost::ref(rot_ang), _1));

	ros::spin();
	return 0;
}
