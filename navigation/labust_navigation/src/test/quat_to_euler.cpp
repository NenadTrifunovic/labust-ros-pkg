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
 *  Author : Dula Nad
 *  Created: 26.03.2013.
 *********************************************************************/
#include <labust/tools/conversions.hpp>

#include <auv_msgs/RPY.h>
#include <sensor_msgs/Imu.h>

#include <ros/ros.h>

void handleImu(ros::Publisher& rpy, const sensor_msgs::Imu::ConstPtr& data)
{
	double roll,pitch,yaw;
	labust::tools::eulerZYXFromQuaternion(data->orientation, roll, pitch, yaw);
	geometry_msgs::Quaternion q = data->orientation;
	//roll = atan2(2*(q.y*q.z - q.x*q.w),1-2*(q.x*q.x + q.y*q.y));
	//pitch = -asin(2*(q.x*q.z + q.y*q.w));
	//yaw = atan2(2*(q.x*q.y-q.w*q.z),1-2*(q.y*q.y+q.z*q.z));
	auv_msgs::RPY out;
	out.roll = roll/M_PI*180;
	out.pitch = pitch/M_PI*180;
	out.yaw = yaw/M_PI*180;
	rpy.publish(out);

	ROS_INFO("R=%f,P=%f,Y=%f",roll, pitch, yaw);
};



int main(int argc, char* argv[])
{
	ros::init(argc,argv,"quat_to_euler");

	ros::NodeHandle nh;

	//Publishers
	ros::Publisher rpy = nh.advertise<auv_msgs::RPY>("rpy",1);
	ros::Subscriber imu = nh.subscribe<sensor_msgs::Imu>("imu", 1, boost::bind(&handleImu,boost::ref(rpy),_1));

	ros::spin();

	return 0;
}

