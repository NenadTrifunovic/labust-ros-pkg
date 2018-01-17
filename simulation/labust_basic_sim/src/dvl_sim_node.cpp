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
#include <labust/simulation/NoiseModel.hpp>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

#include <Eigen/Dense>

using labust::simulation::NoiseGenerators;

struct DvlSim
{
	DvlSim():
		maxBottomLock(40),
		maxDepth(30),
		dvl_pub(1),
		lastPubTime(ros::Time::now()),
		tf_prefix(""),
		sigma(0)
	{
		ros::NodeHandle nh,ph("~");

		ph.param("MaxBottomLock",maxBottomLock, maxBottomLock);
		ph.param("MaxDepth", maxDepth, maxDepth);
		ph.param("dvl_pub", dvl_pub, dvl_pub);
		std::vector<double> offset(3,0), orot(3,0);
		ph.param("offset", offset, offset);
		ph.param("orot", orot, orot);
		ph.param("sigma", sigma, sigma);

        for(int i=0; i<3; ++i) gen.addNew(0,sigma);

		this->offset<<offset[0],offset[1],offset[2];
		labust::tools::quaternionFromEulerZYX(M_PI*orot[0]/180,
				M_PI*orot[1]/180,
				M_PI*orot[2]/180,
				this->orot);

		std::string key;
		if (nh.searchParam("tf_prefix", key)) nh.getParam(key, tf_prefix);

		odom = nh.subscribe<nav_msgs::Odometry>("meas_odom",1,&DvlSim::onOdom, this);
		dvl_nu = nh.advertise<geometry_msgs::TwistStamped>("dvl",1);
		dvl_ned = nh.advertise<geometry_msgs::TwistStamped>("dvl_ned",1);
		altitude_pub = nh.advertise<std_msgs::Float32>("altitude",1);
		dvl_bottom = nh.advertise<std_msgs::Bool>("dvl_bottom",1);

		last_pos[0] = last_pos[1] = last_pos[2] = 0;
	}

	void onOdom(const typename nav_msgs::Odometry::ConstPtr& msg)
	{
		//Get sampling time
		double dT = (ros::Time::now() - lastTime).toSec();
		lastTime = ros::Time::now();
		//Get bottom lock
		bool hasBottomLock = (maxDepth - msg->pose.pose.position.z) < maxBottomLock;
		//Get rotation local->base_link
		Eigen::Quaternion<double> qv(msg->pose.pose.orientation.w,
				msg->pose.pose.orientation.x,
				msg->pose.pose.orientation.y,
				msg->pose.pose.orientation.z);
		//Set first the water lock speeds
		Eigen::Vector3d nu_b;
		nu_b<<msg->twist.twist.linear.x,
				msg->twist.twist.linear.y,
				msg->twist.twist.linear.z;
		Eigen::Vector3d nu_l = qv.toRotationMatrix()*nu_b;
		if (hasBottomLock)
		{
			double pos[3];
			labust::tools::pointToVector(msg->pose.pose.position, pos);
			for (int i=0; i<3; ++i)
			{
				nu_l[i] = (pos[i] - last_pos[i])/dT;
				last_pos[i] = pos[i];
			}
			nu_b = qv.toRotationMatrix().transpose()*nu_l;
		}
		nu_b += Eigen::Vector3d(gen(0),gen(1),gen(2));
		//Incorporate offsets for DVL frame
		Eigen::Vector3d nur;
		labust::tools::pointToVector(msg->twist.twist.angular, nur);
		Eigen::Matrix3d omega;
		enum {p=0,q,r};
		omega<<0,-nur(r),nur(q),
				nur(r),0,-nur(p),
				-nur(q),nur(p),0;
		nu_b = orot.toRotationMatrix().transpose()*(nu_b + omega*offset);
		nu_l = qv.toRotationMatrix()*nu_b;

		double lp((ros::Time::now() - lastPubTime).toSec());
		if (lp >= 1/dvl_pub)
		{
			lastPubTime = ros::Time::now();
			geometry_msgs::TwistStamped::Ptr dvl(new geometry_msgs::TwistStamped());
			dvl->header.stamp = msg->header.stamp;
			dvl->header.frame_id = tf_prefix + msg->header.frame_id;
			labust::tools::vectorToPoint(nu_l, dvl->twist.linear);
			dvl_ned.publish(dvl);
			dvl.reset(new geometry_msgs::TwistStamped());
			dvl->header.stamp = msg->header.stamp;
			dvl->header.frame_id = tf_prefix + "dvl_frame";
			labust::tools::vectorToPoint(nu_b, dvl->twist.linear);
			dvl_nu.publish(dvl);

			if (hasBottomLock)
			{
				std_msgs::Float32::Ptr altitude(new std_msgs::Float32());
				altitude->data = maxDepth - msg->pose.pose.position.z;
				altitude_pub.publish(altitude);
				std_msgs::Bool bl;
				bl.data = true;
				dvl_bottom.publish(bl);
			}
			else
			{
				std_msgs::Bool bl;
				bl.data = false;
				dvl_bottom.publish(bl);
			}
		}
	}

private:
	ros::Subscriber odom;
	ros::Publisher dvl_nu, altitude_pub, dvl_ned, dvl_bottom;
	ros::Time lastTime;
	Eigen::Vector3d offset;
	Eigen::Quaternion<double> orot;
	double last_pos[3];
	double dvl_pub;
	ros::Time lastPubTime;
	double maxBottomLock, maxDepth;
	std::string tf_prefix;
	NoiseGenerators gen;
	double sigma;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"dvl_sim");
	ros::NodeHandle nh;
	DvlSim dvl;
	ros::spin();
	return 0;
}


