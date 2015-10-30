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
#include <labust/simulation/dvl_sim.h>

using namespace labust::simulation;

DVLSim::DVLSim()
{
	//Initialize the noise generators with minimal noise
	for(int i=0; i<noise_gen.size(); ++i)
	{
		noise_gen[i].reset(new NoiseGenerator(rd,
				boost::normal_distribution<>(0,1e-5)));
	}

	//Initialize the measurement mapping output
	this->map_meas.resize(nstate);
	map_meas<<m_u,m_v,m_w,m_alt;
	this->sim_meas = Eigen::VectorXd::Zero(nstate);
};

void DVLSim::step(const vector& eta,
		const vector& nu,
		const vector& nuacc,
		const EnvironmentModel& env)
{
	//Calculate the position derivative to get speed over-ground
	for (int i=0; i<3; ++i) sim_meas(i) = (eta(i) - last_eta(i))/env.Ts;
	last_eta = eta;

	//Rotate into body frame
	//sim_meas.head(3) = R*sim_meas.head(3);
	//Calculate measured speed on sensor due to movement and lever arm
	Eigen::Matrix3d omega;
	enum {p=3,q,r};
	omega<<0,-nu(r),nu(q),
			nu(r),0,-nu(p),
			-nu(q),nu(p),0;
	sim_meas.head(3) = orot.toRotationMatrix()*(sim_meas.head(3) + omega*offset);

	//Calculate the altitude


};

bool DVLSim::setNoise(const Eigen::VectorXd& mean, const Eigen::VectorXd& var)
{
	if ((mean.size() != var.size()) || (mean.size() != nstate)) return false;

	for(int i=0; i<noise_gen.size(); ++i)
	{
		noise_gen[i].reset(new NoiseGenerator(rd,
				boost::normal_distribution<>(mean(i),var(i))));
	}

	return true;
};

/*
		maxBottomLock(100),
		maxDepth(100),
		dvl_pub(1)
	{
		ros::NodeHandle nh,ph("~");

		ph.param("MaxBottomLock",maxBottomLock, maxBottomLock);
		ph.param("MaxDepth", maxDepth, maxDepth);
		ph.param("DvlPub", dvl_pub, dvl_pub);

		odom = nh.subscribe<nav_msgs::Odometry>("meas_odom",1,&DvlSim::onOdom, this);
		dvl_nu = nh.advertise<geometry_msgs::TwistStamped>("dvl",1);
		dvl_ned = nh.advertise<geometry_msgs::TwistStamped>("dvl_ned",1);
		altitude_pub = nh.advertise<std_msgs::Float32>("altitude",1);
		dvl_bottom = nh.advertise<std_msgs::Bool>("dvl_bottom",1);

		last_pos[0] = last_pos[1] = last_pos[2] = 0;
	}

	void onOdom(const typename nav_msgs::Odometry::ConstPtr& msg)
	{
		static int i=0;
		i=++i%dvl_pub;

		if (i == 0)
		{
			double dT = (ros::Time::now() - lastTime).toSec();
			lastTime = ros::Time::now();
			geometry_msgs::TwistStamped::Ptr dvl(new geometry_msgs::TwistStamped());
			dvl->header.stamp = ros::Time::now();
			dvl->header.frame_id = msg->header.frame_id;
			double pos[3],v[3];
			labust::tools::pointToVector(msg->pose.pose.position, pos);
			for (int i=0; i<3; ++i)
			{
				v[i] = (pos[i] - last_pos[i])/dT;
				last_pos[i] = pos[i];
			}
			labust::tools::vectorToPoint(v, dvl->twist.linear);
			dvl_ned.publish(dvl);

			dvl.reset(new geometry_msgs::TwistStamped());
			dvl->header.stamp = ros::Time::now();
			dvl->header.frame_id = msg->child_frame_id;
			//Calculate body-fixed speeds
			Eigen::Quaternion<double> q(msg->pose.pose.orientation.w,
					msg->pose.pose.orientation.x,
					msg->pose.pose.orientation.y,
					msg->pose.pose.orientation.z);
			Eigen::Vector3d nu = q.matrix().transpose() * Eigen::Vector3d(v[0],v[1],v[2]);
			dvl->twist.linear.x = nu(0);
			dvl->twist.linear.y = nu(1);
			dvl->twist.linear.z = nu(2);
			dvl_nu.publish(dvl);

			if ((maxDepth - msg->pose.pose.position.z) < maxBottomLock)
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
	double last_pos[3];
	int dvl_pub;
	double maxBottomLock, maxDepth;
};
*/


