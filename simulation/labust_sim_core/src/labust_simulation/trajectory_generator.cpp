/*********************************************************************
 *  trajectory_generator.cpp
 *
 *  Created on: Oct 5, 2015
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, LABUST, UNIZG-FER
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
#include <Eigen/Dense>
#include <ros/ros.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <underwater_msgs/USBLFix.h>
#include <labust/math/NumberManipulation.hpp>

#include <auv_msgs/BodyVelocityReq.h>

namespace labust
{
	namespace sim
	{
		class TrajectoryGenerator{

		public:

			TrajectoryGenerator()
			{

				ros::NodeHandle nh;

				//subVehiclePos = nh.subscribe<auv_msgs::NavSts>("pos_first",1,&USBLSim::onVehiclePos,this);
				//subTargetPos = nh.subscribe<auv_msgs::NavSts>("pos_second",1,&USBLSim::onTargetPos,this);
				subStartGenerator = nh.subscribe<std_msgs::Bool>("start_trajectory_generator",1,&TrajectoryGenerator::onStartGenerator,this);

				pubNuRef = nh.advertise<auv_msgs::BodyVelocityReq>("diver/nuRef",1);
				//TODO Add position reference
				pubStateRef = nh.advertise<auv_msgs::NavSts>("diver/stateRef",1);
			}

			~TrajectoryGenerator(){}

			void start()
			{

				ros::NodeHandle ph("~");
				double Ts(0.5);
				ph.param("Ts",Ts,Ts);
				ros::Rate rate(1/Ts);

				double surge_speed;
				ph.getParam("surge_speed",surge_speed);
				double surge_time;
				ph.getParam("surge_time",surge_time);
				double turn_radius;
				ph.getParam("turn_radius",turn_radius);

				double yaw_time = turn_radius*M_PI/surge_speed;

				unsigned int i = 0;
				double time = 0;
				while (ros::ok())
				{
					time = Ts*i++;
					auv_msgs::BodyVelocityReq::Ptr nuRef(new auv_msgs::BodyVelocityReq);
					nuRef->twist.linear.x = surge_speed;
					nuRef->twist.angular.z = 0;
					if(time > surge_time &&  time < surge_time+yaw_time)
					{
						nuRef->twist.angular.z = -surge_speed/turn_radius;
					}

					pubNuRef.publish(nuRef);
					rate.sleep();
					ros::spinOnce();
				}
			}

			void onVehiclePos(const auv_msgs::NavSts::ConstPtr& data)
			{
				//vehPos << data->position.north, data->position.east, data->position.depth;
				//vehYaw = data->orientation.yaw;
			}

			void onStartGenerator(const std_msgs::Bool::ConstPtr& data)
			{

			}

			/*
			 * Class variables
			 */

			ros::Subscriber subVehiclePos, subTargetPos;
			ros::Subscriber subStartGenerator;
			ros::Publisher pubNuRef, pubStateRef;



			Eigen::Vector3d vehPos, tarPos;

		};
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"trajectory_generator");
	labust::sim::TrajectoryGenerator usblSim;
	usblSim.start();
	return 0;
}












