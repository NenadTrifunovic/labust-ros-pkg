/*********************************************************************
 * docking.cpp
 *
 *  Created on: Aug 23, 2016
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, LABUST, UNIZG-FER
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
#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>
#include <boost/array.hpp>

#include <labust/primitive/PrimitiveBase.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/math/Line.hpp>
#include <labust/math/Signum.hpp>
#include <labust/tools/conversions.hpp>

#include <ros/ros.h>
#include <navcon_msgs/DockingAction.h>
#include <navcon_msgs/EnableControl.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <auv_msgs/BodyVelocityReq.h>

namespace labust
{
	namespace primitive
	{
		/*************************************************************
		 *** Docking primitive class
		 ************************************************************/
		struct Docking : protected ExecutorBase<navcon_msgs::DockingAction>
		{
			typedef navcon_msgs::DockingGoal Goal;
			typedef navcon_msgs::DockingResult Result;
			typedef navcon_msgs::DockingFeedback Feedback;

			enum { hdg, numcnt};
			enum {xp = 0, yp, zp};
			enum {IDLE = 0, SEARCH, APPROACH};

			Docking():ExecutorBase("docking"),
						 processNewGoal(false),
						 new_meas(false),
						 docking_state(IDLE),
						 horizontal_meas(0.0),
						 vertical_meas(0.0),
						 size_meas(0.0),
						 last_direction(0){}


			void init()
			{
				ros::NodeHandle nh,ph("~");

				/*** Initialize controller names ***/
				controllers.name.resize(numcnt);
				controllers.state.resize(numcnt, false);
				controllers.name[hdg] = "HDG_enable";

				pub_docking_arm = nh.advertise<std_msgs::Float32MultiArray>("dock_out",1);
			
				size_flag=0;
			}

			void onGoal()
			{
				boost::mutex::scoped_lock l(state_mux);
				/*** Set the flag to avoid disabling controllers on preemption ***/
				processNewGoal = true;
				Goal::Ptr new_goal = boost::make_shared<Goal>(*(aserver->acceptNewGoal()));
				processNewGoal = false;

				/*** Display goal info ***/
				ROS_INFO("docking: Primitive action goal received.");


				/*** Check if course keeping is possible. ***/
//				if (new_goal->speed == 0)
//				{
//					ROS_ERROR("Cannot perform course keeping without forward speed.");
//					aserver->setAborted(Result(), "Forward speed is zero.");
//				}

				ros::NodeHandle nh;
				sub_vertical = nh.subscribe<std_msgs::Float32>("docking_vertical",1,&Docking::onVerticalMeasurement,this);
				sub_horizontal = nh.subscribe<std_msgs::Float32>("docking_horizontal",1,&Docking::onHorizontalMeasurement,this);
				sub_size = nh.subscribe<std_msgs::Float32>("size",1,&Docking::onSizeMeasurement,this);



			    goal = new_goal;

				/*** Update reference ***/
				//stateRef.publish(step(lastState));

				/*** Enable controllers depending on the primitive subtype ***/
				controllers.state[hdg] = false; //&& goal->axis_enable.x && goal->axis_enable.y;;
				this->updateControllers();
			}

			void onPreempt()
			{
				ROS_WARN("docking: Goal preempted.");
				if (!processNewGoal)
				{
					goal.reset();
					ROS_INFO("docking: Stopping controllers.");
					controllers.state.assign(numcnt, false);
					this->updateControllers();
				}
				else
				{
					//ROS_ERROR("New goal processing.");
				}
				aserver->setPreempted();
			};

			void updateControllers()
			{
				ros::NodeHandle nh;
				ros::ServiceClient cl;
				navcon_msgs::EnableControl a;

				/*** Enable or disable hdg controller ***/
				cl = nh.serviceClient<navcon_msgs::EnableControl>(std::string(controllers.name[hdg]).c_str());
				a.request.enable = controllers.state[hdg];
				cl.call(a);
			}

			void onStateHat(const auv_msgs::NavSts::ConstPtr& estimate)
			{
				/*** Enable mutex ***/
				boost::mutex::scoped_lock l(state_mux);

				if(aserver->isActive())
				{
					/*** Publish reference for high-level controller ***/
					//stateRef.publish(step(*estimate));

					/*** Publish reference for low-level controller ***/
					nuRef.publish(step(*estimate));

					/*** Publish reference for docking arm ***/
					std_msgs::Float32MultiArray docking_arm_ref;
					docking_arm_ref.data.push_back(1.0);
					docking_arm_ref.data.push_back(0.0);
					docking_arm_ref.data.push_back(0.0);
					docking_arm_ref.data.push_back(0.0);
					pub_docking_arm.publish(docking_arm_ref);

				    /*** Check if goal (docking) is achieved ***/


					/*** If goal is completed ***/
					//if (size_meas>20000) size_flag++;
					//if (size_meas>size_max) size_max=size_meas;
					if ((vertical_meas<0.4) && (size_meas>5))
					{
						ROS_ERROR("Closing docking arm.");
						/*** Publish reference for docking arm ***/
						std_msgs::Float32MultiArray docking_arm_ref;
						docking_arm_ref.data.push_back(0.0);
						docking_arm_ref.data.push_back(0.0);
						docking_arm_ref.data.push_back(0.0);
						docking_arm_ref.data.push_back(0.0);
						pub_docking_arm.publish(docking_arm_ref);

						//result.distance = distVictory;
						aserver->setSucceeded(result);
						goal.reset();
						ROS_INFO("docking: Goal completed. Stopping controllers.");
						controllers.state.assign(numcnt, false);
						this->updateControllers();
						return;
					}

					//size_last=size_meas;
					/*** Publish primitive feedback ***/
					Feedback feedback;
					aserver->publishFeedback(feedback);
						this->updateControllers();
				}
				lastState = *estimate;
			}

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& state)
			{
				if((ros::Time::now()-new_meas_time).toSec() < 1  && new_meas == true)
				{
					docking_state == APPROACH;
					ROS_ERROR("Approach state.");
					return approachState();
				}
				else
				{
					docking_state == SEARCH;
					ROS_ERROR("Search state.");
					new_meas = false;
					return searchState();
				}
			}

			auv_msgs::BodyVelocityReqPtr searchState()
			{
				double search_yaw_speed = 0.01;
				auv_msgs::BodyVelocityReqPtr ref(new auv_msgs::BodyVelocityReq());
				ref->twist.linear.x = 0;
				ref->twist.linear.y = 0;
				ref->twist.angular.z = (last_direction != 0)?last_direction*search_yaw_speed:search_yaw_speed;
				ref->header.frame_id = tf_prefix + "local";
				ref->header.stamp = ros::Time::now();
				return ref;
			}

			auv_msgs::BodyVelocityReqPtr approachState()
			{
				double gain = 0.2;
				double surge_gain = 0.1;
				double stdev = 0.1;
				auv_msgs::BodyVelocityReqPtr ref(new auv_msgs::BodyVelocityReq());
				ref->twist.linear.x = surge_gain*std::exp(-(std::pow(horizontal_meas,2))/(2*std::pow(stdev,2)));
				ref->twist.linear.y = 0;
				//ref->twist.angular.z = gain*horizontal_meas;
				ref->twist.angular.z = gain*std::tanh(horizontal_meas);

				ref->header.frame_id = tf_prefix + "local";
				ref->header.stamp = ros::Time::now();
				return ref;
			}

			void onVerticalMeasurement(const std_msgs::Float32::ConstPtr& data)
			{
				vertical_meas = data->data;
				new_meas = true;
				new_meas_time = ros::Time::now();
				ROS_ERROR("Received vertical measurement: %f", vertical_meas);
			}

			void onHorizontalMeasurement(const std_msgs::Float32::ConstPtr& data)
			{
				horizontal_meas = data->data;
				new_meas = true;
				new_meas_time = ros::Time::now();
				last_direction = labust::math::sgn(horizontal_meas);
				ROS_ERROR("Received horizontal measurement: %f", horizontal_meas);
			}

			void onSizeMeasurement(const std_msgs::Float32::ConstPtr& data)
			{
				size_meas = data->data;
				//new_meas = true;
				//new_meas_time = ros::Time::now();
				//last_direction = labust::math::sgn(horizontal_meas);
				ROS_ERROR("Received size measurement: %f", size_meas);
			}

			Result result;

		private:

			geometry_msgs::Point lastPosition;
			labust::math::Line line, bearing_to_endpoint;
			tf2_ros::StaticTransformBroadcaster broadcaster;
			Goal::ConstPtr goal;
			auv_msgs::NavSts lastState;
			boost::mutex state_mux;
			navcon_msgs::ControllerSelectRequest controllers;
			bool processNewGoal;

			ros::Subscriber sub_vertical, sub_horizontal, sub_size;
			ros::Publisher pub_docking_arm;

			double vertical_meas, horizontal_meas, size_meas;
			bool new_meas;
			ros::Time new_meas_time;
			int docking_state, last_direction, size_flag, size_max;
		};
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"docking");
	labust::primitive::PrimitiveBase<labust::primitive::Docking> primitive;
	ros::spin();
	return 0;
}








