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
#include <std_msgs/Bool.h>
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

				kinect_reached=false;
				pullback_done=false;

				pub_docking_arm = nh.advertise<std_msgs::Float32MultiArray>("dock_out",1);
				pub_kinect_servo = nh.advertise<std_msgs::Float32>("kinect_out",1);
				
				//std_msgs::Float32MultiArray docking_arm_ref = std_msgs::Float32MultiArray();
				//std_msgs::Float32 servo_ref = std_msgs::Float32();
				
				docking_arm_ref.data.push_back(0.0);
				docking_arm_ref.data.push_back(0.0);
				docking_arm_ref.data.push_back(0.0);
				docking_arm_ref.data.push_back(0.0);

				struct timespec timeOut,remains;
                                timeOut.tv_sec = 0;
                                timeOut.tv_nsec = 50000000; /* 50 milliseconds */
				servo_ref.data=0.0;
				for (int i=0;i<10;i++)
                                                {
                                                pub_kinect_servo.publish(servo_ref);
                                                nanosleep(&timeOut, &remains);
                                                }
				
				kinect_servo_pos[0]=0.1;
				kinect_servo_pos[1]=0.46;
				kinect_servo_pos[2]=0.75;
				kinect_servo_pos[3]=1.0;
				ROS_ERROR("DOCKING SERVO VALUES INITED %f, %f, %f, %f", kinect_servo_pos[0],kinect_servo_pos[1],kinect_servo_pos[2],kinect_servo_pos[3]);
			}

			void onGoal()
			{
				kinect_reached=false;
				boost::mutex::scoped_lock l(state_mux);
				/*** Set the flag to avoid disabling controllers on preemption ***/
				processNewGoal = true;
				ROS_ERROR("INIT GOAL");
				Goal::Ptr new_goal = boost::make_shared<Goal>(*(aserver->acceptNewGoal()));
				processNewGoal = false;
				/*** Display goal info ***/
				docking_state==IDLE;
				ROS_INFO("docking: Primitive action goal received.");
				//slot=static_cast<int>(goal->docking_slot);
				//ROS_ERROR("SLOT %d",slot);
				//servo_ref.data = kinect_servo_pos.data[slot];
                                //ROS_ERROR("SERVO VAL %f %d",servo_ref.data, slot);
                                //pub_kinect_servo.publish(servo_ref);

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
				sub_manual = nh.subscribe<std_msgs::Bool>("manual",1,&Docking::onManual,this);



			    goal = new_goal;

				/*** Update reference ***/
				//stateRef.publish(step(lastState));
				struct timespec timeOut,remains;
                                timeOut.tv_sec = 0;
                                timeOut.tv_nsec = 900000000; /* 90 milliseconds */
				//kinect_reached=false;
				servo_ref.data=0.0;
                                for (int i=0;i<15;i++)
                                                {
                                                pub_kinect_servo.publish(servo_ref);
                                                nanosleep(&timeOut, &remains);
                                                }
				timeOut.tv_nsec = 200000000;
				nanosleep(&timeOut, &remains);
				slot=static_cast<int>(goal->docking_slot);
                                ROS_ERROR("SLOT %d",slot);
                                //ROS_ERROR("SERVO %f", kinect_servo_pos.data);
                                servo_ref.data=kinect_servo_pos[slot];
				ROS_ERROR("SERVO VAL %f %d",servo_ref.data, slot);
				//kinect_reached=false;
				//for (int i=0;i<10;i++) 
                                //                {
                                                pub_kinect_servo.publish(servo_ref);
                                //                nanosleep(&timeOut, &remains);
                                //                }

                                //pub_kinect_servo.publish(servo_ref);
				timeOut.tv_nsec=900000000;
				for (int i=0;i<15;i++) nanosleep(&timeOut,&remains);
				kinect_reached=true;
				new_meas=false;
				start_time=ros::Time::now();
				ROS_ERROR("Kinect timeout over. Ready to start.");
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
				struct timespec timeOut,remains;
				timeOut.tv_sec = 0;
				timeOut.tv_nsec = 50000000; /* 50 milliseconds */
				//nanosleep(&timeOut, &remains);
				if (!manual_on)
				{
					pub_kinect_servo.publish(servo_ref);
					pub_docking_arm.publish(docking_arm_ref);
				}
				if(aserver->isActive())
				{
					/*** Publish reference for high-level controller ***/
					//stateRef.publish(step(*estimate));
					ROS_ERROR("State updated.");	
					/*** If desired action is undocking ***/
					if(!goal->docking_action)
					{
						ROS_ERROR("Undocking mussel.");
						/*** Publish reference for docking arm ***/
						
						docking_arm_ref.data.at(slot) = 1.0;
						//pub_docking_arm.publish(docking_arm_ref);
						//for (int i=0;i<10;i++) 
						//{
						//pub_docking_arm.publish(docking_arm_ref);
						//nanosleep(&timeOut, &remains);
						//}
						//pub_docking_arm.publish(docking_arm_ref);

						ROS_ERROR("Published arm ref");
						/*** Move backwards to dislodge mussel ***/	
						nuRef.publish(pullbackState());
						
						if (pullback_done)
						{
						//result.distance = distVictory;
						docking_arm_ref.data.at(slot)=0.0;
						//pub_docking_arm.publish(docking_arm_ref);
						aserver->setSucceeded(result);
						goal.reset();
						new_meas=false;
						vertical_meas=0.0;
						ROS_INFO("docking: Goal completed. Stopping controllers.");
						controllers.state.assign(numcnt, false);
						this->updateControllers();
						return;
						}
					}
					else if (kinect_reached)
					{
					
					/*** Publish reference for low-level controller ***/
					nuRef.publish(step(*estimate));
					
					/*** Publish reference for docking arm ***/
					docking_arm_ref.data.at(slot) = 1.0;
					//pub_docking_arm.publish(docking_arm_ref);
					//for (int i=0;i<10;i++) 
                                        //        {
                                                //pub_docking_arm.publish(docking_arm_ref);
                                        //        nanosleep(&timeOut, &remains);
                                        //       }


				    /*** Check if goal (docking) is achieved ***/


					/*** If goal is completed ***/
					if(vertical_meas>0.8 && vertical_meas<0.9 && kinect_reached)
					{
						/*** Publish reference for docking arm ***/
						docking_arm_ref.data.at(slot) = 0.0;
						//pub_docking_arm.publish(docking_arm_ref);
						//for (int i=0;i<10;i++) 
                                                //{
                                                //pub_docking_arm.publish(docking_arm_ref);
                                                //nanosleep(&timeOut, &remains);
                                                //}
						ROS_ERROR("VERTICAL DOCKING THESHOLD REACHED");
						//result.distance = distVictory;
						aserver->setSucceeded(result);
						goal.reset();
						ROS_INFO("docking: Goal completed. Stopping controllers.");
						controllers.state.assign(numcnt, false);
						this->updateControllers();
						vertical_meas=0.0;
						horizontal_meas=0.0;
						new_meas=false;
						//pub_docking_arm.publish(docking_arm_ref);
						return;
					}
					}

					/*** Publish primitive feedback ***/
					Feedback feedback;
					aserver->publishFeedback(feedback);
				}
				else if (goal != 0)
				{
						goal.reset();
						ROS_INFO("docking: Stopping controllers.");
						controllers.state.assign(numcnt, false);
						this->updateControllers();
				}
				lastState = *estimate;
			}

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& state)
			{
				if((ros::Time::now()-new_meas_time).toSec() < 1  && new_meas == true && kinect_reached == true )
				{
					docking_state == APPROACH;
					ROS_ERROR("Approach state.");
					return approachState();
				}
				else if (kinect_reached)
				{
					docking_state == SEARCH;
					ROS_ERROR("Search state.");
					new_meas = false;
					return searchState();
				}
			}

			auv_msgs::BodyVelocityReqPtr searchState()
			{
				double search_yaw_speed = goal->search_yaw_rate;
				auv_msgs::BodyVelocityReqPtr ref(new auv_msgs::BodyVelocityReq());
				ref->twist.linear.x = 0;
				ref->twist.linear.y = 0;
				ref->twist.angular.z = (last_direction != 0)?last_direction*search_yaw_speed:search_yaw_speed;
				ref->header.frame_id = tf_prefix + "local";
				ref->header.stamp = ros::Time::now();
				return ref;
			}
			
			auv_msgs::BodyVelocityReqPtr pullbackState()
			{
				
				double surge_gain = goal->max_surge_speed;
				auv_msgs::BodyVelocityReqPtr ref(new auv_msgs::BodyVelocityReq());
				double angle = goal->docking_slot*M_PI/2+M_PI;
				double surge_val = 0.1;
				ROS_ERROR("Pulling back from mussel");
				Eigen::Vector2f out, in;
				Eigen::Matrix2f R;
				in<<surge_val,0;
				R<<cos(angle),-sin(angle),sin(angle),cos(angle);
				out = R*in;
				ROS_ERROR("Pullback reference: %f, %f",out[0],out[1]);
				//R<<cos(M_PI),-sin(M_PI),sin(M_PI),cos(M_PI);
				//out = R*out;
				
				if((ros::Time::now()-start_time).toSec() < 8)
				{
					ROS_ERROR("Pullback started.");
					ref->twist.linear.x = out[0];
					ref->twist.linear.y = out[1];
					pullback_done=false;
				}
				else
				{
					ROS_ERROR("Pullback stopped.");
					ref->twist.linear.x = 0.0;
					ref->twist.linear.y = 0.0;
					pullback_done = true;
					docking_arm_ref.data.at(slot)=0.0;
					pub_docking_arm.publish(docking_arm_ref);
				}
				
				ref->header.frame_id = tf_prefix + "local";
				ref->header.stamp = ros::Time::now();
				return ref;
			}

			auv_msgs::BodyVelocityReqPtr approachState()
			{
				double gain = goal->max_yaw_rate;
				double surge_gain = goal->max_surge_speed;
				double stdev = goal->surge_stdev;
				stdev=0.2;
				auv_msgs::BodyVelocityReqPtr ref(new auv_msgs::BodyVelocityReq());
				double angle = goal->docking_slot*M_PI/2;
				double surge_val = surge_gain*std::exp(-(std::pow(horizontal_meas,2))/(2*std::pow(stdev,2)));
				ROS_ERROR("Moving towards mussel");			

				Eigen::Vector2f out, in;
				Eigen::Matrix2f R;
				in<<surge_val,0;
				R<<cos(angle),-sin(angle),sin(angle),cos(angle);
				out = R*in;
				
				//ref->twist.angular.z = gain*horizontal_meas;
				ref->twist.angular.z = gain*std::tanh(horizontal_meas);
				ref->twist.linear.x = out[0];
				ref->twist.linear.y = out[1];

				ref->header.frame_id = tf_prefix + "local";
				ref->header.stamp = ros::Time::now();
				return ref;
			}

			void onVerticalMeasurement(const std_msgs::Float32::ConstPtr& data)
			{
				if (kinect_reached)
				{
				vertical_meas = data->data;
				new_meas = true;
				new_meas_time = ros::Time::now();
				ROS_ERROR("Received vertical measurement: %f", vertical_meas);
				}
			}

			void onHorizontalMeasurement(const std_msgs::Float32::ConstPtr& data)
			{
				if (kinect_reached)
				{
				horizontal_meas = data->data;
				new_meas = true;
				new_meas_time = ros::Time::now();
				last_direction = labust::math::sgn(horizontal_meas);
				last_direction = 1;
				ROS_ERROR("Received horizontal measurement: %f", horizontal_meas);
				}
			}

			void onSizeMeasurement(const std_msgs::Float32::ConstPtr& data)
			{
				if (kinect_reached) 
				{
				//size_meas = data->data;
				//new_meas = true;
				//new_meas_time = ros::Time::now();
				//last_direction = labust::math::sgn(horizontal_meas);
				ROS_ERROR("Received size measurement: %f", size_meas);
				}
			}
			
			void onManual(const std_msgs::Bool::ConstPtr& data)
			{
				manual_on = data->data;
				ROS_ERROR("Received manual on: %d", manual_on);
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
			std_msgs::Float32MultiArray docking_arm_ref;
			float kinect_servo_pos[4];
			std_msgs::Float32 servo_ref;

			ros::Subscriber sub_vertical, sub_horizontal, sub_size, sub_manual;
			ros::Publisher pub_docking_arm, pub_kinect_servo;

			double vertical_meas, horizontal_meas, size_meas;
			bool new_meas, kinect_reached, pullback_done, manual_on;
			ros::Time new_meas_time, start_time, initial_timeout;
			int docking_state, last_direction, slot;
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








