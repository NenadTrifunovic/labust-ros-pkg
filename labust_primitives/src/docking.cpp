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
#include <labust/tools/conversions.hpp>

#include <ros/ros.h>
#include <navcon_msgs/DockingAction.h>
#include <navcon_msgs/EnableControl.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <std_msgs/Float32.h>
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
						 processNewGoal(false){}

			void init()
			{
				ros::NodeHandle nh,ph("~");

				/*** Initialize controller names ***/
				controllers.name.resize(numcnt);
				controllers.state.resize(numcnt, false);
				controllers.name[hdg] = "HDG_enable";
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

//				if ((goal == 0) || (new_goal->T1.point.x != goal->T1.point.x)
//								|| (new_goal->T1.point.y != goal->T1.point.y)
//								|| (new_goal->T2.point.x != goal->T2.point.x)
//								|| (new_goal->T2.point.y != goal->T2.point.y)
//								|| (new_goal->heading != goal->heading)
//								|| (new_goal->speed != goal->speed))
//				{

			    goal = new_goal;

//				if(goal->axis_enable.x && goal->axis_enable.y)
//				{
//					/*** Calculate new course line ***/
//					Eigen::Vector3d T1,T2;
//					T1 << new_goal->T1.point.x, new_goal->T1.point.y, 0;
//					T2 << new_goal->T2.point.x, new_goal->T2.point.y, 0;
//					line.setLine(T1,T2);
//
//					geometry_msgs::TransformStamped transform;
//					transform.transform.translation.x = T1(xp);
//					transform.transform.translation.y = T1(yp);
//					transform.transform.translation.z = T1(zp);
//					labust::tools::quaternionFromEulerZYX(0, 0, line.gamma(),
//							transform.transform.rotation);
//					transform.child_frame_id = tf_prefix + "course_frame";
//					transform.header.frame_id = tf_prefix + "local";
//					transform.header.stamp = ros::Time::now();
//					broadcaster.sendTransform(transform);
//				}

					/*** Update reference ***/
					stateRef.publish(step(lastState));

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
					/*** Publish reference for low-level controller ***/


					/*** Publish reference for high-level controller ***/
					//stateRef.publish(step(*estimate));

					/*
					 *
					 *  Ovdje treba osnovnu logiku ukljuciti
					 *
					 */

				    /*** Check if goal (docking) is achieved ***/

					/*** If goal is completed ***/
					if(false)
					{
						//result.position.point.x = estimate->position.north;
						//result.position.point.y = estimate->position.east;
						//result.distance = distVictory;
						//result.bearing = bearing_to_endpoint.gamma();
						aserver->setSucceeded(result);
						goal.reset();
						ROS_INFO("docking: Goal completed. Stopping controllers.");
						controllers.state.assign(numcnt, false);
						this->updateControllers();
						return;
					}

					/*** Publish primitive feedback ***/
					Feedback feedback;
					//feedback.distance = distVictory;
					//feedback.bearing = bearing_to_endpoint.gamma();
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

			auv_msgs::NavStsPtr step(const auv_msgs::NavSts& state)
			{

				if(docking_state == IDLE)
				{

				}
				else if(docking_state == SEARCH)
				{

				}
				else if(docking_state == APPROACH)
				{

				}


				auv_msgs::NavStsPtr ref(new auv_msgs::NavSts());

				return ref;
			}


			void idleState()
			{

			}

			void searchState()
			{

				auv_msgs::BodyVelocityReqPtr ref(new auv_msgs::BodyVelocityReq());

				ref->twist.angular.z;// = ;

				ref->header.frame_id = tf_prefix + "local";
				ref->header.stamp = ros::Time::now();
				//return ref;

			}

			void approachState()
			{
				auv_msgs::BodyVelocityReqPtr ref(new auv_msgs::BodyVelocityReq());

				ref->twist.linear.x;// = ;
				ref->twist.angular.z;// = ;

				ref->header.frame_id = tf_prefix + "local";
				ref->header.stamp = ros::Time::now();
				//return ref;
			}

			void onVerticalMeasurement(const std_msgs::Float32::ConstPtr& data)
			{
				vertical_meas = data->data;
				new_meas = true;
			}

			void onHorizontalMeasurement(const std_msgs::Float32::ConstPtr& data)
			{
				horizontal_meas = data->data;
				new_meas = true;
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

			double vertical_meas, horizontal_meas;
			bool new_meas;
			int docking_state;
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








