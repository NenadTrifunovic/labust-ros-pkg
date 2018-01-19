/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, LABUST, UNIZG-FER
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
#include <labust_msgs/GoToPointAction.h>
#include <labust_msgs/EnableControl.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/Int32.h>


namespace labust
{
	namespace primitive
	{
		/*************************************************************
		 *** Go2Point primitive class
		 ************************************************************/
		struct GoToPoint : protected ExecutorBase<labust_msgs::GoToPointAction>
		{
			typedef labust_msgs::GoToPointGoal Goal;
			typedef labust_msgs::GoToPointResult Result;
			typedef labust_msgs::GoToPointFeedback Feedback;

			enum {ualf = 0, falf, hdg, depth, numcnt};
			enum {xp = 0, yp, zp};
			enum {x=0,y,z,roll,pitc,yaw};

			GoToPoint():ExecutorBase("go2point"),
						 underactuated(false),
						 processNewGoal(false),
						 lastDistance(0.0),
						 distVictory(0.0),
						 Ddistance(0.0){}

			void init()
			{
				ros::NodeHandle nh,ph("~");

				/*** Initialize controller names ***/
				controllers.name.resize(numcnt);
				controllers.state.resize(numcnt, false);
				controllers.name[ualf] = "UALF_enable";
				controllers.name[falf] = "FALF_enable";
				controllers.name[hdg] = "HDG_enable";
				controllers.name[depth] = "DEPTH_enable";

				pub_goto_status = nh.advertise<std_msgs::Int32>("goto_status",1);
				mission_status.data=0;
				pub_goto_status.publish(mission_status);

			}

			void onGoal()
			{
				boost::mutex::scoped_lock l(state_mux);
				/*** Set the flag to avoid disabling controllers on preemption ***/
				processNewGoal = true;
				Goal::Ptr new_goal = boost::make_shared<Goal>(*(aserver->acceptNewGoal()));
				processNewGoal = false;
				mission_status.data=1;
                                pub_goto_status.publish(mission_status);


				/*** Display goal info ***/
				ROS_INFO("go2point: Primitive action goal received.");

				// Vidi kakav redoslijed
				/*** Check primitive subtype ***/
				switch(new_goal->subtype)
				{
					case Goal::GO2POINT_UA:
						underactuated = true;
						ROS_ERROR("go2point: underactuated");
						break;
					case Goal::GO2POINT_FA:
						underactuated = false;
						ROS_ERROR("go2point: Fully actuated");

						new_goal->heading = atan2(new_goal->T2.point.y-new_goal->T1.point.y,new_goal->T2.point.x-new_goal->T1.point.x);
						break;
					case Goal::GO2POINT_FA_HDG:
						underactuated = false;
						if(new_goal->ref_type != Goal::CONSTANT)
							//connectTopics();
						break;
				}

				/*** Check if course keeping is possible. ***/
				if (new_goal->speed == 0)
				{
					ROS_ERROR("Cannot perform go2point without forward speed.");
					aserver->setAborted(Result(), "Forward speed is zero.");
				}
				else if(new_goal->victory_radius <= 0)
				{
					ROS_ERROR("Cannot perform go2point without victory radius.");
					aserver->setAborted(Result(), "Victory radius is zero.");
				}
//				if ((goal == 0) || (new_goal->T1.point.x != goal->T1.point.x)
//								|| (new_goal->T1.point.y != goal->T1.point.y)
//								|| (new_goal->T2.point.x != goal->T2.point.x)
//								|| (new_goal->T2.point.y != goal->T2.point.y)
//								|| (new_goal->heading != goal->heading)
//								|| (new_goal->speed != goal->speed))
//				{

			    goal = new_goal;

				if(goal->axis_enable[x] && goal->axis_enable[y])
				{
					/*** Calculate new course line ***/
					Eigen::Vector3d T1,T2;
					T1 << new_goal->T1.point.x, new_goal->T1.point.y, 0;
					T2 << new_goal->T2.point.x, new_goal->T2.point.y, 0;
					line.setLine(T1,T2);

					geometry_msgs::TransformStamped transform;
					transform.transform.translation.x = T1(xp);
					transform.transform.translation.y = T1(yp);
					transform.transform.translation.z = T1(zp);
					labust::tools::quaternionFromEulerZYX(0, 0, line.gamma(),
							transform.transform.rotation);
					transform.child_frame_id = tf_prefix + "course_frame";
					transform.header.frame_id = tf_prefix + "map_ned";
					transform.header.stamp = ros::Time::now();
					broadcaster.sendTransform(transform);
				}

					/*** Update reference ***/
					stateRef.publish(step(lastState));

					/*** Enable controllers depending on the primitive subtype ***/
					if (!underactuated)
					{
						/*** Fully actuated ***/
						controllers.state[falf] = true && goal->axis_enable[x] && goal->axis_enable[y];
						controllers.state[hdg] = true && goal->axis_enable[x] && goal->axis_enable[y];
						controllers.state[depth] = goal->axis_enable[z];
					}
					else
					{
						/*** Under actuated ***/
						double delta = labust::math::wrapRad(lastState.orientation.z - line.gamma());
						if (std::abs(delta) < M_PI_2)
						{
							controllers.state[ualf] = true && goal->axis_enable[x] && goal->axis_enable[y];
							controllers.state[hdg] = false && goal->axis_enable[x] && goal->axis_enable[y];
							controllers.state[depth] = goal->axis_enable[z];
						}
					}
					this->updateControllers();
			}

			void onPreempt()
			{
				ROS_WARN("go2point: Goal preempted.");
				if (!processNewGoal)
				{
					goal.reset();
					ROS_INFO("go2point: Stopping controllers.");
					controllers.state.assign(numcnt, false);
					this->updateControllers();
				}
				else
				{
					//ROS_ERROR("New goal processing.");
				}
				mission_status.data=2;
                                pub_goto_status.publish(mission_status);
				aserver->setPreempted();
			};

			void updateControllers()
			{
				ros::NodeHandle nh;
				ros::ServiceClient cl;

				/*** Enable or disable ualf/falf controller ***/
				if(underactuated)
					cl = nh.serviceClient<labust_msgs::EnableControl>(std::string(controllers.name[ualf]).c_str());
				else
					cl = nh.serviceClient<labust_msgs::EnableControl>(std::string(controllers.name[falf]).c_str());

				labust_msgs::EnableControl a;
				a.request.enable = controllers.state[ualf] || controllers.state[falf];
				cl.call(a);

				/*** Enable or disable hdg controller ***/
				cl = nh.serviceClient<labust_msgs::EnableControl>(std::string(controllers.name[hdg]).c_str());
				a.request.enable = controllers.state[hdg];
				cl.call(a);

				/*** Enable or disable depth controller ***/
				cl = nh.serviceClient<labust_msgs::EnableControl>(std::string(controllers.name[depth]).c_str());
				a.request.enable = controllers.state[depth];
				cl.call(a);
			}

			void onStateHat(const auv_msgs::NavigationStatus::ConstPtr& estimate)
			{
				/*** Enable mutex ***/
				boost::mutex::scoped_lock l(state_mux);

				if(aserver->isActive())
				{
					/*** Publish reference for high-level controller ***/
					stateRef.publish(step(*estimate));

				    /*** Check if goal (victory radius) is achieved ***/
					Eigen::Vector3d deltaVictory;
					deltaVictory<<
							(goal->axis_enable[x]?goal->T2.point.x-estimate->position.north:0.0),
							(goal->axis_enable[y]?goal->T2.point.y-estimate->position.east:0.0),
							(goal->axis_enable[z]?goal->T2.point.z-estimate->position.depth:0.0);

					distVictory = deltaVictory.norm();
					Ddistance = distVictory - lastDistance;
					lastDistance = distVictory;

					/*** If goal is completed ***/
					if(distVictory < goal->victory_radius)
					{
						result.position.point.x = estimate->position.north;
						result.position.point.y = estimate->position.east;
						result.distance = distVictory;
						result.bearing = bearing_to_endpoint.gamma();
						aserver->setSucceeded(result);
						goal.reset();
						ROS_INFO("go2point: Goal completed. Stopping controllers.");
						mission_status.data=3;
		                                pub_goto_status.publish(mission_status);
						controllers.state.assign(numcnt, false);
						this->updateControllers();
						return;
					}

					/*** Publish primitive feedback ***/
					Feedback feedback;
					feedback.distance = distVictory;
					feedback.bearing = bearing_to_endpoint.gamma();
					aserver->publishFeedback(feedback);
				}
				else if (goal != 0)
				{
						goal.reset();
						ROS_INFO("go2point: Stopping controllers.");
						controllers.state.assign(numcnt, false);
						this->updateControllers();
				}
				lastState = *estimate;
			}

			auv_msgs::NavigationStatusPtr step(const auv_msgs::NavigationStatus& state)
			{
				auv_msgs::NavigationStatusPtr ref(new auv_msgs::NavigationStatus());
				auv_msgs::NavigationStatusPtr ref_local(new auv_msgs::NavigationStatus());

				ref->header.frame_id = tf_prefix + "course_frame";

				/*** Handle depth ***/
				if(goal->axis_enable[z])
				{
					ref->position.depth = goal->T2.point.z;

					//ref_local->position.depth = goal->T2.point.z;
					//ref_local->header.frame_id = tf_prefix + "local";
					//ref_local->header.stamp = ros::Time::now();
					//stateRef.publish(ref_local); //TODO Change how this is done
				}


				/*** Set course frame references ***/
				if(goal->axis_enable[x] && goal->axis_enable[y])
				{
					ref->position.east = 0;
					ref->body_velocity.x = goal->speed;
					ref->orientation.z = goal->heading;


					/*** Calculate bearing to endpoint ***/
					Eigen::Vector3d T1,T2;
					T1 << state.position.north, state.position.east, 0;
					T2 << goal->T2.point.x, goal->T2.point.y, 0;
					bearing_to_endpoint.setLine(T1,T2);

					/*** If victory radius is missed change course  ***/
					if(std::abs(labust::math::wrapRad(bearing_to_endpoint.gamma() - line.gamma())) > 60*M_PI/180 && Ddistance > 0)
					{

						ROS_ERROR("go2point: Victory radius missed. Changing course.");
						line = bearing_to_endpoint;

						geometry_msgs::TransformStamped transform;
						transform.transform.translation.x = T1(xp);
						transform.transform.translation.y = T1(yp);
						transform.transform.translation.z = T1(zp);
						labust::tools::quaternionFromEulerZYX(0, 0, line.gamma(),
								transform.transform.rotation);
						transform.child_frame_id = tf_prefix + "course_frame";
						transform.header.frame_id = tf_prefix + "map_ned";
						transform.header.stamp = ros::Time::now();
						broadcaster.sendTransform(transform);
					}

					/*** Check underactuated behaviour ***/
					if (underactuated)
					{
						ref->orientation.z = line.gamma();
						double delta = labust::math::wrapRad(state.orientation.z - line.gamma());
						ROS_DEBUG("Delta, gamma: %f, %f",delta, line.gamma());

						if (controllers.state[hdg] && (std::abs(delta) < M_PI/3))
						{
								/*** Disable hdg and activate ualf ***/
								controllers.state[hdg] = false;
								controllers.state[ualf] = true;
								this->updateControllers();
								ref->header.frame_id = tf_prefix + "course_frame";
						}
						else if (std::abs(delta) >= M_PI/2)
						{
								/*** Deactivate ualf and activate hdg ***/
								controllers.state[hdg] = true;
								controllers.state[ualf] = false;
								this->updateControllers();
								ref->header.frame_id = tf_prefix + "map_ned";
						}
					}
				}
				ref->header.stamp = ros::Time::now();
				return ref;
			}

			Result result;

		private:

			geometry_msgs::Point lastPosition;
			labust::math::Line line, bearing_to_endpoint;
			tf2_ros::StaticTransformBroadcaster broadcaster;
			Goal::ConstPtr goal;
			auv_msgs::NavigationStatus lastState;
			boost::mutex state_mux;
			labust_msgs::ControllerSelectRequest controllers;
			double distVictory, lastDistance, Ddistance;
			bool processNewGoal, underactuated;
			ros::Publisher pub_goto_status;
			std_msgs::Int32 mission_status;
		};
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"go2point");
	labust::primitive::PrimitiveBase<labust::primitive::GoToPoint> primitive;
	ros::spin();
	return 0;
}








