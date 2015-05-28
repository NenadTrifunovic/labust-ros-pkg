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
#include <labust/primitive/PrimitiveBase.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/math/Line.hpp>
#include <labust/tools/conversions.hpp>

#include <Eigen/Dense>
#include <navcon_msgs/DynamicPositioningAction.h>
#include <navcon_msgs/EnableControl.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>
#include <boost/array.hpp>

namespace labust
{
	namespace primitive
	{

		struct DynamicPositionining : protected ExecutorBase<navcon_msgs::DynamicPositioningAction>{

			typedef navcon_msgs::DynamicPositioningGoal Goal;
			typedef navcon_msgs::DynamicPositioningResult Result;

			enum {ualf=0,falf, fadp, hdg, numcnt};

			DynamicPositionining():
				ExecutorBase("dynamic_positioning"),
				underactuated(true),
				headingEnabled(false),
				processNewGoal(false){};

			void init(){
				ros::NodeHandle ph("~");

				/*** Initialize controller names ***/
				controllers.name.resize(numcnt);
				controllers.state.resize(numcnt, false);

				controllers.name[fadp] = "FADP_enable";
				controllers.name[hdg] = "HDG_enable";
			}

			void onGoal(){

				boost::mutex::scoped_lock l(state_mux);
				ROS_DEBUG("On goal.");
				//Set the flag to avoid disabling controllers on preemption

			//	if(aserver->isActive()){

				//	aserver->setPreempted();
				//}

				processNewGoal = true;
				Goal::ConstPtr new_goal = aserver->acceptNewGoal();
				processNewGoal = false;


				//if ((goal == 0) || (new_goal->course != goal->course))
//				if ((goal == 0) || (new_goal->T1.point.x != goal->T1.point.x)
//								|| (new_goal->T1.point.y != goal->T1.point.y)
//								|| (new_goal->yaw != goal->yaw))
//				{

				if (goal == 0){

					//Save new goal
					goal = new_goal;

					//Update reference
					stateRef.publish(step(lastState));
                    controllers.state[fadp] = true;
                    controllers.state[hdg] = true;


					this->updateControllers();
				}

				//Save new goal
				goal = new_goal;
			}

			void onPreempt()
			{
				ROS_ERROR("Preempted.");
				if (!processNewGoal)
				{
					//ROS_ERROR("Stopping controllers.");
					//controllers.state.assign(numcnt, false);
					//this->updateControllers();
				}
				else
				{
					//ROS_ERROR("New goal processing.");
				}
				aserver->setPreempted();
			};

			void updateControllers(){

				/* Enable high level controllers */
				ros::NodeHandle nh;
				ros::ServiceClient cl;

				cl = nh.serviceClient<navcon_msgs::EnableControl>(std::string(controllers.name[fadp]));
				navcon_msgs::EnableControl a;
                a.request.enable =  controllers.state[fadp];
				cl.call(a);

				cl = nh.serviceClient<navcon_msgs::EnableControl>(std::string(controllers.name[hdg]));
                a.request.enable =  controllers.state[fadp];
				cl.call(a);
			}

			void onStateHat(const auv_msgs::NavSts::ConstPtr& estimate){

				boost::mutex::scoped_lock l(state_mux);

				if (aserver->isActive()){

					stateRef.publish(step(*estimate));
				}
				else if (goal != 0){

						goal.reset();
						ROS_INFO("Stopping controllers.");
						controllers.state.assign(numcnt, false);
						this->updateControllers();
				}

				lastState = *estimate;
			}

			auv_msgs::NavStsPtr step(const auv_msgs::NavSts& state)
			{
				auv_msgs::NavStsPtr ref(new auv_msgs::NavSts());

				ref->position.north = goal->T1.point.x;
				ref->position.east = goal->T1.point.y;
				ref->orientation.yaw = goal->yaw;
				ref->header.frame_id = "local";
				ref->header.stamp = ros::Time::now();

				return ref;
			}

		private:
			geometry_msgs::Point lastPosition;
			labust::math::Line line;
			tf2_ros::StaticTransformBroadcaster broadcaster;
			bool underactuated;
			bool headingEnabled;
			bool processNewGoal;
			Goal::ConstPtr goal;
			auv_msgs::NavSts lastState;
			boost::mutex state_mux;
			navcon_msgs::ControllerSelectRequest controllers;
		};
	}
}

int main(int argc, char* argv[]){

	ros::init(argc,argv,"dynamic_positioning");

	labust::primitive::PrimitiveBase<labust::primitive::DynamicPositionining> primitive;
	ros::spin();

	return 0;
}



