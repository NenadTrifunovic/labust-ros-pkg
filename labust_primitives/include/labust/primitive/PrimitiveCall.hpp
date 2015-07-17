/*********************************************************************
 * PrimitiveCall.hpp
 *
 *  Created on: May 27, 2015
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
*********************************************************************/

#ifndef PRIMITIVECALL_HPP_
#define PRIMITIVECALL_HPP_

#include <labust/primitive/PrimitiveCallBase.hpp>

#include <navcon_msgs/GoToPointAction.h>
#include <navcon_msgs/CourseKeepingAction.h>
#include <navcon_msgs/DynamicPositioningAction.h>
#include <navcon_msgs/DOFIdentificationAction.h>

#include <ros/ros.h>

namespace labust
{
	namespace primitive
	{
		class PrimitiveCallGo2Point : public PrimitiveCallBase<navcon_msgs::GoToPointAction,
																 navcon_msgs::GoToPointGoal,
																 navcon_msgs::GoToPointResult,
																 navcon_msgs::GoToPointFeedback>
		{
		public:
			PrimitiveCallGo2Point():PrimitiveCallBase("go2point")
			{

			}

			~PrimitiveCallGo2Point(){};

		protected:
			/***  Callback called once when the goal completes ***/
			void doneCb(const actionlib::SimpleClientGoalState& state, const Result::ConstPtr& result)
			{
				if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					ROS_ERROR("Go2PointFA - Finished in state [%s]", state.toString().c_str());
					publishEventString("/PRIMITIVE_FINISHED");
				}
			}

			/*** Callback called once when the goal becomes active ***/
			void activeCb()
			{
			ROS_ERROR("Goal just went active go2point_FA");
			}

			/*** Callback called every time feedback is received for the goal ***/
			void feedbackCb(const Feedback::ConstPtr& feedback)
			{
				   ROS_ERROR("Feedback - distance: %f", feedback->distance);
			}
		};

		class PrimitiveCallCourseKeeping : public PrimitiveCallBase<navcon_msgs::CourseKeepingAction,
																	  navcon_msgs::CourseKeepingGoal,
																	  navcon_msgs::CourseKeepingResult,
																	  navcon_msgs::CourseKeepingFeedback>
		{
		public:
			PrimitiveCallCourseKeeping():PrimitiveCallBase("course_keeping")
			{

			}

			~PrimitiveCallCourseKeeping(){};

		protected:
			/***  Callback called once when the goal completes ***/
			void doneCb(const actionlib::SimpleClientGoalState& state, const Result::ConstPtr& result)
			{
				if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					publishEventString("/PRIMITIVE_FINISHED");
				}
			}

			/*** Callback called once when the goal becomes active ***/
			void activeCb()
			{
			ROS_ERROR("Goal just went active go2point_FA");
			}

			/*** Callback called every time feedback is received for the goal ***/
			void feedbackCb(const Feedback::ConstPtr& feedback)
			{
			// ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
			//if((counter++)%10 == 0)
			//ROS_ERROR("Feedback - distance: %f", feedback->distance);
			}
		};

		class PrimitiveCallDynamicPositioning : public PrimitiveCallBase<navcon_msgs::DynamicPositioningAction,
																	  	  navcon_msgs::DynamicPositioningGoal,
																	  	  navcon_msgs::DynamicPositioningResult,
																	  	  navcon_msgs::DynamicPositioningFeedback>
		{
		public:
			PrimitiveCallDynamicPositioning():PrimitiveCallBase("dynamic_positioning")
			{

			}

			~PrimitiveCallDynamicPositioning(){};

		protected:
			/***  Callback called once when the goal completes ***/
			void doneCb(const actionlib::SimpleClientGoalState& state, const Result::ConstPtr& result)
			{
				if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					ROS_ERROR("Dynamic Positioning - Finished in state [%s]", state.toString().c_str());
					publishEventString("/PRIMITIVE_FINISHED");
				}
			}

			/*** Callback called once when the goal becomes active ***/
			void activeCb()
			{
			ROS_ERROR("Goal just went active dynamic_positioning");
			}

			/*** Callback called every time feedback is received for the goal ***/
			void feedbackCb(const Feedback::ConstPtr& feedback)
			{
			// ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
			//if((counter++)%10 == 0)
			//ROS_ERROR("Feedback - distance: %f", feedback->distance);
			}
		};

		class PrimitiveCallDOFIdentification : public PrimitiveCallBase<navcon_msgs::DOFIdentificationAction,
																	  	  navcon_msgs::DOFIdentificationGoal,
																	  	  navcon_msgs::DOFIdentificationResult,
																	  	  navcon_msgs::DOFIdentificationFeedback>
		{
		public:
			PrimitiveCallDOFIdentification():PrimitiveCallBase("Identification")
			{

			}

			~PrimitiveCallDOFIdentification(){};

		protected:
			/***  Callback called once when the goal completes ***/
			void doneCb(const actionlib::SimpleClientGoalState& state, const Result::ConstPtr& result)
			{
				if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					ROS_ERROR("iso - Finished in state [%s]", state.toString().c_str());
					ROS_ERROR("Result - DOF: %d, alpha: %f, beta: %f, betaa: %f, delta %f, wn: %f", result->dof,result->alpha, result->beta, result->betaa, result->delta, result->wn);
					publishEventString("/PRIMITIVE_FINISHED");
				}
			}

			/*** Callback called once when the goal becomes active ***/
			void activeCb()
			{
			ROS_ERROR("Goal just went active go2point_FA");
			}

			/*** Callback called every time feedback is received for the goal ***/
			void feedbackCb(const Feedback::ConstPtr& feedback)
			{
				   ROS_ERROR("Feedback - dof: %d, error: %f, oscilation_num: %d", feedback->dof, feedback->error, feedback->oscillation_num);
			}
		};
	}
}




#endif /* PRIMITIVECALL_HPP_ */
