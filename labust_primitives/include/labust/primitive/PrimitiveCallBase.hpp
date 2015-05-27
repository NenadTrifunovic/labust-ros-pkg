/*********************************************************************
 * PrimitiveCallBase.hpp
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

#ifndef PRIMITIVECALLBASE_HPP_
#define PRIMITIVECALLBASE_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

namespace labust
{
	namespace primitive
	{
		/**
		* The primitive call base class.
		*/
		template <class ActionType, class ActionGoal, class ActionResult, class ActionFeedback>
		class PrimitiveCallBase
		{
		protected:
			typedef ActionType Action;
			typedef actionlib::SimpleActionClient<Action> ActionClient;
			typedef ActionGoal Goal;
			typedef ActionResult Result;
			typedef ActionFeedback Feedback;

			/**
			 * Main constructor
			 */
			PrimitiveCallBase(const std::string& name):primitiveName(name),
														  ac(primitiveName.c_str())
			{
				//TODO Throw exception
				ROS_INFO("Waiting for action server to start.");
				ac.waitForServer(); //will wait for infinite time
				ROS_INFO("Action server started, sending goal.");
			}

			~PrimitiveCallBase(){}

			void callPrimitiveAction(Goal goal)
			{
				ac.sendGoal(goal,
							boost::bind(&PrimitiveCallBase::doneCb, this, _1, _2),
							boost::bind(&PrimitiveCallBase::activeCb, this),
							boost::bind(&PrimitiveCallBase::feedbackCb, this, _1));
			}


			// Called once when the goal completes
			virtual void doneCb(const actionlib::SimpleClientGoalState& state, const typename Result::ConstPtr& result)
			{
				ROS_ERROR("Finished in state [%s]", state.toString().c_str());
			}

			// Called once when the goal becomes active
			virtual void activeCb()
			{
				ROS_ERROR("Goal just went active.");
			}

			// Called every time feedback is received for the goal
			virtual void feedbackCb(const typename Feedback::ConstPtr& feedback)
			{

			}
			/**
			 * The name identifier.
			 */
			std::string primitiveName;
			/**
			 * The action client.
			 */
			ActionClient ac;
			/**
			 * The action var.
			 */
			Action action;
			/**
			* The identification action server.
			*/
			//ActionServerPtr aserver;
			/**
			* The service client for controller activation/deactivation
			*/
			ros::ServiceClient control_manager;
		};
	}
}



#endif /* PRIMITIVECALLBASE_HPP_ */
