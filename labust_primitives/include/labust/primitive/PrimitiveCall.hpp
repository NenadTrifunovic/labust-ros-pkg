/*
 * PrimitiveCall.hpp
 *
 *  Created on: May 27, 2015
 *      Author: filip
 */

#ifndef PRIMITIVECALL_HPP_
#define PRIMITIVECALL_HPP_

#include <labust/primitive/PrimitiveCallBase.hpp>

#include <navcon_msgs/GoToPointAction.h>


namespace labust
{
	namespace primitive
	{
		class PrimitiveCallGo2Point : protected PrimitiveCallBase<navcon_msgs::GoToPointAction,
																	navcon_msgs::GoToPointGoal,
																	navcon_msgs::GoToPointResult,
																	navcon_msgs::GoToPointFeedback>
		{
		public:
			PrimitiveCallGo2Point():PrimitiveCallBase("go2point")
			{

			}

			~PrimitiveCallGo2Point(){};

			void start(double north1, double east1, double north2, double east2, double speed, double heading, double radius)
			{
				//LLcfg.LL_VELconfigure(true,2,2,0,0,0,2);
				Goal goal;
				goal.ref_type = Goal::CONSTANT;
				goal.subtype = Goal::GO2POINT_FA;


				goal.T1.point.x = north1;
				goal.T1.point.y = east1;
				goal.T1.point.z = 0;
				goal.T2.point.x = north2;
				goal.T2.point.y = east2;
				goal.T2.point.z = 0;
				goal.heading = heading;
				goal.speed = speed;
				goal.victory_radius = radius;

				this->callPrimitiveAction(goal);


//							boost::this_thread::sleep(boost::posix_time::milliseconds(200));
//							ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
//
//							enableController("UALF_enable",false);
//							enableController("HDG_enable",false);
//							LLcfg.LL_VELconfigure(false,1,1,0,0,0,1);

			}


			// Called once when the goal completes
			void doneCb(const actionlib::SimpleClientGoalState& state, const Result::ConstPtr& result)
			{
			ROS_ERROR("Go2PointFA - Finished in state [%s]", state.toString().c_str());

			if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
			mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");

			}

			// Called once when the goal becomes active
			void activeCb()
			{
			ROS_ERROR("Goal just went active go2point_FA");
			}

			// Called every time feedback is received for the goal
			void feedbackCb(const Feedback::ConstPtr& feedback)
			{
			// ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
			//if((counter++)%10 == 0)
			//ROS_ERROR("Feedback - distance: %f", feedback->distance);
			}
		};
	}
}




#endif /* PRIMITIVECALL_HPP_ */
