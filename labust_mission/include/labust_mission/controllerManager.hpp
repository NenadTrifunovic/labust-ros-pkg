//TODO Dokumentiraj prototipe funkcija
/*********************************************************************
 * controllerManager.hpp
 *
 *  Created on: Feb 28, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, LABUST, UNIZG-FER
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

#ifndef CONTROLLERMANAGER_HPP_
#define CONTROLLERMANAGER_HPP_

/*********************************************************************
 *** Includes
 ********************************************************************/

#include <cmath>

//#include <auv_msgs/NavSts.h>
#include <auv_msgs/Bool6Axis.h>
#include <navcon_msgs/EnableControl.h>
#include <navcon_msgs/ConfigureAxes.h>
#include <navcon_msgs/ConfigureVelocityController.h>

#include <labust_mission/labustMission.hpp>
#include <labust_mission/lowLevelConfigure.hpp>
#include <labust/primitive/PrimitiveCall.hpp>
#include <labust_mission/utils.hpp>

//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>

#include <navcon_msgs/CourseKeepingAction.h>
#include <navcon_msgs/DynamicPositioningAction.h>
#include <navcon_msgs/GoToPointAction.h>
#include <navcon_msgs/DOFIdentificationAction.h>

/*********************************************************************
 *** ControllerManager class definition
 *********************************************************************/

namespace labust
{
	namespace controller
	{

		class ControllerManager {

		public:

			/*********************************************************
			 *** Class functions
			 ********************************************************/

			/* Constructor */
			ControllerManager(ros::NodeHandle nh);

			/* Initial configuration */
			void start();

			/*********************************************************
			 *** Primitive function calls
			 ********************************************************/

			/* Go to point fully actuated primitive */
			void go2point_FA_hdg(bool enable, double north1, double east1, double north2, double east2, double speed, double heading, double radius);

			/* Go to point fully actuated primitive */
			void go2point_FA(bool enable, double north1, double east1, double north2, double east2, double speed, double radius);

			/* Go to point underactuated primitive */
			void go2point_UA(bool enable, double north1, double east1, double north2, double east2, double speed, double radius);

			/* Dynamic positioning primitive */
			void dynamic_positioning(bool enable, double north, double east, double heading);

			/* Course keeping fully actuated  primitive */
			void course_keeping_FA(bool enable, double course, double speed, double heading);

			/* Course keeping underactuated  primitive */
			void course_keeping_UA(bool enable, double course, double speed);

			/* Self-oscillations identification */
			void ISOprimitive(bool enable, int dof, double command, double hysteresis, double reference, double sampling_rate);

			/*********************************************************
			 *** High Level Controllers
			 ********************************************************/

			/* Line following fully actuated controller*/
			void LF_FAcontroller(bool enable);

			/* Line following underactuated controller */
			void LF_UAcontroller(bool enable);

			/* Heading controller */
			void HDGcontroller(bool enable);

			/* Dynamic positioning controller */
			void DPcontroller(bool enable);

			/* Depth controller */
			void DEPTHcontroller(bool enable);

			/* Altitude controller */
			void ALTcontroller(bool enable);

			/* ES tracking controller */
			void EScontroller(bool enable);

			/* ES EKF tracking controller */
			void ES_EKFcontroller(bool enable);

			/*********************************************************
			 *** Low Level Controllers
			 ********************************************************/

			/* Low-level velocity controller */
			void LL_VELcontroller(bool enable);

			/*********************************************************
			 *** Helper functions
			 ********************************************************/

			/* Publish reference */
			void publishRef(auv_msgs::NavSts setRef);

			/* Enable controller */
			void enableController(const std::string serviceName, bool enable);

			/*********************************************************
			 *** Class variables
			 ********************************************************/

		private:

			ros::Publisher pubStateRef;
			labust::LowLevelConfigure LLcfg;

			labust::primitive::PrimitiveCallGo2Point Go2Point;
			labust::primitive::PrimitiveCallCourseKeeping CourseKeeping;
			labust::primitive::PrimitiveCallDynamicPositioning DynamicPositioning;
			labust::primitive::PrimitiveCallDOFIdentification DOFIdentification;

		};
	}
}

using namespace labust::controller;

	/*
	 * Constructor
	 */
	ControllerManager::ControllerManager(ros::NodeHandle nh): LLcfg(nh)
	{

	}


	/*********************************************************
	 *** Controller primitives masks
	 ********************************************************/
	/*
	 * Course keeping fully actuated primitive
	 */
	void ControllerManager::go2point_FA_hdg(bool enable, double north1, double east1, double north2, double east2, double speed, double heading, double radius)
	{
		typedef navcon_msgs::GoToPointGoal Goal;
		if(enable)
		{
			Goal goal;

			goal.ref_type = Goal::CONSTANT;
			goal.subtype = Goal::GO2POINT_FA_HDG;

			goal.T1.point.x = north1;
			goal.T1.point.y = east1;
			goal.T1.point.z = 0;
			goal.T2.point.x = north2;
			goal.T2.point.y = east2;
			goal.T2.point.z = 0;
			goal.heading = heading;
			goal.speed = speed;
			goal.victory_radius = radius;

			Go2Point.start(goal);
		}
		else
		{
			Go2Point.stop();
		}
	}

	/*
	 * Course keeping fully actuated primitive
	 */
	void ControllerManager::go2point_FA(bool enable, double north1, double east1, double north2, double east2, double speed, double radius)
	{
		typedef navcon_msgs::GoToPointGoal Goal;
		if(enable)
		{
			Goal goal;

			goal.ref_type = Goal::CONSTANT;
			goal.subtype = Goal::GO2POINT_FA;

			goal.T1.point.x = north1;
			goal.T1.point.y = east1;
			goal.T1.point.z = 0;
			goal.T2.point.x = north2;
			goal.T2.point.y = east2;
			goal.T2.point.z = 0;
			goal.heading = atan2(east2-east1,north2-north1);;
			goal.speed = speed;
			goal.victory_radius = radius;

			Go2Point.start(goal);
		}
		else
		{
			Go2Point.stop();
		}
	}

	/*
	 * Course keeping underactuated primitive
	 */
	void ControllerManager::go2point_UA(bool enable, double north1, double east1, double north2, double east2, double speed, double radius)
	{
		typedef navcon_msgs::GoToPointGoal Goal;
		if(enable)
		{
			Goal goal;

			goal.ref_type = Goal::CONSTANT;
			goal.subtype = Goal::GO2POINT_UA;

			goal.T1.point.x = north1;
			goal.T1.point.y = east1;
			goal.T1.point.z = 0;
			goal.T2.point.x = north2;
			goal.T2.point.y = east2;
			goal.T2.point.z = 0;
			goal.heading = atan2(east2-east1,north2-north1);;
			goal.speed = speed;
			goal.victory_radius = radius;

			Go2Point.start(goal);
		}
		else
		{
			Go2Point.stop();
		}
	}

	void ControllerManager::dynamic_positioning(bool enable, double north, double east, double heading)
	{
		typedef navcon_msgs::DynamicPositioningGoal Goal;
		if(enable)
		{
			Goal goal;

			//goal.ref_type = Goal::CONSTANT;
			//goal.subtype = Goal::GO2POINT_UA;

			goal.T1.point.x = north;
			goal.T1.point.y = east;
			goal.T1.point.z = 0;
			goal.yaw = heading;

			DynamicPositioning.start(goal);
		}
		else
		{
			DynamicPositioning.stop();
		}
	}

	/*
	 * Course keeping fully actuated primitive
	 */
	void ControllerManager::course_keeping_FA(bool enable, double course, double speed, double heading)
	{
		typedef navcon_msgs::CourseKeepingGoal Goal;
		if(enable)
		{
			Goal goal;

			goal.ref_type = Goal::CONSTANT;
			goal.subtype = Goal::COURSE_KEEPING_FA;

			goal.course = course;
			goal.speed = speed;
			goal.yaw = heading;

			CourseKeeping.start(goal);
		}
		else
		{
			CourseKeeping.stop();
		}
	}

	/*
	 * Course keeping underactuated primitive
	 */
	void ControllerManager::course_keeping_UA(bool enable, double course, double speed)
	{
		typedef navcon_msgs::CourseKeepingGoal Goal;
		if(enable)
		{
			Goal goal;

			goal.ref_type = Goal::CONSTANT;
			goal.subtype = Goal::COURSE_KEEPING_UA;

			goal.course = course;
			goal.speed = speed;
			goal.yaw = 0;

			CourseKeeping.start(goal);
		}
		else
		{
			CourseKeeping.stop();
		}
	}

	void ControllerManager::ISOprimitive(bool enable, int dof, double command, double hysteresis, double reference, double sampling_rate){

		if(enable){

			//self.velconName = rospy.get_param("~velcon_name","velcon")
			//self.model_update = rospy.Publisher("model_update", ModelParamsUpdate)
			ros::NodeHandle nh, ph("~");
			string velconName;
			ph.param<string>("velcon_name",velconName,"velcon");

            const char *names[7] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw", "Altitude"};

			/* configure velocity controller for identification */
			int velcon[6] = {0,0,0,0,0,0};
			if(dof == navcon_msgs::DOFIdentificationGoal::Altitude){
				velcon[navcon_msgs::DOFIdentificationGoal::Heave] = 3;
			} else {
				velcon[dof] = 3;
			}

			string tmp = velconName + "/" + names[dof] + "_ident_amplitude";
			nh.setParam(tmp.c_str(), command);
			tmp.assign(velconName + "/" + names[dof] + "_ident_hysteresis");
			nh.setParam(tmp.c_str(), hysteresis);
			tmp.assign(velconName + "/" + names[dof] + "_ident_ref");
			nh.setParam(tmp.c_str(), reference);

			LLcfg.LL_VELconfigure(true,velcon[0], velcon[1], velcon[2], velcon[3], velcon[4], velcon[5]);

			//ROS_ERROR("DOF = %d, command = %f, hysteresis = %f, reference = %f, sampling_rate = %f", dof, command, hysteresis, reference, sampling_rate);

			navcon_msgs::DOFIdentificationGoal goal;
			goal.command = command;
			goal.dof = dof;
			goal.hysteresis = hysteresis;
			goal.reference = reference;
			goal.sampling_rate = sampling_rate;

			DOFIdentification.start(goal);

		} else {

			DOFIdentification.stop();
			//LLcfg.LL_VELconfigure(false,1,1,0,0,0,1);
		}

	}

	/*********************************************************
	 * High Level Controllers
	 ********************************************************/

	/*
	 * Line following fully actuated controller
	 */
	void ControllerManager::LF_FAcontroller(bool enable){

	}

	/*
	 * Line following under actuated controller
	 */
	void ControllerManager::LF_UAcontroller(bool enable){


	}

	/*
	 * Heading controller
	 */
	void ControllerManager::HDGcontroller(bool enable){

		if(enable){

			/* Configure velocity controller */
			LLcfg.LL_VELconfigure(true,0,0,0,0,0,2);

			/* Enable controller */
			HDGenable = true;
			enableController("HDG_enable",true);

		} else {

			/* Configure velocity controller */
			LLcfg.LL_VELconfigure(false,0,0,0,0,0,1);

			/* Disable controller */
			HDGenable = false;
			enableController("HDG_enable",false);
		}
	}

	/*
	 * Dynamic positioning controller
	 */
	void ControllerManager::DPcontroller(bool enable){

		if(enable){

			LLcfg.LL_VELconfigure(true,2,2,0,0,0,0);

			/* Enable controller */
			DPenable = true;
			enableController("FADP_enable",true);

		} else {

			LLcfg.LL_VELconfigure(false,1,1,0,0,0,0);

			/* Disable controller */
			DPenable = false;
			enableController("FADP_enable",false);
		}
	}

	/*
	 * Depth controller
	 */
	void ControllerManager::DEPTHcontroller(bool enable){

	}

	/*
	 * Altitude controller
	 */
	void ControllerManager::ALTcontroller(bool enable){

	}

	void ControllerManager::EScontroller(bool enable){

		if(enable){

				LLcfg.LL_VELconfigure(true,2,2,0,0,0,0);

				/* Enable controller */
				DPenable = true;
				enableController("FADP_enable",true);

			} else {

				LLcfg.LL_VELconfigure(false,1,1,0,0,0,0);

				/* Disable controller */
				DPenable = false;
				enableController("FADP_enable",false);
			}

	}

	void ControllerManager::ES_EKFcontroller(bool enable){

		if(enable){

				LLcfg.LL_VELconfigure(true,2,2,0,0,0,0);

				/* Enable controller */
				DPenable = true;
				enableController("ESC_enable",true);

			} else {

				LLcfg.LL_VELconfigure(false,1,1,0,0,0,0);

				/* Disable controller */
				DPenable = false;
				enableController("ESC_EKF_enable",false);
			}

	}


	/*****************************************************************
	 ***  Helper functions
	 ****************************************************************/

	/*
	 * Publish high-level controller reference
	 */
	void ControllerManager::publishRef(auv_msgs::NavSts setRef){

		setRef.header.stamp = ros::Time::now();
		setRef.header.frame_id = "local";

		/* Publish reference*/
		pubStateRef.publish(setRef);
	}

	/*
	 * Enable high-level controller
	 */
	void ControllerManager::enableController(const std::string serviceName, bool enable){

		ros::NodeHandle nh;
		navcon_msgs::EnableControl enabler;
		enabler.request.enable = enable;
		ros::ServiceClient clientControllerEnabler = nh.serviceClient<navcon_msgs::EnableControl>(serviceName);
		utilities::callService<navcon_msgs::EnableControl>(clientControllerEnabler,enabler);
	}



#endif /* CONTROLLERMANAGER_HPP_ */
