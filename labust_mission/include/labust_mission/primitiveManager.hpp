/*********************************************************************
 * PrimitiveManager.hpp
 *
 *  Created on: Jul 13, 2015
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015-2016, LABUST, UNIZG-FER
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

#ifndef PRIMITIVEMANAGER_HPP_
#define PRIMITIVEMANAGER_HPP_

/*********************************************************************
 *** Includes
 ********************************************************************/

#include <cmath>

#include <auv_msgs/Bool6Axis.h>
#include <navcon_msgs/EnableControl.h>

#include <navcon_msgs/CourseKeepingAction.h>
#include <navcon_msgs/DynamicPositioningAction.h>
#include <navcon_msgs/GoToPointAction.h>
#include <navcon_msgs/DOFIdentificationAction.h>
#include <navcon_msgs/TrackDiverAction.h>
#include <caddy_msgs/follow_sectionAction.h>
#include <navcon_msgs/DockingAction.h>

#include <labust/mission/labustMission.hpp>
#include <labust_mission/lowLevelConfigure.hpp>
#include <labust_mission/utils.hpp>

#include <labust/primitive/PrimitiveCall.hpp>
#include <labust/primitive/PrimitiveMapGenerator.h>


/*********************************************************************
 *** ControllerManager class definition
 *********************************************************************/

namespace labust
{
	namespace controller
	{
		class PrimitiveManager
		{
		public:

			/*********************************************************
			 *** Class functions
			 ********************************************************/

			/** Constructor */
			PrimitiveManager();

			~PrimitiveManager();

			/** Enable manual control */
			void enableManual(bool flag);

			/*********************************************************
			 *** Primitive function calls
			 ********************************************************/

			/*** Go2point primitive ***/
			void go2point_disable();
			void go2point(
					bool enable_fully_actuated,
					double north1,
					double east1,
					double depth1,
					double north2,
					double east2,
					double depth2,
					double heading,
					double speed,
					double victory_radius,
					bool north_enable,
					bool east_enable,
					bool depth_enable,
					bool heading_enable,
					bool altitude_enable,
					std::string heading_topic,
					std::string speed_topic
					);


			/** Dynamic positioning primitive */
			void dynamic_positioning_disable();
			void dynamic_positioning(
					double north,
					double east,
					double depth,
					double heading,
					bool north_enable,
					bool east_enable,
					bool depth_enable,
					bool heading_enable,
					bool altitude_enable,
					bool target_topic_enable,
					bool track_heading_enable,
					std::string target_topic,
					std::string heading_topic
					);

			/** Course keeping primitive */
			void course_keeping_disable();
			void course_keeping(
					double course,
					double speed,
					double heading
					);

			/** Self-oscillations identification */
			void ISOprimitive_disable();
			void ISOprimitive(
					int dof,
					double command,
					double hysteresis,
					double reference,
					double sampling_rate
					);

			/** Pointer primitive */
			void pointer_disable();
			void pointer(
					double radius,
					double vertical_offset,
					double guidance_target_x,
					double guidance_target_y,
					double guidance_target_z,
					bool guidance_enable,
					bool wrapping_enable,
					bool streamline_orientation,
					std::string guidance_topic,
					std::string radius_topic
					);

			/** Follow primitive */
			void follow_disable();
			void follow(
					double xrefpoint,
					double yrefpoint,
					double xs,
					double ys,
					double xc,
					double yc,
					double xe,
					double ye,
					double Vl,
					double direction,
					double R0
					);

			/** Docking primitive */
			void docking_disable();
			void docking(
				  bool docking_action,
				  double docking_slot,
          double search_yaw_rate,
          double max_yaw_rate,
          double max_surge_speed,
          double surge_stdev				
					);

			/*********************************************************
			 *** Class variables
			 ********************************************************/

		private:

			labust::primitive::PrimitiveCallGo2Point Go2Point;
			labust::primitive::PrimitiveCallCourseKeeping CourseKeeping;
			labust::primitive::PrimitiveCallDynamicPositioning DynamicPositioning;
			//labust::primitive::PrimitiveCallDOFIdentification DOFIdentification;
			labust::primitive::PrimitiveCallPointer Pointer;
			labust::primitive::PrimitiveCallFollow Follow;
			labust::primitive::PrimitiveCallDocking Docking;


			labust::LowLevelConfigure LLcfg;
		};
	}
}

using namespace labust::controller;


PrimitiveManager::PrimitiveManager()
{

}

PrimitiveManager::~PrimitiveManager()
{

}

void PrimitiveManager::enableManual(bool flag)
{
	LLcfg.LL_VELconfigure(flag,1,1,1,1,1,1);
}
/*********************************************************
 *** Controller primitives masks
 ********************************************************/

/*** Go2point primitive ***/
void PrimitiveManager::go2point_disable()
{
	Go2Point.stop();
	LLcfg.LL_VELconfigure(true,1,1,1,1,1,1);
}
void PrimitiveManager::go2point(
		bool enable_fully_actuated,
		double north1,
		double east1,
		double depth1,
		double north2,
		double east2,
		double depth2,
		double heading,
		double speed,
		double victory_radius,
		bool north_enable,
		bool east_enable,
		bool depth_enable,
		bool heading_enable,
		bool altitude_enable,
		std::string heading_topic,
		std::string speed_topic)
{
	typedef navcon_msgs::GoToPointGoal Goal;
	Goal goal;

	goal.ref_type = Goal::CONSTANT;
	goal.subtype = enable_fully_actuated?static_cast<uint8_t>(Goal::GO2POINT_FA):static_cast<uint8_t>(Goal::GO2POINT_UA);

	goal.T1.point.x = north1;
	goal.T1.point.y = east1;
	goal.T1.point.z = depth1;
	goal.T2.point.x = north2;
	goal.T2.point.y = east2;
	goal.T2.point.z = depth2;

	goal.heading = heading;
	goal.speed = speed;
	goal.victory_radius = victory_radius;

	goal.axis_enable.x = north_enable;
	goal.axis_enable.y = east_enable;
	goal.axis_enable.z = depth_enable;
	goal.axis_enable.roll = false;
	goal.axis_enable.pitch = false;
	goal.axis_enable.yaw = heading_enable; /*** Enables fully actuated control with independent heading control. ***/

	goal.altitude = altitude_enable;

	goal.heading_topic = heading_topic;
	goal.speed_topic = speed_topic;

	LLcfg.LL_VELconfigure(true,north_enable?2:1,east_enable?2:1,depth_enable?2:1,1,1,2);
	Go2Point.start(goal);
}

void PrimitiveManager::dynamic_positioning_disable()
{
	DynamicPositioning.stop();
	LLcfg.LL_VELconfigure(true,1,1,1,1,1,1);
}
void PrimitiveManager::dynamic_positioning(
		double north,
		double east,
		double depth,
		double heading,
		bool north_enable,
		bool east_enable,
		bool depth_enable,
		bool heading_enable,
		bool altitude_enable,
		bool target_topic_enable,
		bool track_heading_enable,
		std::string target_topic,
		std::string heading_topic)
{
	typedef navcon_msgs::DynamicPositioningGoal Goal;
	Goal goal;

	goal.T1.point.x = north;
	goal.T1.point.y = east;
	goal.T1.point.z = depth;
	goal.yaw = heading;

	goal.axis_enable.x = north_enable;
	goal.axis_enable.y = east_enable;
	goal.axis_enable.z = depth_enable;
	goal.axis_enable.roll = false;
	goal.axis_enable.pitch = false;
	goal.axis_enable.yaw = heading_enable; /*** Enables fully actuated control with independent heading control. ***/

	goal.altitude = altitude_enable;
	goal.track_heading_enable = track_heading_enable; /*** Use topic as heading reference. ***/
	goal.target_topic_enable = track_heading_enable; /*** Use topic as target reference. ***/


	goal.heading_topic = heading_topic;
	goal.target_topic = target_topic;

	LLcfg.LL_VELconfigure(true,north_enable?2:1,east_enable?2:1,depth_enable?2:1,1,1,heading_enable?2:1);
	DynamicPositioning.start(goal);
}


/*** Course keeping  primitive ***/
void PrimitiveManager::course_keeping_disable()
{
	CourseKeeping.stop();
}
void PrimitiveManager::course_keeping(double course, double speed, double heading)
{
	typedef navcon_msgs::CourseKeepingGoal Goal;

	Goal goal;

	goal.ref_type = Goal::CONSTANT;
	goal.subtype = Goal::COURSE_KEEPING_FA;

	goal.course = course;
	goal.speed = speed;
	goal.yaw = heading;

	CourseKeeping.start(goal);
}

void PrimitiveManager::pointer_disable()
{
	Pointer.stop();
	LLcfg.LL_VELconfigure(true,1,1,1,1,1,1);
}
void PrimitiveManager::pointer(double radius, double vertical_offset, double guidance_target_x, double guidance_target_y, double guidance_target_z, bool guidance_enable, bool wrapping_enable, bool streamline_orientation, std::string guidance_topic, std::string radius_topic)
{
	typedef navcon_msgs::TrackDiverGoal Goal;

	Goal goal;

	goal.radius = radius;
	goal.vertical_offset = vertical_offset;
	goal.guidance_target.x = guidance_target_x;
	goal.guidance_target.y = guidance_target_y;
	goal.guidance_target.z = guidance_target_z;

	goal.guidance_enable = guidance_enable;
	goal.wrapping_enable = wrapping_enable;
	goal.streamline_orientation = streamline_orientation;

	goal.guidance_topic = guidance_topic;
	goal.radius_topic = radius_topic;

	LLcfg.LL_VELconfigure(true,2,2,2,1,1,2);
	Pointer.start(goal);
}

void PrimitiveManager::follow_disable()
{
	Follow.stop();

	ros::NodeHandle nh;
	ros::ServiceClient cl;

	navcon_msgs::EnableControl a;
	/*** Enable or disable hdg controller ***/
	cl = nh.serviceClient<navcon_msgs::EnableControl>("HDG_enable");
	a.request.enable = false;
	cl.call(a);

	cl = nh.serviceClient<navcon_msgs::EnableControl>("ALT_enable");
	a.request.enable = false;
	cl.call(a);

	LLcfg.LL_VELconfigure(true,1,1,1,1,1,1);
}
void PrimitiveManager::follow(double xrefpoint, double yrefpoint, double xs, double ys, double xc, double yc, double xe, double ye, double Vl, double direction, double R0)
{
	typedef caddy_msgs::follow_sectionGoal Goal;

	Goal goal;

	goal.follow_section.xrefpoint = xrefpoint;
	goal.follow_section.yrefpoint = yrefpoint;
	goal.follow_section.xs = xs;
	goal.follow_section.ys = ys;
	goal.follow_section.xc = xc;
	goal.follow_section.yc = yc;
	goal.follow_section.xe = xe;
	goal.follow_section.ye = ye;
	goal.follow_section.Vl = Vl;
	goal.follow_section.direction = direction;
	goal.follow_section.R0 = R0;

	LLcfg.LL_VELconfigure(true,2,1,2,1,1,2);

	ros::NodeHandle nh;
	ros::ServiceClient cl;

	navcon_msgs::EnableControl a;
	/*** Enable or disable hdg controller ***/
	cl = nh.serviceClient<navcon_msgs::EnableControl>("HDG_enable");
	a.request.enable = true;
	cl.call(a);

	cl = nh.serviceClient<navcon_msgs::EnableControl>("ALT_enable");
	a.request.enable = true;
	cl.call(a);

	Follow.start(goal);
}

//	void PrimitiveManager::ISOprimitive(bool enable, int dof, double command, double hysteresis, double reference, double sampling_rate){
//
//		if(enable){
//
//			//self.velconName = rospy.get_param("~velcon_name","velcon")
//			//self.model_update = rospy.Publisher("model_update", ModelParamsUpdate)
//			ros::NodeHandle nh, ph("~");
//			string velconName;
//			ph.param<string>("velcon_name",velconName,"velcon");
//
//            const char *names[7] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw", "Altitude"};
//
//			/* configure velocity controller for identification */
//			int velcon[6] = {0,0,0,0,0,0};
//			if(dof == navcon_msgs::DOFIdentificationGoal::Altitude){
//				velcon[navcon_msgs::DOFIdentificationGoal::Heave] = 3;
//			} else {
//				velcon[dof] = 3;
//			}
//
//			string tmp = velconName + "/" + names[dof] + "_ident_amplitude";
//			nh.setParam(tmp.c_str(), command);
//			tmp.assign(velconName + "/" + names[dof] + "_ident_hysteresis");
//			nh.setParam(tmp.c_str(), hysteresis);
//			tmp.assign(velconName + "/" + names[dof] + "_ident_ref");
//			nh.setParam(tmp.c_str(), reference);
//
//			LLcfg.LL_VELconfigure(true,velcon[0], velcon[1], velcon[2], velcon[3], velcon[4], velcon[5]);
//
//			//ROS_ERROR("DOF = %d, command = %f, hysteresis = %f, reference = %f, sampling_rate = %f", dof, command, hysteresis, reference, sampling_rate);
//
//			navcon_msgs::DOFIdentificationGoal goal;
//			goal.command = command;
//			goal.dof = dof;
//			goal.hysteresis = hysteresis;
//			goal.reference = reference;
//			goal.sampling_rate = sampling_rate;
//
//			DOFIdentification.start(goal);
//
//		} else {
//
//			DOFIdentification.stop();
//			//LLcfg.LL_VELconfigure(false,1,1,0,0,0,1);
//		}
//
//	}

/*** Docking primitive ***/
void PrimitiveManager::docking_disable()
{
	Docking.stop();
	LLcfg.LL_VELconfigure(true,1,1,1,1,1,1);
}
void PrimitiveManager::docking(
	  bool docking_action,
	  double docking_slot,
    double search_yaw_rate,
    double max_yaw_rate,
    double max_surge_speed,
    double surge_stdev					
		)
{
	typedef navcon_msgs::DockingGoal Goal;
	Goal goal;
	goal.docking_action = docking_action;
	goal.docking_slot = docking_slot;
	goal.search_yaw_rate = search_yaw_rate;
	goal.max_yaw_rate = max_yaw_rate;
	goal.max_surge_speed = max_surge_speed;
	goal.surge_stdev = surge_stdev;

	LLcfg.LL_VELconfigure(true,2,2,1,1,1,2);
	Docking.start(goal);
}


#endif /* PRIMITIVEMANAGER_HPP_ */
