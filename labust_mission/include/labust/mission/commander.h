/*****************+***************************************************
 * commander.h
 *
 *  Created on: Apr 21, 2016
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

#ifndef LABUST_ROS_PKG_LABUST_MISSION_INCLUDE_LABUST_MISSION_COMMANDER_H_
#define LABUST_ROS_PKG_LABUST_MISSION_INCLUDE_LABUST_MISSION_COMMANDER_H_

#include <ros/ros.h>
#include <labust_mission/maneuverGenerator.hpp>
#include <boost/lexical_cast.hpp>

#include <misc_msgs/Go2depthService.h>
#include <misc_msgs/Go2pointService.h>
#include <misc_msgs/Go2pointPrimitiveService.h>
#include <misc_msgs/PointerPrimitiveService.h>
#include <misc_msgs/DynamicPositioningPrimitiveService.h>
#include <misc_msgs/DockingPrimitiveService.h>
#include <misc_msgs/LawnmoverService.h>
#include <misc_msgs/StartParser.h>

#include <std_srvs/Trigger.h>


/*********************************************************************
 *** Commander class definition
 ********************************************************************/

namespace labust
{
	namespace mission
	{
		class Commander
		{
		public:

			/*****************************************************************
			 ***  Class functions
			 ****************************************************************/

			Commander(std::string xml_path);

			~Commander();

			/*****************************************************************
			 ***  Full primitive services
			 ****************************************************************/

			bool go2pointPrimitiveService(misc_msgs::Go2pointPrimitiveService::Request &req, misc_msgs::Go2pointPrimitiveService::Response &res);

			bool pointerPrimitiveService(misc_msgs::PointerPrimitiveService::Request &req, misc_msgs::PointerPrimitiveService::Response &res);

			bool dynamicPositioningPrimitiveService(misc_msgs::DynamicPositioningPrimitiveService::Request &req, misc_msgs::DynamicPositioningPrimitiveService::Response &res);

			bool dockingPrimitiveService(misc_msgs::DockingPrimitiveService::Request &req, misc_msgs::DockingPrimitiveService::Response &res);

			/*****************************************************************
			 ***  Derived services
			 ****************************************************************/

			bool go2pointService(misc_msgs::Go2pointService::Request &req, misc_msgs::Go2pointService::Response &res);

			bool go2depthService(misc_msgs::Go2depthService::Request &req, misc_msgs::Go2depthService::Response &res);

			bool lawnmoverService(misc_msgs::LawnmoverService::Request &req, misc_msgs::LawnmoverService::Response &res);

			/*****************************************************************
			 ***  General services
			 ****************************************************************/

			bool stopService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

			bool pauseService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

			bool continueService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

			/*****************************************************************
			 ***  Helper functions
			 ****************************************************************/

			void publishEventString(std::string event);

			void publishStartParser(std::string xml);

			void saveAndRequestAction(std::string name);

			/*********************************************************************
			 ***  Class variables
			 ********************************************************************/

		private:

			/*** Subscribers ***/

			/*** Publishers ***/
			ros::Publisher pubStatus;
			ros::Publisher pubEventString, pubStartParser;

			/*** Full action service calls ***/
			ros::ServiceServer srvGo2pointPrimitive;
			ros::ServiceServer srvPointerPrimitive;
			ros::ServiceServer srvDynamicPositioningPrimitive;
			//ros::ServiceServer srvCourseKeeping;
			ros::ServiceServer srvDockingPrimitive;


			/*** Masked service calls ***/
			ros::ServiceServer srvGo2point;
			ros::ServiceServer srvDepth;
			ros::ServiceServer srvLawnmover;

			/*** Control service calls ***/
			ros::ServiceServer srvStop;
			ros::ServiceServer srvPause;
			ros::ServiceServer srvContinue;

			ros::ServiceServer srvStatus; // To bi trebao mission exec publishati.

			/*** Maneuver generator class ***/
			labust::maneuver::ManeuverGenerator MG;

			/*** Path for saving mission xml files ***/
			std::string xml_save_path;

		};

		Commander::Commander(std::string xml_path):
				MG(xml_path),
				xml_save_path("")
		{
			ros::NodeHandle nh;

			nh.param("mission_save_path",xml_save_path,xml_save_path);

			/*** Subscribers ***/
			//subStateHatAbs = nh.subscribe<auv_msgs::NavSts>("stateHatAbs",1,&DataEventManager::onStateHat,this);

			/*** Publishers ***/
			pubEventString = nh.advertise<std_msgs::String>("eventString",1);
			pubStartParser = nh.advertise<misc_msgs::StartParser>("startParser",1);

			/*** Services ***/
			srvGo2pointPrimitive = nh.advertiseService("commander/primitive/go2point", &Commander::go2pointPrimitiveService,this);
			srvPointerPrimitive = nh.advertiseService("commander/primitive/pointer", &Commander::pointerPrimitiveService,this);
			srvDynamicPositioningPrimitive = nh.advertiseService("commander/primitive/dynamic_positioning", &Commander::dynamicPositioningPrimitiveService,this);
			srvDockingPrimitive = nh.advertiseService("commander/primitive/docking", &Commander::dockingPrimitiveService,this);

			srvGo2point = nh.advertiseService("commander/go2point", &Commander::go2pointService,this);
			srvDepth = nh.advertiseService("commander/go2depth", &Commander::go2depthService,this);
			srvLawnmover = nh.advertiseService("commander/lawnmover", &Commander::lawnmoverService,this);

			srvStop = nh.advertiseService("commander/stop_mission", &Commander::stopService,this);
			srvPause = nh.advertiseService("commander/pause_mission", &Commander::pauseService,this);
			srvContinue = nh.advertiseService("commander/continue_mission", &Commander::continueService,this);
		}

		Commander::~Commander()
		{

		}

		/*** go2depth service ***/
		bool Commander::go2depthService(misc_msgs::Go2depthService::Request &req, misc_msgs::Go2depthService::Response &res)
		{
			/*** Generate mission xml file ***/
			MG.writeXML.addMission();
			MG.generateGo2Point(true,0,0,req.depth,0,0.2,0.1,false,false,true,false,false,"","");
			/*** Request mission execution ***/
			saveAndRequestAction("go2depth");

			res.status = true;
			return true;
		}

		/*** go2point service ***/
		bool Commander::go2pointService(misc_msgs::Go2pointService::Request &req, misc_msgs::Go2pointService::Response &res)
		{
			/*** Generate mission xml file ***/
			MG.writeXML.addMission();
			MG.generateGo2Point_FA(req.point.x, req.point.y, req.point.z, req.speed, 1.5);

			/*** Request mission execution ***/
			saveAndRequestAction("go2point");

			res.status = true;
			return true;
		}

		/*** go2point service ***/
		bool Commander::go2pointPrimitiveService(misc_msgs::Go2pointPrimitiveService::Request &req, misc_msgs::Go2pointPrimitiveService::Response &res)
		{
			/*** Generate mission xml file ***/
			MG.writeXML.addMission();
			MG.generateGo2Point(req.fully_actuated_enable,
					req.point.x,
					req.point.y,
					req.point.z,
					req.heading,
					req.speed,
					req.victory_radius,
					req.north_enable,
					req.east_enable,
					req.depth_enable,
					req.heading_enable,
					req.altitude_enable,
					req.heading_topic,
					req.speed_topic);

			/*** Request mission execution ***/
			saveAndRequestAction("go2depth");

			res.status = true;
			return true;
		}

		/*** Pointer service ***/
		bool Commander::pointerPrimitiveService(misc_msgs::PointerPrimitiveService::Request &req, misc_msgs::PointerPrimitiveService::Response &res)
		{
			/*** Generate mission xml file ***/
			MG.writeXML.addMission();
			MG.generatePointer(req.radius,
					req.vertical_offset,
					req.guidance_target.x,
					req.guidance_target.y,
					req.guidance_target.z,
					req.guidance_enable,
					req.wrapping_enable,
					req.streamline_orientation,
					req.guidance_topic,
					req.radius_topic);

			/*** Request mission execution ***/
			saveAndRequestAction("pointer");

			res.status = true;
			return true;
		}

        /*** Dynamic positioning service ***/
        bool Commander::dynamicPositioningPrimitiveService(misc_msgs::DynamicPositioningPrimitiveService::Request &req, misc_msgs::DynamicPositioningPrimitiveService::Response &res)
        {
            /*** Generate mission xml file ***/
            MG.writeXML.addMission();
            MG.generateDynamicPositioning(
                            req.north,
                            req.east,
                            req.depth,
                            req.heading,
                            req.north_enable,
                            req.east_enable,
                            req.depth_enable,
                            req.heading_enable,
                            req.altitude_enable,
                            req.target_topic_enable,
                            req.track_heading_enable,
                            req.point_to_target,
                            req.target_topic,
                            req.heading_topic);

            /*** Request mission execution ***/
            saveAndRequestAction("dynamic_positioning");

            res.status = true;
            return true;
        }

        /*** Docking service ***/
        bool Commander::dockingPrimitiveService(misc_msgs::DockingPrimitiveService::Request &req, misc_msgs::DockingPrimitiveService::Response &res)
        {
            /*** Generate mission xml file ***/
            MG.writeXML.addMission();
            MG.generateDocking(
                            //req.north,
                            //req.east,
  								);

            /*** Request mission execution ***/
            saveAndRequestAction("docking");

            res.status = true;
            return true;
        }

		/*** Lawnmover service ***/
		bool Commander::lawnmoverService(misc_msgs::LawnmoverService::Request &req, misc_msgs::LawnmoverService::Response &res)
		{
			/*** Generate mission xml file ***/
			MG.writeXML.addMission();
			MG.generateRows(
					req.start_position.x,
					req.start_position.y,
					req.start_position.z,
					req.speed,
					req.victory_radius,
					req.width,
					req.length,
					req.horizontal_step,
					1.0, 			//md->alternation,
					0.0,			//md->coff,
					true,			//(md->flags & md->FLG_SQUARE_CURVE),
					req.bearing,	//md->bearing*180/M_PI,
					0.0,			//md->cross_angle,
					false);			//!(md->flags & md->FLG_CURVE_RIGHT));

			/*** Request mission execution ***/
			saveAndRequestAction("lawnmover");

			res.status = true;
			return true;
		}

		/*** Service that stops mission execution ***/
		bool Commander::stopService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
		{
			publishEventString("/STOP");
			return true;
		}

		/*** Service that pauses mission execution ***/
		bool Commander::pauseService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
		{
			publishEventString("/PAUSE");
			return true;
		}

		/*** Service that continues mission execution after pause command ***/
		bool Commander::continueService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
		{
			publishEventString("/CONTINUE");
			return true;
		}

		/*****************************************************************
		 ***  Helper functions
		 ****************************************************************/

		void Commander::publishStartParser(std::string xml)
		{
			misc_msgs::StartParser msg;
			msg.method = misc_msgs::StartParser::FILENAME;
			msg.missionData = xml.c_str();
			pubStartParser.publish(msg);
		}

		void Commander::publishEventString(std::string event)
		{
			std_msgs::String msg;
			msg.data = event.c_str();
			pubEventString.publish(msg);
		}

		void Commander::saveAndRequestAction(std::string name)
		{
			std::string filename = xml_save_path+name+"_"+boost::lexical_cast<std::string>(ros::Time::now())+".xml";
			MG.writeXML.saveXML(filename.c_str());
			publishStartParser(filename.c_str());
			publishEventString("/START_PARSER");
			ROS_INFO("Commander: %s request",name.c_str());
		}
	}
}

#endif /* LABUST_ROS_PKG_LABUST_MISSION_INCLUDE_LABUST_MISSION_COMMANDER_H_ */
