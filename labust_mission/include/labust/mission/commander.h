/*
 * commander.h
 *
 *  Created on: Apr 21, 2016
 *      Author: filip
 */

#ifndef LABUST_ROS_PKG_LABUST_MISSION_INCLUDE_LABUST_MISSION_COMMANDER_H_
#define LABUST_ROS_PKG_LABUST_MISSION_INCLUDE_LABUST_MISSION_COMMANDER_H_

#include <ros/ros.h>
#include <labust_mission/maneuverGenerator.hpp>
#include <boost/lexical_cast.hpp>

#include <misc_msgs/Go2depthService.h>
#include <misc_msgs/Go2pointService.h>
#include <misc_msgs/PointerService.h>
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

			Commander();

			~Commander();

			bool go2pointService(misc_msgs::Go2pointService::Request &req, misc_msgs::Go2pointService::Response &res);

			bool go2depthService(misc_msgs::Go2depthService::Request &req, misc_msgs::Go2depthService::Response &res);

			bool pointerService(misc_msgs::PointerService::Request &req, misc_msgs::PointerService::Response &res);

			bool stopService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

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

			/*** Services ***/
			ros::ServiceServer srvDepth;
			ros::ServiceServer srvGo2point;
			ros::ServiceServer srvPointer;
			ros::ServiceServer srvLawnomver;
			ros::ServiceServer srvStop;

			ros::ServiceServer srvStatus; // To bi trebao mission exec publishati.

			labust::maneuver::ManeuverGenerator MG;

			std::string xml_save_path;

		};

		Commander::Commander()
		{

			ros::NodeHandle nh;

			xml_save_path = "/home/filip/";

			/*** Subscribers ***/
			//subStateHatAbs = nh.subscribe<auv_msgs::NavSts>("stateHatAbs",1,&DataEventManager::onStateHat,this);
			//subMissionSetup = nh.subscribe<misc_msgs::MissionSetup>("missionSetup",1,&DataEventManager::onMissionSetup,this);
			//subExternalEvents= nh.subscribe<misc_msgs::ExternalEvent>("externalEvent",1, &DataEventManager::onExternalEvent, this);
			//subEventString = nh.subscribe<std_msgs::String>("eventString",1, &DataEventManager::onEventString, this);

			/*** Publishers ***/
			//pubDataEventsContainer = nh.advertise<misc_msgs::DataEventsContainer>("dataEventsContainer",1);
			pubEventString = nh.advertise<std_msgs::String>("eventString",1);
			pubStartParser = nh.advertise<misc_msgs::StartParser>("startParser",1);


			/*** Services ***/
			//srvEvaluateExpression = nh.advertiseService("evaluate_expression", &DataEventManager::expressionEvaluationService,this);
			srvDepth = nh.advertiseService("commander/go2depth", &Commander::go2depthService,this);
			srvGo2point = nh.advertiseService("commander/go2point", &Commander::go2pointService,this);
			srvPointer = nh.advertiseService("commander/pointer", &Commander::pointerService,this);
			//srvLawnomver = nh.advertiseService("commander/lawnmover", &Commander::lawnmoverService,this);
			srvStop = nh.advertiseService("commander/stop", &Commander::stopService,this);


		}

		Commander::~Commander()
		{

		}

		/** Service that evaluates string expression */
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

		/** Service that evaluates string expression */
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


		/** Service that evaluates string expression */
		bool Commander::pointerService(misc_msgs::PointerService::Request &req, misc_msgs::PointerService::Response &res)
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

		/** Service that evaluates string expression */
//		bool Commander::lawnmoverService(misc_msgs::EvaluateExpression::Request &req, misc_msgs::EvaluateExpression::Response &res){
//
//			MG.writeXML.addMission();
//			MG.generateRows(posxy.first, posxy.second, md->speed, 1.0, md->width, md->length, md->hstep, md->alternation,
//											md->coff, (md->flags & md->FLG_SQUARE_CURVE), md->bearing*180/M_PI, md->cross_angle, !(md->flags & md->FLG_CURVE_RIGHT));
//			MG.writeXML.saveXML(xml_save_path.c_str());
//
//			publishStartParser(xml_save_path.c_str());
//
//			publishEventString("/START_PARSER");
//			ROS_INFO("Commander: depth request");
//
//			res.result = EE.evaluateStringExpression(req.expression);
//			return true;
//		}

		/** Service that evaluates string expression */
		bool Commander::stopService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
		{
			publishEventString("/STOP");
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
