/*********************************************************************
 * missionExecution.hpp
 *
 *  Created on: Apr 22, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2016, LABUST, UNIZG-FER
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

#ifndef MISSIONEXECUTION_HPP_
#define MISSIONEXECUTION_HPP_


#include <labust_mission/primitiveManager.hpp>
#include <labust/primitive/PrimitiveMapGenerator.h>

#include <exprtk/exprtk.hpp>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>
#include <labust/mission/labustMission.hpp>

extern decision_making::EventQueue* mainEventQueue;

namespace ser = ros::serialization;

/*********************************************************************
 ***  MissionExecution class definition
 ********************************************************************/

namespace labust
{
	namespace mission
	{
		class MissionExecution
		{
		public:

			/*****************************************************************
			 ***  Class functions
			 ****************************************************************/

			MissionExecution(ros::NodeHandle& nh, std::string xml_path);

		    void evaluatePrimitive(string primitiveString);

			/*****************************************************************
			 ***  State machine primitive states
			 ****************************************************************/

		    void execute_primitive();

		    void dynamic_postitioning_state();

		    void go2point_state();

		    void course_keeping_state();

		    void iso_state();

		    void follow_state();

		    void pointer_state();

			/*****************************************************************
			 ***  ROS Subscriptions Callback
			 ****************************************************************/

			void onStateHat(const auv_msgs::NavSts::ConstPtr& data);

			void onDataEventsContainer(const misc_msgs::DataEventsContainer::ConstPtr& data);

			void onEventString(const std_msgs::String::ConstPtr& msg);

			void onReceivePrimitive(const misc_msgs::SendPrimitive::ConstPtr& data);

			/*********************************************************************
			 *** Helper functions
			 ********************************************************************/

			void requestPrimitive();

			void setTimeout(double timeout);

			void onTimeout(const ros::TimerEvent& timer);

			void onPrimitiveEndReset();

			/*********************************************************************
			 ***  Class variables
			 ********************************************************************/

			/** Controller manager class */
			labust::controller::PrimitiveManager PM;

			/** ROS Node handle */
			ros::NodeHandle nh_;

			/** Timers */
			ros::Timer timer;

			/** Publishers */
			ros::Publisher pubRequestPrimitive, pubEventString;

			/** Subscribers */
			ros::Subscriber subDataEventsContainer, subEventString, subReceivePrimitive, subStateHat;

			/** Services */
			ros::ServiceClient srvExprEval;

			/** stateHat container */
			auv_msgs::NavSts state;

			/** Remember last primitive end point */
			auv_msgs::NED oldPosition;

			/*** Remember position when paused ***/
			auv_msgs::NED pause_position;

			/** Store last received primitive */
			misc_msgs::SendPrimitive receivedPrimitive;

			/** Vectors */
			vector<string> eventsActive, primitiveStrContainer;

			/** Map for storing last primitive floating point data */
			map<string, double> primitiveMap;

			/** Map for storing last primitive string data */
			map<string, string> primitiveStringMap;

			/** Map for storing last primitive bool data */
			map<string, bool> primitiveBoolMap;

			/** Execution flags */
			bool checkEventFlag, timeoutActive;

			/** Next primitive to request */
			int nextPrimitive;

			/** Mission state flag */
			bool missionActive;

			/*** ***/
			labust::primitive::PrimitiveMapGenerator PrimitiveMapGenerator;

		};

		/*****************************************************************
		 ***  Class functions
		 ****************************************************************/

		MissionExecution::MissionExecution(ros::NodeHandle& nh, std::string xml_path):checkEventFlag(false),
																	nextPrimitive(1),
																	timeoutActive(false),
																	missionActive(false),
																	PrimitiveMapGenerator(xml_path)
		{
			/** Subscribers */
			subEventString = nh.subscribe<std_msgs::String>("eventString",3, &MissionExecution::onEventString, this);
			subReceivePrimitive = nh.subscribe<misc_msgs::SendPrimitive>("sendPrimitive",1, &MissionExecution::onReceivePrimitive, this);
			subDataEventsContainer = nh.subscribe<misc_msgs::DataEventsContainer>("dataEventsContainer",1, &MissionExecution::onDataEventsContainer, this);
			subStateHat = nh.subscribe<auv_msgs::NavSts>("stateHat",1, &MissionExecution::onStateHat, this);

			/** Publishers */
			pubRequestPrimitive = nh.advertise<std_msgs::UInt16>("requestPrimitive",1);
			pubEventString = nh.advertise<std_msgs::String>("eventString",1);

			/** Services */
			srvExprEval = nh.serviceClient<misc_msgs::EvaluateExpression>("evaluate_expression");

			/** Define primitive parameters  */
			primitiveMap = PrimitiveMapGenerator.getPrimitiveDoubleMap();
			primitiveStringMap = PrimitiveMapGenerator.getPrimitiveStringMap();
			primitiveBoolMap = PrimitiveMapGenerator.getPrimitiveBoolMap();

//			primitiveMap.insert(std::pair<string, double>("xrefpont", 0.0));
//			primitiveMap.insert(std::pair<string, double>("yrefpoint", 0.0));
//			primitiveMap.insert(std::pair<string, double>("xs", 0.0));
//			primitiveMap.insert(std::pair<string, double>("ys", 0.0));
//			primitiveMap.insert(std::pair<string, double>("xc", 0.0));
//			primitiveMap.insert(std::pair<string, double>("yc", 0.0));
//			primitiveMap.insert(std::pair<string, double>("xe", 0.0));
//			primitiveMap.insert(std::pair<string, double>("ye", 0.0));
//			primitiveMap.insert(std::pair<string, double>("Vl", 0.0));
//			primitiveMap.insert(std::pair<string, double>("direction", 0.0));
//			primitiveMap.insert(std::pair<string, double>("R0", 0.0));
		}

	    void MissionExecution::evaluatePrimitive(string primitiveString)
	    {
	    	/*** Reset data ***/
	    	primitiveMap["timeout"] = 0;

			misc_msgs::EvaluateExpression evalExpr;
			primitiveStrContainer = labust::utilities::split(primitiveString, ':');

			for(vector<string>::iterator it = primitiveStrContainer.begin(); it != primitiveStrContainer.end(); it = it + 2)
			{
				/*** Handle string type parameters ***/
				size_t found = (*(it+1)).find_first_of('#');
				if(found != string::npos)
				{
					primitiveStringMap[*it] =  (*(it+1)).erase(found,1);
					ROS_INFO("Mission execution: String evaluation %s: %s", (*it).c_str(),primitiveStringMap[*it].c_str());
					continue;
				}

				evalExpr.request.expression = (*(it+1)).c_str();

				if(primitiveMap.find(*it) != primitiveMap.end())
				{
					/*** Handle double type parameters ***/
					primitiveMap[*it] =  (labust::utilities::callService(srvExprEval, evalExpr)).response.result;
					ROS_INFO("Mission execution: Double evaluation %s: %f", (*it).c_str(),primitiveMap[*it]);
				}
				else if(primitiveBoolMap.find(*it) != primitiveBoolMap.end())
				{
					/*** Handle bool type parameters ***/
					primitiveBoolMap[*it] =  bool((labust::utilities::callService(srvExprEval, evalExpr)).response.result);
					ROS_INFO("Mission execution: Bool evaluation %s: %d", (*it).c_str(),primitiveBoolMap[*it]);
				}
			}
		}

		/*****************************************************************
		 ***  State machine primitive states
		 ****************************************************************/

	    void MissionExecution::execute_primitive()
	    {

	    }

	    void MissionExecution::dynamic_postitioning_state(){

	    	/*** Evaluate primitive data with current values ***/
			evaluatePrimitive(receivedPrimitive.primitiveString.data);
	    	/*** Activate primitive timeout ***/
	    	if(!timeoutActive && primitiveMap["timeout"] > 0)
	    		setTimeout(primitiveMap["timeout"]);
			/*** Activate primitive ***/
			PM.dynamic_positioning(
					primitiveMap["north"],
					primitiveMap["east"],
					primitiveMap["depth"],
					primitiveMap["heading"],
					primitiveBoolMap["north_enable"],
					primitiveBoolMap["east_enable"],
					primitiveBoolMap["depth_enable"],
					primitiveBoolMap["heading_enable"],
					primitiveBoolMap["altitude_enable"],
					primitiveBoolMap["track_heading_enable"],
					primitiveStringMap["target_topic"],
					primitiveStringMap["heading_topic"]
					);

			/*** Save current position ***/
			oldPosition.north = primitiveMap["north"];
			oldPosition.east = primitiveMap["east"];
			oldPosition.depth = primitiveMap["depth"];
	    }

	    void MissionExecution::go2point_state()
	    {
	    	/** Evaluate primitive data with current values */
			evaluatePrimitive(receivedPrimitive.primitiveString.data);
	    	/** Activate primitive timeout */
			if(!timeoutActive && primitiveMap["timeout"] > 0)
				setTimeout(primitiveMap["timeout"]);
			/** Activate primitive */
			PM.go2point(
					primitiveBoolMap["fully_actuated_enable"],
					oldPosition.north,
					oldPosition.east,
					oldPosition.depth,
					primitiveMap["north"],
					primitiveMap["east"],
					primitiveMap["depth"],
					primitiveMap["heading"],
					primitiveMap["speed"],
					primitiveMap["victory_radius"],
					primitiveBoolMap["north_enable"],
					primitiveBoolMap["east_enable"],
					primitiveBoolMap["depth_enable"],
					primitiveBoolMap["heading_enable"],
					primitiveBoolMap["altitude_enable"],
					primitiveStringMap["heading_topic"],
					primitiveStringMap["speed_topic"]
					);

			oldPosition.north = primitiveMap["north"];
			oldPosition.east = primitiveMap["east"];
			oldPosition.depth = primitiveMap["depth"];
	    }

	    void MissionExecution::course_keeping_state(){

//			evaluatePrimitive(receivedPrimitive.primitiveString.data);
//	    	/** Activate primitive timeout */
//			if(!timeoutActive && primitiveMap["timeout"] > 0)
//				setTimeout(primitiveMap["timeout"]);
//			PM.course_keeping_FA(true, primitiveMap["course"], primitiveMap["speed"], primitiveMap["heading"]);
	    }

	    void MissionExecution::iso_state(){

//	    	/*** Evaluate primitive data with current values ***/
//			evaluatePrimitive(receivedPrimitive.primitiveString.data);
//	    	/*** Activate primitive timeout ***/
//			if(!timeoutActive && primitiveMap["timeout"] > 0)
//				setTimeout(primitiveMap["timeout"]);
//			/*** Activate primitive ***/
//			PM.ISOprimitive(true, primitiveMap["dof"], primitiveMap["command"], primitiveMap["hysteresis"], primitiveMap["reference"], primitiveMap["sampling_rate"]);
	    }

	    void MissionExecution::follow_state()
	    {
//			evaluatePrimitive(receivedPrimitive.primitiveString.data);
//	    	/* Activate primitive timeout */
//			if(!timeoutActive && primitiveMap["timeout"] > 0)
//				setTimeout(primitiveMap["timeout"]);
//			PM.follow(true, primitiveMap["xrefpoint"], primitiveMap["yrefpoint"], primitiveMap["xs"], primitiveMap["ys"], primitiveMap["xc"], primitiveMap["yc"], primitiveMap["xe"], primitiveMap["ye"], primitiveMap["Vl"], primitiveMap["direction"], primitiveMap["R0"]);

	    }

	    void MissionExecution::pointer_state()
	    {
			evaluatePrimitive(receivedPrimitive.primitiveString.data);
	    	/*** Activate primitive timeout */
			if(!timeoutActive && primitiveMap["timeout"] > 0)
				setTimeout(primitiveMap["timeout"]);

 			PM.pointer(
 					primitiveMap["radius"],
					primitiveMap["vertical_offset"],
					primitiveMap["guidance_target_x"],
					primitiveMap["guidance_target_y"],
					primitiveMap["guidance_target_z"],
					primitiveBoolMap["guidance_enable"],
					primitiveBoolMap["wrapping_enable"],
					primitiveBoolMap["streamline_orientation"],
					primitiveStringMap["guidance_topic"],
					primitiveStringMap["radius_topic"]);
	    }

		/*****************************************************************
		 ***  ROS Subscriptions Callback
		 ****************************************************************/

		/** DataEventsContainer callback  */
		void MissionExecution::onDataEventsContainer(const misc_msgs::DataEventsContainer::ConstPtr& data){

			/** If primitive has active events */
			if(checkEventFlag){

				/** Reset flag and counters */
				int flag = 0, i = 0;

				for(std::vector<uint8_t>::iterator it = receivedPrimitive.event.onEventNext.begin() ;
														it != receivedPrimitive.event.onEventNext.end(); ++it){

					/** For each primitive event check if it is true */
					if(data->eventsVar[receivedPrimitive.event.onEventNextActive[i++]-1] == 1){

						flag = 1;
						nextPrimitive = *it;
						ROS_ERROR("Event active:: %d", receivedPrimitive.event.onEventNextActive[i-1]);

						onPrimitiveEndReset();
						mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");
					}

					/** First true event has priority */
					if (flag) break;
				}
			}
		}

		/** ReceivePrimitive topic callback */
		void MissionExecution::onReceivePrimitive(const misc_msgs::SendPrimitive::ConstPtr& data)
		{
			receivedPrimitive = *data;

			/** Check if received primitive has active events */
			if(receivedPrimitive.event.onEventNextActive.empty() == 0)
			{
				checkEventFlag = true;
			}

			/** Call primitive */
			if(data->primitiveID != none)
			{
				string id_string(PRIMITIVES[data->primitiveID]);
				id_string = "/" + boost::to_upper_copy(id_string);
				mainEventQueue->riseEvent(id_string.c_str());
			}
			else
			{
				ROS_INFO("Mission ended.");
				std_msgs::String msg;
				msg.data = "/STOP";
				pubEventString.publish(msg);
			}
		}

		/*
		 * Collect state measurements
		 */
		void MissionExecution::onStateHat(const auv_msgs::NavSts::ConstPtr& data)
		{
			state = *data;
		}

		/*********************************************************************
		 *** Helper functions
		 ********************************************************************/

		/** EventString topic callback */
		void MissionExecution::onEventString(const std_msgs::String::ConstPtr& msg){

			if(strcmp(msg->data.c_str(),"/START_DISPATCHER") == 0 && missionActive)
			{
				mainEventQueue->riseEvent("/STOP");
				onPrimitiveEndReset();
				nextPrimitive = 1;
			}
			else if(strcmp(msg->data.c_str(),"/STOP") == 0)
			{
				onPrimitiveEndReset();
				nextPrimitive = 1;
			}
			else if(strcmp(msg->data.c_str(),"/PAUSE") == 0)
			{
				onPrimitiveEndReset();
				nextPrimitive--;
			}
			else if(strcmp(msg->data.c_str(),"/MANUAL_ENABLE") == 0 && !missionActive)
			{
				PM.enableManual(true);
			}
			else if(strcmp(msg->data.c_str(),"/MANUAL_DISABLE") == 0 && !missionActive)
			{
				PM.enableManual(false);
			}

			mainEventQueue->riseEvent(msg->data.c_str());
			ROS_INFO("Mission execution: Received mission control command: %s",msg->data.c_str());
		}

		/** Request new primitive */
		void MissionExecution::requestPrimitive(){

			std_msgs::UInt16 req;
			req.data = nextPrimitive++;
			pubRequestPrimitive.publish(req);
		}

		/** Set primitive timeout */
		void MissionExecution::setTimeout(double timeout){

		   	if(timeout != 0){
		   		ROS_WARN("Setting timeout: %f", timeout);
				timer = nh_.createTimer(ros::Duration(timeout), &MissionExecution::onTimeout, this, true);
				timeoutActive = true;
		   	}
		}

		/** On timeout finish primitive execution */
		void MissionExecution::onTimeout(const ros::TimerEvent& timer){

			ROS_WARN("Timeout");
			onPrimitiveEndReset();
			mainEventQueue->riseEvent("/TIMEOUT");
		}

		/** Reset timers and flags */
		void MissionExecution::onPrimitiveEndReset(){

			/** Stop timeout timer */
			timer.stop();
		    /** Reset execution flags */
			timeoutActive = checkEventFlag = false;
			missionActive = false;
		}
	}
}

#endif /* MISSIONEXECUTION_HPP_ */
