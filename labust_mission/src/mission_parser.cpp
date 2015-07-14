//\todo napravi da parser pamti pokazivac na zadnji poslani node koko se ne bi svaki put trebalo parsati korz cijeli xml
//\todo napraviti missionParser (tj klasu koja direktno poziva primitive) klasu tako da extenda razlicite klase za parsanje (u buducnosti pojednostavljeno
//prebacivanje na razlicite mission planere)
//\todo Pogledja treba li poslati sve evente kao jednu poruku na poectku, kako bi se kasnije smanjila kolicina podataka koja se salje skupa s primitivom
//\TODO Dodati mogucnost odabira vise evenata na koje primitiv može reagirati. (Nema potrebe za svaki slučaj nanovo definirati event)

/*********************************************************************
 * mission_parser.cpp
 *
 *  Created on: Apr 18, 2014
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

#include <labust_mission/labustMission.hpp>

#include <misc_msgs/StartParser.h>
#include <misc_msgs/EvaluateExpression.h>

#include <tinyxml2.h>

using namespace tinyxml2;

namespace labust
{
	namespace mission
	{

		/*********************************************************************
		 ***  MissionParser class definition
		 ********************************************************************/

		class MissionParser
		{
		public:

			/*****************************************************************
			 ***  Class functions
			 ****************************************************************/

			MissionParser();

			~MissionParser(){};

			void sendPrimitve();

			int parseMission(int id);

			int parseEvents();

			int parseMissionParam();

			void onRequestPrimitive(const std_msgs::UInt16::ConstPtr& req);

			void onEventString(const std_msgs::String::ConstPtr& msg);

			void onReceiveMission(const misc_msgs::StartParser::ConstPtr& msg);

			/*****************************************************************
			 ***  Helper functions
			 ****************************************************************/

			//void serializePrimitive(int id, vector<uint8_t> serializedData);

			void onEventNextParse(XMLElement *elem2);

			void resetParser();

			/*****************************************************************
			 ***  Class variables
			 ****************************************************************/

			int ID, lastID, eventID;
			double newTimeout;

			string missionEvents, missionParams;

			vector<uint8_t> onEventNextActive, onEventNext;

			ros::Publisher pubSendPrimitive, pubRiseEvent, pubMissionSetup;
			ros::Subscriber subRequestPrimitive, subEventString, subReceiveXmlPath;
			ros::ServiceClient srvExprEval;

			auv_msgs::NED offset;
			uint16_t breakpoint;

			/** Send primitive to mission execution as string with general data */
			stringstream primitiveString;

			PrimitiveParams PP;

			XMLDocument xmlDoc;
		};


		/*****************************************************************
		 ***  Class functions
		 ****************************************************************/

		MissionParser::MissionParser():ID(0),
										   lastID(0),
										   newTimeout(0),
										   eventID(0),
										   breakpoint(1),
										   missionEvents("")
		{
			ros::NodeHandle nh;

			/** Subscribers */
			subRequestPrimitive = nh.subscribe<std_msgs::UInt16>("requestPrimitive",1,&MissionParser::onRequestPrimitive, this);
			subEventString = nh.subscribe<std_msgs::String>("eventString",1,&MissionParser::onEventString, this);
			subReceiveXmlPath = nh.subscribe<misc_msgs::StartParser>("startParse",1,&MissionParser::onReceiveMission, this);

			/** Publishers */
			pubSendPrimitive = nh.advertise<misc_msgs::SendPrimitive>("sendPrimitive",1);
			pubRiseEvent = nh.advertise<std_msgs::String>("eventString",1);
			pubMissionSetup = nh.advertise<misc_msgs::MissionSetup>("missionSetup",1);

			/** Service */
			srvExprEval = nh.serviceClient<misc_msgs::EvaluateExpression>("evaluate_expression");
		}

		void MissionParser::sendPrimitve()
		{
			ROS_ERROR("%d",ID);
			int id = parseMission(ID);
			ROS_ERROR("%s", PRIMITIVES[id]);

			if(id != none){

				misc_msgs::SendPrimitive sendContainer;
				sendContainer.primitiveID = id;
				//sendContainer.primitiveData = serializedData; /* Remove from msg */
				sendContainer.event.timeout = newTimeout;
				sendContainer.event.onEventNextActive = onEventNextActive;
				sendContainer.event.onEventNext = onEventNext;

				sendContainer.primitiveString.data = primitiveString.str();

				pubSendPrimitive.publish(sendContainer);

			} else {

				ROS_ERROR("Mission ended.");
				std_msgs::String tmp;
				tmp.data = "/STOP"; //TODO Change to /missionEnded
				pubRiseEvent.publish(tmp);
			}

		}

		/* Function for parsing primitives in XML mission file */
		int MissionParser::parseMission(int id)
		{
		   XMLNode *mission;
		   XMLNode *primitive;
		   XMLNode *primitiveParam;

		   /* Find mission node */
		   mission = xmlDoc.FirstChildElement("main")->FirstChildElement("mission");
		   if(mission)
		   {
			   /* Loop through primitive nodes */
			   for (primitive = mission->FirstChildElement("primitive"); primitive != NULL; primitive = primitive->NextSiblingElement())
			   {
				   XMLElement *elem = primitive->ToElement();
				   string primitiveName = elem->Attribute("name");
				   ROS_INFO("%s", primitiveName.c_str());

				   primitiveParam = primitive->FirstChildElement("id");
				   XMLElement *elemID = primitiveParam->ToElement();

				   /* If ID is correct process primitive data */
				   string id_string = static_cast<ostringstream*>( &(ostringstream() << id) )->str();
				   string tmp = elemID->GetText();

				   if (tmp.compare(id_string) == 0)
				   {
					   /** Reset data */
						newTimeout = 0;
						onEventNextActive.clear();
						onEventNext.clear();

						primitiveString.str(string());

						/** Initialize service call data */
						misc_msgs::EvaluateExpression evalExpr;

						for(int i = 1; i <= primitiveNum; ++i){

							if(primitiveName.compare(PRIMITIVES[i]) == 0)
							{
							   for (primitiveParam = primitive->FirstChildElement("param"); primitiveParam != NULL; primitiveParam = primitiveParam->NextSiblingElement())
							   {
								   XMLElement *elem2 = primitiveParam->ToElement();
								   string primitiveParamName = elem2->Attribute("name");

								   for(std::vector<std::string>::iterator it = PP.primitive_params[i].begin(); it != PP.primitive_params[i].end(); ++it)
								   {
									   if(primitiveParamName.compare((*it).c_str()) == 0)
									   {
										   primitiveString << (*it).c_str() <<":"<< elem2->GetText() << ":";
										   break;
									   }
								   }

								   if(primitiveParamName.compare("onEventNext") == 0)
								   {
									   onEventNextParse(elem2);
								   }
							   }

							   return i;
							}
						}
				   }
			   }
			   return none;

		   }
		   else
		   {
			   ROS_ERROR("No mission defined");
			   return -1;
		   }
		}

		void MissionParser::onEventNextParse(XMLElement *elem2)
		{
			string primitiveNext = elem2->GetText();

			if(primitiveNext.empty()==0)
			{
				if(strcmp(primitiveNext.c_str(),"bkp") == 0)
				{
					onEventNext.push_back(breakpoint);
				}
				else
				{
					onEventNext.push_back(atoi(primitiveNext.c_str()));
				}
			}
			else
			{
					onEventNext.push_back(ID+1); // provjeri teba li ovo
			}
			onEventNextActive.push_back(atoi(elem2->Attribute("event")));
		}


		/*************************************************************
		 *** Initial parse of XML mission file
		 ************************************************************/

		/* Parse events on mission load */
		int MissionParser::parseEvents()
		{

		   XMLNode *events;
		   XMLNode *event;
		   XMLNode *primitiveParam;

		   /* Find events node */
		   events = xmlDoc.FirstChildElement("main")->FirstChildElement("events");
		   if(events)
		   {
			   for (event = events->FirstChildElement("event"); event != NULL; event = event->NextSiblingElement())
			   {
				   //eventsContainer.push_back(event->ToElement()->GetText());
				   missionEvents.append(event->ToElement()->GetText());
				   missionEvents.append(":");
			   }
		   }
		   else
		   {
			   ROS_ERROR("No events defined");
			   return -1;
		   }
		   return 0;
		}

		/* Parse mission parameters on mission load */
		int MissionParser::parseMissionParam()
		{
		   XMLNode *events;
		   XMLNode *event;
		   XMLNode *primitiveParam;

		   /* Find Mission param node */
		   events = xmlDoc.FirstChildElement("main")->FirstChildElement("params");
		   if(events)
		   {
			   for (event = events->FirstChildElement("param"); event != NULL; event = event->NextSiblingElement())
			   {
				   missionParams.append(event->ToElement()->Attribute("name"));
				   missionParams.append(":");
				   missionParams.append(event->ToElement()->GetText());
				   missionParams.append(":");
			   }
		   }
		   else
		   {
			   ROS_ERROR("No mission parameters defined");
			   return -1;
		   }
		   return 0;
		}

		void MissionParser::resetParser()
		{
			/*** On mission stop reset variables */
			ID = 0;
			missionParams.clear();
			missionEvents.clear();
			xmlDoc.Clear();
		}

		/*************************************************************
		 *** ROS subscriptions
		 ************************************************************/

		void MissionParser::onRequestPrimitive(const std_msgs::UInt16::ConstPtr& req)
		{
			if(req->data > 0)
			{
				ROS_INFO("REQUESTED PRIMITIVE ID: %d", req->data); //TODO
				sendPrimitve();
			}
			else
			{
				ROS_FATAL("REQUESTED INVALID ID");
			}
		}

		void MissionParser::onEventString(const std_msgs::String::ConstPtr& msg)
		{
			if(strcmp(msg->data.c_str(),"/STOP") == 0)
			{
				resetParser();
			}
		}

		void MissionParser::onReceiveMission(const misc_msgs::StartParser::ConstPtr& msg)
		{
			XMLError err_status;

			if(msg->method == misc_msgs::StartParser::FILENAME)
			{
				err_status = xmlDoc.LoadFile(msg->missionData.c_str());
			}
			else if(msg->method == misc_msgs::StartParser::STRING)
			{
				err_status = xmlDoc.Parse(msg->missionData.c_str());
			}

			if(err_status == XML_SUCCESS)
			{
				/* Set mission start offset */
				if(msg->relative)
				{
					offset.north = -msg->startPosition.north;
					offset.east = -msg->startPosition.east;
				}
				else
				{
					offset.north = offset.east = 0;
				}

				/* On XML load parse mission parameters and mission events */
				parseEvents();
				parseMissionParam();

				/* Publish mission setup */
				misc_msgs::MissionSetup missionSetup;
				missionSetup.missionEvents = missionEvents;
				missionSetup.missionParams = missionParams;
				missionSetup.missionOffset = offset;
				pubMissionSetup.publish(missionSetup);
			}
			else
			{
				ROS_FATAL("INVALID MISSION PARSER REQUEST");
			}
		}
	}
}

/*********************************************************************
 ***  Main function
 ********************************************************************/

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mission_parser");
	labust::mission::MissionParser MP;
	ros::spin();
	return 0;
}






