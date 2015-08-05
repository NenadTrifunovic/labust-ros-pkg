/*********************************************************************
 * data_event_manager.cpp
 *
 *  Created on: Jun 30, 2014
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

#include<labust_mission/labustMission.hpp>
#include<labust_mission/dataManager.hpp>
#include<labust_mission/eventEvaluation.hpp>

/*********************************************************************
 *** DataEventManager class definition
 ********************************************************************/

class DataEventManager{

public:
	DataEventManager():missionLoaded(false){

		ros::NodeHandle nh;

		/** Subscribers */
		subStateHatAbs = nh.subscribe<auv_msgs::NavSts>("stateHatAbs",1,&DataEventManager::onStateHat,this);
		subMissionSetup = nh.subscribe<misc_msgs::MissionSetup>("missionSetup",1,&DataEventManager::onMissionSetup,this);
		subExternalEvents= nh.subscribe<misc_msgs::ExternalEvent>("externalEvent",1, &DataEventManager::onExternalEvent, this);
		subEventString = nh.subscribe<std_msgs::String>("eventString",1, &DataEventManager::onEventString, this);

		/** Publishers */
		pubDataEventsContainer = nh.advertise<misc_msgs::DataEventsContainer>("dataEventsContainer",1);

		/** Services */
		srvEvaluateExpression = nh.advertiseService("evaluate_expression", &DataEventManager::expressionEvaluationService,this);
	}

	/** Callback for evaluating states and event condition */
	void onStateHat(const auv_msgs::NavSts::ConstPtr& data){

		/** Update all data in DataManager */
		DM.updateStateVar(data);

		if(missionLoaded){
			/** Update EventEvaluation symbol table and evaluate events */
			EE.updateSymbolTable(DM.getStateVar(),DM.getMissionVar(),DM.getMissionVarNames());
			DM.updateEventsVar(EE.evaluateEvents(eventsContainer));

			/** Publish events data and events states */
			publishDataEvent(DM.getStateVar(), DM.getMissionVar(), DM.getEventsVar());
		}
	}

	/** Publish data and events */
	void publishDataEvent(vector<double> stateVar, vector<double> missionVar, vector<uint8_t> eventsVar){

		misc_msgs::DataEventsContainer dataEventsContainer;
		dataEventsContainer.stateVar.data  = stateVar;
		dataEventsContainer.eventsVar = eventsVar;
		dataEventsContainer.missionVar.data = missionVar;

		pubDataEventsContainer.publish(dataEventsContainer);
	}

	/** On external change of mission variable update data manager */
	void onExternalEvent(const misc_msgs::ExternalEvent::ConstPtr& data){

		if((data->id > 0) && (data->id <= DM.getMissionVar().size()))
			DM.updateMissionVar(data->id, data->value);
	}

	/** Callback that initializes mission parameters and events */
	void onMissionSetup(const misc_msgs::MissionSetup::ConstPtr& data){

		parseMissionParam(data->missionParams.c_str());
		parseMissionEvents(data->missionEvents.c_str());

		/** Initialize symbol table */
		EE.initializeSymbolTable(DM.getStateVar(), DM.getMissionVar(), DM.getMissionVarNames());

		ROS_ERROR("Mission successfully loaded");
		missionLoaded = true;
	}

	void onEventString(const std_msgs::String::ConstPtr& msg){

		ROS_INFO("EventString: %s",msg->data.c_str());
		if(strcmp(msg->data.c_str(),"/STOP") == 0){
			missionLoaded = false;
			DM.reset();
			EE.resetSymbolTable();
		}
	}

	/** Service that evaluates string expression */
	bool expressionEvaluationService(misc_msgs::EvaluateExpression::Request &req, misc_msgs::EvaluateExpression::Response &res){

		res.result = EE.evaluateStringExpression(req.expression);
		return true;
	}

	/*****************************************************************
	 *** Helper functions
	 ****************************************************************/

	/** Parse string with mission parameters sent from mission parser */
	void parseMissionParam(string missionParamString){

		if(missionParamString.empty() == 0){

			vector<string> tmp;
			tmp = labust::utilities::split(missionParamString.c_str(), ':');

			for(vector<string>::iterator it = tmp.begin(); it != tmp.end(); it=it+2){

				DM.setMissionVarNames((*(it)).c_str()); /** Assign mission variable name */
				DM.setMissionVar(atof((*(it+1)).c_str())); /** Assign mission variable value */
			}
		}
	}

	/** Parse string with mission events sent from mission parser */
	void parseMissionEvents(string missionEventsString){

		if(missionEventsString.empty() == 0){
			eventsContainer = labust::utilities::split(missionEventsString.c_str(), ':');
			vector<uint8_t> tmp(eventsContainer.size(),0);
			DM.updateEventsVar(tmp); /** Resize event states vector */
		}
	}

	/*****************************************************************
	 *** Class variables
	 ****************************************************************/

private:

	/** Data container class */
	labust::data::DataManager DM;

	/** Class for event and expression evaluation */
	labust::event::EventEvaluation EE;

	/** Subscribers */
	ros::Subscriber subStateHatAbs, subMissionSetup, subExternalEvents, subEventString;

	/** Publishers */
	ros::Publisher pubDataEvent, pubDataEventsContainer;

	/** Services */
	ros::ServiceServer srvEvaluateExpression;

	/** Mission events container */
	vector<string> eventsContainer;

	/** Mission active status flag */
	bool missionLoaded;
};

/*********************************************************************
 *** Main
 ********************************************************************/

int main(int argc, char **argv){

	ros::init(argc, argv, "data_event_manager");
	ros::NodeHandle nh;
	DataEventManager DEM;
	ros::spin();
	return 0;
}


