/*********************************************************************
 * mission_exec.cpp
 *
 *  Created on: Mar 24, 2014
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
#include <labust_mission/primitiveManager.hpp>
#include <labust_mission/missionExecution.hpp>

#include <tinyxml2.h>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace decision_making;
using namespace tinyxml2;

/*********************************************************************
*** Global variables
*********************************************************************/

EventQueue* mainEventQueue;
labust::mission::MissionExecution* ME = NULL;

struct MainEventQueue{
MainEventQueue(){ mainEventQueue = new RosEventQueue(); }
~MainEventQueue(){ delete mainEventQueue; }
};

/*********************************************************************
 *** Finite State Machine
 *********************************************************************/

	/* Mission selection  */
	FSM(MissionSelect)
	{
		FSM_STATES
		{
			/*** Execution states */
			Wait_state,
			Dispatcher_state,
			placeholder_state,
			/*** Primitive states */
			go2point_FA_state,
			dynamic_positioning_state,
			course_keeping_state,
			//iso_state,
			follow_state,
			pointer_state
		}
		FSM_START(Wait_state);
		FSM_BGN
		{
			FSM_STATE(Wait_state)
			{
				ROS_ERROR("Mission waiting...");

				FSM_ON_STATE_EXIT_BGN{

					ROS_ERROR("Starting mission...");
					/** Wait for data and events initialization */
					//ros::Rate(ros::Duration(1.0)).sleep(); Vidjeti je li potrebno

					/** Get current vehicle position */
					ME->oldPosition.north = ME->state.position.north;
					ME->oldPosition.east = ME->state.position.east;
					ME->oldPosition.depth = ME->state.position.depth;

				}FSM_ON_STATE_EXIT_END

				FSM_TRANSITIONS
				{
					FSM_ON_EVENT("/START_DISPATCHER", FSM_NEXT(Dispatcher_state));
				}
			}
			FSM_STATE(Dispatcher_state)
			{
				ROS_ERROR("Dispatcher active");
				ME->missionActive = true;
				ME->requestPrimitive();

				FSM_TRANSITIONS
				{
					FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
					FSM_ON_EVENT("/PLACEHOLDER", FSM_NEXT(placeholder_state));
					FSM_ON_EVENT("/GO2POINT_FA", FSM_NEXT(go2point_FA_state));
					FSM_ON_EVENT("/DYNAMIC_POSITIONING", FSM_NEXT(dynamic_positioning_state));
					//FSM_ON_EVENT("/COURSE_KEEPING", FSM_NEXT(course_keeping_state));
					//FSM_ON_EVENT("/ISO", FSM_NEXT(iso_state));
					FSM_ON_EVENT("/FOLLOW", FSM_NEXT(follow_state));
					FSM_ON_EVENT("/POINTER", FSM_NEXT(pointer_state));
				}
			}
			FSM_STATE(placeholder_state)
			{
				ROS_ERROR("Placeholder active");

				FSM_TRANSITIONS
				{
					FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
					FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
					FSM_ON_EVENT("/TIMEOUT", FSM_NEXT(Dispatcher_state));
				}
			}
			FSM_STATE(go2point_FA_state)
			{
				ROS_ERROR("go2point_FA primitive active");

				ME->go2point_FA_state();

				FSM_ON_STATE_EXIT_BGN{

					ME->CM.go2point_FA(false,0,0,0,0,0,0);

				}FSM_ON_STATE_EXIT_END

				FSM_TRANSITIONS
				{
					FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
					FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
					FSM_ON_EVENT("/TIMEOUT", FSM_NEXT(Dispatcher_state));

				}
			}
			FSM_STATE(dynamic_positioning_state)
			{
				ROS_ERROR("dynamic_positioning primitive active");

				ME->dynamic_postitioning_state();

				FSM_ON_STATE_EXIT_BGN{

					ME->CM.dynamic_positioning(false,0,0,0);

				}FSM_ON_STATE_EXIT_END

				FSM_TRANSITIONS
				{
					FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
					FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
					FSM_ON_EVENT("/TIMEOUT", FSM_NEXT(Dispatcher_state));
				}
			}
/*			FSM_STATE(course_keeping_state)
			{
				ROS_ERROR("course_keeping_FA primitive active");

				ME->course_keeping_FA_state();

				FSM_ON_STATE_EXIT_BGN{

					ME->CM.course_keeping_FA(false,0,0,0);

					ME->oldPosition.north = ME->state.position.north;
					ME->oldPosition.east = ME->state.position.east;
					ME->oldPosition.depth = ME->state.position.depth;

				}FSM_ON_STATE_EXIT_END

				FSM_TRANSITIONS
				{
					FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
					FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
					FSM_ON_EVENT("/TIMEOUT", FSM_NEXT(Dispatcher_state));
				}
			}*/
/*			FSM_STATE(iso_state)
			{
				ROS_ERROR("iso primitive active");

				ME->iso_state();


				FSM_ON_STATE_EXIT_BGN{

					ME->CM.ISOprimitive(false,0,0,0,0,0);

					ME->oldPosition.north = ME->state.position.north;
					ME->oldPosition.east = ME->state.position.east;
					ME->oldPosition.depth = ME->state.position.depth;

				}FSM_ON_STATE_EXIT_END

				FSM_TRANSITIONS
				{
					FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
					FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
					FSM_ON_EVENT("/TIMEOUT", FSM_NEXT(Dispatcher_state));
				}
			}*/
			FSM_STATE(follow_state)
			{
				ROS_ERROR("follow primitive active");

				ME->follow_state();


				FSM_ON_STATE_EXIT_BGN
				{
					ME->CM.follow(false,0,0,0,0,0,0,0,0,0,0,0);
				}
				FSM_ON_STATE_EXIT_END

				FSM_TRANSITIONS
				{
					FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
					FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
					FSM_ON_EVENT("/TIMEOUT", FSM_NEXT(Dispatcher_state));
				}
			}
			FSM_STATE(pointer_state)
			{
				ROS_ERROR("pointer primitive active");

				ME->pointer_state();


				FSM_ON_STATE_EXIT_BGN
				{
					ME->CM.pointer(false,0,0,0,0,0,0,0,0,"","");
				}
				FSM_ON_STATE_EXIT_END

				FSM_TRANSITIONS
				{
					FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
					FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
					FSM_ON_EVENT("/TIMEOUT", FSM_NEXT(Dispatcher_state));
				}
			}
		}
		FSM_END
	}


/*********************************************************************
 ***  Main function
 ********************************************************************/

int main(int argc, char** argv){

	ros::init(argc, argv, "ControllerFSM");
	ros_decision_making_init(argc, argv);
	ros::NodeHandle nh;

	/* Start Mission Execution */
	labust::mission::MissionExecution MissExec(nh);
	ME = &MissExec;

	/* Global event queue */
	MainEventQueue meq;

	/* Start state machine */
	ros::AsyncSpinner spinner(2);
	spinner.start();
	FsmMissionSelect(NULL, mainEventQueue);
	spinner.stop();

	return 0;
}



