
///\todo  Dodati prebacivanje u slucaju gubitka informacija od quadrotora
///\todo ubaciti deklaraciju primitive struct-ova u klasu controllerManager
///\todo dodati dva prototipa za pozivanje primitiva (direktno zadavanje i preko kompleksnijih struktura podataka pointStamped)
///\todo kako bi se dodali razliciti prototipovi potrebno definirati actionClient kao clana klase
//\todo odluciti da li dodati mjerenja unutar missionExecutiona ili controllerManagera
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

#include <auv_msgs/NavSts.h>
#include <auv_msgs/Bool6Axis.h>
#include <navcon_msgs/EnableControl.h>
#include <navcon_msgs/ConfigureAxes.h>

#include <labust_mission/labustMission.hpp>
#include <labust_mission/primitiveActionClient.hpp>
#include <labust_mission/utils.hpp>
#include <labust_mission/lowLevelConfigure.hpp>

#include <navcon_msgs/ConfigureVelocityController.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <navcon_msgs/CourseKeepingAction.h>
#include <navcon_msgs/DynamicPositioningAction.h>
#include <navcon_msgs/GoToPointAction.h>
#include <navcon_msgs/DOFIdentificationAction.h>

/*********************************************************************
 ***  Global variables
 ********************************************************************/

/* Primitive callbacks */
utils::CourseKeepingFA_CB CK_FA;
utils::CourseKeepingUA_CB CK_UA;
utils::DPprimitive_CB DP_FA;
utils::Go2PointFA_CB G2P_FA;
utils::Go2PointUA_CB G2P_UA;
utils::ISO_CB ISO;


//typedef actionlib::SimpleActionClient<navcon_msgs::CourseKeepingAction> Client;
//actionlib::SimpleActionClient<navcon_msgs::GoToPointAction> *ac_ptr;

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
			 *** Controller primitives
			 ********************************************************/

			/* Go to point fully actuated primitive */
			void go2point_FA_hdg(bool enable, double north1, double east1, double north2, double east2, double speed, double heading, double radius);

			/* Go to point fully actuated primitive */
			void go2point_FA(bool enable, double north1, double east1, double north2, double east2, double speed, double radius);
			//void go2point_FA(bool enable);

			/* Go to point underactuated primitive */
			void go2point_UA(bool enable, double north1, double east1, double north2, double east2, double speed, double radius);
			//void go2point_FA(bool enable);

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

			/* Get state estimates used in control loop */
			void stateHatCallback(const auv_msgs::NavSts::ConstPtr& data);

			/* Get absolute state estimates */
			void stateHatAbsCallback(const auv_msgs::NavSts::ConstPtr& data);


			/*********************************************************
			 *** Class variables
			 ********************************************************/

			bool LF_FAenable;
			bool LF_UAenable;
			bool HDGenable;
			bool DPenable;
			bool DEPTHenable;
			bool ALTenable;
			bool LL_VELenable;
			double Xpos, Ypos, YawPos;

			auv_msgs::NED posVariance;
			auv_msgs::NavSts meas;

		private:

			ros::Publisher pubStateRef;
			ros::Subscriber subStateHat;
			ros::Subscriber subStateHatAbs;
			labust::LowLevelConfigure LLcfg;

			actionlib::SimpleActionClient<navcon_msgs::GoToPointAction> ac;
			actionlib::SimpleActionClient<navcon_msgs::GoToPointAction> ac2;
			actionlib::SimpleActionClient<navcon_msgs::DynamicPositioningAction> ac3;
			actionlib::SimpleActionClient<navcon_msgs::CourseKeepingAction> ac4;
			actionlib::SimpleActionClient<navcon_msgs::CourseKeepingAction> ac5;
			actionlib::SimpleActionClient<navcon_msgs::DOFIdentificationAction> ac6;

		};

	}

}

using namespace labust::controller;

	/*
	 * Constructor
	 */
	ControllerManager::ControllerManager(ros::NodeHandle nh): LF_FAenable(false),
											LF_UAenable(false),
											HDGenable(false),
											DPenable(false),
											DEPTHenable(false),
											ALTenable(false),
											LL_VELenable(false),
											Xpos(0.0),
											Ypos(0.0),
											YawPos(0.0),
											ac("go2point_FA", true),
											ac2("go2point_UA", true),
											ac3("DPprimitive", true),
											ac4("course_keeping_FA", true),
											ac5("course_keeping_UA", true),
											ac6("Identification", true),
											LLcfg(nh)
	{

		/** Publishers */
		pubStateRef = nh.advertise<auv_msgs::NavSts>("stateRef", 1);

		/** Subscribers */
		subStateHat = nh.subscribe<auv_msgs::NavSts>("stateHat",1, &ControllerManager::stateHatCallback,this);
		subStateHatAbs = nh.subscribe<auv_msgs::NavSts>("stateHatAbs",1, &ControllerManager::stateHatAbsCallback,this);
	}


	/*********************************************************
	 *** Controller primitives
	 ********************************************************/

	/*
	 * Course keeping fully actuated primitive
	 */
	void ControllerManager::go2point_FA_hdg(bool enable, double north1, double east1, double north2, double east2, double speed, double heading, double radius){

		//static actionlib::SimpleActionClient<navcon_msgs::GoToPointAction> ac("go2point_FA", true);

		if(enable){

			ROS_INFO("Waiting for action server to start.");
			ac.waitForServer(); //will wait for infinite time
			ROS_INFO("Action server started, sending goal.");

			LLcfg.LL_VELconfigure(true,2,2,0,0,0,2);

			navcon_msgs::GoToPointGoal goal;
			goal.T1.point.x = north1;
			goal.T1.point.y = east1;
			goal.T2.point.x = north2;
			goal.T2.point.y = east2;
			goal.yaw = heading;
			goal.speed = speed;
			goal.radius = radius;


			ac.sendGoal(goal,
							boost::bind(&utils::Go2PointFA_CB::doneCb, G2P_FA, _1, _2),
							boost::bind(&utils::Go2PointFA_CB::activeCb, G2P_FA),
							boost::bind(&utils::Go2PointFA_CB::feedbackCb, G2P_FA, _1));

		} else {

			boost::this_thread::sleep(boost::posix_time::milliseconds(200));
			ac.cancelGoalsAtAndBeforeTime(ros::Time::now());

			enableController("UALF_enable",false);
			enableController("HDG_enable",false);
			LLcfg.LL_VELconfigure(false,1,1,0,0,0,1);
		}
	}

	/*
	 * Course keeping fully actuated primitive
	 */
	void ControllerManager::go2point_FA(bool enable, double north1, double east1, double north2, double east2, double speed, double radius){

		//static actionlib::SimpleActionClient<navcon_msgs::GoToPointAction> ac("go2point_FA", true);

		if(enable){

			ROS_INFO("Waiting for action server to start.");
			ac.waitForServer(); //will wait for infinite time
			ROS_INFO("Action server started, sending goal.");

			LLcfg.LL_VELconfigure(true,2,2,0,0,0,2);

			navcon_msgs::GoToPointGoal goal;
			goal.T1.point.x = north1;
			goal.T1.point.y = east1;
			goal.T2.point.x = north2;
			goal.T2.point.y = east2;
			goal.yaw = atan2(east2-east1,north2-north1);
			goal.speed = speed;
			goal.radius = radius;


			ac.sendGoal(goal,
							boost::bind(&utils::Go2PointFA_CB::doneCb, G2P_FA, _1, _2),
							boost::bind(&utils::Go2PointFA_CB::activeCb, G2P_FA),
							boost::bind(&utils::Go2PointFA_CB::feedbackCb, G2P_FA, _1));

		} else {

			boost::this_thread::sleep(boost::posix_time::milliseconds(200));
			ac.cancelGoalsAtAndBeforeTime(ros::Time::now());

			enableController("UALF_enable",false);
			enableController("HDG_enable",false);
			LLcfg.LL_VELconfigure(false,1,1,0,0,0,1);
		}
	}



	/*
	 * Course keeping underactuated primitive
	 */
	void ControllerManager::go2point_UA(bool enable, double north1, double east1, double north2, double east2, double speed, double radius){

		//static actionlib::SimpleActionClient<navcon_msgs::GoToPointAction> ac2("go2point_UA", true);

		if(enable){

			ROS_INFO("Waiting for action server to start.");
			ac2.waitForServer(); //will wait for infinite time
			ROS_INFO("Action server started, sending goal.");

			LLcfg.LL_VELconfigure(true,2,2,0,0,0,2);

			double tmp;
			tmp = atan2(east2-east1, north2-north1);


			navcon_msgs::GoToPointGoal goal;
			goal.T1.point.x = north1;
			goal.T1.point.y = east1;
			goal.T2.point.x = north2;
			goal.T2.point.y = east2;

			goal.yaw = tmp; // Underactuated
			goal.speed = speed;
			goal.radius = radius;

			if(ac2.getState() == actionlib::SimpleClientGoalState::ACTIVE){

				ac2.cancelGoal();
				ac2.sendGoal(goal,
								boost::bind(&utils::Go2PointUA_CB::doneCb, G2P_UA, _1, _2),
								boost::bind(&utils::Go2PointUA_CB::activeCb, G2P_UA),
								boost::bind(&utils::Go2PointUA_CB::feedbackCb, G2P_UA, _1));


			} else {
			ac2.sendGoal(goal,
							boost::bind(&utils::Go2PointUA_CB::doneCb, G2P_UA, _1, _2),
							boost::bind(&utils::Go2PointUA_CB::activeCb, G2P_UA),
							boost::bind(&utils::Go2PointUA_CB::feedbackCb, G2P_UA, _1));
			}

		} else {

			boost::this_thread::sleep(boost::posix_time::milliseconds(200));
			ac2.cancelGoalsAtAndBeforeTime(ros::Time::now());

			enableController("UALF_enable_1",false);
			enableController("HDG_enable",false);
			LLcfg.LL_VELconfigure(false,1,1,0,0,0,1);
		}
	}

	void ControllerManager::dynamic_positioning(bool enable, double north, double east, double heading){

		//static actionlib::SimpleActionClient<navcon_msgs::DynamicPositioningAction> ac3("DPprimitive", true);

		if(enable){

			ROS_INFO("Waiting for action server to start.");
			ac3.waitForServer(); //will wait for infinite time

			ROS_INFO("Action server started, sending goal.");

			//DPcontroller(true);
			//HDGcontroller(true);

			LLcfg.LL_VELconfigure(true,2,2,0,0,0,2);

			navcon_msgs::DynamicPositioningGoal goal;
			goal.T1.point.x = north;
			goal.T1.point.y = east;
			goal.yaw = heading;

			if(ac3.getState() == actionlib::SimpleClientGoalState::ACTIVE){

							//ac3.cancelGoal();


							ac3.cancelGoalsAtAndBeforeTime(ros::Time::now());
					      //  boost::this_thread::sleep(boost::posix_time::milliseconds(200));
							ac3.sendGoal(goal,
										boost::bind(&utils::DPprimitive_CB::doneCb, DP_FA, _1, _2),
										boost::bind(&utils::DPprimitive_CB::activeCb, DP_FA),
										boost::bind(&utils::DPprimitive_CB::feedbackCb, DP_FA, _1));


						} else {

			ac3.sendGoal(goal,
						boost::bind(&utils::DPprimitive_CB::doneCb, DP_FA, _1, _2),
						boost::bind(&utils::DPprimitive_CB::activeCb, DP_FA),
						boost::bind(&utils::DPprimitive_CB::feedbackCb, DP_FA, _1));
						}
		} else {

			boost::this_thread::sleep(boost::posix_time::milliseconds(200));
			ac3.cancelGoalsAtAndBeforeTime(ros::Time::now());
			//DPcontroller(false);
			//HDGcontroller(false);

			enableController("FADP_enable",false);
			enableController("HDG_enable",false);
			LLcfg.LL_VELconfigure(false,1,1,0,0,0,1);
		}
	}


	/*
	 * Course keeping fully actuated primitive
	 */
	void ControllerManager::course_keeping_FA(bool enable, double course, double speed, double heading){

		//static actionlib::SimpleActionClient<navcon_msgs::CourseKeepingAction> ac4("course_keeping_FA", true);

		if(enable){

			ROS_INFO("Waiting for action server to start.");
			ac4.waitForServer(); //will wait for infinite time

			ROS_INFO("Action server started, sending goal.");

		    LLcfg.LL_VELconfigure(true,2,2,0,0,0,2);

		    navcon_msgs::CourseKeepingGoal goal;
		    goal.course = course;
		    goal.speed = speed;
		    goal.yaw = heading;

		  	ac4.sendGoal(goal,
						boost::bind(&utils::CourseKeepingFA_CB::doneCb, CK_FA, _1, _2),
						boost::bind(&utils::CourseKeepingFA_CB::activeCb, CK_FA),
						boost::bind(&utils::CourseKeepingFA_CB::feedbackCb, CK_FA, _1));
		} else {

			boost::this_thread::sleep(boost::posix_time::milliseconds(200));
			ac4.cancelGoalsAtAndBeforeTime(ros::Time::now());

			enableController("UALF_enable",false);
			enableController("HDG_enable",false);
			LLcfg.LL_VELconfigure(false,1,1,0,0,0,1);
  		}
	}

	/*
	 * Course keeping underactuated primitive
	 */
	void ControllerManager::course_keeping_UA(bool enable, double course, double speed){

		//static actionlib::SimpleActionClient<navcon_msgs::CourseKeepingAction> ac5("course_keeping_UA", true);

		if(enable){

			ROS_ERROR("Waiting for action server to start.");
			  // wait for the action server to start
			ac5.waitForServer(); //will wait for infinite time
			ROS_ERROR("Action server started, sending goal.");

			LLcfg.LL_VELconfigure(true,2,2,0,0,0,2);


			navcon_msgs::CourseKeepingGoal goal;
		    goal.course = course;
		    goal.speed = speed;
		    goal.yaw = course;

		    ac5.sendGoal(goal,
						boost::bind(&utils::CourseKeepingUA_CB::doneCb, CK_UA, _1, _2),
						boost::bind(&utils::CourseKeepingUA_CB::activeCb, CK_UA),
						boost::bind(&utils::CourseKeepingUA_CB::feedbackCb, CK_UA, _1));

		} else {

			boost::this_thread::sleep(boost::posix_time::milliseconds(200));
			ac5.cancelGoalsAtAndBeforeTime(ros::Time::now());

			enableController("UALF_enable_1",false);
			enableController("HDG_enable",false);
			LLcfg.LL_VELconfigure(false,1,1,0,0,0,1);
		}
	}

	void ControllerManager::ISOprimitive(bool enable, int dof, double command, double hysteresis, double reference, double sampling_rate){

		//static actionlib::SimpleActionClient<navcon_msgs::GoToPointAction> ac("go2point_FA", true);

		if(enable){

			//self.velconName = rospy.get_param("~velcon_name","velcon")
			//self.model_update = rospy.Publisher("model_update", ModelParamsUpdate)
			ros::NodeHandle nh, ph("~");
			string velconName;
			ph.param<string>("velcon_name",velconName,"velcon");

            const char *names[7] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw", "Altitude"};

			ROS_INFO("Waiting for action server to start.");
			ac6.waitForServer(); //will wait for infinite time
			ROS_INFO("Action server started, sending goal.");

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

			ac6.sendGoal(goal,
							boost::bind(&utils::ISO_CB::doneCb, ISO, _1, _2),
							boost::bind(&utils::ISO_CB::activeCb, ISO),
							boost::bind(&utils::ISO_CB::feedbackCb, ISO, _1));

		} else {

			boost::this_thread::sleep(boost::posix_time::milliseconds(200));
			ac6.cancelGoalsAtAndBeforeTime(ros::Time::now());

			enableController("UALF_enable",false);
			enableController("HDG_enable",false);
			LLcfg.LL_VELconfigure(false,1,1,0,0,0,1);
		}

	}

//    def onModelUpdate(self, result, use_linear):
//        update = ModelParamsUpdate()
//        update.dof = result.dof
//        update.alpha = result.alpha
//        update.beta = result.beta
//        update.betaa = result.betaa
//        update.delta = result.delta
//        update.wn = result.wn
//        update.use_linear = use_linear
//        self.model_update.publish(update)

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

	/*
	 * Low-level velocity controller
	 */
	void ControllerManager::LL_VELcontroller(bool enable){

//		if(enable){
//			utils::callService<navcon_msgs::ConfigureVelocityController>(clientConfigureVelocitiyController,velConConf);
//		} else {
//			for( int i = 0; i<=5; i++){
//				velConConf.request.desired_mode[i] = 0;
//			}
//				utils::callService<navcon_msgs::ConfigureVelocityController>(clientConfigureVelocitiyController,velConConf);
//		}
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

	/*
	 * Collect state measurements
	 */
	void ControllerManager::stateHatCallback(const auv_msgs::NavSts::ConstPtr& data){

		posVariance = data->position_variance;
	}

	void ControllerManager::stateHatAbsCallback(const auv_msgs::NavSts::ConstPtr& data){

		Xpos = data->position.north;
		Ypos = data->position.east;
		YawPos = data->orientation.yaw;
	}


#endif /* CONTROLLERMANAGER_HPP_ */
