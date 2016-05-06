/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010-2016, LABUST, UNIZG-FER
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
#ifndef LABUST_MISSION_CADDY_MISSIONS_H
#define LABUST_MISSION_CADDY_MISSIONS_H
#include <auv_msgs/NavSts.h>
#include <std_msgs/UInt8.h>
#include <ros/ros.h>

namespace labust
{
	namespace mission
	{
		/**
		 * The class implements a small CADDY mission controller to start
		 * and stop lawn-mower missions launched from the topside.
		 * The controller incorporates minimalistic safety features for
		 * underwater operation.
		 */
		class CaddyMissions
		{
		  enum {IDLE=0,LAWN_MOWER,STOP};
		  enum {POSITION_UPDATE=1, AC_UPDATE=10};
		  enum {u=0,v,w,p,q,r};
		  enum {DONT_CARE=-1, DISABLED=0, VELCON=2};

		public:
			///Main constructor
		    CaddyMissions();
		    ///Main destructor
		    ~CaddyMissions();
		    ///Initialize the mission controller
			void onInit();

		private:
			///Handle incoming commands.
			void onSurfaceCmd(const std_msgs::UInt8::ConstPtr& cmd);
			///Monitor the state estimation.
			void onPosition(const auv_msgs::NavSts::ConstPtr& msg);
			///Surface connection testing over WiFi network.
			bool checkNetwork();
			///Safety check that all controllers are connected
			bool testControllers();
			///Turn-off all controllers
			void stopControllers();
			///The timer callback
			void onTimer(const ros::TimerEvent& event);

			///Safety timer
			ros::Timer safety;
			///Last arrived position
			ros::Time last_position;
			///Last cmd arrived
			ros::Time last_cmd;

			/// Safety timeout indicator
			ros::Publisher timeout_pub;
			/// Lawn-mower interrupt publisher
			ros::Publisher lawnmower_pub;
			/// Position estimate subscriber
			ros::Subscriber position_sub;
            /// Surface command subscriber
            ros::Subscriber surfacecmd_sub;

			/// Speed controller services.
			ros::ServiceClient velcon;
			/// Heading controller services.
			ros::ServiceClient hdgcon;
			/// Altitude controller services.
			ros::ServiceClient altcon;
			/// Depth controller service.
			ros::ServiceClient depthcon;

			/// The reference IP address on network
			std::string ipaddress;
		};
	}
}

/* LABUST_MISSION_CADDY_MISSIONS_H */
#endif