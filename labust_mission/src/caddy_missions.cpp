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
#include <labust/mission/caddy_missions.h>
#include <navcon_msgs/ConfigureVelocityController.h>
#include <navcon_msgs/EnableControl.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Bool.h>

#include <cstdio>

using labust::mission::CaddyMissions;

CaddyMissions::CaddyMissions():
    ipaddress("10.0.10.1")
{
  this->onInit();
}

CaddyMissions::~CaddyMissions()
{
  safety.stop();
}

void CaddyMissions::onInit()
{
	ros::NodeHandle nh,ph("~");

	//Setup parameters
	ph.param("ipaddress",ipaddress,ipaddress);

	//Initialize publishers
	timeout_pub = nh.advertise<std_msgs::Bool>("mission_timeout", 1);
	lawnmower_pub = nh.advertise<std_msgs::Bool>("stop_follow_section", 1);

	//Initialize service clients
	velcon = nh.serviceClient<navcon_msgs::ConfigureVelocityController>("ConfigureVelocityController", true);
	hdgcon = nh.serviceClient<navcon_msgs::EnableControl>("HDG_enable", true);
	altcon = nh.serviceClient<navcon_msgs::EnableControl>("ALT_enable", true);

	//Initialze subscribers
    position_sub = nh.subscribe<auv_msgs::NavSts>("position", 1, &CaddyMissions::onPosition,this);
    surfacecmd_sub = nh.subscribe<std_msgs::UInt8>("surface_cmd", 1, &CaddyMissions::onSurfaceCmd,this);

    //Initialize timer
    safety = nh.createTimer(ros::Duration(0.5),&CaddyMissions::onTimer, this, false, true);
}

void CaddyMissions::onSurfaceCmd(const std_msgs::UInt8::ConstPtr& cmd)
{
  last_cmd = ros::Time::now();

  if (cmd->data == LAWN_MOWER)
  {
    if (testControllers())
    {
      ROS_INFO("Setup controller for lawn-mower mission.");
      navcon_msgs::ConfigureVelocityController srv;
      for(int i=0; i<6; ++i) srv.request.desired_mode[i] = DONT_CARE;
      srv.request.desired_mode[u] = VELCON;
      srv.request.desired_mode[v] = DISABLED;
      srv.request.desired_mode[r] = VELCON;
      bool velconOK = velcon.call(srv);
      navcon_msgs::EnableControl flag;
      flag.request.enable = true;
      bool hdgconOK = hdgcon.call(flag);
      bool altconOK = altcon.call(flag);

      if (altconOK && hdgconOK && velconOK)
      {
        ROS_INFO("Lawn-mower setup complete.");
      }
      else
      {
        ROS_ERROR("Lawn-mower setup failed.");
        this->stopControllers();
      }
    }
    else
    {
      ROS_ERROR("Controller or position estimate not ready for lawn-mower mission.");
    }
  }
  else if (cmd->data == STOP)
  {
    this->stopControllers();
    //Stop the lawn-mower
    std_msgs::Bool flag;
    flag.data = false;
    //Added a hack to kill the lawnmower
    for(int i=0; i<10; ++i)
    {
      lawnmower_pub.publish(flag);
      ros::Duration(0.1).sleep();
    }
  }
}

void CaddyMissions::stopControllers()
{
  if (testControllers())
  {
    navcon_msgs::ConfigureVelocityController srv;
    for(int i=0; i<6; ++i) srv.request.desired_mode[i] = DISABLED;
    velcon.call(srv);
    navcon_msgs::EnableControl flag;
    flag.request.enable = false;
    hdgcon.call(flag);
    altcon.call(flag);
  }
}

bool CaddyMissions::testControllers()
{
  bool retVal = velcon.exists() && hdgcon.exists() && altcon.exists();
  return retVal && ((ros::Time::now() - last_position).toSec() < POSITION_UPDATE);
}

void CaddyMissions::onPosition(const auv_msgs::NavSts::ConstPtr& msg)
{
  last_position = ros::Time::now();
}

void CaddyMissions::onTimer(const ros::TimerEvent& event)
{
  bool noac_update = (ros::Time::now() - last_cmd).toSec() > AC_UPDATE;
  bool nonet_update = !checkNetwork();
  if (noac_update && nonet_update)
  {
    ROS_WARN("Timeout triggered - stopping controllers.");
    this->stopControllers();
  }
}

bool CaddyMissions::checkNetwork()
{
  FILE* in;
  char buff[512];

  std::string command = "fping -c2 -t300 " + ipaddress + " 2>&1";
  //ROS_INFO("Command: %s", command.c_str());
  if (!(in = popen(command.c_str(), "r"))) return false;

  //std::stringstream sout;
  //while(fgets(buff, sizeof(buff), in)!=NULL) sout << buff;
  //ROS_INFO("%s",sout.str().c_str());

  return pclose(in) == 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "caddy_missions");
    CaddyMissions mission_controller;
    ros::spin();
    return 0;
}
