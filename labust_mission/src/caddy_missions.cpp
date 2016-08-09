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
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <std_srvs/Trigger.h>
#include <misc_msgs/Go2depthService.h>
#include <misc_msgs/Go2pointService.h>
#include <misc_msgs/PointerPrimitiveService.h>
#include <misc_msgs/DynamicPositioningPrimitiveService.h>

#include <cstdio>

using labust::mission::CaddyMissions;

CaddyMissions::CaddyMissions():
                ipaddress("10.0.10.1"),
                pointer_radius(4.0),
                mission_state(IDLE),
                go_and_carry_substate(NONE)
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
  ph.param("pointer_radius", pointer_radius, pointer_radius);

  //Initialize publishers
  timeout_pub = nh.advertise<std_msgs::Bool>("mission_timeout", 1);
  lawnmower_pub = nh.advertise<std_msgs::Bool>("stop_follow_section", 1);

  event_string_pub = nh.advertise<std_msgs::String>("eventString", 1);

  //Initialize service clients
  dpcon = nh.serviceClient<navcon_msgs::EnableControl>("FADP_enable");
  hdgcon = nh.serviceClient<navcon_msgs::EnableControl>("HDG_enable");
  altcon = nh.serviceClient<navcon_msgs::EnableControl>("ALT_enable");
  depthcon = nh.serviceClient<navcon_msgs::EnableControl>("DEPTH_enable");
  vtcon = nh.serviceClient<navcon_msgs::EnableControl>("VT_enable");
  velcon = nh.serviceClient<navcon_msgs::ConfigureVelocityController>("ConfigureVelocityController");

  pointer_srv = nh.serviceClient<misc_msgs::PointerPrimitiveService>("commander/primitive/pointer");
  go2depth_srv = nh.serviceClient<misc_msgs::Go2depthService>("commander/go2depth");
  go2point_srv = nh.serviceClient<misc_msgs::Go2pointService>("commander/go2point");
  dp_srv = nh.serviceClient<misc_msgs::DynamicPositioningPrimitiveService>("commander/primitive/dynamic_positioning");
  stop_srv = nh.serviceClient<std_srvs::Trigger>("commander/stop_mission");
  pause_srv = nh.serviceClient<std_srvs::Trigger>("commander/pause_mission");
  continue_srv = nh.serviceClient<std_srvs::Trigger>("commander/continue_mission");

  //Initialze subscribers
  position_sub = nh.subscribe<auv_msgs::NavSts>("position", 1, &CaddyMissions::onPosition,this);
  surfacecmd_sub = nh.subscribe<std_msgs::UInt8>("surface_cmd", 1, &CaddyMissions::onSurfaceCmd,this);

  emergency_sub = nh.subscribe<std_msgs::Int32>("mission_controller/primitives/emergency", 1, &CaddyMissions::onEmergency,this);
  go_and_carry_sub = nh.subscribe<std_msgs::Int32>("mission_controller/primitives/go_and_carry", 1, &CaddyMissions::onGoAndCarry,this);
  guide_me_sub = nh.subscribe<std_msgs::Int32>("mission_controller/primitives/guide_me", 1, &CaddyMissions::onGuideMe,this);
  take_photo_sub = nh.subscribe<std_msgs::Int32>("mission_controller/primitives/take_photo", 1, &CaddyMissions::onTakePhoto, this);
  mosaic_sub = nh.subscribe<std_msgs::Int32>("mission_controller/primitives/mosaic", 1, &CaddyMissions::onMosaic, this);
  idle_sub = nh.subscribe<std_msgs::Int32>("mission_controller/primitives/idle", 1, &CaddyMissions::onIdle,this);

  vehicle_state_sub = nh.subscribe<auv_msgs::NavSts>("position", 1, &CaddyMissions::onVehicleState,this);

  //Initialize timer
  safety = nh.createTimer(ros::Duration(1.0),&CaddyMissions::onTimer, this, false, true);
}

void CaddyMissions::onSurfaceCmd(const std_msgs::UInt8::ConstPtr& cmd)
{
  /*  last_cmd = ros::Time::now();

  if (cmd->data == LAWN_MOWER)
  {
    if (testControllers())
    {
      ROS_INFO("Setup controller for lawn-mower mission.");
      navcon_msgs::ConfigureVelocityController srv;
      for(int i=0; i<6; ++i) srv.request.desired_mode[i] = DONT_CARE;
      srv.request.desired_mode[u] = VELCON;
      srv.request.desired_mode[v] = DISABLED;
      srv.request.desired_mode[w] = VELCON;
      srv.request.desired_mode[r] = VELCON;
      bool velconOK = velcon.call(srv);
      navcon_msgs::EnableControl flag;
      flag.request.enable = true;
      bool hdgconOK = hdgcon.call(flag);
      bool altconOK = altcon.call(flag);
      bool depthconOK = depthcon.call(flag);

      if (altconOK && hdgconOK && velconOK && depthconOK)
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
    for(int i=0; i<50; ++i)
    {
      lawnmower_pub.publish(flag);
      ros::Duration(0.01).sleep();
    }
  }*/
}

void CaddyMissions::onTakePhoto(const std_msgs::Int32::ConstPtr& data)
{
}

void CaddyMissions::onMosaic(const std_msgs::Int32::ConstPtr& data)
{
  if(data->data == 1)
  {
    this->stopControllers();
    // Start the primitive
    ROS_INFO("Setup controllers for Mosaicing mission.");
    navcon_msgs::ConfigureVelocityController srv;
    for(int i=0; i<6; ++i) srv.request.desired_mode[i] = DONT_CARE;
    srv.request.desired_mode[u] = VELCON;
    srv.request.desired_mode[v] = DISABLED;
    srv.request.desired_mode[w] = VELCON;
    srv.request.desired_mode[r] = VELCON;
    bool velconOK = velcon.call(srv);
    navcon_msgs::EnableControl flag;
    flag.request.enable = true;
    bool hdgconOK = hdgcon.call(flag);
    bool altconOK = altcon.call(flag);

    if (altconOK && hdgconOK && velconOK)
    {
      ROS_INFO("Mosaic controller setup complete.");
      mission_state = MOSAIC;
    }
    else
    {
      ROS_ERROR("Mosaic controller setup failed.");
      this->stopControllers();
    }
  }
}

void CaddyMissions::onEmergency(const std_msgs::Int32::ConstPtr& data)
{

}

void CaddyMissions::onGoAndCarry(const std_msgs::Int32::ConstPtr& data)
{
  if ((data->data == 1) && (mission_state != GO_AND_CARRY))
  {
    // Turn off primitives and controllers
    this->stopControllers();

    ROS_INFO("Setup controllers for go and carry mission.");
    navcon_msgs::ConfigureVelocityController srv;
    for(int i=0; i<6; ++i) srv.request.desired_mode[i] = DONT_CARE;
    srv.request.desired_mode[u] = VELCON;
    srv.request.desired_mode[v] = VELCON;
    srv.request.desired_mode[w] = VELCON;
    srv.request.desired_mode[r] = VELCON;
    bool velconOK = velcon.call(srv);
    navcon_msgs::EnableControl flag;
    flag.request.enable = true;
    bool hdgconOK = hdgcon.call(flag);
    bool depthconOK = depthcon.call(flag);
    bool dpconOK = dpcon.call(flag);

    if (hdgconOK && velconOK && depthconOK && dpconOK)
    {
      ROS_INFO("Go and carry controller setup complete.");
      mission_state = GO_AND_CARRY;
    }
    else
    {
      ROS_ERROR("Go and carry controller setup failed.");
      this->stopControllers();
    }
  }
}

void CaddyMissions::onGuideMe(const std_msgs::Int32::ConstPtr& data)
{
  if ((data->data == 1) && (mission_state != GUIDE_ME))
  {
    // Turn off primitives and controllers
    this->stopControllers();

    // Start the primitive
    ROS_INFO("Setup Pointer primitive.");
    misc_msgs::PointerPrimitiveService srv_data;
    srv_data.request.radius = pointer_radius;
    srv_data.request.radius_topic = "diver_distance";
    srv_data.request.vertical_offset = 0;
    srv_data.request.guidance_enable = false;
    srv_data.request.guidance_target.x = 0;
    srv_data.request.guidance_target.y = 0;
    srv_data.request.guidance_target.z = 0;
    srv_data.request.guidance_topic = "guide_target";
    srv_data.request.streamline_orientation = false;
    srv_data.request.wrapping_enable = false;

    pointer_srv.call(srv_data);

    ROS_INFO("Setup controllers for Guide-Me mission.");
    navcon_msgs::ConfigureVelocityController srv;
    for(int i=0; i<6; ++i) srv.request.desired_mode[i] = DONT_CARE;
    srv.request.desired_mode[u] = VELCON;
    srv.request.desired_mode[v] = VELCON;
    srv.request.desired_mode[w] = VELCON;
    srv.request.desired_mode[r] = VELCON;
    bool velconOK = velcon.call(srv);
    navcon_msgs::EnableControl flag;
    flag.request.enable = true;
    bool hdgconOK = hdgcon.call(flag);
    bool vtconOK = vtcon.call(flag);

    if (hdgconOK && velconOK && vtconOK)
    {
      ROS_INFO("Guide-Me controller setup complete.");
      mission_state = GUIDE_ME;
    }
    else
    {
      ROS_ERROR("Guide-Me controller setup failed.");
      this->stopControllers();
    }
  }
}

void CaddyMissions::onIdle(const std_msgs::Int32::ConstPtr& data)
{
  if(data->data == 1)
  {
    if(mission_state != IDLE)
    {
      std_srvs::Trigger srv_data;
      stop_srv.call(srv_data);

      mission_state = IDLE;
    }
  }
}

void CaddyMissions::onVehicleState(const auv_msgs::NavSts::ConstPtr& data)
{
  vehicle_state = *data;
}


void CaddyMissions::stopControllers()
{
  // Stop primitives
  std_srvs::Trigger srv_data;
  stop_srv.call(srv_data);
  mission_state = IDLE;
  // Stop controllers
  navcon_msgs::ConfigureVelocityController srv;
  for(int i=0; i<6; ++i) srv.request.desired_mode[i] = DISABLED;
  velcon.call(srv);
  navcon_msgs::EnableControl flag;
  flag.request.enable = false;
  dpcon.call(flag);
  hdgcon.call(flag);
  altcon.call(flag);
  depthcon.call(flag);
  vtcon.call(flag);
}

bool CaddyMissions::testControllers()
{
  bool retVal = velcon.exists() && hdgcon.exists() && altcon.exists() && depthcon.exists();
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
    std_msgs::Bool timeout;
    timeout.data = true;
    timeout_pub.publish(timeout);
    //this->stopControllers();
  }
}

bool CaddyMissions::checkNetwork()
{
  FILE* in;
  char buff[512];

  std::string command = "fping -c2 -t800 " + ipaddress + " 2>&1";
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
