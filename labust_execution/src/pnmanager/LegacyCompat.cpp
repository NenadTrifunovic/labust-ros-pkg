/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
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
 *
 *  Author: Dula Nad
 *  Created: 30.10.2013.
 *********************************************************************/
#include <labust/control/LegacyCompat.hpp>
#include <navcon_msgs/ConfigureVelocityController.h>
#include <navcon_msgs/EnableControl.h>
#include <navcon_msgs/RegisterController_v3.h>
#include <ros/ros.h>

using namespace labust::control;

LegacyCompat::LegacyCompat()
{
	//Configure identfications and low-level
	std::string ident_names[]={"ident_X","ident_Y","ident_Z","ident_K",
			"ident_M", "ident_N"};
	std::string names[]={"surge","sway","heave","roll_rate",
			"pitch_rate", "yaw_rate"};
	std::string baseResources[] = {"X","Y","Z","K","M","N"};

	int idx[] = {0,1,2,3,4,5};

	for (int i=0; i<6; ++i)
	{
		lowLevel.insert(std::make_pair(names[i], idx[i]));
		ident.insert(std::make_pair(ident_names[i], idx[i]));
		std::vector<std::string> temp(1,baseResources[i]);
		dependencies.push_back(DepPair(names[i],temp));
		dependencies.push_back(DepPair(ident_names[i],temp));
	}

	//Configure high-level
	enum{numHL = 6};
	std::string hlnames[]={"fadp","ualf","falf","heading","depth","altitude"};
	std::string hlservice[]={"FADP_enable","UALF_enable","FALF_enable",
			"HDG_enable","DEPTH_enable","ALT_enable"};
	std::vector<std::string> hldeps[]={{"surge", "sway"}, {"yaw_rate", "surge"},
			{"surge", "sway"},{"yaw_rate"},{"heave"},{"heave"}};

	for (int i=0; i<numHL; ++i)
	{
		hlLevel.insert(std::make_pair(hlnames[i],hlservice[i]));
		dependencies.push_back(DepPair(hlnames[i],hldeps[i]));
	}

	this->onInit();
};

bool LegacyCompat::callService(const std::string& name, int state)
{
	navcon_msgs::ConfigureVelocityController srv;
	srv.request.desired_mode.assign(-1);
	LLMap::const_iterator it;
	HLMap::const_iterator itH;
	ros::NodeHandle nh;

	bool success = true;
	if ((it = lowLevel.find(name)) != lowLevel.end())
	{
		//Only 0-DISABLE, 1-MANUAL, 2-EXTERNAL are supported
		if (state > 2) state = 0;
		srv.request.desired_mode[it->second] = state;
	}
	else if ((it = ident.find(name)) != ident.end())
	{
		srv.request.desired_mode[it->second] = 3;
	}
	else if ((itH = hlLevel.find(name)) != hlLevel.end())
	{
		if (state == 1)
		{
			ROS_ERROR("Manual speed control is not implemented in legacy.");
			state = 0;
		}

		//Call service
		ros::ServiceClient client =
				nh.serviceClient<navcon_msgs::EnableControl>(itH->second);
		navcon_msgs::EnableControl req;
		req.request.enable = state > 0;
		if ((success = client.call(req)))
		{
			ROS_ERROR("Failed service call to: %s",itH->second.c_str());
		}

		return success;
	}

	//Configure the VelocityController
	ros::ServiceClient client =
			nh.serviceClient<navcon_msgs::ConfigureVelocityController>(
					"ConfigureVelocityController");
	if ((success = client.call(srv)))
	{
		ROS_ERROR("Failed service call to: ConfigureVelocityController");
	}

	return success;
}

void LegacyCompat::registerAll(boost::function<bool(navcon_msgs::RegisterController_v3::Request&,
		navcon_msgs::RegisterController_v3::Response&)> callback)
{
	ros::NodeHandle nh;
	//Call service
	ros::ServiceClient client =
			nh.serviceClient<navcon_msgs::RegisterController_v3>("register_controller", true);

	for(DepMap::const_iterator it=dependencies.begin();
			it != dependencies.end();
			++it)
	{
		navcon_msgs::RegisterController_v3 reg;
		reg.request.name = it->first;
		reg.request.used_resources.assign(it->second.begin(), it->second.end());

		std::cout<<"Registering "<<it->first<<std::endl;

		while (!callback(reg.request, reg.response))
		{
			std::cout<<"Unable to register controller."<<std::endl;
			ros::Duration(1.0).sleep();
		}
	}
}


