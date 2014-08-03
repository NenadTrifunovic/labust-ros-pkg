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
#include <labust/control/ControllerManager.hpp>
#include <navcon_msgs/ControllerState.h>

using namespace labust::control;

ControllerManager::ControllerManager()
{
	this->onInit();
};

void ControllerManager::onInit()
{
	ros::NodeHandle nh;
	//Setup services
	controllerSelect = nh.advertiseService("select_controller",
			&ControllerManager::onControllerSelect, this);
	registerController = nh.advertiseService("register_controller",
			&ControllerManager::onRegisterController, this);
	unregisterController = nh.advertiseService("unregister_controller",
			&ControllerManager::onUnRegisterController, this);

	//Setup publisher
	controllerState = nh.advertise<navcon_msgs::ControllerState>("controller_state",1);

	///\todo ADD AVAILABLE DOF RESOURCES TO CONTROLLER GRAPH
}

bool ControllerManager::onControllerSelect(navcon_msgs::ControllerSelect::Request& req,
				navcon_msgs::ControllerSelect::Response& resp)
{
	return true;
}

bool ControllerManager::onRegisterController(navcon_msgs::RegisterController_v3::Request& req,
		navcon_msgs::RegisterController_v3::Response& resp)
{
	///\todo Check that the controller already exists
	///\todo Check for un-met dependencies
	///\todo Dismiss controller registration if double or unmet dependencies
	///\todo Add the accepted controller to the Petri-Net and dependency graphs

	//Check if the controller with same name is already registered.
	///\todo Move validity checking into graph class
	if (cgraph.controllers.find(req.name) != cgraph.controllers.end())
	{
		ROS_WARN("Controller with name %s already exists.", req.name.c_str());
		resp.reply = navcon_msgs::RegisterController_v3::Response::ALREADY_REGISTERED;
		return true;
	}

	///\todo Move all dependency names into one string vector
	//Check if dependencies are satisfied
	for (int i=0; i<req.used_other.size(); ++i)
	{
		if (cgraph.placeMap.find(req.used_other[i]) == cgraph.placeMap.end())
		{
			ROS_ERROR("The controller %s is missing dependency %s.",
					req.name.c_str(),
					req.used_other[i].c_str());
			resp.reply = navcon_msgs::RegisterController_v3::Response::MISSING_DEPENDENCY;
			return true;
		}
	}

/*
	if (resp.unmet_cnt.size())
	{
		resp.accepted = false;
		return true;
	}

	//Add the controller
	ROS_INFO("Adding controller %s.",req.name.c_str());
	resp.accepted = true;
	controllers[req.name].info = req;
	depGraph.addToGraph(req);
	//pnGraph.addToGraph(req);
	//pnCon.addToPNGraph(req);

	ros::Time now = ros::Time::now();
	pnCon.addToGraph(req);
	double addgraph_dT = (ros::Time::now() - now).toSec();
	now = ros::Time::now();
	pnCon.addToPNGraph(req);
	double addpngraph_dT = (ros::Time::now() - now).toSec();
	//pnCon.reachability();
	double classic_dT = (ros::Time::now() - now).toSec();
	now = ros::Time::now();
	//pnCon.addToRGraph2(req.name);
	pnCon.addToRGraph(req.name);
	double incremental_dT = (ros::Time::now() - now).toSec();

	//pnCon.addToRGraph();
	//addToMatrix(req.name);
	names.push_back(req.name);

	std_msgs::String out;
	std::fstream dep_file("dep_graph.dot",std::ios::out);
	std::fstream pn_file("pn_graph.dot",std::ios::out);
	std::fstream r_file("r_graph.dot",std::ios::out);
	std::string temp;
	depGraph.getDotDesc(temp);
	dep_file<<temp;
	out.data = temp;
	depGraphPub.publish(out);
	pnCon.getDotDesc2(temp);
	pn_file<<temp;
	out.data = temp;
	pnGraphPub.publish(out);
	pnCon.getDotDesc(temp);
	r_file<<temp;

	std::ofstream prof_file("profile.csv", std::ios::app);
	prof_file<<addgraph_dT<<","<<addpngraph_dT<<","<<classic_dT<<","<<incremental_dT<<std::endl;

	return true;
*/
	cgraph.controllers[req.name] = req;
	return true;
}

bool ControllerManager::onUnRegisterController(navcon_msgs::RegisterController_v3::Request& req,
		navcon_msgs::RegisterController_v3::Response& resp)
{
	///\todo Check that the controller already exists
	///\todo Find all controllers that depend on this controller and remove them ?
	///\todo Remove the controller from the Petri-Net and dependency graphs
	return true;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"controller_manager");
	labust::control::ControllerManager exec;
	ros::spin();
	return 0;
}
