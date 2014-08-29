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

#include <navcon_msgs/ControllerState.h>
#include <std_msgs/String.h>

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
	graphDesc = nh.advertise<std_msgs::String>("graph_desc",1);

	//Register available DOFs as resources
	///\todo Make this static const
	std::vector<std::string> dofs({"X","Y","Z","K","M","N"});
	std::vector<int> availableRes({0,1,5});
	for (int i=0; i < availableRes.size(); ++i)
		cgraph.addResource(dofs[availableRes[i]]);
}

bool ControllerManager::onControllerSelect(navcon_msgs::ControllerSelect::Request& req,
				navcon_msgs::ControllerSelect::Response& resp)
{
	for (int i=0; i<req.name.size(); ++i)
	{
		int type = 0;
		///\todo Check if controller exists
		///\todo Check if controller is already active
		///\todo Check if the controller is in the same mode
		///\todo Move checking and activation request processing to cgraph
		switch (req.state[i])
		{
		case req.EXTERNAL:
			//Activate the controller
		  type = ControllerGraph::ACTIVATE;
			break;
		case req.MANUAL:
			//Deactivate higher level dependencies
	  	type = ControllerGraph::FORCE;
			break;
		case req.TRACKING:
		case req.DISABLED:
	  	type = ControllerGraph::DEACTIVATE;
			break;
		default:
			ROS_INFO("Unprocessed request %d", req.state[i]);
			continue;
			break;
		}

		ControllerGraph::CASequencePtr activation = cgraph.get_firing_pn(req.name[i], type);
		navcon_msgs::ControllerState::Ptr out(new navcon_msgs::ControllerState());

		///\todo Make this a special case of a more general approach
		//Doing a single activation at a time
		//The activation and de-activation of controllers inside the current scheme
		for(int k=0; k<activation->size(); ++k)
		{
			std::string name = activation->at(k).first;
			bool activate = activation->at(k).second;
			int state = req.state[i];
			int type = state;

			if (name == req.name[i])
			{
				if (activate &&
						((state == req.DISABLED) ||
						(state == req.TRACKING)))
				{
					std::runtime_error("Conflict: PN wants to activate controller"
							" while the user requested deactivation.");
				}

				if (!activate &&
						((state == req.MANUAL) ||
						(state == req.EXTERNAL)))
				{
					std::runtime_error("Conflict: PN wants to deactivate controller"
							" while the user requested activation.");
				}
			}
			else
			{
				//No direct request
				if (activate)
					type = req.ACTIVATE;
				else
					type = req.DEACTIVATE;
			}

			out->name.push_back(name);
			out->info.push_back(navcon_msgs::ControllerInfo());
			out->info.rbegin()->state = type;

			if (!legacy.callService(name, type))
			{
				ROS_ERROR("Failed firing sequence.");
				///\todo Redo error/recover marking, etc.
			}
		}

		//Send the updates for this sequence
		controllerState.publish(out);
	}

	return true;
}

bool ControllerManager::onRegisterController(navcon_msgs::RegisterController_v3::Request& req,
		navcon_msgs::RegisterController_v3::Response& resp)
{
	resp.reply = cgraph.addToGraph(req);
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
