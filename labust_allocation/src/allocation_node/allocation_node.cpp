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
 *********************************************************************/
#include <labust/allocation/allocation_node.h>
#include <labust/tools/conversions.hpp>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

using namespace labust::allocation;

AllocationNode::AllocationNode():
					timeout(0.5),
					allocation_mode(STANDARD),
					alloc_loader("labust_allocation",
							"labust::allocation::AllocationInterface")
{
	this->onInit();
}

AllocationNode::~AllocationNode(){}

void AllocationNode::onInit()
{
	ros::NodeHandle nh, ph("~");
	try
	{
		//Configure comms
		std::string allocplug("NONE");
		ph.param("allocation_plugin", allocplug, allocplug);
		alloc = alloc_loader.createInstance(allocplug);
		if ((alloc == 0) || !alloc->configure(nh, ph))
			throw std::runtime_error("AllocationNode: failed to configure the allocation class.");
		ROS_INFO("AllocationNode: Comms plugin: '%s' loaded", allocplug.c_str());
	}
	catch(pluginlib::PluginlibException& ex)
	{
		//handle the class failing to load
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}

	//Initialize subscribers and publishers
	tau_sub = nh.subscribe("tau_in", 1, &AllocationNode::onTau,this);
	mode_sub = nh.subscribe("allocation_mode", 1, &AllocationNode::onAllocationMode,this);
	tauach_pub = nh.advertise<auv_msgs::BodyForceRequest>("tau_ach",1);
	pwm_pub = nh.advertise<std_msgs::Float32MultiArray>("pwm_out",1);
}

void AllocationNode::onAllocationMode(const std_msgs::Int32::ConstPtr mode)
{
    if(mode->data == STANDARD || mode->data == THRUSTER_TEST)
    {
        allocation_mode = mode->data;
    }
}

void AllocationNode::onTau(const auv_msgs::BodyForceRequest::ConstPtr tau)
{

    if(allocation_mode == STANDARD)
    {
	//Reset timer

	//Load into vector
	Eigen::VectorXd tauv(AllocationInterface::N+1);
	labust::tools::pointToVector(tau->wrench.force, tauv);
	labust::tools::pointToVector(tau->wrench.torque, tauv, AllocationInterface::K);
	//Perform allocation
	const std::vector<double>& pwm = alloc->allocate(tauv);

	//Publish PWM data
	std_msgs::Float32MultiArray::Ptr pwmd(new std_msgs::Float32MultiArray());
	pwmd->data.assign(pwm.begin(),pwm.end());
	pwm_pub.publish(pwmd);

	//Publish the achieved tau
	auv_msgs::BodyForceRequest::Ptr tau_ach(new auv_msgs::BodyForceRequest());
	const Eigen::VectorXd& tauA = alloc->tauA();
	labust::tools::vectorToPoint(tauA, tau_ach->wrench.force);
	labust::tools::vectorToPoint(tauA, tau_ach->wrench.torque, AllocationInterface::K);
	const std::vector<bool>& wdp = alloc->windup();
	labust::tools::vectorToPoint(wdp, tau_ach->disable_axis);
	labust::tools::vectorToRPY(wdp, tau_ach->disable_axis, AllocationInterface::K);
	//Backward compatibility
	Eigen::VectorXd tsgn(tauA);
	for (int i=0; i<tsgn.size(); ++i)	tsgn(i)=((tsgn(i)>=0)?wdp[i]:-wdp[i]);
	labust::tools::vectorToPoint(tsgn, tau_ach->windup);
	labust::tools::vectorToRPY(tsgn, tau_ach->windup, AllocationInterface::K);
	tau_ach->header.stamp = ros::Time::now();
	tauach_pub.publish(tau_ach);
    } 
    else if(allocation_mode == THRUSTER_TEST)
    {

        ROS_WARN("allocation_node: THRUSTER_TEST mode active!");

    }
    else
    {
        ROS_FATAL("allocation_node: Wrong operation mode!");
        ROS_WARN("allocation_node: Switching to STANDARD allocation mode.");
        allocation_mode = STANDARD;
    }
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"allocation_node");

	labust::allocation::AllocationNode node;
	ros::spin();

	return 0;
}
