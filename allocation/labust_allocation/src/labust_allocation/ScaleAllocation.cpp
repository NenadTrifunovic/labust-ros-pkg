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
 *  Created: 06.03.2013.
 *********************************************************************/
#include <labust/allocation/ScaleAllocation.hpp>
#include <labust/allocation/ThrusterModels.hpp>
#include <labust/tools/conversions.hpp>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int16MultiArray.h>

#include <boost/bind.hpp>

using namespace labust::allocation;

ScaleAllocation::ScaleAllocation(){}

void ScaleAllocation::onInit()
{
	ros::NodeHandle nh = this->getNodeHandle();

	//Setup allocation
	alloc.configure(nh);

	//Initalize publishers
	tauAch = nh.advertise<auv_msgs::BodyForceReq>("tauAch",1);
	revsAch = nh.advertise<std_msgs::Int16MultiArray>("revs",1);

	//Initialize subscribers
	tauIn = nh.subscribe<auv_msgs::BodyForceReq>("tauOut", 1,
			boost::bind(&ScaleAllocation::onTauIn, this, _1));
}

void ScaleAllocation::onTauIn(const auv_msgs::BodyForceReq::ConstPtr tauIn)
{
	labust::simulation::vector tau;
	labust::tools::pointToVector(tauIn->wrench.force, tau);
	labust::tools::pointToVector(tauIn->wrench.torque, tau, 3);

	//Scaling allocation
	//Extract desired dofs
	Eigen::VectorXd vi(Eigen::VectorXd::Zero(alloc.dofs.size()));
	for (int i=0; i<alloc.dofs.size(); ++i) vi(i)=tau(alloc.dofs(i));

	//Calculate desired thrust for thrusters
	Eigen::VectorXd thrustAch = alloc.Binv*vi;
	Eigen::VectorXd satAch = labust::simulation::vector::Zero();

	//Find maximum scale
	double scale_max = 1;
	for (size_t i=0; i<thrustAch.rows();++i)
	{
		double scale = fabs((thrustAch(i)>0)?thrustAch(i)/alloc.tmax(i):thrustAch(i)/alloc.tmin(i));
		if (scale>scale_max) scale_max=scale;
	}
	//Normalize all to the maximum scaling
	thrustAch = thrustAch/scale_max;

	//Recover the full tau vector
	vi = alloc.B*thrustAch;
	tau = labust::simulation::vector::Zero();
	for (int i=0; i<alloc.dofs.size(); ++i) tau(alloc.dofs(i)) = vi(i);

	if (scale_max > 1) for (int i=0; i<alloc.dofs.size(); ++i) satAch(alloc.dofs(i)) = ((vi(i)>0)?1:-1);

	//Publish data
	auv_msgs::BodyForceReq::Ptr tauR(new auv_msgs::BodyForceReq());
	labust::tools::vectorToPoint(tau, tauR->wrench.force);
	labust::tools::vectorToPoint(tau, tauR->wrench.torque, 3);
	labust::tools::vectorToPoint(satAch, tauR->windup);
	labust::tools::vectorToRPY(satAch, tauR->windup, 3);
	labust::tools::vectorToDisableAxis(satAch, tauR->disable_axis);
	tauAch.publish(tauR);

	//Publish revolutions
	std_msgs::Int16MultiArray::Ptr revs(new std_msgs::Int16MultiArray());
	for (int i=0; i<thrustAch.size(); ++i) revs->data.push_back(alloc.pwmscaler*AffineModel::getRevs(thrustAch(i)));
	revsAch.publish(revs);
}

//Export the nodelet
PLUGINLIB_DECLARE_CLASS(allocation, Scaling, labust::allocation::ScaleAllocation, nodelet::Nodelet)
