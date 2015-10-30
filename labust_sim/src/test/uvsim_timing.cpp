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
 *  Created: 01.02.2013.
 *********************************************************************/
#include <labust/simulation/RBModel.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <ros/ros.h>

using namespace labust::simulation;

void configureModel(const ros::NodeHandle& nh, RBModel& model)
{
	labust::tools::loadDynamicsParams(nh, model);

	nh.param("sampling_time",model.dT, model.dT);
	nh.param("coupled",model.isCoupled,model.isCoupled);
	Eigen::Vector3d bdg;
	labust::tools::getMatrixParam(nh,"bounding_ellipsoid",bdg);
	model.ae = bdg(0);
	model.be = bdg(1);
	model.ce = bdg(2);
	labust::tools::getMatrixParam(nh,"eta0",model.eta0);
	labust::tools::getMatrixParam(nh,"nu0",model.nu0);
	labust::tools::getMatrixParam(nh,"current",model.current);

	typedef Eigen::Matrix<double,6,1> NoiseVec;
	NoiseVec pn(NoiseVec::Zero()),mn(NoiseVec::Zero());
	labust::tools::getMatrixParam(nh,"process_noise",pn);
	labust::tools::getMatrixParam(nh,"measurement_noise",mn);
	model.noise.setNoiseParams(pn,mn);

	//Populate allocation
	int alloc_type(-1);
	Eigen::MatrixXd alloc;
	Eigen::MatrixXi dofs, groups;
	nh.param("allocation_type",alloc_type,-1);
    labust::tools::getMatrixParam(nh,"allocation_matrix",alloc);
	labust::tools::getMatrixParam(nh,"allocation_dofs",dofs);
	labust::tools::getMatrixParam(nh,"allocation_thruster_groups",groups);

	model.allocator.init(alloc_type, alloc, dofs, groups);
	model.init();
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "uvsim_timing");
	ros::NodeHandle nh;
	RBModel model;
	configureModel(nh, model);
	model.dT = 0.1;
	model.isCoupled = true;
	std::cout<<model.Ma<<std::endl<<std::endl<<model.Mrb<<std::endl;
	//return 1;
	ros::Time last(ros::Time::now());
	double max(0);
	double min(1.0);
	const int maxit=100000000;
	int it(0);
	double sum(0);
	while (ros::ok && it < maxit)
	{
		++it;
		vector tau(vector::Zero());
		model.step(tau);
		double dt((ros::Time::now() - last).toSec());
		if (dt > max) max = dt;
		if (dt < min) min = dt;
		last = ros::Time::now();
		sum += dt;
	}

	ROS_INFO("Min-Max-Average time: min=%f, max=%f, avg=%f, sum=%f", min, max, sum/it, sum);

	return 0;
}
