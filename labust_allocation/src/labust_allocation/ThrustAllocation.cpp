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
#include <labust/allocation/ThrustAllocation.hpp>
#include <labust/allocation/ThrusterModels.hpp>
#include <labust/tools/MatrixLoader.hpp>

#include <cmath>

using namespace labust::allocation;

ThrustAllocation::ThrustAllocation():
  dofs(2),
  B(2,2),
  W(Eigen::Matrix2d::Identity()),
  tmax(2),
  tmin(2),
  pwmscaler(255)
{
	dofs<<0,5;
	B<<1,1,1,-1;
	Binv = B.inverse();
	tmax<<1,1;
	tmin = -tmax;
};

void ThrustAllocation::configure(ros::NodeHandle& nh)
{
	//Populate allocation
  labust::tools::getMatrixParam(nh,"allocation_matrix",B);
  Binv = B.transpose()*(B*B.transpose()).inverse();
	labust::tools::getMatrixParam(nh,"allocation_dofs",dofs);

	assert((B.cols() == tmax.size()) &&
					"Allocation matrix must have number of columns equal to number of thrusters.");
	assert((B.rows() == dofs.size()) &&
					"Allocation matrix must have the same number of rows as controllable DOFs.");

	if (nh.hasParam("thrust_max"))
	{
		labust::tools::getMatrixParam(nh,"thrust_max",tmax);
	}
	else
	{
		tmax = Eigen::VectorXd::Ones(B.cols());
	}

	if (nh.hasParam("thrust_min"))
	{
		labust::tools::getMatrixParam(nh,"thrust_min",tmin);
	}
	else
	{
		tmin = -tmax;
	}

	nh.param("pwmscaler",pwmscaler,pwmscaler);
}
