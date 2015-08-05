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
 *  FOR A PARTICULAR PURPOSE ARROS paketiE DISCLAIMED. IN NO EVENT SHALL THE
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
#include <labust/allocation/thruster_configuration.h>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/math/NumberManipulation.hpp>

using namespace labust::allocation;

ThrusterConfiguration::ThrusterConfiguration():
	Us(1),
	Un(1),
	adapt(true){}

bool ThrusterConfiguration::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Get the supply voltage
	ph.param("Us",Us,Us);
	ph.param("Un",Un,Un);
	ph.param("adapt_pwm",adapt,adapt);

	//Read thruster gains
	std::vector<double> Kp,Kn;
	ph.param("K", Kp, Kp);
	ph.param("Kp", Kp, Kp);
	ph.param("Kn", Kn, Kp);
	//Read the allocation matrix
	labust::tools::getMatrixParam(ph,"B", _B);

	//Check validity
	int sz = Kp.size();
	bool valid = (sz != 0);
	valid = valid && (sz == Kn.size());
	valid = valid && (sz == _B.cols());

	if (!valid)
	{
		ROS_ERROR("ThrusterConfigration: Configuration is not valid. Check all vector sizes.");
		return false;
	}

	//Calculate the inverse
	_Binv = _B.transpose()*(_B*_B.transpose()).inverse();

	//Set mapping defaults
	std::vector<double> pwm_max(sz,1.0), pwm_direction(sz,1);
	std::vector<int> mapping(sz);
	//Add +1 to start calculating from 1
	for (int i=0; i<mapping.size(); ++i) mapping[i]=i+1;

	//Read mapping values
	ph.param("pwm_mapping", mapping, mapping);
	ph.param("pwm_direction", pwm_direction, pwm_direction);
	ph.param("pwm_max", pwm_max, pwm_max);
	std::vector<double> pwm_min;
	for(int i=0; i<pwm_max.size(); ++i) pwm_min.push_back(-pwm_max[i]);
	ph.param("pwm_min", pwm_min, pwm_min);

	//Check validity
	valid = (sz == mapping.size());
	valid = valid && (sz == pwm_direction.size());
	valid = valid && (sz == pwm_max.size());
	valid = valid && (sz == pwm_min.size());
	if (!valid)
	{
		ROS_ERROR("ThrusterConfigration: Configuration is not valid. Check all vector sizes.");
		return false;
	}
	//Sanity check on minimum/maximum
	for (int i=0; i<pwm_max.size(); ++i)
	{
		if (pwm_max[i] < pwm_min[i])
		{
			ROS_ERROR("Minimum PWM is larger than maximum PWM.");
			return false;
		}
	}

	for(int i=0; i<sz; ++i)
	{
		//Substract -1 to start calculating from 0
		thrusters.push_back(EThruster(
				Kp[i], Kn[i], mapping[i]-1,
				pwm_max[i], pwm_min[i], pwm_direction[i]));
	}

	//Setup outputs
	pwm_out.resize(sz,0.0);
	F_a.resize(sz);
	F_max.resize(sz);
	F_min.resize(sz);
	//Init maximum/minimum to default
	this->updateMinMax();

	//Setup subscribers
	voltage_sub = nh.subscribe("supply_voltage",1,
			&ThrusterConfiguration::onSupplyVoltage, this);

	return true;
}

const std::vector<double>& ThrusterConfiguration::pwm(const Eigen::VectorXd& F)
{
	bool nofail(false);
	//Sanity check
	if ((nofail = (F.size() == thrusters.size())))
	{
		for(int i=0; i<thrusters.size(); ++i)
		{
			EThruster& t(thrusters[i]);
			//Calculate
			double pwmi = t.pwm(F(i),Us);
			//Limit and correct direction
			pwmi = labust::math::coerce(pwmi, t.pwm_min, t.pwm_max);
			//Assign the achieved force
			F_a(i) = t.F(pwmi, Us);
			//Assign the pwm output
			pwm_out[t.pwm_id] = t.pwm_dir * pwmi;
			//Sanity check
			if (isnan(pwm_out[t.pwm_id]))
			{
				nofail = false;
				break;
			}
		}
	}

	if (!nofail)
	{
		//Zero all
		for(int i=0; i<pwm_out.size(); ++i)
		{
			F_a(i) = 0;
			pwm_out[i]=0;
		}
	}

	T_a = _B*F_a;

	return pwm_out;
}

void ThrusterConfiguration::updateMinMax()
{
	//Update the new maximums
	for(int i=0; i<thrusters.size(); ++i)
	{
		EThruster& t(thrusters[i]);
		if (adapt)
		{
			//Adapt to achieved nominal voltage
			t.pwm_max = labust::math::coerce(Un/Us,-1.0,1.0);
			t.pwm_min = labust::math::coerce(-Un/Us,-1.0,1.0);
		}
		F_max[i] = t.F(t.pwm_max,Us);
		F_min[i] = t.F(t.pwm_min,Us);
	};
}
