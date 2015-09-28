/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2015, LABUST, UNIZG-FER
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
#include <labust/allocation/x2d_vp_qp.h>
#include <labust/math/NumberManipulation.hpp>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

#include <boost/array.hpp>
#include <Eigen/Dense>

#include <cmath>
#include <vector>

using namespace labust::allocation;

X2dVPQP::X2dVPQP():
				qp(CGAL::EQUAL, true, -1.0, true, 1.0),
				_windup(6,false),
				minN(0.2),
				tau_ach(Eigen::VectorXd::Zero(6)){}

bool X2dVPQP::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Load the thruster configuration
	bool valid = thrusters.configure(nh, ph);
	if (!valid) return false;
	//Read the minimum torque
	ph.param("min_torque", minN, minN);

	//Setup quadratic program (A=B)
	//\todo WARNING- ASSUMED A 4x3 MATRIX HERE
	for (int i=0; i<thrusters.B().rows(); ++i)
		for (int j=0; j<thrusters.B().cols(); ++j)
			qp.set_a(j, i,  thrusters.B()(i,j));

	//Setup 2*D matrix; D=I
	for(int i=0; i<thrusters.B().cols(); ++i) qp.set_d(i,i,1.0);

	return valid;
}

const std::vector<double>& X2dVPQP::allocate(const Eigen::VectorXd& tau)
{
	//Update the limits
	const Eigen::Vector4d& fmax_h(thrusters.maxF().segment(0,4));
	const Eigen::Vector4d& fmin_h(thrusters.minF().segment(0,4));
	xconf.updateLimits(fmin_h, fmax_h);
	//Coerce the desired into a feasible solution
	Eigen::Vector3d taud, tauh;
	taud<<tau(X), tau(Y), tau(N);
	xconf.coerce2poly(taud, minN, tauh);
	//Setup quadratic program limits
	for (int i=0; i<fmax_h.size(); ++i)
	{
		qp.set_u(i, true, fmax_h(i));
		qp.set_l(i, true, fmin_h(i));
	}
	//Setup equality condition
	for (int i=0; i<tauh.size(); ++i) qp.set_b(i, tauh(i));
	//Solve quadratic program
	Eigen::Vector4d tT(Eigen::Vector4d::Zero());
	Solution s = CGAL::solve_quadratic_program(qp, ET());
	if (!s.is_infeasible())
	{
		Solution::Variable_value_iterator fs = s.variable_values_begin();
		int i = 0;
		while (fs != s.variable_values_end())	tT(i++) = CGAL::to_double(*fs++);

		if (saturate(tT, fmin_h, fmax_h))
		{
			ROS_ERROR("QuadProg: Calculated thrust vector is unachivable.");
		}
	}
	else
	{
		ROS_ERROR("Infeasible solution.");
	}
	//Calculate achieved tau
	Eigen::Vector3d tauhf = thrusters.B()*tT;
	//Test discrepancy
	double ep = 0.001;
	if ((tauh - tauhf).cwiseAbs().maxCoeff() > ep)
	{
		ROS_ERROR("Contract breached. Expected: (%f, %f, %f) "
							"and got: (%f, %f, %f)",
							tauh(Xh) , tauh(Yh) , tauh(Nh),
							tauhf(Xh), tauhf(Yh), tauhf(Nh));
	}
	//Primitive windup detection
	Eigen::Vector3d arem = (taud - tauhf).cwiseAbs();
	const double sm_th(0.001);
	_windup[X] = _windup[Y] = (arem(Xh) > sm_th) || (arem(Yh) > sm_th);
	_windup[N] = (arem(Nh) > sm_th);
	//Copy to external vector
	tau_ach(X) = tauhf(Xh);
	tau_ach(Y) = tauhf(Yh);
	tau_ach(N) = tauhf(Nh);

	//Separate vertical handling
	Eigen::Matrix2d Bv;
	double l5(0.25),l6(0.55);
	Bv<<1.0,1.0,
			-l5,l6;
	Eigen::Vector2d tauv;
	tauv<<tau(Z), tau(M);
	const Eigen::VectorXd& tmaxv(thrusters.maxF().segment(4,2));
	const Eigen::VectorXd& tminv(thrusters.minF().segment(4,2));
	Eigen::Vector2d tTV = Bv.inverse()*tauv;
	double scale=1.0;
	double scalef=1.0;
	for (int i=0;i<tTV.size(); ++i)
	{
		if (tTV(i) >= 0)
			scale = tTV(i)/tmaxv(i);
		else
			scale = tTV(i)/tminv(i);

		if (scale > scalef) scalef=scale;
	}
	tTV = tTV/scalef;
	_windup[Z] = scalef > 1.0;
	_windup[M] = scalef > 1.0;
	tauv = Bv*tTV;
	//Export final forces
	tau_ach[Z] = tauv(0);
	tau_ach[M] = tauv(1);

	//Assemble final thrust vector
	Eigen::Matrix<double, 6,1> tTT(6);
	tTT<<tT,tTV;

	return thrusters.pwm(tTT);
}

template <class TypeV, class TypeMinMax>
bool X2dVPQP::saturate(TypeV& t, const TypeMinMax& pmin, const TypeMinMax& pmax)
{
	double scalef(1.0), scale(0.0);
	for (int i=0;i<t.size(); ++i)
	{
		if (t(i) >= 0)
			scale = t(i)/pmax(i);
		else
			scale = t(i)/pmin(i);

		if (scale > scalef) scalef=scale;
	}
	t = t/scalef;

	return (scalef > 1.0);
}


PLUGINLIB_EXPORT_CLASS(labust::allocation::X2dVPQP, labust::allocation::AllocationInterface)
