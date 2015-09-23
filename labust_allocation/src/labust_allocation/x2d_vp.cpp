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
#include <labust/allocation/x2d_adaptive.h>
#include <labust/math/NumberManipulation.hpp>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

#include <boost/array.hpp>
#include <Eigen/Dense>

#include <cmath>
#include <vector>

using namespace labust::allocation;

X2dAdaptive::X2dAdaptive():
		_windup(6,false),
		minN(1),
		daisy_chain(true),
		multi_chain(true),
		tau_ach(Eigen::VectorXd::Zero(6)){}

bool X2dAdaptive::configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
{
	//Load the thruster configuration
	bool valid = thrusters.configure(nh, ph);
	if (!valid) return false;
	//Read the minimum torque
	ph.param("min_torque", minN, minN);
	ph.param("daisy_chain", daisy_chain, daisy_chain);
	ph.param("multi_chain", multi_chain, multi_chain);
	Eigen::Vector3d tn(0,0,minN);
	tnmax = (thrusters.Binv()*tn).cwiseAbs();
	tnmin = -tnmax;

	return valid;
}

const std::vector<double>& X2dAdaptive::allocate(const Eigen::VectorXd& tau)
{
	//ROS_ERROR("Pint 1");
	//Get overall operational limits
	const Eigen::VectorXd& tmax(thrusters.maxF().segment(0,4));
	const Eigen::VectorXd& tmin(thrusters.minF().segment(0,4));
	//ROS_ERROR("Pint 2");
	//Get XY operational limits
	Eigen::Vector4d txymax(tmax - tnmax);
	Eigen::Vector4d txymin(tmin - tnmin);
	Eigen::Vector4d tnmax(this->tnmax);
	Eigen::Vector4d tnmin(this->tnmin);

	//Separate variables
	Eigen::Vector3d tauN, tauXY;
	tauN<<0.0,0.0,tau(N);
	tauXY<<tau(X),tau(Y),0.0;
	//Perform inverse allocation
	Eigen::Vector4d tN, tXY;
	tN = thrusters.Binv()*tauN;
	tXY = thrusters.Binv()*tauXY;

	std::ostringstream os;

	//ROS_ERROR("Pint 3");
	//Test for saturation
	bool satN(false),satXY(false);

	for (int i=0; i<tN.size(); ++i)
	{
		satN = satN || ((tN(i) > tnmax(i)) || (tN(i) < tnmin(i)));
	  satXY = satXY || ((tXY(i) > txymax(i)) || (tXY(i) < txymin(i)));
	}

	if (!satN && !satXY)
	{
	    //Everything is within limits
	    ROS_DEBUG("Everything allocated without saturation.");
	}
	else if (satN && !satXY)
	{
	    ROS_DEBUG("Yaw DoF is saturated.");
	    this->recalcOpLimits(tXY,txymax,txymin,&tnmax,&tnmin);
	    satN = saturate(tN, tnmin, tnmax);
	}
	else if (satXY && !satN)
	{
	    ROS_DEBUG("XY DoF are saturated.");
	    this->recalcOpLimits(tN,tnmax,tnmin,&txymax,&txymin);
	    satXY = saturate(tXY, txymin, txymax);
	    //os<<"tmin:"<<tmin<<std::endl;
	    //os<<"tmax:"<<tmax<<std::endl;
	    //os<<"tXY:"<<tXY<<std::endl;
	    //os<<"tN:"<<tN<<std::endl;
	}
	else if (satXY && satN)
	{
	    ROS_DEBUG("All DoFs are saturated.");
	    satN = saturate(tN, tnmin, tnmax);
	    satXY = saturate(tXY, txymin, txymax);
	}
	Eigen::VectorXd tT = tXY+tN;
	os<<"Final:"<<tT<<std::endl;
	Eigen::VectorXd tauA = thrusters.B()*tT;
	//ROS_ERROR("Stuff:%s",os.str().c_str());

	//Make a second run if nothing was achieved
	if (daisy_chain && (satXY || satN))
	{
		//ROS_ERROR("Second run.");
		Eigen::VectorXd tauS(3);
		tauS<<tau(X),tau(Y),tau(N);
		bool ssat = this->secondRun(tauS, tmax, tmin, &tauA, &tT);
		Eigen::VectorXd arem = (tauS - tauA).cwiseAbs();
		const double sm_th(0.001);
		satXY = (arem(X) > sm_th) || (arem(Y) > sm_th);
		satN = (arem(Z) > sm_th);
	}

	_windup[X] = _windup[Y] = satXY;
	_windup[N] = satN;

	//Copy to external vector
	enum {Xi=0,Yi=1,Ni=2};
	tau_ach(X) = tauA(Xi);
	tau_ach(Y) = tauA(Yi);
	tau_ach(N) = tauA(Ni);

	//The vertical patch
	const Eigen::VectorXd& tmaxv(thrusters.maxF().segment(4,2));
	const Eigen::VectorXd& tminv(thrusters.minF().segment(4,2));
	Eigen::Vector2d tTV;
	tTV<<tau(Z)/2,tau(Z)/2;
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
       	tau_ach(Z) = tTV.sum();

       	//ROS_ERROR("Pint 4");

       	Eigen::VectorXd tTT(6);
       	tTT<<tT,tTV;

	//ROS_ERROR("Pint 5");

	return thrusters.pwm(tTT);
}

void X2dAdaptive::recalcOpLimits(Eigen::Vector4d& used, Eigen::Vector4d& pmax,
		Eigen::Vector4d& pmin, Eigen::Vector4d* cmax, Eigen::Vector4d* cmin)
{
  for (int i=0; i<used.size(); ++i)
  {
      if (used(i) >= 0)
      {
          double delta = pmax(i) - used(i);
          (*cmax)(i) += delta;
          (*cmin)(i) += pmin(i);
      }
      else
      {
          double delta = pmin(i) - used(i);
          (*cmin)(i) += delta;
          (*cmax)(i) += pmax(i);
      }
  }
}

bool X2dAdaptive::saturateN(Eigen::Vector4d& t,
		const Eigen::Vector4d& pmin, const Eigen::Vector4d& pmax)
{
	bool retVal(false);
  for (int i=0; i<t.size(); ++i)
  {
  	double tn = labust::math::coerce(t(i), pmin(i), pmax(i));
  	if (tn != t(i)) retVal |= true;
  	t(i) = tn;
  }
  return retVal;
}

bool X2dAdaptive::saturate(Eigen::Vector4d& t,
		const Eigen::Vector4d& pmin, const Eigen::Vector4d& pmax)
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

bool X2dAdaptive::secondRun(const Eigen::VectorXd& tau,
		const Eigen::VectorXd& tmax, const Eigen::VectorXd& tmin,
		Eigen::VectorXd* tauA, Eigen::VectorXd* tT)
{
	bool retVal;
	bool do_loop(true);

	//Small floating point
	const double sm_th(0.1);

	while (do_loop)
	{
		Eigen::VectorXd tauR(tau - (*tauA));
		//If the request is almost satisfied
		if (tauR.norm() < sm_th)
		{
			ROS_DEBUG("Norm is smaller that threshold: %f", tauR.norm());
			retVal = false;
			break;
		}

		std::vector<double> tdmax,tdmin;
		std::vector<int> notsat;

		for(int i=0; i<tT->size(); ++i)
		{
			if ((*tT)(i) >= 0)
			{
				double cmax = tmax(i) - (*tT)(i);
				//If not saturated
				if (cmax > sm_th)
				{
					notsat.push_back(i);
					tdmax.push_back(cmax);
					tdmin.push_back(tmin(i));
				}
			}
			else
			{
				double cmin = tmin(i) - (*tT)(i);
				if (cmin < -sm_th)
				{
					notsat.push_back(i);
					tdmin.push_back(cmin);
					tdmax.push_back(tmax(i));
				}
			}
		}

		if (!multi_chain && (notsat.size() < (tT->size()-1)))
		{
			ROS_DEBUG("Less than %d are saturated. Daisy chain allocation complete.", int(tT->size()-1));
			retVal = true;
			break;
		}
		if (notsat.size() == 1) {retVal = true; break;};

		//Assemble the reduced allocation matrix
		Eigen::MatrixXd Bd(tau.size(), notsat.size());
		for (int i=0; i<notsat.size(); ++i)
		{
			Bd.col(i) = thrusters.B().col(notsat[i]);
		}

		Eigen::MatrixXd Bdinv;
		if (notsat.size() <= (tT->size()-2))
		{
			Bdinv = (Bd.transpose()*Bd).inverse()*Bd.transpose();
		}
		else
		{
			Bdinv = Bd.transpose()*(Bd*Bd.transpose()).inverse();
		}

		//Sanity check
		if (isnan(Bdinv.norm()))
		{
			ROS_ERROR("Singular inverse allocation matrix. Aborting daisy chain.");
			retVal = true;
			break;
		}

		Eigen::VectorXd tTd = Bdinv*tauR;

		//std::stringstream out;
		//out<<tTd;
		//ROS_ERROR("Td (%d): %s",tTd.size(),out.str().c_str());

		double scalef(1.0),scale(0.0);
    for (int i=0; i< tTd.size(); ++i)
    {
        if (tTd(i) >= 0)
            scale = tTd(i)/tdmax[i];
        else
            scale = tTd(i)/tdmin[i];

        if (scale > scalef) scalef=scale;
    }
    tTd = tTd/scalef;

    Eigen::VectorXd tTn(*tT);
    for (int i=0; i< notsat.size(); ++i)
    {
    	tTn(notsat[i]) += tTd(i);
    }
    Eigen::VectorXd tauf = thrusters.B()*(tTn);
    Eigen::VectorXd tauRf= tau - tauf;

    enum {Xi=0,Yi=1,Ni=2};
    //Test if we gained a monotonous increase (scaling will be perserved then)
    //ROS_ERROR("X: %f, %f", fabs(tauR(Xi)), fabs(tauRf(Xi)));
    //ROS_ERROR("Y: %f, %f", fabs(tauR(Yi)), fabs(tauRf(Yi)));
    //ROS_ERROR("N: %f, %f", fabs(tauR(Ni)), fabs(tauRf(Ni)));
    bool limit = fabs(tauR(Xi)) - fabs(tauRf(Xi)) < -sm_th;
    limit = limit || (fabs(tauR(Yi)) - fabs(tauRf(Yi)) < -sm_th);
		limit = limit || (fabs(tauR(Ni)) - fabs(tauRf(Ni)) < -sm_th);

    if (limit)
    {
    	ROS_DEBUG("Scaling or yaw contract broken. Skipping this contribution.");
    	retVal = true;
    	break;
    }

    if ((tTd.norm() < sm_th))
    {
    	ROS_DEBUG("The daisy chain allocation is not contributing any change.");
    	retVal = true;
    	break;
    }

    (*tT) = tTn;
    (*tauA) = tauf;
	}

	return retVal;
}

PLUGINLIB_EXPORT_CLASS(labust::allocation::X2dAdaptive, labust::allocation::AllocationInterface)
