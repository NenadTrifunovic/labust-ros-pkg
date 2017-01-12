/*********************************************************************
 *  TwoVehicleLocalizationModel.cpp
 *
 *  Created on: Sep 22, 2015
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015-2016, LABUST, UNIZG-FER
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
#include <labust/navigation/TwoVehicleLocalizationModel.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/math/Signum.hpp>



using namespace labust::navigation;

#include <vector>

#include <ros/ros.h>

TwoVehicleLocalizationModel::TwoVehicleLocalizationModel():
		dvlModel(1),
		xdot(0),
		ydot(0),
		bearing_wrap_index(0){

	this->initModel();
};

TwoVehicleLocalizationModel::~TwoVehicleLocalizationModel(){};

void TwoVehicleLocalizationModel::initModel()
{
  x = vector::Zero(stateNum);
  xdot = 0;
  ydot = 0;
  /*** Setup the transition matrix ***/
  derivativeAW();
  R0 = R;
  V0 = V;
  //std::cout<<"R:"<<R<<"\n"<<V<<std::endl;
}

void TwoVehicleLocalizationModel::calculateXYInovationVariance(const TwoVehicleLocalizationModel::matrix& P, double& xin,double &yin)
{
	xin = sqrt(P(xp,xp)) + sqrt(R0(xp,xp));
	yin = sqrt(P(yp,yp)) + sqrt(R0(yp,yp));
}


void TwoVehicleLocalizationModel::calculateUVInovationVariance(const TwoVehicleLocalizationModel::matrix& P, double& uin,double &vin)
{
	uin = sqrt(P(u,u)) + sqrt(R0(u,u));
	//vin = sqrt(P(v,v)) + sqrt(R0(v,v));
}

void TwoVehicleLocalizationModel::step(const input_type& input)
{
  xdot = x(u)*cos(x(psi));
  ydot = x(u)*sin(x(psi));
  x(xp) += Ts * xdot;
  x(yp) += Ts * ydot;
  x(zp) += Ts * x(w);
  x(psi) += 0;

  x(hdg) += Ts*x(r);

  x(u) += 0;
  x(w) += 0;
  x(r) += 0;

  /*** Target ***/
  xdot = x(ub)*cos(x(psib));
  ydot = x(ub)*sin(x(psib));
  x(xb) += Ts * xdot;
  x(yb) += Ts * ydot;

  /*** Limit target depth ***/
  //x(zb) += Ts * x(wb);
  x(zb) += 0;
  if(x(zb)<0.0)
	x(zb)=0.0;

  /*** Limit target course ***/
  //x(psib) += Ts * x(rb);
  x(psib) += 0;


  /*** Limit target surge speed ***/
  x(ub) += 0;
  if(x(ub)>0.4)
	x(ub)=0.4;
  if(x(ub)<-0.4)
	x(ub)=-0.4;

  /*** ***/
  x(hdgb) += 0;

  /*** Limit target heave speed ***/
  //x(wb) = 0;

  /*** Limit target yaw speed ***/
  //x(rb) = 0;

  xk_1 = x;

  derivativeAW();
}

void TwoVehicleLocalizationModel::derivativeAW()
{
	A = matrix::Identity(stateNum, stateNum);

	A(xp,u) = Ts*cos(x(psi));
	A(xp,psi) = Ts*(-x(u)*sin(x(psi)));

	A(yp,u) = Ts*sin(x(psi));
	A(yp,psi) = Ts*(x(u)*cos(x(psi)));

	A(zp,w) = Ts;
	//A(psi,r) = Ts;
	A(hdg,r) = Ts;


	A(xb,ub) = Ts*cos(x(psib));
	A(xb,psib) = Ts*(-x(ub)*sin(x(psib)));

	A(yb,ub) = Ts*sin(x(psib));
	A(yb,psib) = Ts*(x(ub)*cos(x(psib)));

	//A(zb,wb) = Ts;
	//A(psib,rb) = Ts;
}

const TwoVehicleLocalizationModel::output_type& TwoVehicleLocalizationModel::update(vector& measurements, vector& newMeas)
{

	//if (dvlModel != 0)
	derivativeH();

	std::vector<size_t> arrived;
	std::vector<double> dataVec;

	for (size_t i=0; i<newMeas.size(); ++i)
	{
		if (newMeas(i) && !std::isnan(measurements(i)))
		{
			arrived.push_back(i);
			dataVec.push_back(measurements(i));
			newMeas(i) = 0;
		} else if(std::isnan(measurements(i)))
		{
			newMeas(i) = 0;
			ROS_ERROR("NaN measurement arrived.");
		}
	}






	measurement.resize(arrived.size());
	H = matrix::Zero(arrived.size(),stateNum);
	y = vector::Zero(arrived.size());
	R = matrix::Zero(arrived.size(),arrived.size());
	V = matrix::Zero(arrived.size(),arrived.size());

	for (size_t i=0; i<arrived.size();++i)
	{

		/*** Check for angle wrapping ***/
		if(arrived[i] == bearing && std::abs(dataVec.at(i)-ynl(arrived[i]))>M_PI)
		{
			ROS_ERROR("DEBUG BEARING RAZLIKA %f - %f = %f",dataVec.at(i),ynl(arrived[i]),dataVec.at(i)-ynl(arrived[i]));

			double Delta = 2*M_PI - std::abs(dataVec.at(i)) - std::abs(ynl(arrived[i]));
			dataVec.at(i) = 0;
			ynl(arrived[i]) = -1*labust::math::sgn<double>(ynl(arrived[i]))*Delta;

			ROS_ERROR("DELTA: %f, ynl: %f", Delta, ynl(arrived[i]));
		}

		measurement(i) = dataVec[i];

//		if (dvlModel != 0)
//		{
			H.row(i)=Hnl.row(arrived[i]);
			y(i) = ynl(arrived[i]);
//		}
//		else
//		{
//			H(i,arrived[i]) = 1;
//			y(i) = x(arrived[i]);
//		}


		for (size_t j=0; j<arrived.size(); ++j)
		{
			R(i,j)=R0(arrived[i],arrived[j]);
			V(i,j)=V0(arrived[i],arrived[j]);
		}
	}

	/*** Debug Print
	std::cout<<"Setup H:"<<H<<std::endl;
	std::cout<<"Setup R:"<<R<<std::endl;
	std::cout<<"Setup V:"<<V<<std::endl;
	***/
	return measurement;
}

void TwoVehicleLocalizationModel::estimate_y(output_type& y)
{
	y=this->y;
}

void TwoVehicleLocalizationModel::derivativeH()
{
	Hnl = matrix::Zero(measSize,stateNum);
	Hnl.topLeftCorner(stateNum,stateNum) = matrix::Identity(stateNum,stateNum);

	ynl = vector::Zero(measSize);
	ynl.head(stateNum) = matrix::Identity(stateNum,stateNum)*x;

	double rng  = sqrt(pow((x(xb)-x(xp)),2)+pow((x(yb)-x(yp)),2)+pow((x(zb)-x(zp)),2));
	double rng_h  = sqrt(pow((x(xb)-x(xp)),2)+pow((x(yb)-x(yp)),2));
	double delta_x = (x(xb)-x(xp));
	double delta_y = (x(yb)-x(yp));
	double delta_z = (x(zb)-x(zp));

	double eps = 1.0e-25;

    if(rng_h<eps)
    {
    	rng_h = eps;

    	if(std::abs(delta_x)<eps)
    	{
    		delta_x = delta_x<0?-0.5*eps:0.5*eps;
    	}

    	if(std::abs(delta_y)<eps)
    	{
    		delta_y = delta_y<0?-0.5*eps:0.5*eps;
    	}
    }

    if(rng<eps)
    {
    	rng = eps;

    	if(std::abs(delta_z)<eps)
    	{
    		delta_z = delta_z<0?-eps:eps;
    	}
    }

	ynl(range) = rng_h;
	ynl(bearing) = atan2(delta_y,delta_x); /*** Absolute bearing ***/
	//ynl(bearing) = labust::math::wrapRad(atan2(delta_y,delta_x) -1*x(hdg)); /*** Relative bearing ***/
	//ynl(elevation) = asin((x(zp)-x(zb))/rng);

	Hnl(range, xp)  = -(delta_x)/rng_h;
	Hnl(range, yp)  = -(delta_y)/rng_h;
	//Hnl(range, zp)  = -(delta_z)/rng;

	Hnl(range, xb)  = (delta_x)/rng_h;
	Hnl(range, yb)  = (delta_y)/rng_h;
	//Hnl(range, zb)  = (delta_z)/rng;

	Hnl(bearing, xp) = delta_y/(delta_x*delta_x+delta_y*delta_y);
	Hnl(bearing, yp) = -delta_x/(delta_x*delta_x+delta_y*delta_y);
	Hnl(bearing, xb) = -delta_y/(delta_x*delta_x+delta_y*delta_y);
	Hnl(bearing, yb) = delta_x/(delta_x*delta_x+delta_y*delta_y);

	//Hnl(bearing, hdg) = -1;

	ynl(sonar_range) = rng_h;
	ynl(sonar_bearing) = labust::math::wrapRad(atan2(delta_y,delta_x) -1*x(hdg));


	Hnl(sonar_range, xp)  = -(delta_x)/rng_h;
	Hnl(sonar_range, yp)  = -(delta_y)/rng_h;
	//Hnl(sonar_range, zp)  = -(delta_z)/rng;

	Hnl(sonar_range, xb)  = (delta_x)/rng_h;
	Hnl(sonar_range, yb)  = (delta_y)/rng_h;
	//Hnl(sonar_range, zb)  = (delta_z)/rng;

	Hnl(sonar_bearing, xp) = delta_y/(delta_x*delta_x+delta_y*delta_y);
	Hnl(sonar_bearing, yp) = -delta_x/(delta_x*delta_x+delta_y*delta_y);
	Hnl(sonar_bearing, xb) = -delta_y/(delta_x*delta_x+delta_y*delta_y);
	Hnl(sonar_bearing, yb) = delta_x/(delta_x*delta_x+delta_y*delta_y);

	Hnl(sonar_bearing, hdg) = -1;
	
    ynl(camera_range) = rng_h;
	ynl(camera_bearing) = labust::math::wrapRad(atan2(delta_y,delta_x) -1*x(hdg));
    ynl(camera_hdgb) = x(hdgb);

	Hnl(camera_range, xp)  = -(delta_x)/rng_h;
	Hnl(camera_range, yp)  = -(delta_y)/rng_h;
	//Hnl(camera_range, zp)  = -(delta_z)/rng;

	Hnl(camera_range, xb)  = (delta_x)/rng_h;
	Hnl(camera_range, yb)  = (delta_y)/rng_h;
	//Hnl(camera_range, zb)  = (delta_z)/rng;

	Hnl(camera_bearing, xp) = delta_y/(delta_x*delta_x+delta_y*delta_y);
	Hnl(camera_bearing, yp) = -delta_x/(delta_x*delta_x+delta_y*delta_y);
	Hnl(camera_bearing, xb) = -delta_y/(delta_x*delta_x+delta_y*delta_y);
	Hnl(camera_bearing, yb) = delta_x/(delta_x*delta_x+delta_y*delta_y);

	Hnl(camera_bearing, hdg) = -1;
    Hnl(camera_hdgb,hdgb) = 1;

    //ynl(psib) = x(hdgb);
    //Hnl(psib,hdgb) = 1;

}

