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
*  Created on: Feb 25, 2013
*  Author: Dula Nad
*
*  Modified by: Filip Mandic
*********************************************************************/
#include <labust/navigation/RelativeTrackingModel.hpp>
#include <vector>
#include <iostream>
#include <ros/ros.h>

using namespace labust::navigation;

RelativeTrackingModel::RelativeTrackingModel():
		dvlModel(0),
		xdot(0),
		ydot(0)
{
	this->initModel();
};

RelativeTrackingModel::~RelativeTrackingModel(){};

void RelativeTrackingModel::initModel()
{
  //std::cout<<"Init model."<<std::endl;
  x = vector::Zero(stateNum);
  xdot = 0;
  ydot = 0;
  //Setup the transition matrix
  derivativeAW();
  R0 = R;
  V0 = V;

  //std::cout<<"R:"<<R<<"\n"<<V<<std::endl;
}

//void RelativeTrackingModel::calculateXYInovationVariance(const RelativeTrackingModel::matrix& P, double& xin,double &yin)
//{
//	xin = sqrt(P(xp,xp)) + sqrt(R0(xp,xp));
//	yin = sqrt(P(yp,yp)) + sqrt(R0(yp,yp));
//}

//double RelativeTrackingModel::calculateAltInovationVariance(const RelativeTrackingModel::matrix& P)
//{
//	return 0; //sqrt(P(altitude,altitude)) + sqrt(R0(altitude,altitude));
//}

//void RelativeTrackingModel::calculateUVInovationVariance(const RelativeTrackingModel::matrix& P, double& uin,double &vin)
//{
//	uin = sqrt(P(u,u)) + sqrt(R0(u,u));// Greska??? napomena: ispravljeno
//	vin = sqrt(P(v,v)) + sqrt(R0(v,v));
//}

void RelativeTrackingModel::step(const input_type& input){

	/*****************************************************************
	*** Relative target tracking
	*****************************************************************/

	/* States --- delta_x ,delta_y, delta_z, psi_t, u_t, w_t, r_t */
	/* Inputs ---	x_dot, y_dot, psi, X, Y, Z */

	x(delta_x) += Ts*(input(x_dot) - x(u_t)*cos(x(psi_t)));
	x(delta_y) += Ts*(input(y_dot) - x(u_t)*sin(x(psi_t)));
	x(delta_z) += Ts*x(w_t);
	x(psi_t) += Ts*x(r_t);
	x(u_t) += Ts*(-surge.Beta(x(u_t))/surge.alpha*x(u_t) + 1/surge.alpha * input(X));
	x(w_t) += Ts*(-heave.Beta(x(w_t))/heave.alpha*x(w_t) + 1/heave.alpha * (input(Z))); /* Buoyancy ignored */
	x(r_t) += Ts*(-yaw.Beta(x(r_t))/yaw.alpha*x(r_t) + 1/yaw.alpha * input(N));

	xk_1 = x;

	derivativeAW();
};

void RelativeTrackingModel::derivativeAW(){

	/*****************************************************************
	*** Relative target tracking
	*****************************************************************/

	/* States --- delta_x ,delta_y, delta_z, psi_t, u_t, w_t, r_t */

	A = matrix::Identity(stateNum, stateNum);

	A(delta_x, psi_t) = Ts*x(u_t)*sin(x(psi_t));
	A(delta_x, u_t) = -Ts*cos(x(psi_t));

	A(delta_y, psi_t) = -Ts*x(u_t)*cos(x(psi_t));
	A(delta_y, u_t) = -Ts*sin(x(psi_t));

	A(delta_z, w_t) = Ts;

	A(psi_t,r_t) = Ts;

	A(u_t, u_t) = 1-Ts*(surge.beta + 2*surge.betaa*fabs(x(u_t)))/surge.alpha;

	A(w_t,w_t) = 1-Ts*(heave.beta + 2*heave.betaa*fabs(x(w_t)))/heave.alpha;

	A(r_t,r_t) = 1-Ts*(yaw.beta + 2*yaw.betaa*fabs(x(r_t)))/yaw.alpha;

}

const RelativeTrackingModel::output_type& RelativeTrackingModel::update(vector& measurements, vector& newMeas)
{
	std::vector<size_t> arrived;
	std::vector<double> dataVec;

	for (size_t i=0; i<newMeas.size(); ++i)
	{

		if (newMeas(i))
		{
			arrived.push_back(i);
			dataVec.push_back(measurements(i));
			newMeas(i) = 0;
		}
	}


	//ROS_ERROR("broj mjerenja: %d", arrived.size());
	//for( std::vector<size_t>::const_iterator it = arrived.begin(); it != arrived.end(); ++it)
	//	ROS_ERROR("%d", *it);


	//if (dvlModel != 0) derivativeH();

	derivativeH();

	measurement.resize(arrived.size());
	H = matrix::Zero(arrived.size(),stateNum);
	y = vector::Zero(arrived.size());
	R = matrix::Zero(arrived.size(),arrived.size());
	V = matrix::Zero(arrived.size(),arrived.size());

	for (size_t i=0; i<arrived.size();++i)
	{
		measurement(i) = dataVec[i];

		//if (dvlModel != 0)
		//{
			H.row(i)=Hnl.row(arrived[i]);
			y(i) = ynl(arrived[i]);
		//}
		//else
		//{
		//	H(i,arrived[i]) = 1;
		//	y(i) = x(arrived[i]);
		//}

		for (size_t j=0; j<arrived.size(); ++j)
		{
			R(i,j)=R0(arrived[i],arrived[j]);
			V(i,j)=V0(arrived[i],arrived[j]);
		}
	}

	//std::cout<<"Setup H:"<<H<<std::endl;
	//std::cout<<"Setup R:"<<R<<std::endl;
	//std::cout<<"Setup V:"<<V<<std::endl;

	return measurement;
}

void RelativeTrackingModel::estimate_y(output_type& y){

  y=this->y;
}

void RelativeTrackingModel::derivativeH(){

	//Hnl = matrix::Zero(measSize,stateNum);
	//ynl = vector::Zero(measSize);

	Hnl=matrix::Identity(stateNum,stateNum);
	ynl = Hnl*x;

	/* Measurement vector --- d, theta, depth, psi_tm, delta_xm, delat_ym, delta_zm*/

	ynl(d) = sqrt(pow(x(delta_x),2)+pow(x(delta_y),2)+pow(x(delta_z),2));
	ynl(theta) = atan2(x(delta_y),x(delta_x));
	ynl(depth) = x(delta_z);
	ynl(psi_tm) = x(psi_t);
	ynl(delta_xm) = x(delta_x);
	ynl(delta_ym) = x(delta_y);
	ynl(delta_zm) = x(delta_z);

	Hnl(d, delta_x)  = (x(delta_x))/sqrt(pow(x(delta_x),2)+pow(x(delta_y),2)+pow(x(delta_z),2));
	Hnl(d, delta_y)  = (x(delta_y))/sqrt(pow(x(delta_x),2)+pow(x(delta_y),2)+pow(x(delta_z),2));
	Hnl(d, delta_z)  = (x(delta_z))/sqrt(pow(x(delta_x),2)+pow(x(delta_y),2)+pow(x(delta_z),2));

	Hnl(theta, delta_x) = -x(delta_y)/(pow(x(delta_x),2)*(pow(x(delta_y)/x(delta_x),2) + 1));
	Hnl(theta, delta_y) = 1/(x(delta_x)*(pow(x(delta_y)/x(delta_x),2) + 1));

	Hnl(depth, delta_z) = 1;

	Hnl(psi_tm, psi_t) = 1;

	Hnl(delta_xm, delta_x) = 1;

	Hnl(delta_ym, delta_y) = 1;

	Hnl(delta_zm, delta_z) = 1;
}

