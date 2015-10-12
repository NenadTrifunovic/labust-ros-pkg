/*********************************************************************
 *  SimpleRelativeLocalizationModel.cpp
 *
 *  Created on: Sep 22, 2015
 *      Author: Filip Mandic
 *
 ********************************************************************/

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
#include <labust/navigation/SimpleRelativeLocalizationModel.hpp>

using namespace labust::navigation;

#include <vector>

#include <ros/ros.h>

SimpleRelativeLocalizationModel::SimpleRelativeLocalizationModel():
		dvlModel(1),
		xdot(0),
		ydot(0){

	this->initModel();
};

SimpleRelativeLocalizationModel::~SimpleRelativeLocalizationModel(){};

void SimpleRelativeLocalizationModel::initModel(){

  x = vector::Zero(stateNum);
  xdot = 0;
  ydot = 0;
  /*** Setup the transition matrix ***/
  derivativeAW();
  R0 = R;
  V0 = V;
  //std::cout<<"R:"<<R<<"\n"<<V<<std::endl;
}

void SimpleRelativeLocalizationModel::calculateXYInovationVariance(const SimpleRelativeLocalizationModel::matrix& P, double& xin,double &yin){

	xin = sqrt(P(xp,xp)) + sqrt(R0(xp,xp));
	yin = sqrt(P(yp,yp)) + sqrt(R0(yp,yp));
}


void SimpleRelativeLocalizationModel::calculateUVInovationVariance(const SimpleRelativeLocalizationModel::matrix& P, double& uin,double &vin){

	//uin = sqrt(P(u,u)) + sqrt(R0(u,u));
	//vin = sqrt(P(v,v)) + sqrt(R0(v,v));
}

void SimpleRelativeLocalizationModel::step(const input_type& input){



  //xdot = x(u)*cos(x(psi));
  //ydot = x(u)*sin(x(psi));
  x(xb) += Ts * input(x_dot);
  x(yb) += Ts * input(y_dot);
 // x(xb) += 0;
 // x(yb) += 0;
  x(xp) += 0;
  x(yp) += 0;

  xk_1 = x;

  derivativeAW();
}

void SimpleRelativeLocalizationModel::derivativeAW(){

	A = matrix::Identity(stateNum, stateNum);

//	A(xp,u) = Ts*cos(x(psi));
//	//A(xp,v) = -Ts*sin(x(psi));
//	A(xp,psi) = Ts*(-x(u)*sin(x(psi)));
//	//A(xp,xc) = Ts;
//
//	A(yp,u) = Ts*sin(x(psi));
//	//A(yp,v) = Ts*cos(x(psi));
//	A(yp,psi) = Ts*(x(u)*cos(x(psi)));
//	//A(yp,yc) = Ts;
//
//	A(zp,w) = Ts;
//	A(psi,r) = Ts;
//
//	A(xb,ub) = Ts*cos(x(psib));
//	//A(xp,v) = -Ts*sin(x(psi));
//	A(xb,psib) = Ts*(-x(ub)*sin(x(psib)));
//	//A(xp,xc) = Ts;
//
//	A(yb,ub) = Ts*sin(x(psib));
//	//A(yp,v) = Ts*cos(x(psi));
//	A(yb,psib) = Ts*(x(ub)*cos(x(psib)));
//	//A(yp,yc) = Ts;
//
//	A(zb,wb) = Ts;
//	A(psib,rb) = Ts;
}

const SimpleRelativeLocalizationModel::output_type& SimpleRelativeLocalizationModel::update(vector& measurements, vector& newMeas){

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

	if (dvlModel != 0) derivativeH();

	measurement.resize(arrived.size());
	H = matrix::Zero(arrived.size(),stateNum);
	y = vector::Zero(arrived.size());
	R = matrix::Zero(arrived.size(),arrived.size());
	V = matrix::Zero(arrived.size(),arrived.size());

	for (size_t i=0; i<arrived.size();++i)
	{
		measurement(i) = dataVec[i];

		if (dvlModel != 0)
		{
			H.row(i)=Hnl.row(arrived[i]);
			y(i) = ynl(arrived[i]);
		}
		else
		{
			H(i,arrived[i]) = 1;
			y(i) = x(arrived[i]);
		}

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

void SimpleRelativeLocalizationModel::estimate_y(output_type& y){
	y=this->y;
}

void SimpleRelativeLocalizationModel::derivativeH(){
	Hnl = matrix::Zero(measSize,stateNum);
	Hnl.topLeftCorner(stateNum,stateNum) = matrix::Identity(stateNum,stateNum);

	ynl = vector::Zero(measSize);
	ynl.head(stateNum) = matrix::Identity(stateNum,stateNum)*x;

/*
	double rng  = sqrt(pow(x(xp),2)+pow(x(yp),2));
	double delta_x = x(xp);
	double delta_y = x(yp);

	double eps = 0.000000001;

	if(rng<eps)
		rng = eps;

	if(abs(delta_x)<eps)
		delta_x = (delta_x<0)?-eps:eps;

	if(abs(delta_y)<eps)
		delta_y = (delta_y<0)?-eps:eps;

	ynl(range) = rng;

	Hnl(range, xp)  = x(xp)/rng;
	Hnl(range, yp)  = x(yp)/rng;
*/

double rng  = sqrt(pow((x(xp)-x(xb)),2)+pow((x(yp)-x(yb)),2));
	double delta_x = (x(xp)-x(xb));
	double delta_y = (x(yp)-x(yb));

	double eps = 0.000000001;

	if(rng<eps)
		rng = eps;

	if(abs(delta_x)<eps)
		delta_x = (delta_x<0)?-eps:eps;

	if(abs(delta_y)<eps)
		delta_y = (delta_y<0)?-eps:eps;

	ynl(range) = rng;

	//Hnl(range, xp)  = -(x(xb)-x(xp))/rng;
	//Hnl(range, yp)  = -(x(yb)-x(yp))/rng;

	//Hnl(range, xb)  = (x(xb)-x(xp))/rng;
	//Hnl(range, yb)  = (x(yb)-x(yp))/rng;

	Hnl(range, xp)  = delta_x/rng;
	Hnl(range, yp)  = delta_y/rng;

	Hnl(range, xb)  = -delta_x/rng;
	Hnl(range, yb)  = -delta_y/rng;

}

