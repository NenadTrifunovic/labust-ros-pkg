/*********************************************************************
 *  EKF3D_USBLModel.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: Dula Nad
 *
 *   Modified on: Feb 27, 2015
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
#include <labust/navigation/EKF_3D_USBLModel.hpp>

#include <labust/navigation/SSModel.hpp>

namespace labust
{
  namespace navigation
  {
    /**
     * This class implements a EKF filter for the LDTravOcean ROV.
     */
    class GenericModel : public SSModel<double>
    {
    	typedef SSModel<double> Base;
    public:
      typedef vector input_type;
      typedef vector output_type;

      struct ModelParams
      {
    	  ModelParams():
    		  alpha(1),
    		  beta(1),
    		  betaa(0){};

    	  ModelParams(double alpha, double beta, double betaa):
    		  alpha(alpha),
    		  beta(beta),
    		  betaa(betaa){}

    	  inline double Beta(double val)
    	  {
    		  return beta + betaa*fabs(val);
    	  }

    	  double alpha, beta, betaa;
      };

      enum {u=0,v,w,p,q,r,xp,yp,zp,phi,theta,psi,xc,yc,b,buoyancy,roll_restore,pitch_restore,altitude,xb,yb,zb,stateNum};
      enum {X=0,Y,Z,Kroll,M,N,inputSize};
      enum {range=stateNum,bearing,elevation,measSize};

      //const char *VarNames[]  = {"u","v","w","p","q","r","xp","yp","zp","phi","theta","psi","xc","yc","b","buoyancy","roll_restore","pitch_restore","altitude","xb","yb","zb","range","bearing","elevation"};


      /*** The default constructor. ***/
      GenericModel();

      /*** Generic destructor. ***/
      ~GenericModel();

      /*** Process model ***/
      void derivativeAX();
      void derivativeAW();
      void derivativeHX();
      void derivativeHV();

    };
  }
}

      /**
       * Perform a prediction step based on the system input.
       *
       * \param u System input.
       */
      void step(const input_type& input);
      /**
       * Calculates the estimated output of the model.
       *
       * \param y Inserts the estimated output values here.
       */
      void estimate_y(output_type& y);
      /**
       * Initialize the model to default values
       */
      void initModel();

      /**
       * Setup the measurement matrix for available measurements.
       */
      const output_type& update(vector& measurements, vector& newMeas);

      /**
       * Set the model parameters.
       */
      void setParameters(const ModelParams& surge,
    		  const ModelParams& sway,
    		  const ModelParams& heave,
    		  const ModelParams& roll,
    		  const ModelParams& pitch,
    		  const ModelParams& yaw)
      {
    	  this->surge = surge;
    	  this->sway = sway;
    	  this->heave = heave;
    	  this->roll = roll;
    	  this->pitch = pitch;
    	  this->yaw = yaw;
      }

      void calculateXYInovationVariance(const matrix& P, double& xin,double &yin);
      void calculateUVInovationVariance(const matrix& P, double& uin,double &vin);
      double calculateAltInovationVariance(const matrix& P);

      /**
       * Return the speeds in the local frame.
       */
      inline void getNEDSpeed(double& xdot, double& ydot)
      {
      	xdot = this->xdot;
      	ydot = this->ydot;
      }

      inline void useDvlModel(int flag){this->dvlModel = flag;};
      inline void setDVLRotationTrustFactor(double trustf){this->trustf = trustf;};

      inline void setSwayCorrection(bool use_sc,
      		double acc_port, double acc_starboard,
					double vec_port, double vec_starboard)
      {
      	this->use_sc = use_sc;
      	this->acc_port = acc_port;
      	this->acc_starboard = acc_starboard;
      	this->vec_port = vec_port;
      	this->vec_starboard = vec_starboard;
      };

    protected:
     /**
       * Calculate the Jacobian matrices.
       */
      void derivativeAW();
      /**
        * Calculate the Jacobian matrices.
        */
       void derivativeHV(int num);
       /**
        * Calculate the nonlinear H derivative.
        */
       void derivativeH();
      /**
       * The model parameters.
       */
      ModelParams surge,sway,heave,roll,pitch,yaw;
      /**
       * The newest measurement.
       */
      output_type measurement;
      /**
       * The NED speeds.
       */
      double xdot,ydot;
      ///DVL rotation trust factor
      double trustf;
      ///The sway correction factors
      double use_sc, acc_port, acc_starboard,
			vec_port, vec_starboard;
      /**
       * The DVL linear/nonlinear flag.
       */
      int dvlModel;
      /**
       * The nonlinear H.
       */
      matrix Hnl;
      /**
       * The nonlinear and final y.
       */
      vector ynl,y;
    };
  }
}

using namespace labust::navigation;

#include <vector>

#include <ros/ros.h>

EKF_3D_USBLModel::EKF_3D_USBLModel():
		dvlModel(0),
		xdot(0),
		ydot(0){

	this->initModel();
};

EKF_3D_USBLModel::~EKF_3D_USBLModel(){};

void EKF_3D_USBLModel::initModel(){

  x = vector::Zero(stateNum);
  xdot = 0;
  ydot = 0;
  /*** Setup the transition matrix ***/
  derivativeAW();
  R0 = R;
  V0 = V;
  //std::cout<<"R:"<<R<<"\n"<<V<<std::endl;
}

void EKF_3D_USBLModel::calculateXYInovationVariance(const EKF_3D_USBLModel::matrix& P, double& xin,double &yin){

	xin = sqrt(P(xp,xp)) + sqrt(R0(xp,xp));
	yin = sqrt(P(yp,yp)) + sqrt(R0(yp,yp));
}

double EKF_3D_USBLModel::calculateAltInovationVariance(const EKF_3D_USBLModel::matrix& P){

	return sqrt(P(altitude,altitude)) + sqrt(R0(altitude,altitude));
}

void EKF_3D_USBLModel::calculateUVInovationVariance(const EKF_3D_USBLModel::matrix& P, double& uin,double &vin){

	uin = sqrt(P(u,u)) + sqrt(R0(v,v));
	vin = sqrt(P(v,v)) + sqrt(R0(v,v));
}

void EKF_3D_USBLModel::step(const input_type& input){

  x(u) += Ts*(-surge.Beta(x(u))/surge.alpha*x(u) + 1/surge.alpha * input(X));
  x(v) += Ts*(-sway.Beta(x(v))/sway.alpha*x(v) + 1/sway.alpha * input(Y));
  x(w) += Ts*(-heave.Beta(x(w))/heave.alpha*x(w) + 1/heave.alpha * (input(Z) + x(buoyancy)));
  //x(p) += Ts*(-roll.Beta(x(p))/roll.alpha*x(p) + 1/roll.alpha * (input(Kroll) + x(roll_restore)));
  //x(q) += Ts*(-pitch.Beta(x(p))/pitch.alpha*x(q) + 1/pitch.alpha * (input(M) + x(pitch_restore)));
  x(r) += Ts*(-yaw.Beta(x(r))/yaw.alpha*x(r) + 1/yaw.alpha * input(N) + x(b));

  xdot = x(u)*cos(x(psi)) - x(v)*sin(x(psi)) + x(xc);
  ydot = x(u)*sin(x(psi)) + x(v)*cos(x(psi)) + x(yc);
  x(xp) += Ts * xdot;
  x(yp) += Ts * ydot;
  x(zp) += Ts * x(w);
  x(altitude) += -Ts * x(w);
  //\todo This is not actually true since angles depend on each other
  //\todo Also x,y are dependent on the whole rotation matrix.
  //\todo We make a simplification here for testing with small angles ~10°
  //x(phi) += Ts * x(p);
  //x(theta) += Ts * x(q);
  x(psi) += Ts * x(r);

  xk_1 = x;

  derivativeAW();
}

void EKF_3D_USBLModel::derivativeAW(){

	A = matrix::Identity(stateNum, stateNum);

	A(u,u) = 1-Ts*(surge.beta + 2*surge.betaa*fabs(x(u)))/surge.alpha;
	A(v,v) = 1-Ts*(sway.beta + 2*sway.betaa*fabs(x(v)))/sway.alpha;
	A(w,w) = 1-Ts*(heave.beta + 2*heave.betaa*fabs(x(w)))/heave.alpha;
	A(w,buoyancy) = Ts/heave.alpha;
	//A(p,p) = 1-Ts*(roll.beta + 2*roll.betaa*fabs(x(p)))/roll.alpha;
	//A(p,roll_restore) = Ts/roll.alpha;
	//A(q,q) = 1-Ts*(pitch.beta + 2*pitch.betaa*fabs(x(q)))/pitch.alpha;
	//A(q,pitch_restore) = Ts/pitch.alpha;
	A(r,r) = 1-Ts*(yaw.beta + 2*yaw.betaa*fabs(x(r)))/yaw.alpha;
	A(r,b) = Ts;

	A(xp,u) = Ts*cos(x(psi));
	A(xp,v) = -Ts*sin(x(psi));
	A(xp,psi) = Ts*(-x(u)*sin(x(psi)) - x(v)*cos(x(psi)));
	A(xp,xc) = Ts;

	A(yp,u) = Ts*sin(x(psi));
	A(yp,v) = Ts*cos(x(psi));
	A(yp,psi) = Ts*(x(u)*cos(x(psi)) - x(v)*sin(x(psi)));
	A(yp,yc) = Ts;

	A(zp,w) = Ts;
	//\todo If you don't want the altitude to contribute comment this out.
	A(altitude,w) = -Ts;

	//A(phi,p) = Ts;
	//A(theta,q) = Ts;
	A(psi,r) = Ts;
}

const EKF_3D_USBLModel::output_type& EKF_3D_USBLModel::update(vector& measurements, vector& newMeas){

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

void EKF_3D_USBLModel::estimate_y(output_type& y){
	y=this->y;
}

void EKF_3D_USBLModel::derivativeH(){

	Hnl = matrix::Zero(measSize,stateNum); // Prije je bilo identity
	Hnl.topLeftCorner(stateNum,stateNum) = matrix::Identity(stateNum,stateNum);

	ynl = vector::Zero(measSize);
	ynl.head(stateNum) = matrix::Identity(stateNum,stateNum)*x;

	switch (dvlModel){
	case 1:
		/*** Correct the nonlinear part ***/
		ynl(u) = x(u)+x(xc)*cos(x(psi))+x(yc)*sin(x(psi));
		ynl(v) = x(v)-x(xc)*sin(x(psi))+x(yc)*cos(x(psi));

		//Correct for the nonlinear parts
		Hnl(u,u) = 1;
		Hnl(u,xc) = cos(x(psi));
		Hnl(u,yc) = sin(x(psi));
		Hnl(u,psi) = -x(xc)*sin(x(psi)) + x(yc)*cos(x(psi));

		Hnl(v,v) = 1;
		Hnl(v,xc) = -sin(x(psi));
		Hnl(v,yc) = cos(x(psi));
		Hnl(v,psi) = -x(xc)*cos(x(psi)) - x(yc)*sin(x(psi));
		break;

	case 2:
		/*** Correct the nonlinear part ***/
		y(u) = x(u)*cos(x(psi)) - x(v)*sin(x(psi)) + x(xc);
		y(v) = x(u)*sin(x(psi)) + x(v)*cos(x(psi)) + x(yc);

	    /*** Correct for the nonlinear parts ***/
		Hnl(u,xc) = 1;
		Hnl(u,u) = cos(x(psi));
		Hnl(u,v) = -sin(x(psi));
		Hnl(u,psi) = -x(u)*sin(x(psi)) - x(v)*cos(x(psi));

		Hnl(v,yc) = 1;
		Hnl(v,u) = sin(x(psi));
		Hnl(v,v) = cos(x(psi));
		Hnl(v,psi) = x(u)*cos(x(psi)) - x(v)*sin(x(psi));
		break;
	}

//	double rng  = sqrt(pow((x(xp)-x(xb)),2)+pow((x(yp)-x(yb)),2)+pow((x(zp)-x(zb)),2));
//
//	//if(rng == 0) rng = 0.1;
//
//	if(rng<0.0001){
//		rng = 0.0001;
//	}
//
//	double delta_xbp = x(xb)-x(xp);
//	//ROS_ERROR("delta: %f",delta_xbp);
//	if(abs(delta_xbp)<0.000000001){
//		delta_xbp = (delta_xbp<0)?-0.000000001:0.000000001;
//	}
//
//	//ROS_ERROR("delta after: %f",delta_xbp);
//
//	ynl(range) = rng;
//	ynl(bearing) = atan2((x(yp)-x(yb)),(x(xp)-x(xb)))-0*x(psi);
//	ynl(elevation) = asin((x(zp)-x(zb))/rng);
//
//	Hnl(range, xp)  = (x(xp)-x(xb))/rng;
//	Hnl(range, yp)  = (x(yp)-x(yb))/rng;
//	Hnl(range, zp)  = (x(zp)-x(zb))/rng;
//
//	Hnl(range, xb)  = -(x(xp)-x(xb))/rng;
//	Hnl(range, yb)  = -(x(yp)-x(yb))/rng;
//	Hnl(range, zb)  = -(x(zp)-x(zb))/rng;
//
//	Hnl(bearing, xp) = (x(yb)-x(yp))/(pow(delta_xbp,2)*(pow((x(yb)-x(yp))/delta_xbp,2) + 1));
//	Hnl(bearing, yp) = -1/((delta_xbp)*(pow((x(yb) - x(yp))/delta_xbp,2) + 1));
//	Hnl(bearing, xb) = -(x(yb) - x(yp))/(pow(delta_xbp,2)*(pow((x(yb) - x(yp))/delta_xbp,2) + 1));
//	Hnl(bearing, yb) = 1/((delta_xbp)*(pow((x(yb) - x(yp))/delta_xbp,2) + 1));
//	//Hnl(bearing, psi) = -1;


	double rng  = sqrt(pow((x(xp)-x(xb)),2)+pow((x(yp)-x(yb)),2)+pow((x(zp)-x(zb)),2));
	double delta_x = (x(xb)-x(xp));
	double delta_y = (x(yb)-x(yp));

	if(rng<0.00001)
		rng = 0.00001;

	if(abs(delta_x)<0.00001)
		delta_x = (delta_x<0)?-0.00001:0.00001;

	if(abs(delta_y)<0.00001)
		delta_y = (delta_y<0)?-0.00001:0.00001;

	ynl(range) = rng;
	ynl(bearing) = atan2(delta_y,delta_x) -1*x(psi);
	ynl(elevation) = asin((x(zp)-x(zb))/rng);

	Hnl(range, xp)  = -(x(xb)-x(xp))/rng;
	Hnl(range, yp)  = -(x(yb)-x(yp))/rng;
	Hnl(range, zp)  = -(x(zb)-x(zp))/rng;

	Hnl(range, xb)  = (x(xb)-x(xp))/rng;
	Hnl(range, yb)  = (x(yb)-x(yp))/rng;
	Hnl(range, zb)  = (x(zb)-x(zp))/rng;

	Hnl(bearing, xp) = delta_y/(delta_x*delta_x+delta_y*delta_y);
	Hnl(bearing, yp) = -delta_x/(delta_x*delta_x+delta_y*delta_y);
	Hnl(bearing, xb) = -delta_y/(delta_x*delta_x+delta_y*delta_y);
	Hnl(bearing, yb) = delta_x/(delta_x*delta_x+delta_y*delta_y);

	Hnl(bearing, psi) = -1;



	// Nadi gresku u elevationu i sredi singularitete
//	double part1 = (x(zb) - x(zp))/(sqrt(1 - pow((x(zb) - x(zp)),2)/pow(rng,2))*(pow((rng),3)));
//	double part2 = (x(xb)*x(xb) - 2*x(xb)*x(xp) + x(xp)*x(xp) + x(yb)*x(yb) - 2*x(yb)*x(yp) + x(yp)*x(yp))/(sqrt(1 - pow((x(zb) - x(zp)),2)/pow(rng,2))*(pow((rng),3)));
//
//	Hnl(elevation,xp) = -((x(xb) - x(xp))*part1);
//	Hnl(elevation,yp) = -((x(yb) - x(yp))*part1);
//	Hnl(elevation,zp) = part2;
//	Hnl(elevation,xb) = ((x(xb) - x(xp))*part1);
//	Hnl(elevation,yb) = ((x(yb) - x(yp))*part1);
//	Hnl(elevation,zb) = -part2;
}

