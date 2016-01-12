/*********************************************************************
 *  EstimatorCore.cpp
 *
 *  Created on: Jan 12, 2016
 *      Authors: Dula Nad, Filip Mandic
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, LABUST, UNIZG-FER
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

#include <labust/navigation/KFCore.hpp>

#include <deque>


namespace labust
{
	namespace navigation
	{
		//enum{X=0,Y,Z,K,M,N, DoF};
		class EstimatorCore{

			typedef labust::navigation::KFCore<labust::navigation::EKF_3D_USBLModel> Estimator;

			struct FilterState{

				FilterState(){

				}

				~FilterState(){

				}

				Estimator::vector input;
				Estimator::vector measurement;
				Estimator::vector new_measurement;
				Estimator::vector state;
				Estimator::matrix Pcov;
				Estimator::matrix Rcov;

			};

		public:
			EstimatorCore(){

			}

			~EstimatorCore(){

			}

			void setMeasurementVector(Estimator::vector measurement);

			void setStateVector(Estimator::vector state);

			void setInputVector(Estimator::vector input);

			Estimator::vector getStateVector();

			void estimatorStep();

		private:

			void storeCurrentData();

			FilterState _state;
			std::deque<FilterState> _past_states;


		};
	}
}

using namespace labust::navigation;

void EstimatorCore::setMeasurementVector(Estimator::vector measurement, Estimator::vector new_measurement)
{
	_state.measurement = measurement;
	_state.new_measurement = new_measurement;
}

void EstimatorCore::setStateVector(Estimator::vector state)
{
	_state.state = state;
}

void EstimatorCore::setInputVector(Estimator::vector input)
{
	_state.input = input;
}

void EstimatorCore::storeCurrentData()
{
	/*** Store filter data ***/
	FilterState state;
	state.input = tauIn;
	state.meas = measurements;
	state.newMeas = newMeas;

	/*** Check if there are delayed measurements, and disable them in current step ***/
	bool new_delayed(false);
	for(size_t i=0; i<_state.new_measurement.size(); ++i)
	{
		if(_state.new_measurement(i))
		{
			_state.new_measurement(i) = -1;
			new_delayed = true;
		}
	}

	/*** Store x, P, R data ***/
	state.state = nav.getState();
	state.Pcov = nav.getStateCovariance();
	//state.Rcov = ; // In case of time-varying measurement covariance

	//ROS_ERROR_STREAM(state.Pcov);
	/*** Limit queue size ***/
	if(_past_states.size()>1000){
		_past_states.pop_front();
		//ROS_ERROR("Pop front");
	}
	_past_states.push_back(state);
}



void EstimatorCore::estimatorStep()
{
	storeCurrentData();



			if(new_delayed && enableDelay)
			{
				/*** Check for maximum delay ***/
				int delaySteps = measDelay.maxCoeff();

				/*** If delay is bigger then buffer size assume that it is max delay ***/
				if(delaySteps >= pastStates.size())
					delaySteps = pastStates.size()-1;

				/*** Position delayed measurements in past states container
				     and store past states data in temporary stack ***/
				std::stack<FilterState> tmp_stack;
				for(size_t i=0; i<=delaySteps; i++){

					KFNav::vector tmp_cmp;
					tmp_cmp.setConstant(KFNav::measSize, i);
					if((measDelay.array() == tmp_cmp.array()).any() && i != 0){
						FilterState tmp_state = pastStates.back();
						for(size_t j=0; j<measDelay.size(); ++j){
							if(measDelay(j) == i){
								tmp_state.newMeas(j) = 1;
								tmp_state.meas(j) = measurements(j);

								if(enableRejection)
								{
								/////////////////////////////////////////
								/// Outlier test
								/////////////////////////////////////////
								if(j == KFNav::range)
								{
									const KFNav::vector& x = tmp_state.state; ///// Treba li jos predikciju napravit?
									double range = measurements(j);

									Eigen::VectorXd input(Eigen::VectorXd::Zero(3));
										//Eigen::VectorXd output(Eigen::VectorXd::Zero(1));


									input << x(KFNav::xp)-x(KFNav::xb), x(KFNav::yp)-x(KFNav::yb), x(KFNav::zp)-x(KFNav::zb);
									//input << x(KFNav::u), x(KFNav::yp)-x(KFNav::yb), x(KFNav::zp)-x(KFNav::zb);

									//output << measurements(KFNav::range);
									double y_filt, sigma, w;
									OR.step(input, measurements(KFNav::range), &y_filt, &sigma, &w);
									//ROS_INFO("Finished outlier rejection");

									std_msgs::Float32 rng_msg;
									//ROS_ERROR("w: %f",w);

									double w_limit =  0.3*std::sqrt(float(sigma));
									w_limit = (w_limit>1.0)?1.0:w_limit;
									if(w>w_limit)
									{
										rng_msg.data = range;
										pubRangeFiltered.publish(rng_msg);
										//pubRangeFiltered.publish(rng_msg);

									} else {
										tmp_state.newMeas(j) = 0;
									}
									//std_msgs::Float32 rng_msg;

									rng_msg.data = w_limit;
									pubwk.publish(rng_msg);

									rng_msg.data = range;
									pubRange.publish(rng_msg);

								}

								if(j == KFNav::bearing && tmp_state.newMeas(j-1) == 0)
								{
									tmp_state.newMeas(j) = 0;

								}
								}

								//////////////////////////////////////////

								//ROS_ERROR("Dodano mjerenje. Delay:%d",i);
							}
						}
						tmp_stack.push(tmp_state);
					} else {
						tmp_stack.push(pastStates.back());
					}
					pastStates.pop_back();
				}

				/*** Load past state and covariance for max delay time instant ***/
				FilterState state_p = tmp_stack.top();
				nav.setStateCovariance(state_p.Pcov);
				nav.setState(state_p.state);

				/*** Pass through stack data and recalculate filter states ***/
				while(!tmp_stack.empty()){
					state_p = tmp_stack.top();
					tmp_stack.pop();
					pastStates.push_back(state_p);

					/*** Estimation ***/
					nav.predict(state_p.input);
					bool newArrived(false);
					for(size_t i=0; i<state_p.newMeas.size(); ++i)	if ((newArrived = state_p.newMeas(i))) break;
					if (newArrived)	nav.correct(nav.update(state_p.meas, state_p.newMeas));
				}
			}else{
				/*** Estimation ***/
				nav.predict(tauIn);
				bool newArrived(false);
				boost::mutex::scoped_lock l(meas_mux);
				for(size_t i=0; i<newMeas.size(); ++i)	if ((newArrived = newMeas(i))) break;

				//Update sensor flag
				updateDVL = !newMeas(KFNav::r);

				std::ostringstream out;
				out<<newMeas;
				ROS_DEBUG("Measurements %d:%s",KFNav::psi, out.str().c_str());

				if (newArrived)	nav.correct(nav.update(measurements, newMeas));
				KFNav::vector tcstate = nav.getState();
				if (tcstate(KFNav::buoyancy) < -30) tcstate(KFNav::buoyancy) = -10;
				if (tcstate(KFNav::buoyancy) > 0) tcstate(KFNav::buoyancy) = 0;
				if (tcstate(KFNav::altitude) < 0) tcstate(KFNav::altitude) = 0;
				nav.setState(tcstate);
				l.unlock();
			}

			newMeas.setZero(); // razmisli kako ovo srediti
			measDelay.setZero();
			publishState();

			//ROS_ERROR_STREAM(nav.getStateCovariance());

			//Send the base-link transform
			geometry_msgs::TransformStamped transform;
			KFNav::vectorref cstate = nav.getState();
			//Update DVL sensor
			if (updateDVL) dvl.current_r(cstate(KFNav::r));

			//local -> base_pose
			transform.transform.translation.x = cstate(KFNav::xp);
			transform.transform.translation.y = cstate(KFNav::yp);
			transform.transform.translation.z = cstate(KFNav::zp);
			labust::tools::quaternionFromEulerZYX(0, 0, 0, transform.transform.rotation);
			if(absoluteEKF){
				transform.child_frame_id = "base_pose_usbl_abs";
			} else{
				transform.child_frame_id = "base_pose_usbl";
			}
			transform.header.frame_id = "local";
			transform.header.stamp = ros::Time::now();
			broadcaster.sendTransform(transform);

			//local -> base_pose
			transform.transform.translation.x = 0;
			transform.transform.translation.y = 0;
			transform.transform.translation.z = 0;
			labust::tools::quaternionFromEulerZYX(cstate(KFNav::phi),
					cstate(KFNav::theta),
					cstate(KFNav::psi),
					transform.transform.rotation);
			if(absoluteEKF){
				transform.child_frame_id = "base_link_usbl_abs";
			} else{
				transform.child_frame_id = "base_link_usbl";
			}
			transform.header.frame_id = "base_pose_usbl";
			broadcaster.sendTransform(transform);

			rate.sleep();
			/*** Get current time (for delay calculation) ***/
			currentTime = ros::Time::now().toSec();
			ros::spinOnce();
}

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

