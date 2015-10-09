/*********************************************************************
 *  EKF_3D_USBL.cpp
 *
 *  Created on: Mar 26, 2013
 *      Author: Dula Nad
 *
 *   Modified on: Feb 27, 2015
 *      Author: Filip Mandic
 *
 *   Description:
 *   	6-DOF EKF navigation filter with raw range and bearing measurements
 *
 *   	Relative mode:
 *
 *   	Absolute mode:
 *
 ********************************************************************/
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, LABUST, UNIZG-FER
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
#include <labust/navigation/TwoVehicleLocalization.hpp>
#include <labust/navigation/TwoVehicleLocalizationModel.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/simulation/DynamicsParams.hpp>
#include <labust/navigation/KFModelLoader.hpp>

#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyForceReq.h>
#include <underwater_msgs/USBLFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <ros/ros.h>

#include <boost/bind.hpp>
#include <math.h>

using namespace labust::navigation;



Estimator3D::Estimator3D():

		tauIn(KFNav::vector::Zero(KFNav::inputSize)),
		measurements(KFNav::vector::Zero(KFNav::measSize)),
		newMeas(KFNav::vector::Zero(KFNav::measSize)),
		measDelay(KFNav::vector::Zero(KFNav::measSize)),
		enableDelay(false),
		enableRange(true),
		enableBearing(true),
		enableElevation(false),
		enableRejection(false),
		delay_time(0.0),
		dvl_model(1),
		OR(3,0.95){this->onInit();};

void Estimator3D::onInit()
{
	ros::NodeHandle nh, ph("~");

	/** Configure the navigation */
	configureNav(nav,nh);

	/** Publishers */
	pubLocalStateHat = nh.advertise<auv_msgs::NavSts>("localStateHat",1);
	pubSecondStateHat = nh.advertise<auv_msgs::NavSts>("secondStateHat",1);

	pubLocalStateMeas = nh.advertise<auv_msgs::NavSts>("localMesurement",1);
	pubSecondStateMeas = nh.advertise<auv_msgs::NavSts>("secondMesurement",1);

	pubRange = nh.advertise<std_msgs::Float32>("range_meas",1);
	pubBearing = nh.advertise<std_msgs::Float32>("bearing_meas",1);

	pubCondP = nh.advertise<std_msgs::Float32>("condP",1);
	pubCondPxy = nh.advertise<std_msgs::Float32>("condPxy",1);
	pubCost = nh.advertise<std_msgs::Float32>("cost",1);

	//pubRangeFiltered = nh.advertise<std_msgs::Float32>("range_filtered",1);
	//pubwk = nh.advertise<std_msgs::Float32>("w_limit",1);

	/** Subscribers */
	subLocalStateHat = nh.subscribe<auv_msgs::NavSts>("stateHat", 1, &Estimator3D::onLocalStateHat,this);

	subSecond_heading = nh.subscribe<std_msgs::Float32>("out_acoustic_heading", 1, &Estimator3D::onSecond_heading,this);
	subSecond_position = nh.subscribe<geometry_msgs::Point>("out_acoustic_position", 1, &Estimator3D::onSecond_position,this);
	subSecond_speed = nh.subscribe<std_msgs::Float32>("out_acoustic_speed", 1, &Estimator3D::onSecond_speed,this);
	subSecond_usbl_fix = nh.subscribe<underwater_msgs::USBLFix>("usbl_fix", 1, &Estimator3D::onSecond_usbl_fix,this);

	resetTopic = nh.subscribe<std_msgs::Bool>("reset_nav_covariance", 1, &Estimator3D::onReset,this);

	/** Enable USBL measurements */
	ph.param("delay", enableDelay, enableDelay);
	ph.param("delay_time", delay_time, delay_time);
	ph.param("range", enableRange, enableRange);
	ph.param("bearing", enableBearing, enableBearing);
	ph.param("elevation", enableElevation, enableElevation);

	/** Enable outlier rejection */
	ph.param("meas_outlier_rejection", enableRejection, enableRejection);
}

void Estimator3D::onReset(const std_msgs::Bool::ConstPtr& reset)
{
   if (reset->data)
      nav.setStateCovariance(10000*KFNav::matrix::Identity(KFNav::stateNum, KFNav::stateNum));
}

void Estimator3D::configureNav(KFNav& nav, ros::NodeHandle& nh)
{
	ROS_INFO("Configure navigation.");

	/** No dynamic params initialization */

	nav.initModel();
	labust::navigation::kfModelLoader(nav, nh, "ekfnav_twl");
}

/*********************************************************************
 *** Measurement callback
 ********************************************************************/

void Estimator3D::onLocalStateHat(const auv_msgs::NavSts::ConstPtr& data)
{
	measurements(KFNav::xp) = data->position.north;
	newMeas(KFNav::xp) = 1;

	measurements(KFNav::yp) = data->position.east;
	newMeas(KFNav::yp) = 1;

	measurements(KFNav::zp) = data->position.depth;
	newMeas(KFNav::zp) = 1;

	Eigen::Matrix2d R;
	double yaw = labust::math::wrapRad(data->orientation.yaw);
	R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
	Eigen::Vector2d in, out;
	in << data->gbody_velocity.x, data->gbody_velocity.y;
	out = R*in;


	measurements(KFNav::psi) = std::atan2(out(1),out(0));
	newMeas(KFNav::psi) = 1;

	measurements(KFNav::u) = std::sqrt(std::pow(out(0),2)+std::pow(out(1),2));
	newMeas(KFNav::u) = 1;

	measurements(KFNav::w) = data->gbody_velocity.z;
	newMeas(KFNav::w) = 1;

	measurements(KFNav::r) = data->orientation_rate.yaw;
	newMeas(KFNav::r) = 1;
};


void Estimator3D::onSecond_heading(const std_msgs::Float32::ConstPtr& data)
{
	measurements(KFNav::psib) = data->data;
	newMeas(KFNav::psib) = 1;
}

void Estimator3D::onSecond_position(const geometry_msgs::Point::ConstPtr& data)
{
	/*measurements(KFNav::xb) = data->x;
	newMeas(KFNav::xb) = 1;

	measurements(KFNav::yb) = data->y;
	newMeas(KFNav::yb) = 1;*/

	measurements(KFNav::zb) = data->z;
	newMeas(KFNav::zb) = 1;
}

void Estimator3D::onSecond_speed(const std_msgs::Float32::ConstPtr& data)
{
	measurements(KFNav::ub) = data->data;
	newMeas(KFNav::ub) = 1;
}

void Estimator3D::onSecond_usbl_fix(const underwater_msgs::USBLFix::ConstPtr& data)
{

	/*** Calculate measurement delay ***/
	//double delay = calculateDelaySteps(data->header.stamp.toSec(), currentTime);
	// Totalno nepotrebno
	double delay = double(calculateDelaySteps(currentTime-delay_time, currentTime));

	double bear = 360 - data->bearing;
	double elev = 180 - data->elevation;

	const KFNav::vector& x = nav.getState();

	//labust::math::wrapRad(measurements(KFNav::psi));

	ROS_ERROR("RANGE: %f, BEARING: %f deg %f rad", data->range, labust::math::wrapRad(bear*M_PI/180), labust::math::wrapRad(bear*M_PI/180+x(KFNav::psi)));
	/*** Get USBL measurements ***/
	measurements(KFNav::range) = (data->range > 0.1)?data->range:0.1;
	newMeas(KFNav::range) = enableRange;
	measDelay(KFNav::range) = delay;


	//measurements(KFNav::bearing) = labust::math::wrapRad(bear*M_PI/180+x(KFNav::psi));
	measurements(KFNav::bearing) = labust::math::wrapRad(bear*M_PI/180);
	newMeas(KFNav::bearing) = enableBearing;
	measDelay(KFNav::bearing) = delay;

	measurements(KFNav::elevation) = elev*M_PI/180;
	newMeas(KFNav::elevation) = enableElevation;
	measDelay(KFNav::elevation) = delay;

	/** Relative distance measurements */

	/*measurements(KFNav::xb) = x(KFNav::xp) - data->relative_position.x;
	newMeas(KFNav::xb) = 1;
	measurements(KFNav::yb) = x(KFNav::yp) - data->relative_position.y;
	newMeas(KFNav::yb) = 1;
	measurements(KFNav::zb) = x(KFNav::zp) - data->relative_position.z;
	newMeas(KFNav::zb) = 1; */

}

/*********************************************************************
 *** Helper functions
 ********************************************************************/

void Estimator3D::processMeasurements()
{
	/*** Publish local measurements ***/
	auv_msgs::NavSts::Ptr meas(new auv_msgs::NavSts());
	meas->body_velocity.x = measurements(KFNav::u);
	meas->body_velocity.z = measurements(KFNav::w);

	meas->position.north = measurements(KFNav::xp);
	meas->position.east = measurements(KFNav::yp);
	meas->position.depth = measurements(KFNav::zp);

	meas->orientation.yaw = labust::math::wrapRad(measurements(KFNav::psi));
	meas->orientation_rate.yaw = measurements(KFNav::r);

	meas->header.stamp = ros::Time::now();
	meas->header.frame_id = "local";
	pubLocalStateMeas.publish(meas);

	/*** Publish second vehicle measurements ***/
	auv_msgs::NavSts::Ptr meas2(new auv_msgs::NavSts());
	meas2->body_velocity.x = measurements(KFNav::ub);
	meas2->body_velocity.z = measurements(KFNav::wb);

	meas2->position.north = measurements(KFNav::xb);
	meas2->position.east = measurements(KFNav::yb);
	meas2->position.depth = measurements(KFNav::zb);

	meas2->orientation.yaw = labust::math::wrapRad(measurements(KFNav::psib));
	meas2->orientation_rate.yaw = measurements(KFNav::rb);

	meas2->header.stamp = ros::Time::now();
	meas2->header.frame_id = "local";
	pubSecondStateMeas.publish(meas2);
}

void Estimator3D::publishState()
{
	auv_msgs::NavSts::Ptr state(new auv_msgs::NavSts());
	const KFNav::vector& estimate = nav.getState();
	state->gbody_velocity.x = estimate(KFNav::u);
	state->gbody_velocity.z = estimate(KFNav::w);

	state->orientation.yaw = labust::math::wrapRad(estimate(KFNav::psi));
	state->orientation_rate.yaw = estimate(KFNav::r);

	state->position.north = estimate(KFNav::xp);
	state->position.east = estimate(KFNav::yp);
	state->position.depth = estimate(KFNav::zp);

	const KFNav::matrix& covariance = nav.getStateCovariance();
	state->position_variance.north = covariance(KFNav::xp, KFNav::xp);
	state->position_variance.east = covariance(KFNav::yp, KFNav::yp);
	state->position_variance.depth = covariance(KFNav::zp,KFNav::zp);
	state->orientation_variance.yaw =  covariance(KFNav::psi, KFNav::psi);

	state->header.stamp = ros::Time::now();
	state->header.frame_id = "local";
	pubLocalStateHat.publish(state);

	/*** Publish second vehicle states */
	auv_msgs::NavSts::Ptr state2(new auv_msgs::NavSts());
	state2->gbody_velocity.x = estimate(KFNav::ub);
	state2->gbody_velocity.z = estimate(KFNav::wb);

	state2->orientation.yaw = labust::math::wrapRad(estimate(KFNav::psib));
	state2->orientation_rate.yaw = estimate(KFNav::rb);

	state2->position.north = estimate(KFNav::xb);
	state2->position.east = estimate(KFNav::yb);
	state2->position.depth = estimate(KFNav::zb);

	state2->position_variance.north = covariance(KFNav::xb, KFNav::xb);
	state2->position_variance.east = covariance(KFNav::yb, KFNav::yb);
	state2->position_variance.depth = covariance(KFNav::zb,KFNav::zb);
	state2->orientation_variance.yaw =  covariance(KFNav::psib, KFNav::psib);

	state2->header.stamp = ros::Time::now();
	state2->header.frame_id = "local";
	pubSecondStateHat.publish(state2);
}

void Estimator3D::calculateConditionNumber(){

	KFNav::matrix P = nav.getStateCovariance();

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(P);
	double cond1 = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);

	Eigen::Matrix2d Pxy;
	Pxy << P(KFNav::xp,KFNav::xp), P(KFNav::xp,KFNav::yp),
		   P(KFNav::yp,KFNav::xp), P(KFNav::yp,KFNav::yp);

	Eigen::JacobiSVD<Eigen::MatrixXd> svd2(P);
	double cond2 = svd2.singularValues()(0) / svd2.singularValues()(svd2.singularValues().size()-1);

	double traceP = Pxy.trace();
	double detP = Pxy.determinant();

	double condCost = std::sqrt(traceP*traceP-4*detP);

	std_msgs::Float32::Ptr data(new std_msgs::Float32);

	data->data = cond1;
	pubCondP.publish(data);
	data->data = cond2;
	pubCondPxy.publish(data);
	data->data = condCost;
	pubCost.publish(data);

}

int Estimator3D::calculateDelaySteps(double measTime, double arrivalTime){
				return floor((arrivalTime-measTime)/nav.Ts);
			}

/*********************************************************************
 *** Main loop
 ********************************************************************/
void Estimator3D::start()
{
	ros::NodeHandle ph("~");
	double Ts(0.1);
	ph.param("Ts",Ts,Ts);
	ros::Rate rate(1/Ts);
	nav.setTs(Ts);

	/*** Initialize time (for delay calculation) ***/
	currentTime = ros::Time::now().toSec();

	while (ros::ok()){

		/*** Process measurements ***/
		processMeasurements();

		/*** Store filter data ***/
		FilterState state;
		state.input = tauIn;
		state.meas = measurements;
		state.newMeas = newMeas;

		/*** Check if there are delayed measurements, and disable them in current step ***/
		bool newDelayed(false);
		for(size_t i=0; i<measDelay.size(); ++i){
			if(measDelay(i)){
				state.newMeas(i) = 0;
				newDelayed = true;
			}
		}

		/*** Store x, P, R data ***/
		state.state = nav.getState();
		state.Pcov = nav.getStateCovariance();
		//state.Rcov = ; // In case of time-varying measurement covariance

		//ROS_ERROR_STREAM(state.Pcov);
		/*** Limit queue size ***/
		if(pastStates.size()>100){
			pastStates.pop_front();
			//ROS_ERROR("Pop front");
		}
		pastStates.push_back(state);

		if(newDelayed && enableDelay)
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
			for(size_t i=0; i<newMeas.size(); ++i)	if ((newArrived = newMeas(i))) break;
			if (newArrived)	nav.correct(nav.update(measurements, newMeas));
		}

		newMeas.setZero(); // razmisli kako ovo srediti
		measDelay.setZero();
		publishState();

		//ROS_ERROR_STREAM(nav.getStateCovariance());

		/*** Send the base-link transform ***/
		/*geometry_msgs::TransformStamped transform;
		transform.transform.translation.x = nav.getState()(KFNav::xp);
		transform.transform.translation.y = nav.getState()(KFNav::yp);
		transform.transform.translation.z = nav.getState()(KFNav::zp);
		labust::tools::quaternionFromEulerZYX(0, 0, nav.getState()(KFNav::psi), transform.transform.rotation);
		transform.child_frame_id = "base_link";
		transform.header.frame_id = "local";
		transform.header.stamp = ros::Time::now();
		broadcaster.sendTransform(transform);*/

		rate.sleep();
		/*** Get current time (for delay calculation) ***/
		currentTime = ros::Time::now().toSec();
		ros::spinOnce();
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"nav_3d");
	Estimator3D nav;
	nav.start();
	return 0;
}


