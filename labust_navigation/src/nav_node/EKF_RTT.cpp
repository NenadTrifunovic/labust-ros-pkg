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
 *  Author : Dula Nad
 *  Created: 26.03.2013.
 *********************************************************************/
#include <labust/navigation/EKF_RTT.hpp>
#include <labust/navigation/RelativeTrackingModel.hpp>

#include <labust/tools/GeoUtilities.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/simulation/DynamicsParams.hpp>
#include <labust/navigation/KFModelLoader.hpp>


#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyForceReq.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <underwater_msgs/USBLFix.h>

#include <ros/ros.h>

#include <boost/bind.hpp>

using namespace labust::navigation;

Estimator3D::Estimator3D():
		inputVec(KFNav::vector::Zero(KFNav::inputSize)),
		measurements(KFNav::vector::Zero(KFNav::stateNum)),
		newMeas(KFNav::vector::Zero(KFNav::measSize)),
		alt(0),
		xc(0),
		yc(0){this->onInit();};

void Estimator3D::onInit()
{
	ros::NodeHandle nh, ph("~");
	//Configure the navigation
	configureNav(nav,nh);

	/* Publishers */
	pubStateHat = nh.advertise<auv_msgs::NavSts>("stateHatRelative",1);
	pubStateMeas = nh.advertise<auv_msgs::NavSts>("measRelative",1);

	/* Subscribers */
	subStateHat = nh.subscribe<auv_msgs::NavSts>("stateHat", 1, &Estimator3D::onStateHat, this);
	subCurrentsHat = nh.subscribe<geometry_msgs::TwistStamped>("currentsHat", 1, &Estimator3D::onCurrentsHat, this);
	subTargetTau = nh.subscribe<auv_msgs::BodyForceReq>("bla", 1, &Estimator3D::onTargetTau, this);
	subTargetDepth = nh.subscribe<std_msgs::Float32>("depth", 1,	&Estimator3D::onTargetDepth, this);
	subTargetHeading = nh.subscribe<std_msgs::Float32>("heading", 1, &Estimator3D::onTargetHeading, this);
	subUSBLfix = nh.subscribe<underwater_msgs::USBLFix>("usbl_fix", 1, &Estimator3D::onUSBLfix, this);
	modelUpdate = nh.subscribe<navcon_msgs::ModelParamsUpdate>("model_update", 1, &Estimator3D::onModelUpdate,this);
}

void Estimator3D::configureNav(KFNav& nav, ros::NodeHandle& nh){

	ROS_INFO("Configure navigation.");

	labust::simulation::DynamicsParams params;
	labust::tools::loadDynamicsParams(nh, params);

	ROS_INFO("Loaded dynamics params.");

	this->params[X].alpha = params.m + params.Ma(0,0);
	this->params[X].beta = params.Dlin(0,0);
	this->params[X].betaa = params.Dquad(0,0);

	this->params[Y].alpha = params.m + params.Ma(1,1);
	this->params[Y].beta = params.Dlin(1,1);
	this->params[Y].betaa = params.Dquad(1,1);

	this->params[Z].alpha = params.m + params.Ma(2,2);
	this->params[Z].beta = params.Dlin(2,2);
	this->params[Z].betaa = params.Dquad(2,2);

	this->params[K].alpha = params.Io(0,0) + params.Ma(3,3);
	this->params[K].beta = params.Dlin(3,3);
	this->params[K].betaa = params.Dquad(3,3);

	this->params[M].alpha = params.Io(1,1) + params.Ma(4,4);
	this->params[M].beta = params.Dlin(4,4);
	this->params[M].betaa = params.Dquad(4,4);

	this->params[N].alpha = params.Io(2,2) + params.Ma(5,5);
	this->params[N].beta = params.Dlin(5,5);
	this->params[N].betaa = params.Dquad(5,5);

	nav.setParameters(this->params[X], this->params[Y],
			this->params[Z], this->params[K],
			this->params[M], this->params[N]);

	nav.initModel();
	labust::navigation::kfModelLoader(nav, nh, "ekfnav_rtt");
}

void Estimator3D::onModelUpdate(const navcon_msgs::ModelParamsUpdate::ConstPtr& update)
{
	ROS_INFO("Updating the model parameters for %d DoF.",update->dof);
	params[update->dof].alpha = update->alpha;
	if (update->use_linear)
	{
		params[update->dof].beta = update->beta;
		params[update->dof].betaa = 0;
	}
	else
	{
		params[update->dof].beta = 0;
		params[update->dof].betaa = update->betaa;
	}
	nav.setParameters(this->params[X],this->params[Y],
			this->params[Z], this->params[K],
			this->params[M],this->params[N]);
}

void Estimator3D::onStateHat(const auv_msgs::NavSts::ConstPtr& data){

	u = data->body_velocity.x;
	v = data->body_velocity.y;

	inputVec(KFNav::psi) = data->orientation.yaw;
}

void Estimator3D::onCurrentsHat(const  geometry_msgs::TwistStamped::ConstPtr& data){

	xc = data->twist.linear.x;
	yc = data->twist.linear.y;
}

void Estimator3D::onTargetTau(const auv_msgs::BodyForceReq::ConstPtr& tau)
{
	inputVec(KFNav::X) = tau->wrench.force.x;
	inputVec(KFNav::Z) = tau->wrench.force.z;
	inputVec(KFNav::N) = tau->wrench.torque.z;
}

void Estimator3D::onTargetDepth(const std_msgs::Float32::ConstPtr& data)
{
	measurements(KFNav::depth) = data->data;
	newMeas(KFNav::depth) = 1;
}

void Estimator3D::onTargetHeading(const std_msgs::Float32::ConstPtr& data){

	measurements(KFNav::psi_t) = data->data;
	newMeas(KFNav::psi_t) = 1;
}

void Estimator3D::onUSBLfix(const underwater_msgs::USBLFix::ConstPtr& data){



	measurements(KFNav::d) = data->range;
	newMeas(KFNav::d) = 1;

	measurements(KFNav::theta) = data->bearing;
	newMeas(KFNav::theta) = 1;

	measurements(KFNav::delta_xm) = data->relative_position.x;
	newMeas(KFNav::delta_xm) = 1;

	measurements(KFNav::delta_ym) = data->relative_position.y;
	newMeas(KFNav::delta_ym) = 1;

	measurements(KFNav::delta_zm) = data->relative_position.z;
	newMeas(KFNav::delta_zm) = 1;
}


void Estimator3D::processMeasurements()
{

	//Publish measurements
	//auv_msgs::NavSts::Ptr meas(new auv_msgs::NavSts());
	//meas->body_velocity.x = measurements(KFNav::u);
	//meas->body_velocity.y = measurements(KFNav::v);
	//meas->body_velocity.z = measurements(KFNav::w);

	//meas->position.north = measurements(KFNav::xp);
	//meas->position.east = measurements(KFNav::yp);
	//meas->position.depth = measurements(KFNav::zm);
	//meas->altitude = measurements(KFNav::altitude);

	//meas->orientation.roll = measurements(KFNav::phi);
	//meas->orientation.pitch = measurements(KFNav::theta);
	//meas->orientation.yaw = labust::math::wrapRad(measurements(KFNav::psim));
	//if (useYawRate)	meas->orientation_rate.yaw = measurements(KFNav::r);

	//meas->origin.latitude = gps.origin().first;
	//meas->origin.longitude = gps.origin().second;
	//meas->global_position.latitude = gps.latlon().first;
	//meas->global_position.longitude = gps.latlon().second;

	//meas->header.stamp = ros::Time::now();
	//meas->header.frame_id = "local";
	//stateMeas.publish(meas);
}

void Estimator3D::publishState(){

	auv_msgs::NavSts::Ptr state(new auv_msgs::NavSts());
	const KFNav::vector& estimate = nav.getState();

	state->body_velocity.x = estimate(KFNav::u_t);
	state->body_velocity.z = estimate(KFNav::w_t);

	state->orientation_rate.yaw = estimate(KFNav::r_t);

	state->position.north = estimate(KFNav::delta_x);
	state->position.east = estimate(KFNav::delta_y);
	state->position.depth = estimate(KFNav::delta_z);

	state->orientation.yaw = labust::math::wrapRad(estimate(KFNav::psi_t));

	const KFNav::matrix& covariance = nav.getStateCovariance();
	state->position_variance.north = covariance(KFNav::delta_x, KFNav::delta_x);
	state->position_variance.east = covariance(KFNav::delta_y, KFNav::delta_y);
	state->position_variance.depth = covariance(KFNav::delta_z, KFNav::delta_z);

	state->orientation_variance.yaw =  covariance(KFNav::psi_t, KFNav::psi_t);

	state->header.stamp = ros::Time::now();
	state->header.frame_id = "local";
	pubStateHat.publish(state);
}

void Estimator3D::start(){

	ros::NodeHandle ph("~");
	double Ts(0.1);
	ph.param("Ts",Ts,Ts);
	ros::Rate rate(1/Ts);
	nav.setTs(Ts);

	while (ros::ok()){

		inputVec(KFNav::x_dot) = xc + u*cos(inputVec(KFNav::psi)) - v*sin(inputVec(KFNav::psi));
		inputVec(KFNav::y_dot) = yc + u*sin(inputVec(KFNav::psi)) + v*cos(inputVec(KFNav::psi));

		nav.predict(inputVec);
		processMeasurements();
		bool newArrived(false);
		for(size_t i=0; i<newMeas.size(); ++i)	if ((newArrived = newMeas(i))) break;
		if (newArrived)	nav.correct(nav.update(measurements, newMeas));

		publishState();

		//Send the base-link transform
		geometry_msgs::TransformStamped transform;
		transform.transform.translation.x = nav.getState()(KFNav::delta_x);
		transform.transform.translation.y = nav.getState()(KFNav::delta_y);
		transform.transform.translation.z = nav.getState()(KFNav::delta_z);

		labust::tools::quaternionFromEulerZYX(0.0,
						0.0,
						nav.getState()(KFNav::psi),
						transform.transform.rotation);

		transform.child_frame_id = "base_link_relative";
		transform.header.frame_id = "local";
		transform.header.stamp = ros::Time::now();
		broadcaster.sendTransform(transform);

		rate.sleep();
		ros::spinOnce();
	}
}

int main(int argc, char* argv[]){

	ros::init(argc,argv,"nav_sb");
	Estimator3D nav;
	nav.start();
	return 0;
}


