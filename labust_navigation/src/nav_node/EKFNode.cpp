/*********************************************************************
 *  EKFNode.cpp
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
#include <labust/navigation/EKF_3D_USBL.hpp>
#include <labust/navigation/EKF_3D_USBLModel.hpp>
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
#include <math.h>


#include <labust/navigation/GenericModel.hpp>
#include <labust/navigation/EstimatorContainer.hpp>
#include <labust/navigation/MeasurementHandler.hpp>


using namespace labust::navigation;






class EKFNode{

public:
	EKFNode();

	~EKFNode();

	void start();

	void reset();

	void initializeEstimator();

	void configureModelParameters(KFNav& nav, ros::NodeHandle& nh)

	void processInput();

	void processMeasurement();

	void updateEstimator();

	void publishStateAndMeasurement();

	void publishTransform();

	labust::navigation::EstimatorContainer<labust::navigation::GenericModel> _estimator;

	labust::navigation::MeasurementHandler<labust::navigation::GenericModel> _measurement_handler;

	enum {u=0,v,w,p,q,r,xp,yp,zp,phi,theta,psi,xc,yc,b,buoyancy,roll_restore,pitch_restore,altitude,xb,yb,zb,stateNum};
	enum {X=0,Y,Z,Kroll,M,N,inputSize};
	enum {range=stateNum,bearing,elevation,measSize};



};

void EKFNode::initializeEstimator(){

	ros::NodeHandle ph("~");
	double Ts(0.1);
	ph.param("Ts",Ts,Ts);
	ros::Rate rate(1/Ts);
	nav.setTs(Ts);

	configureModelParameters();

}

void EKFNode::configureModelParameters(KFNav& nav, ros::NodeHandle& nh)
{
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
	labust::navigation::kfModelLoader(nav, nh, "ekfnav_usbl");
}

void EKFNode::onReset(const std_msgs::Bool::ConstPtr& reset)
{
   if (reset->data)
   {

   }
}

void Estimator3D::onTau(const auv_msgs::BodyForceReq::ConstPtr& tau)
{
	tauIn(KFNav::X) = tau->wrench.force.x;
	tauIn(KFNav::Y) = tau->wrench.force.y;
	tauIn(KFNav::Z) = tau->wrench.force.z;
	tauIn(KFNav::Kroll) = tau->wrench.torque.x;
	tauIn(KFNav::M) = tau->wrench.torque.y;
	tauIn(KFNav::N) = tau->wrench.torque.z;
};

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


void EKFNode::start()
{
	initializeEstimator();
	ros::Rate rate(1/nav.getTs);

	while (ros::ok())
	{
		/*** Process inputs ***/
		processInput();
		/*** Process measurements ***/
		processMeasurement();
		/*** Process measurements ***/
		updateEstimator();
		/*** Publish state and measurement vector ***/
		publishStateAndMeasurement();
		/*** Publish transform ***/
		publishTransform();
		/*** Sleep and call spinner ***/
		rate.sleep();
		ros::spinOnce();
	}
}


void EKFNode::processMeasurement()
{
	_measurement_handler.updateMeasurements();
}

void EKFNode::processInput()
{
	//_measurement_handler.updateMeasurements();
}

void EKFNode::updateEstimator()
{
	_estimator.setInputVector(_measurement_handler.getMeasurementVector());
	_estimator.setMeasurementVector(_measurement_handler.getMeasurementVector());
	_estimator.estimatorStep();


}


void EKFNode::publishTransform()
{
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

	transform.child_frame_id = "base_pose_usbl_abs";
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

	transform.child_frame_id = "base_link_usbl_abs";
	transform.header.frame_id = "base_pose_usbl";
	broadcaster.sendTransform(transform);
}

void EKFNode::publishStateAndMeasurement()
{
	auv_msgs::NavSts::Ptr state(new auv_msgs::NavSts());
	const KFNav::vector& estimate = nav.getState();
	state->body_velocity.x = estimate(KFNav::u);
	state->body_velocity.y = estimate(KFNav::v);
	state->body_velocity.z = estimate(KFNav::w);

	Eigen::Matrix2d R;
	double yaw = labust::math::wrapRad(estimate(KFNav::psi));
	R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
	Eigen::Vector2d in, out;
	in << estimate(KFNav::xc), estimate(KFNav::yc);
	out = R.transpose()*in;

	state->gbody_velocity.x = estimate(KFNav::u) + out(0);
	state->gbody_velocity.y = estimate(KFNav::v) + out(1);
	state->gbody_velocity.z = estimate(KFNav::w);

	state->orientation_rate.roll = estimate(KFNav::p);
	state->orientation_rate.pitch = estimate(KFNav::q);
	state->orientation_rate.yaw = estimate(KFNav::r);

	state->position.north = estimate(KFNav::xp);
	state->position.east = estimate(KFNav::yp);
	state->position.depth = estimate(KFNav::zp);
	state->altitude = estimate(KFNav::altitude);

	state->orientation.roll = estimate(KFNav::phi);
	state->orientation.pitch = estimate(KFNav::theta);
	state->orientation.yaw = labust::math::wrapRad(estimate(KFNav::psi));

	state->origin.latitude = gps.origin().first;
  state->origin.longitude = gps.origin().second;
  proj.Reset(state->origin.latitude, state->origin.longitude, gps.origin_h());
	Eigen::Quaternion<double> qrot;
	labust::tools::quaternionFromEulerZYX(M_PI,0,M_PI/2,qrot);
	Eigen::Vector3d ned;
	ned<<state->position.north,
			state->position.east,
			state->position.depth;
	double h;
	Eigen::Vector3d enu =  qrot.toRotationMatrix().transpose()*ned;
  proj.Reverse(enu(0),
  		enu(1),
			enu(2),
			state->global_position.latitude,
			state->global_position.longitude,h);

	const KFNav::matrix& covariance = nav.getStateCovariance();
	state->position_variance.north = covariance(KFNav::xp, KFNav::xp);
	state->position_variance.east = covariance(KFNav::yp, KFNav::yp);
	state->position_variance.depth = covariance(KFNav::zp,KFNav::zp);
	state->orientation_variance.roll =  covariance(KFNav::phi, KFNav::phi);
	state->orientation_variance.pitch =  covariance(KFNav::theta, KFNav::theta);
	state->orientation_variance.yaw =  covariance(KFNav::psi, KFNav::psi);

	state->header.stamp = ros::Time::now();
	state->header.frame_id = "local";
	stateHat.publish(state);

	geometry_msgs::TwistStamped::Ptr current(new geometry_msgs::TwistStamped());
	current->twist.linear.x = estimate(KFNav::xc);
	current->twist.linear.y = estimate(KFNav::yc);
	current->header.stamp = ros::Time::now();
	current->header.frame_id = "local";
	currentsHat.publish(current);

	std_msgs::Float32::Ptr buoyancy(new std_msgs::Float32());
	buoyancy->data = estimate(KFNav::buoyancy);
	buoyancyHat.publish(buoyancy);

	std_msgs::Float32::Ptr turns(new std_msgs::Float32());
	turns->data = estimate(KFNav::psi)/(2*M_PI);
	turns_pub.publish(turns);

	std_msgs::Float32::Ptr altcov(new std_msgs::Float32());
	altcov->data = covariance(KFNav::altitude, KFNav::altitude);
	altitude_cov.publish(altcov);

	/** Observability metric */
        calculateConditionNumber();
}

/*********************************************************************
 *** Model data
 ********************************************************************/

void GenericModel::step(const input_type& input)
{
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

	  ////!!!!!!!!!!!!!!!!!!!!!!xk_1 = x;

	  ///!!!!!!!!!!!!!11derivativeAW();
}

void GenericModel::derivativeAX()
{
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
void GenericModel::derivativeAW()
{

}
void GenericModel::derivativeHX()
{

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
void GenericModel::derivativeHV()
{

}

/*********************************************************************
 *** Main loop
 ********************************************************************/


int main(int argc, char* argv[])
{
	ros::init(argc,argv,"nav_3d");
	Estimator3D nav;
	nav.start();
	return 0;
}


