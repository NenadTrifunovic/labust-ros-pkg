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
//#include <labust/navigation/EKF_3D_USBL.hpp>
//#include <labust/navigation/EKF_3D_USBLModel.hpp>
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
#include <navcon_msgs/ModelParamsUpdate.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <underwater_msgs/USBLFix.h>


#include <tf2_ros/transform_broadcaster.h>


#include <ros/ros.h>

#include <boost/bind.hpp>



#include <labust/navigation/GenericModel.hpp>
#include <labust/navigation/EstimatorContainer.hpp>
#include <labust/navigation/MeasurementHandler.hpp>


using namespace labust::navigation;





template <typename Model>
class EKFNode: public MeasurementHandler
{

	//typedef labust::navigation::GenericModel Model;

public:
	EKFNode();

	~EKFNode();

	void start();

	void reset();

	void onReset(const std_msgs::Bool::ConstPtr& reset);

	void onTau(const auv_msgs::BodyForceReq::ConstPtr& tau);

	void onModelUpdate(const navcon_msgs::ModelParamsUpdate::ConstPtr& update);



	void initializeEstimator();

	void configureModelParameters(ros::NodeHandle& nh);

	void processInput();

	void processMeasurement();

	void updateEstimator();

	void publishStateAndMeasurement();

	void publishTransform();

	void calculateConditionNumber();


	labust::navigation::EstimatorContainer<Model> _estimator;

	//labust::navigation::MeasurementHandler<labust::navigation::GenericModel> _measurement_handler;

	MeasurementHandler _measurement_handler;

	tf2_ros::TransformBroadcaster broadcaster;


};


template <typename Model>
EKFNode<Model>::EKFNode(){

}

template <typename Model>
EKFNode<Model>::~EKFNode(){

}



template <typename Model>
void EKFNode<Model>::initializeEstimator(){

	ros::NodeHandle nh, ph("~");
	double Ts(0.1);
	ph.param("Ts",Ts,Ts);
	ros::Rate rate(1/Ts);
	_estimator.setSamplingTime(Ts);
    configureModelParameters(nh);

}

template <typename Model>
void EKFNode<Model>::configureModelParameters(ros::NodeHandle& nh)
{
	ROS_INFO("Configure navigation.");

	labust::simulation::DynamicsParams params;
	labust::tools::loadDynamicsParams(nh, params);

	//Model model;
	typename Model::ModelParams param[6];


	ROS_INFO("Loaded dynamics params.");

	param[Model::X].alpha = params.m + params.Ma(0,0);
	param[Model::X].beta = params.Dlin(0,0);
	param[Model::X].betaa = params.Dquad(0,0);

	param[Model::Y].alpha = params.m + params.Ma(1,1);
	param[Model::Y].beta = params.Dlin(1,1);
	param[Model::Y].betaa = params.Dquad(1,1);

	param[Model::Z].alpha = params.m + params.Ma(2,2);
	param[Model::Z].beta = params.Dlin(2,2);
	param[Model::Z].betaa = params.Dquad(2,2);

	param[Model::K].alpha = params.Io(0,0) + params.Ma(3,3);
	param[Model::K].beta = params.Dlin(3,3);
	param[Model::K].betaa = params.Dquad(3,3);

	param[Model::M].alpha = params.Io(1,1) + params.Ma(4,4);
	param[Model::M].beta = params.Dlin(4,4);
	param[Model::M].betaa = params.Dquad(4,4);

	param[Model::N].alpha = params.Io(2,2) + params.Ma(5,5);
	param[Model::N].beta = params.Dlin(5,5);
	param[Model::N].betaa = params.Dquad(5,5);

	_estimator.setModelParameters(param[Model::X],param[Model::Y],
			param[Model::Z],param[Model::K],
			param[Model::M], param[Model::N]);

//	_estimator.initializeModel();
//
//	/*** Initialize estimator parameters ***/
//	labust::navigation::kfModelLoader(model, nh, "ekfnav_usbl");
//	_estimator.setEstimatorParameters(model);

}

template <typename Model>
void EKFNode<Model>::onReset(const std_msgs::Bool::ConstPtr& reset)
{
   if (reset->data)
   {

   }
}

template <typename Model>
void EKFNode<Model>::onTau(const auv_msgs::BodyForceReq::ConstPtr& tau)
{
	tauIn(Model::X) = tau->wrench.force.x;
	tauIn(Model::Y) = tau->wrench.force.y;
	tauIn(Model::Z) = tau->wrench.force.z;
	tauIn(Model::K) = tau->wrench.torque.x;
	tauIn(Model::M) = tau->wrench.torque.y;
	tauIn(Model::N) = tau->wrench.torque.z;
};

template <typename Model>
void EKFNode<Model>::onModelUpdate(const navcon_msgs::ModelParamsUpdate::ConstPtr& update)
{
//	ROS_INFO("Updating the model parameters for %d DoF.",update->dof);
//	params[update->dof].alpha = update->alpha;
//	if (update->use_linear)
//	{
//		params[update->dof].beta = update->beta;
//		params[update->dof].betaa = 0;
//	}
//	else
//	{
//		params[update->dof].beta = 0;
//		params[update->dof].betaa = update->betaa;
//	}
//	nav.setParameters(this->params[X],this->params[Y],
//			this->params[Z], this->params[K],
//			this->params[M],this->params[N]);
}

template <typename Model>
void EKFNode<Model>::start()
{
	initializeEstimator();
	ros::Rate rate(1/double(_estimator.getTs()));

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

template <typename Model>
void EKFNode<Model>::processMeasurement()
{
	_measurement_handler.updateMeasurements();
	//MeasurementHandler<Model>::updateMeasurements();
	this->updateMeasurements();

}
template <typename Model>
void EKFNode<Model>::processInput()
{
	//_measurement_handler.updateMeasurements();

}
template <typename Model>
void EKFNode<Model>::updateEstimator()
{
	_estimator.setInputVector(_measurement_handler.getMeasurementVector());
	_estimator.setMeasurementVector(_measurement_handler.getMeasurementVector(), _measurement_handler.getMeasurementTimeVector());
	_estimator.estimatorStep();


}

template <typename Model>
void EKFNode<Model>::publishTransform()
{
//	//Send the base-link transform
//	geometry_msgs::TransformStamped transform;
//	typename Model::vectorref cstate = _estimator.getState();
//	//Update DVL sensor
//	//if (updateDVL) dvl.current_r(cstate(Model::r));
//
//	//local -> base_pose
//	transform.transform.translation.x = cstate(Model::xp);
//	transform.transform.translation.y = cstate(Model::yp);
//	transform.transform.translation.z = cstate(Model::zp);
//	labust::tools::quaternionFromEulerZYX(0, 0, 0, transform.transform.rotation);
//
//	transform.child_frame_id = "base_pose_usbl_abs";
//	transform.header.frame_id = "local";
//	transform.header.stamp = ros::Time::now();
//	broadcaster.sendTransform(transform);
//
//	//local -> base_pose
//	transform.transform.translation.x = 0;
//	transform.transform.translation.y = 0;
//	transform.transform.translation.z = 0;
//	labust::tools::quaternionFromEulerZYX(cstate(Model::phi),
//			cstate(Model::theta),
//			cstate(Model::psi),
//			transform.transform.rotation);
//
//	transform.child_frame_id = "base_link_usbl_abs";
//	transform.header.frame_id = "base_pose_usbl";
//	broadcaster.sendTransform(transform);
}
template <typename Model>
void EKFNode<Model>::publishStateAndMeasurement()
{
//	auv_msgs::NavSts::Ptr state(new auv_msgs::NavSts());
//	const Model::vector& estimate = nav.getState();
//	state->body_velocity.x = estimate(Model::u);
//	state->body_velocity.y = estimate(Model::v);
//	state->body_velocity.z = estimate(Model::w);
//
//	Eigen::Matrix2d R;
//	double yaw = labust::math::wrapRad(estimate(Model::psi));
//	R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
//	Eigen::Vector2d in, out;
//	in << estimate(Model::xc), estimate(Model::yc);
//	out = R.transpose()*in;
//
//	state->gbody_velocity.x = estimate(Model::u) + out(0);
//	state->gbody_velocity.y = estimate(Model::v) + out(1);
//	state->gbody_velocity.z = estimate(Model::w);
//
//	state->orientation_rate.roll = estimate(Model::p);
//	state->orientation_rate.pitch = estimate(Model::q);
//	state->orientation_rate.yaw = estimate(Model::r);
//
//	state->position.north = estimate(Model::xp);
//	state->position.east = estimate(Model::yp);
//	state->position.depth = estimate(Model::zp);
//	state->altitude = estimate(Model::altitude);
//
//	state->orientation.roll = estimate(Model::phi);
//	state->orientation.pitch = estimate(Model::theta);
//	state->orientation.yaw = labust::math::wrapRad(estimate(Model::psi));
//
//	state->origin.latitude = gps.origin().first;
//  state->origin.longitude = gps.origin().second;
//  proj.Reset(state->origin.latitude, state->origin.longitude, gps.origin_h());
//	Eigen::Quaternion<double> qrot;
//	labust::tools::quaternionFromEulerZYX(M_PI,0,M_PI/2,qrot);
//	Eigen::Vector3d ned;
//	ned<<state->position.north,
//			state->position.east,
//			state->position.depth;
//	double h;
//	Eigen::Vector3d enu =  qrot.toRotationMatrix().transpose()*ned;
//  proj.Reverse(enu(0),
//  		enu(1),
//			enu(2),
//			state->global_position.latitude,
//			state->global_position.longitude,h);
//
//	const Model::matrix& covariance = nav.getStateCovariance();
//	state->position_variance.north = covariance(Model::xp, Model::xp);
//	state->position_variance.east = covariance(Model::yp, Model::yp);
//	state->position_variance.depth = covariance(Model::zp,Model::zp);
//	state->orientation_variance.roll =  covariance(Model::phi, Model::phi);
//	state->orientation_variance.pitch =  covariance(Model::theta, Model::theta);
//	state->orientation_variance.yaw =  covariance(Model::psi, Model::psi);
//
//	state->header.stamp = ros::Time::now();
//	state->header.frame_id = "local";
//	stateHat.publish(state);
//
//	geometry_msgs::TwistStamped::Ptr current(new geometry_msgs::TwistStamped());
//	current->twist.linear.x = estimate(Model::xc);
//	current->twist.linear.y = estimate(Model::yc);
//	current->header.stamp = ros::Time::now();
//	current->header.frame_id = "local";
//	currentsHat.publish(current);
//
//	std_msgs::Float32::Ptr buoyancy(new std_msgs::Float32());
//	buoyancy->data = estimate(Model::buoyancy);
//	buoyancyHat.publish(buoyancy);
//
//	std_msgs::Float32::Ptr turns(new std_msgs::Float32());
//	turns->data = estimate(Model::psi)/(2*M_PI);
//	turns_pub.publish(turns);
//
//	std_msgs::Float32::Ptr altcov(new std_msgs::Float32());
//	altcov->data = covariance(Model::altitude, Model::altitude);
//	altitude_cov.publish(altcov);
//
//	/** Observability metric */
//        calculateConditionNumber();
}
template <typename Model>
void EKFNode<Model>::calculateConditionNumber()
{
//	Model::matrix P = nav.getStateCovariance();
//
//	Eigen::JacobiSVD<Eigen::MatrixXd> svd(P);
//	double cond1 = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
//
//	Eigen::Matrix2d Pxy;
//	Pxy << P(Model::xp,Model::xp), P(Model::xp,Model::yp), P(Model::yp,Model::xp), P(Model::yp,Model::yp);
//
//
//	//double P(1,0)*P(0,1);
//
//	//ROS_ERROR_STREAM(Pxy);
//	double traceP = Pxy.trace();
//	double detP = Pxy.determinant();
//
//	double condCost = std::sqrt(traceP*traceP-4*detP);
//
//	Eigen::JacobiSVD<Eigen::MatrixXd> svd2(Pxy);
//	double cond2 = svd2.singularValues()(0) / svd2.singularValues()(svd2.singularValues().size()-1);
//
//	std_msgs::Float32::Ptr data(new std_msgs::Float32);
//
//	data->data = cond1;
////	pubCondP.publish(data);
//	data->data = cond2;
////	pubCondPxy.publish(data);
//	data->data = condCost;
////	pubCost.publish(data);
}



/*********************************************************************
 *** Main loop
 ********************************************************************/


int main(int argc, char* argv[])
{
	ros::init(argc,argv,"navigation");
	EKFNode<GenericModel> nav;
	nav.start();
	return 0;
}


