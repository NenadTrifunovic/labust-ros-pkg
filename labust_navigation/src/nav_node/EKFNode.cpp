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


using namespace labust::navigation;



Estimator3D::Estimator3D():

		tauIn(KFNav::vector::Zero(KFNav::inputSize)),
		measurements(KFNav::vector::Zero(KFNav::measSize)),
		newMeas(KFNav::vector::Zero(KFNav::measSize)),
		measDelay(KFNav::vector::Zero(KFNav::measSize)),
		alt(0),
		useYawRate(false),
		enableDelay(false),
		enableRange(true),
		enableBearing(true),
		enableElevation(false),
		delayTime(0.0),
		delay_time(0.0),
		dvl_model(0),
		enableRejection(false),
		compassVariance(0.3),
		gyroVariance(0.003),
		OR(3,0.95),
		absoluteEKF(false),
		max_dvl(2.0),
		min_altitude(0.5),
		dvl_timeout(5),
		dvl_time(ros::Time::now()),
		alt_sample(true),
		nsamples_alt(10),
		altok_var(2),
		dvl_fp(0.1){this->onInit();};

void Estimator3D::onInit()
{
	ros::NodeHandle nh, ph("~");

	/*** Configure the navigation ***/
	configureNav(nav,nh);

	/*** Publishers ***/
	stateHat = nh.advertise<auv_msgs::NavSts>("stateHat",1);
	stateMeas = nh.advertise<auv_msgs::NavSts>("meas",1);
	currentsHat = nh.advertise<geometry_msgs::TwistStamped>("currentsHat",1);
	buoyancyHat = nh.advertise<std_msgs::Float32>("buoyancy",1);

	turns_pub = nh.advertise<std_msgs::Float32>("turns",1);
	unsafe_dvl = nh.advertise<std_msgs::Bool>("unsafe_dvl",1);
	altitude_cov = nh.advertise<std_msgs::Float32>("altitude_cov",1);

	pubRange = nh.advertise<std_msgs::Float32>("range",1);
	pubRangeFiltered = nh.advertise<std_msgs::Float32>("range_filtered",1);
	pubwk = nh.advertise<std_msgs::Float32>("w_limit",1);
	pubCondP = nh.advertise<std_msgs::Float32>("condP",1);
	pubCondPxy = nh.advertise<std_msgs::Float32>("condPxy",1);
	pubCost = nh.advertise<std_msgs::Float32>("cost",1);

	/*** Subscribers ***/
	tauAch = nh.subscribe<auv_msgs::BodyForceReq>("tauAch", 1, &Estimator3D::onTau,this);
	depth = nh.subscribe<std_msgs::Float32>("depth", 1,	&Estimator3D::onDepth, this);
	altitude = nh.subscribe<std_msgs::Float32>("altitude", 1, &Estimator3D::onAltitude, this);
	modelUpdate = nh.subscribe<navcon_msgs::ModelParamsUpdate>("model_update", 1, &Estimator3D::onModelUpdate,this);
	resetTopic = nh.subscribe<std_msgs::Bool>("reset_nav_covariance", 1, &Estimator3D::onReset,this);
	useGyro = nh.subscribe<std_msgs::Bool>("use_gyro", 1, &Estimator3D::onUseGyro,this);
	subUSBL = nh.subscribe<underwater_msgs::USBLFix>("usbl_fix", 1, &Estimator3D::onUSBLfix,this);
	sub = nh.subscribe<auv_msgs::NED>("quad_delta_pos", 1, &Estimator3D::deltaPosCallback,this);
	subKFmode = nh.subscribe<std_msgs::Bool>("KFmode", 1, &Estimator3D::KFmodeCallback, this);

	useAltSampling = nh.subscribe<std_msgs::Bool>("use_alt_sampling", 1, &Estimator3D::onAltSampling,this);
    altNSample = nh.subscribe<std_msgs::Int32>("alt_n_samples", 1, &Estimator3D::onAltNSamples,this);


	//subSecond_heading = nh.subscribe<std_msgs::Float32>("out_acoustic_heading", 1, &Estimator3D::onSecond_heading,this);
	subSecond_position = nh.subscribe<geometry_msgs::Point>("out_acoustic_position", 1, &Estimator3D::onSecond_position,this);
	//subSecond_speed = nh.subscribe<std_msgs::Float32>("out_acoustic_speed", 1, &Estimator3D::onSecond_speed,this);
	//subSecond_usbl_fix = nh.subscribe<underwater_msgs::USBLFix>("usbl_fix", 1, &Estimator3D::onSecond_usbl_fix,this);

	KFmode = quadMeasAvailable = false;

	/*** Get DVL model ***/
	ph.param("dvl_model",dvl_model, dvl_model);
	nav.useDvlModel(dvl_model);
	ph.param("imu_with_yaw_rate",useYawRate,useYawRate);
	ph.param("compass_variance",compassVariance,compassVariance);
	ph.param("gyro_variance",gyroVariance,gyroVariance);

	ph.param("max_dvl",max_dvl,max_dvl);
	ph.param("min_altitude", min_altitude, min_altitude);
	ph.param("dvl_timeout", dvl_timeout, dvl_timeout);
	double trustf(0);
	ph.param("dvl_rot_trust_factor", trustf, trustf);
	double sway_corr(0);
	ph.param("absoluteEKF", absoluteEKF,absoluteEKF);
	ph.param("altitude_cov_timeout",altitude_timeout, altitude_timeout);
	ph.param("altitude_sampling",alt_sample, alt_sample);

	/*** Enable USBL measurements ***/
	ph.param("delay", enableDelay, enableDelay);
	ph.param("delay_time", delay_time, delay_time);
	ph.param("range", enableRange, enableRange);
	ph.param("bearing", enableBearing, enableBearing);
	ph.param("elevation", enableElevation, enableElevation);
	//ph.param("delayTime", delayTime, delayTime);

	/** Enable outlier rejection */
	ph.param("meas_outlier_rejection", enableRejection, enableRejection);

	/*** Configure handlers. ***/
	gps.configure(nh);
	dvl.configure(nh);
	imu.configure(nh);

	/*** Get initial error covariance. ***/
    Pstart = nav.getStateCovariance();

    nav.setDVLRotationTrustFactor(trustf);

    //Setup sway correction
    bool use_sc(false);
    double acc_port(0.3), acc_starboard(0.3),
  	vec_port(0.07), vec_starboard(0.07);
    ph.param("use_sc",use_sc, use_sc);
    ph.param("acc_port",acc_port, acc_port);
    ph.param("acc_starboard",acc_starboard, acc_starboard);
    ph.param("vec_port",vec_port, vec_port);
    ph.param("vec_starboard",vec_starboard, vec_starboard);
    nav.setSwayCorrection(use_sc, acc_port, acc_starboard,
    		vec_port, vec_starboard);
    //Rstart = nav.R;
    //ROS_ERROR("NAVIGATION");
    ph.param("dvl_fp",dvl_fp, dvl_fp);
}

void Estimator3D::onReset(const std_msgs::Bool::ConstPtr& reset)
{
   if (reset->data)
   {
      nav.setStateCovariance(10000*KFNav::matrix::Identity(KFNav::stateNum, KFNav::stateNum));
   }
}

void Estimator3D::onUseGyro(const std_msgs::Bool::ConstPtr& use_gyro)
{
   if (use_gyro->data)
   {
      nav.R0(KFNav::psi, KFNav::psi) = gyroVariance;
      ROS_INFO("Switch to using gyro measurements.");
   }
   else
   {
      nav.R0(KFNav::psi, KFNav::psi) = compassVariance;
      ROS_INFO("Switch to using compass measurements.");
   }
}

void Estimator3D::onAltSampling(const std_msgs::Bool::ConstPtr& flag)
{
	alt_sample = flag->data;
}

void Estimator3D::onAltNSamples(const std_msgs::Int32::ConstPtr& samples)
{
	nsamples_alt = samples->data;
}

void Estimator3D::configureNav(KFNav& nav, ros::NodeHandle& nh)
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

void Estimator3D::onTau(const auv_msgs::BodyForceReq::ConstPtr& tau)
{
	tauIn(KFNav::X) = tau->wrench.force.x;
	tauIn(KFNav::Y) = tau->wrench.force.y;
	tauIn(KFNav::Z) = tau->wrench.force.z;
	tauIn(KFNav::Kroll) = tau->wrench.torque.x;
	tauIn(KFNav::M) = tau->wrench.torque.y;
	tauIn(KFNav::N) = tau->wrench.torque.z;
};

/*********************************************************************
 *** Measurement callback
 ********************************************************************/

void Estimator3D::onDepth(const std_msgs::Float32::ConstPtr& data)
{
	boost::mutex::scoped_lock l(meas_mux);
	measurements(KFNav::zp) = data->data;
	newMeas(KFNav::zp) = 1;
};

void Estimator3D::onAltitude(const std_msgs::Float32::ConstPtr& data)
{
	boost::mutex::scoped_lock l(meas_mux);
	measurements(KFNav::altitude) = data->data;
	//Skip measurement if minimum altitude is encountered
	if (data->data <= min_altitude) return;
	//Dismiss false altitude
	if (fabs(data->data-nav.getState()(KFNav::altitude)) < 3*nav.calculateAltInovationVariance(nav.getStateCovariance()))
	{
		double malt = data->data;
		bool old = (ros::Time::now() - last_alt).toSec() > altitude_timeout;
		//In case no measurements were received for a long time
		if (old && alt_sample)
		{
			//Add to buffer
			altbuf.push_back(malt);
			//Keep n samples
			if (altbuf.size() > nsamples_alt) altbuf.erase(altbuf.begin());
			//Do not accept the measurement before the sample container is full.
			if (altbuf.size() < nsamples_alt) return;

			//Calculate variance
			double var = pow(labust::math::std2(altbuf),2);
			malt = labust::math::mean(altbuf);
			ROS_INFO("Var: %f, mean: %f", var, malt);
			if (var > altok_var) return;
			altbuf.clear();
		}

		last_alt = ros::Time::now();
		newMeas(KFNav::altitude) = 1;
		alt = malt;
		ROS_DEBUG("Accepted altitude: meas=%f, estimate=%f, variance=%f",
			data->data, nav.getState()(KFNav::altitude), 3*nav.calculateAltInovationVariance(nav.getStateCovariance()));
	}
	else
	{
		ROS_INFO("Dissmissed altitude: meas=%f, estimate=%f, variance=%f",
			data->data, nav.getState()(KFNav::altitude), 10*nav.calculateAltInovationVariance(nav.getStateCovariance()));
	}
};

void Estimator3D::onUSBLfix(const underwater_msgs::USBLFix::ConstPtr& data){

	/*** Calculate measurement delay ***/
	//double delay = calculateDelaySteps(data->header.stamp.toSec(), currentTime);
	double delay = double(calculateDelaySteps(currentTime-delay_time, currentTime));

//	double bear = 360 - data->bearing;
//	double elev = 180 - data->elevation;
	double bear = data->bearing;
	double elev = data->elevation;

	const KFNav::vector& x = nav.getState();
	//labust::math::wrapRad(measurements(KFNav::psi));

	ROS_ERROR("RANGE: %f, BEARING: %f deg %f rad", data->range,bear, labust::math::wrapRad(bear*M_PI/180+x(KFNav::psi)));
	/*** Get USBL measurements ***/
	measurements(KFNav::range) = (data->range > 0.1)?data->range:0.1;
	newMeas(KFNav::range) = enableRange;
	measDelay(KFNav::range) = delay;

	/*Eigen::VectorXd input(Eigen::VectorXd::Zero(3));
	//Eigen::VectorXd output(Eigen::VectorXd::Zero(1));


	input << x(KFNav::xp)-x(KFNav::xb), x(KFNav::yp)-x(KFNav::yb), x(KFNav::zp)-x(KFNav::zb);
	//output << measurements(KFNav::range);
	double y_filt, sigma, w;
	OR.step(input, measurements(KFNav::range), &y_filt, &sigma, &w);
	ROS_INFO("Finished outlier rejection");

	std_msgs::Float32 rng_msg;
	ROS_ERROR("w: %f",w);

	double w_limit =  0.25*std::sqrt(float(sigma));
	w_limit = (w_limit>1.0)?1.0:w_limit;
	if(w>w_limit)
	{
		rng_msg.data = data->range;
		pubRangeFiltered.publish(rng_msg);
	}
	//std_msgs::Float32 rng_msg;

	rng_msg.data = w_limit;
	pubwk.publish(rng_msg);

	rng_msg.data = data->range;
	pubRange.publish(rng_msg);*/

	//measurements(KFNav::bearing) = labust::math::wrapRad(bear*M_PI/180+x(KFNav::psi));
//	measurements(KFNav::bearing) = labust::math::wrapRad(bear*M_PI/180);
	measurements(KFNav::bearing) = bear;
	newMeas(KFNav::bearing) = enableBearing;
	measDelay(KFNav::bearing) = delay;

	measurements(KFNav::elevation) = elev*M_PI/180;
	newMeas(KFNav::elevation) = enableElevation;
	measDelay(KFNav::elevation) = delay;

	/*** Get beacon position ***/
	measurements(KFNav::xb) = 0;
	newMeas(KFNav::xb) = 1;
	measDelay(KFNav::xb) = delay;

	measurements(KFNav::yb) = 0;
	newMeas(KFNav::yb) = 1;
	measDelay(KFNav::yb) = delay;

	measurements(KFNav::zb) = 0;
	newMeas(KFNav::zb) = 1;
	measDelay(KFNav::zb) = delay;

	// Debug print
	//ROS_ERROR("Delay: %f", delay);
	//ROS_ERROR("Range: %f, bearing: %f, elevation: %f", measurements(KFNav::range), measurements(KFNav::bearing), measurements(KFNav::elevation));
	//ROS_ERROR("ENABLED Range: %d, bearing: %d, elevation: %d", enableRange, enableBearing, enableElevation);


	// Relativna mjerenja ukljuciti u model mjerenja...

	//measurements(KFNav::xb) = x(KFNav::xp) - data->relative_position.x;
	//newMeas(KFNav::xb) = 1;

	//measurements(KFNav::yb) = x(KFNav::yp) - data->relative_position.y;
	//newMeas(KFNav::yb) = 1;

	//measurements(KFNav::zb) = x(KFNav::zp) - data->relative_position.z;
	//newMeas(KFNav::zb) = 1;
}

void Estimator3D::onSecond_position(const geometry_msgs::Point::ConstPtr& data)
{
	measurements(KFNav::xb) = data->x;
	newMeas(KFNav::xb) = 1;

	measurements(KFNav::yb) = data->y;
	newMeas(KFNav::yb) = 1;

	measurements(KFNav::zb) = data->z;
	newMeas(KFNav::zb) = 1;
}

void Estimator3D::deltaPosCallback(const auv_msgs::NED::ConstPtr& msg){

	quadMeasAvailable = true;
	deltaXpos = msg->north;
	deltaYpos = msg->east;
}

void Estimator3D::KFmodeCallback(const std_msgs::Bool::ConstPtr& msg){

	if(!absoluteEKF){
		KFmode = msg->data;

		if(KFmode && (KFmodePast xor KFmode)){

			KFmodePast = KFmode;
			KFNav::matrix P = nav.getStateCovariance();
			P(KFNav::xp,KFNav::xp) = 10000;
			P(KFNav::yp,KFNav::yp) = 10000;
			nav.setStateCovariance(P);

			//nav.R0(KFNav::xp,KFNav::xp) = 1;
			//nav.R0(KFNav::xp,KFNav::xp) = 1;
			//nav.R(7,7) = 0.001;


		} else if(!KFmode && (KFmodePast xor KFmode)){

		KFmodePast = KFmode;
			KFNav::matrix P = nav.getStateCovariance();
			P(KFNav::xp,KFNav::xp) = 10000;
			P(KFNav::yp,KFNav::yp) = 10000;
			nav.setStateCovariance(P);

			//nav.setStateCovariance(Pstart);
			//nav.R0 = Rstart;
			//Rstart(KFNav::xp,KFNav::xp)
		}
	}
}

/*********************************************************************
 *** Helper functions
 ********************************************************************/

void Estimator3D::processMeasurements()
{
	boost::mutex::scoped_lock l(meas_mux);
	if(KFmode == true && absoluteEKF == false)
		{
			if ((newMeas(KFNav::xp) = newMeas(KFNav::yp) = quadMeasAvailable)){

				quadMeasAvailable = false;
				measurements(KFNav::xp) = deltaXpos;
				measurements(KFNav::yp) = deltaYpos;
		    }
		} else {

			//GPS measurements
			if ((newMeas(KFNav::xp) = newMeas(KFNav::yp) = gps.newArrived()))
			{
				measurements(KFNav::xp) = gps.position().first;
				measurements(KFNav::yp) = gps.position().second;
			}
		}

	//Altitude measurement check
	if ((ros::Time::now() - last_alt).toSec() > altitude_timeout)
	{
		//Reset estimate to Zero
		//KFNav::vector cstate = nav.getState();
		//cstate(KFNav::altitude) = 2*min_altitude;
		KFNav::matrix ccov = nav.getStateCovariance();
		ccov(KFNav::altitude, KFNav::altitude) = 10000;
		nav.setStateCovariance(ccov);
	}

	//ROS_ERROR("xp: %4.2f, yp: %4.2f, MODE: %d",measurements(KFNav::xp),measurements(KFNav::yp),KFmode );

	//Imu measurements
	if ((newMeas(KFNav::phi) = newMeas(KFNav::theta) = newMeas(KFNav::psi) = imu.newArrived()))
	{
		measurements(KFNav::phi) = imu.orientation()[ImuHandler::roll];
		measurements(KFNav::theta) = imu.orientation()[ImuHandler::pitch];
		measurements(KFNav::psi) = imu.orientation()[ImuHandler::yaw];

		ROS_DEBUG("NEW IMU: r=%f, p=%f, y=%f", imu.orientation()[ImuHandler::roll],
				imu.orientation()[ImuHandler::pitch],
				imu.orientation()[ImuHandler::yaw]);


		if ((newMeas(KFNav::r) = useYawRate))
		{
			measurements(KFNav::r) = imu.rate()[ImuHandler::r];
			//Combine measurement with DVL
			dvl.current_r(measurements(KFNav::r));
		}
	}
	//DVL measurements
	//if ((newMeas(KFNav::u) = newMeas(KFNav::v) = newMeas(KFNav::w) = dvl.NewArrived()))
	if ((newMeas(KFNav::u) = newMeas(KFNav::v) = dvl.newArrived()))
	{
		double vx = dvl.body_speeds()[DvlHandler::u];
		double vy = dvl.body_speeds()[DvlHandler::v];
		double vxe = nav.getState()(KFNav::u);
		double vye = nav.getState()(KFNav::v);

		double rvx(10),rvy(10);
		//Calculate the measured value
		//This depends on the DVL model actually, but lets assume
		nav.calculateUVInovationVariance(nav.getStateCovariance(), rvx, rvy);

		double cpsi = cos(nav.getState()(KFNav::psi));
		double spsi = sin(nav.getState()(KFNav::psi));
		double xc = nav.getState()(KFNav::xc);
		double yc = nav.getState()(KFNav::yc);
		switch (dvl_model)
		{
		  case 1:
		    vxe += xc*cpsi + yc*spsi;
		    vye += -xc*spsi + yc*cpsi;
		    break;
		default: break;
		}

		if ((fabs((vx - vxe)) > dvl_fp*fabs(rvx)) || (fabs((vy - vye)) > dvl_fp*fabs(rvy)))
		{
			ROS_INFO("Outlier rejected: meas=%f, est=%f, tolerance=%f", vx, vxe, dvl_fp*fabs(rvx));
			ROS_INFO("Outlier rejected: meas=%f, est=%f, tolerance=%f", vy, vye, dvl_fp*fabs(rvy));
			newMeas(KFNav::u) = false;
			newMeas(KFNav::v) = false;
		}
		measurements(KFNav::u) = vx;
		measurements(KFNav::v) = vy;

		//Sanity check
		if ((fabs(vx) > max_dvl) || (fabs(vy) > max_dvl))
		{
			ROS_INFO("DVL measurement failed sanity check for maximum speed %f. Got: vx=%f, vy=%f.",max_dvl, vx, vy);
			newMeas(KFNav::u) = false;
			newMeas(KFNav::v) = false;
		}

		if (newMeas(KFNav::u) || newMeas(KFNav::v))
		{
			dvl_time = ros::Time::now();
			std_msgs::Bool data;
			data.data = false;
			unsafe_dvl.publish(data);
		}
		//measurements(KFNav::w) = dvl.body_speeds()[DvlHandler::w];
	}
	else
	{
		if ((ros::Time::now() - dvl_time).toSec() > dvl_timeout)
		{
			std_msgs::Bool data;
			data.data = true;
			unsafe_dvl.publish(data);
		}
	}
	l.unlock();

	//Publish measurements
	auv_msgs::NavSts::Ptr meas(new auv_msgs::NavSts());
	meas->body_velocity.x = measurements(KFNav::u);
	meas->body_velocity.y = measurements(KFNav::v);
	meas->body_velocity.z = measurements(KFNav::w);

	meas->position.north = measurements(KFNav::xp);
	meas->position.east = measurements(KFNav::yp);
	meas->position.depth = measurements(KFNav::zp);
	meas->altitude = measurements(KFNav::altitude);

	meas->orientation.roll = measurements(KFNav::phi);
	meas->orientation.pitch = measurements(KFNav::theta);
	meas->orientation.yaw = labust::math::wrapRad(measurements(KFNav::psi));
	if (useYawRate)	meas->orientation_rate.yaw = measurements(KFNav::r);

	meas->origin.latitude = gps.origin().first;
	meas->origin.longitude = gps.origin().second;
	meas->global_position.latitude = gps.latlon().first;
	meas->global_position.longitude = gps.latlon().second;

	meas->header.stamp = ros::Time::now();
	meas->header.frame_id = "local";
	stateMeas.publish(meas);
}

void Estimator3D::publishState()
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

void Estimator3D::calculateConditionNumber(){

	KFNav::matrix P = nav.getStateCovariance();

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(P);
	double cond1 = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);

	Eigen::Matrix2d Pxy;
	Pxy << P(KFNav::xp,KFNav::xp), P(KFNav::xp,KFNav::yp), P(KFNav::yp,KFNav::xp), P(KFNav::yp,KFNav::yp);


	//double P(1,0)*P(0,1);

	//ROS_ERROR_STREAM(Pxy);
	double traceP = Pxy.trace();
	double detP = Pxy.determinant();

	double condCost = std::sqrt(traceP*traceP-4*detP);
	
	Eigen::JacobiSVD<Eigen::MatrixXd> svd2(Pxy);
	double cond2 = svd2.singularValues()(0) / svd2.singularValues()(svd2.singularValues().size()-1);

	std_msgs::Float32::Ptr data(new std_msgs::Float32);

	data->data = cond1;
//	pubCondP.publish(data);
	data->data = cond2;
//	pubCondPxy.publish(data);
	data->data = condCost;
//	pubCost.publish(data);

}

int Estimator3D::calculateDelaySteps(double measTime, double arrivalTime){
				return floor((arrivalTime-measTime)/nav.Ts);
			}

class EKFNode{

public:
	EKFNode();

	~EKFNode();

	void start();

	void reset();

	void init();

	void processInput();

	void processMeasurement();

	void updateEstimator();

	void publishStateAndMeasurement();

	void publishTransform();

	labust::navigation::EstimatorContainer<labust::navigation::GenericModel> _estimator;

	labust::navigation::MeasurementHandler _measurement_handler;

	enum {u=0,v,w,p,q,r,xp,yp,zp,phi,theta,psi,xc,yc,b,buoyancy,roll_restore,pitch_restore,altitude,xb,yb,zb,stateNum};
	enum {X=0,Y,Z,Kroll,M,N,inputSize};
	enum {range=stateNum,bearing,elevation,measSize};



};

void EKFNode::init(){

	ros::NodeHandle ph("~");
	double Ts(0.1);
	ph.param("Ts",Ts,Ts);
	ros::Rate rate(1/Ts);
	nav.setTs(Ts);
}

void EKFNode::start()
{
	init();
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


