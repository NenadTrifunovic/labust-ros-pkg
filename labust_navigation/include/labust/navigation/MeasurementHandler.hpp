/*
 * MeasurementHandler.hpp
 *
 *  Created on: Jan 13, 2016
 *      Author: filip
 */

#ifndef MEASUREMENTHANDLER_HPP_
#define MEASUREMENTHANDLER_HPP_

#include <labust/navigation/SensorHandlers.hpp>
#include <labust/navigation/SSModel.hpp>


// Temporary
//#include <labust/navigation/EKF_3D_USBL.hpp>
//#include <labust/navigation/EKF_3D_USBLModel.hpp>
//


namespace labust
{
	namespace navigation
	{
		//template <class Model>
		class MeasurementHandler
		{
		public:
			typedef SSModel<>::vector vector;

		private:


//			typedef labust::navigation::KFCore<Model> Estimator;
//			typedef typename labust::navigation::KFCore<Model>::vector vector;
//			typedef typename labust::navigation::KFCore<Model>::vector matrix;
//			typedef typename labust::navigation::KFCore<Model>::measurement_vector measurement_vector;
//			typedef typename labust::navigation::KFCore<Model>::state_vector state_vector;
//			typedef typename labust::navigation::KFCore<Model>::input_vector input_vector;

			struct FilterState
			{
				FilterState(){}

				~FilterState(){}

//				Estimator::vector input;
//				Estimator::vector measurement;
//				Estimator::vector new_measurement; /* Value -1 denotes no measurement, values >=0 denote measurement delay. */
//				Estimator::vector state;
//				Estimator::matrix p_covariance;
//				Estimator::matrix r_covariance;
			};

		public:
			MeasurementHandler(){}

			~MeasurementHandler(){}

			void updateMeasurements();

			vector getMeasurementVector();

			vector getMeasurementTimeVector();


			/**
			 * The GPS handler.
			 */
			GPSHandler gps;
			/**
			 * The Imu handler.
			 */
			ImuHandler imu;
			/**
			 * The DVL handler.
			 */
			DvlHandler dvl;
			/// Maximum dvl speed for sanity checks
			double max_dvl;
			/// Minimum safe altitude that is accepted as measurement
			double min_altitude;
			/// Last DVL measurement received time
			ros::Time dvl_time;
			/// Timeout value for the DVL
			double dvl_timeout;

		};
	}
}

using namespace labust::navigation;

MeasurementHandler::vector MeasurementHandler::getMeasurementVector()
{

}

MeasurementHandler::vector MeasurementHandler::getMeasurementTimeVector()
{

}



//void Estimator3D::onInit()
//{
//	ros::NodeHandle nh, ph("~");
//
//	/*** Configure the navigation ***/
////	configureNav(nav,nh);
//
//	/*** Publishers ***/
////	stateHat = nh.advertise<auv_msgs::NavSts>("stateHat",1);
////	stateMeas = nh.advertise<auv_msgs::NavSts>("meas",1);
////	currentsHat = nh.advertise<geometry_msgs::TwistStamped>("currentsHat",1);
////	buoyancyHat = nh.advertise<std_msgs::Float32>("buoyancy",1);
////
////	turns_pub = nh.advertise<std_msgs::Float32>("turns",1);
////	unsafe_dvl = nh.advertise<std_msgs::Bool>("unsafe_dvl",1);
////	altitude_cov = nh.advertise<std_msgs::Float32>("altitude_cov",1);
////
////	pubRange = nh.advertise<std_msgs::Float32>("range",1);
////	pubRangeFiltered = nh.advertise<std_msgs::Float32>("range_filtered",1);
////	pubwk = nh.advertise<std_msgs::Float32>("w_limit",1);
////	pubCondP = nh.advertise<std_msgs::Float32>("condP",1);
////	pubCondPxy = nh.advertise<std_msgs::Float32>("condPxy",1);
////	pubCost = nh.advertise<std_msgs::Float32>("cost",1);
////
////	/*** Subscribers ***/
////	tauAch = nh.subscribe<auv_msgs::BodyForceReq>("tauAch", 1, &Estimator3D::onTau,this);
////	depth = nh.subscribe<std_msgs::Float32>("depth", 1,	&Estimator3D::onDepth, this);
////	altitude = nh.subscribe<std_msgs::Float32>("altitude", 1, &Estimator3D::onAltitude, this);
////	modelUpdate = nh.subscribe<navcon_msgs::ModelParamsUpdate>("model_update", 1, &Estimator3D::onModelUpdate,this);
////	resetTopic = nh.subscribe<std_msgs::Bool>("reset_nav_covariance", 1, &Estimator3D::onReset,this);
////	useGyro = nh.subscribe<std_msgs::Bool>("use_gyro", 1, &Estimator3D::onUseGyro,this);
////	subUSBL = nh.subscribe<underwater_msgs::USBLFix>("usbl_fix", 1, &Estimator3D::onUSBLfix,this);
////	sub = nh.subscribe<auv_msgs::NED>("quad_delta_pos", 1, &Estimator3D::deltaPosCallback,this);
////	subKFmode = nh.subscribe<std_msgs::Bool>("KFmode", 1, &Estimator3D::KFmodeCallback, this);
////
////	useAltSampling = nh.subscribe<std_msgs::Bool>("use_alt_sampling", 1, &Estimator3D::onAltSampling,this);
////    altNSample = nh.subscribe<std_msgs::Int32>("alt_n_samples", 1, &Estimator3D::onAltNSamples,this);
//
//
//	//subSecond_heading = nh.subscribe<std_msgs::Float32>("out_acoustic_heading", 1, &Estimator3D::onSecond_heading,this);
//	subSecond_position = nh.subscribe<geometry_msgs::Point>("out_acoustic_position", 1, &Estimator3D::onSecond_position,this);
//	//subSecond_speed = nh.subscribe<std_msgs::Float32>("out_acoustic_speed", 1, &Estimator3D::onSecond_speed,this);
//	//subSecond_usbl_fix = nh.subscribe<underwater_msgs::USBLFix>("usbl_fix", 1, &Estimator3D::onSecond_usbl_fix,this);
//
//	KFmode = quadMeasAvailable = false;
//
//	/*** Get DVL model ***/
//	ph.param("dvl_model",dvl_model, dvl_model);
//	nav.useDvlModel(dvl_model);
//	ph.param("imu_with_yaw_rate",useYawRate,useYawRate);
//	ph.param("compass_variance",compassVariance,compassVariance);
//	ph.param("gyro_variance",gyroVariance,gyroVariance);
//
//	ph.param("max_dvl",max_dvl,max_dvl);
//	ph.param("min_altitude", min_altitude, min_altitude);
//	ph.param("dvl_timeout", dvl_timeout, dvl_timeout);
//	double trustf(0);
//	ph.param("dvl_rot_trust_factor", trustf, trustf);
//	double sway_corr(0);
//	ph.param("absoluteEKF", absoluteEKF,absoluteEKF);
//	ph.param("altitude_cov_timeout",altitude_timeout, altitude_timeout);
//	ph.param("altitude_sampling",alt_sample, alt_sample);
//
//	/*** Enable USBL measurements ***/
//	ph.param("delay", enableDelay, enableDelay);
//	ph.param("delay_time", delay_time, delay_time);
//	ph.param("range", enableRange, enableRange);
//	ph.param("bearing", enableBearing, enableBearing);
//	ph.param("elevation", enableElevation, enableElevation);
//	//ph.param("delayTime", delayTime, delayTime);
//
//	/** Enable outlier rejection */
//	ph.param("meas_outlier_rejection", enableRejection, enableRejection);
//
//	/*** Configure handlers. ***/
//	gps.configure(nh);
//	dvl.configure(nh);
//	imu.configure(nh);
//
//	/*** Get initial error covariance. ***/
//    Pstart = nav.getStateCovariance();
//
//    nav.setDVLRotationTrustFactor(trustf);
//
//    //Setup sway correction
//    bool use_sc(false);
//    double acc_port(0.3), acc_starboard(0.3),
//  	vec_port(0.07), vec_starboard(0.07);
//    ph.param("use_sc",use_sc, use_sc);
//    ph.param("acc_port",acc_port, acc_port);
//    ph.param("acc_starboard",acc_starboard, acc_starboard);
//    ph.param("vec_port",vec_port, vec_port);
//    ph.param("vec_starboard",vec_starboard, vec_starboard);
//    nav.setSwayCorrection(use_sc, acc_port, acc_starboard,
//    		vec_port, vec_starboard);
//    //Rstart = nav.R;
//    //ROS_ERROR("NAVIGATION");
//    ph.param("dvl_fp",dvl_fp, dvl_fp);
//}

//void Estimator3D::onReset(const std_msgs::Bool::ConstPtr& reset)
//{
//   if (reset->data)
//   {
//      nav.setStateCovariance(10000*KFNav::matrix::Identity(KFNav::stateNum, KFNav::stateNum));
//   }
//}
//
//void Estimator3D::onUseGyro(const std_msgs::Bool::ConstPtr& use_gyro)
//{
//   if (use_gyro->data)
//   {
//      nav.R0(KFNav::psi, KFNav::psi) = gyroVariance;
//      ROS_INFO("Switch to using gyro measurements.");
//   }
//   else
//   {
//      nav.R0(KFNav::psi, KFNav::psi) = compassVariance;
//      ROS_INFO("Switch to using compass measurements.");
//   }
//}
//
//void Estimator3D::onAltSampling(const std_msgs::Bool::ConstPtr& flag)
//{
//	alt_sample = flag->data;
//}
//
//void Estimator3D::onAltNSamples(const std_msgs::Int32::ConstPtr& samples)
//{
//	nsamples_alt = samples->data;
//}





/*********************************************************************
 *** Measurement callback
 ********************************************************************/

//void Estimator3D::onDepth(const std_msgs::Float32::ConstPtr& data)
//{
////	boost::mutex::scoped_lock l(meas_mux);
////	measurements(KFNav::zp) = data->data;
////	newMeas(KFNav::zp) = 1;
//};
//
//void Estimator3D::onAltitude(const std_msgs::Float32::ConstPtr& data)
//{
////	boost::mutex::scoped_lock l(meas_mux);
////	measurements(KFNav::altitude) = data->data;
////	//Skip measurement if minimum altitude is encountered
////	if (data->data <= min_altitude) return;
////	//Dismiss false altitude
////	if (fabs(data->data-nav.getState()(KFNav::altitude)) < 3*nav.calculateAltInovationVariance(nav.getStateCovariance()))
////	{
////		double malt = data->data;
////		bool old = (ros::Time::now() - last_alt).toSec() > altitude_timeout;
////		//In case no measurements were received for a long time
////		if (old && alt_sample)
////		{
////			//Add to buffer
////			altbuf.push_back(malt);
////			//Keep n samples
////			if (altbuf.size() > nsamples_alt) altbuf.erase(altbuf.begin());
////			//Do not accept the measurement before the sample container is full.
////			if (altbuf.size() < nsamples_alt) return;
////
////			//Calculate variance
////			double var = pow(labust::math::std2(altbuf),2);
////			malt = labust::math::mean(altbuf);
////			ROS_INFO("Var: %f, mean: %f", var, malt);
////			if (var > altok_var) return;
////			altbuf.clear();
////		}
////
////		last_alt = ros::Time::now();
////		newMeas(KFNav::altitude) = 1;
////		alt = malt;
////		ROS_DEBUG("Accepted altitude: meas=%f, estimate=%f, variance=%f",
////			data->data, nav.getState()(KFNav::altitude), 3*nav.calculateAltInovationVariance(nav.getStateCovariance()));
////	}
////	else
////	{
////		ROS_INFO("Dissmissed altitude: meas=%f, estimate=%f, variance=%f",
////			data->data, nav.getState()(KFNav::altitude), 10*nav.calculateAltInovationVariance(nav.getStateCovariance()));
////	}
//};
//
//void Estimator3D::onUSBLfix(const underwater_msgs::USBLFix::ConstPtr& data){
//
////	/*** Calculate measurement delay ***/
////	//double delay = calculateDelaySteps(data->header.stamp.toSec(), currentTime);
////	double delay = double(calculateDelaySteps(currentTime-delay_time, currentTime));
////
//////	double bear = 360 - data->bearing;
//////	double elev = 180 - data->elevation;
////	double bear = data->bearing;
////	double elev = data->elevation;
////
////	const KFNav::vector& x = nav.getState();
////	//labust::math::wrapRad(measurements(KFNav::psi));
////
////	ROS_ERROR("RANGE: %f, BEARING: %f deg %f rad", data->range,bear, labust::math::wrapRad(bear*M_PI/180+x(KFNav::psi)));
////	/*** Get USBL measurements ***/
////	measurements(KFNav::range) = (data->range > 0.1)?data->range:0.1;
////	newMeas(KFNav::range) = enableRange;
////	measDelay(KFNav::range) = delay;
////
////	/*Eigen::VectorXd input(Eigen::VectorXd::Zero(3));
////	//Eigen::VectorXd output(Eigen::VectorXd::Zero(1));
////
////
////	input << x(KFNav::xp)-x(KFNav::xb), x(KFNav::yp)-x(KFNav::yb), x(KFNav::zp)-x(KFNav::zb);
////	//output << measurements(KFNav::range);
////	double y_filt, sigma, w;
////	OR.step(input, measurements(KFNav::range), &y_filt, &sigma, &w);
////	ROS_INFO("Finished outlier rejection");
////
////	std_msgs::Float32 rng_msg;
////	ROS_ERROR("w: %f",w);
////
////	double w_limit =  0.25*std::sqrt(float(sigma));
////	w_limit = (w_limit>1.0)?1.0:w_limit;
////	if(w>w_limit)
////	{
////		rng_msg.data = data->range;
////		pubRangeFiltered.publish(rng_msg);
////	}
////	//std_msgs::Float32 rng_msg;
////
////	rng_msg.data = w_limit;
////	pubwk.publish(rng_msg);
////
////	rng_msg.data = data->range;
////	pubRange.publish(rng_msg);*/
////
////	//measurements(KFNav::bearing) = labust::math::wrapRad(bear*M_PI/180+x(KFNav::psi));
//////	measurements(KFNav::bearing) = labust::math::wrapRad(bear*M_PI/180);
////	measurements(KFNav::bearing) = bear;
////	newMeas(KFNav::bearing) = enableBearing;
////	measDelay(KFNav::bearing) = delay;
////
////	measurements(KFNav::elevation) = elev*M_PI/180;
////	newMeas(KFNav::elevation) = enableElevation;
////	measDelay(KFNav::elevation) = delay;
////
////	/*** Get beacon position ***/
////	measurements(KFNav::xb) = 0;
////	newMeas(KFNav::xb) = 1;
////	measDelay(KFNav::xb) = delay;
////
////	measurements(KFNav::yb) = 0;
////	newMeas(KFNav::yb) = 1;
////	measDelay(KFNav::yb) = delay;
////
////	measurements(KFNav::zb) = 0;
////	newMeas(KFNav::zb) = 1;
////	measDelay(KFNav::zb) = delay;
////
////	// Debug print
////	//ROS_ERROR("Delay: %f", delay);
////	//ROS_ERROR("Range: %f, bearing: %f, elevation: %f", measurements(KFNav::range), measurements(KFNav::bearing), measurements(KFNav::elevation));
////	//ROS_ERROR("ENABLED Range: %d, bearing: %d, elevation: %d", enableRange, enableBearing, enableElevation);
////
////
////	// Relativna mjerenja ukljuciti u model mjerenja...
////
////	//measurements(KFNav::xb) = x(KFNav::xp) - data->relative_position.x;
////	//newMeas(KFNav::xb) = 1;
////
////	//measurements(KFNav::yb) = x(KFNav::yp) - data->relative_position.y;
////	//newMeas(KFNav::yb) = 1;
////
////	//measurements(KFNav::zb) = x(KFNav::zp) - data->relative_position.z;
////	//newMeas(KFNav::zb) = 1;
//}
//
////void Estimator3D::onSecond_position(const geometry_msgs::Point::ConstPtr& data)
////{
////	measurements(KFNav::xb) = data->x;
////	newMeas(KFNav::xb) = 1;
////
////	measurements(KFNav::yb) = data->y;
////	newMeas(KFNav::yb) = 1;
////
////	measurements(KFNav::zb) = data->z;
////	newMeas(KFNav::zb) = 1;
////}

/*********************************************************************
 *** Helper functions
 ********************************************************************/

void MeasurementHandler::updateMeasurements()
{
//	boost::mutex::scoped_lock l(meas_mux);
//	if(KFmode == true && absoluteEKF == false)
//		{
//			if ((newMeas(KFNav::xp) = newMeas(KFNav::yp) = quadMeasAvailable)){
//
//				quadMeasAvailable = false;
//				measurements(KFNav::xp) = deltaXpos;
//				measurements(KFNav::yp) = deltaYpos;
//		    }
//		} else {
//
//			//GPS measurements
//			if ((newMeas(KFNav::xp) = newMeas(KFNav::yp) = gps.newArrived()))
//			{
//				measurements(KFNav::xp) = gps.position().first;
//				measurements(KFNav::yp) = gps.position().second;
//			}
//		}
//
//	//Altitude measurement check
//	if ((ros::Time::now() - last_alt).toSec() > altitude_timeout)
//	{
//		//Reset estimate to Zero
//		//KFNav::vector cstate = nav.getState();
//		//cstate(KFNav::altitude) = 2*min_altitude;
//		KFNav::matrix ccov = nav.getStateCovariance();
//		ccov(KFNav::altitude, KFNav::altitude) = 10000;
//		nav.setStateCovariance(ccov);
//	}
//
//	//ROS_ERROR("xp: %4.2f, yp: %4.2f, MODE: %d",measurements(KFNav::xp),measurements(KFNav::yp),KFmode );
//
//	//Imu measurements
//	if ((newMeas(KFNav::phi) = newMeas(KFNav::theta) = newMeas(KFNav::psi) = imu.newArrived()))
//	{
//		measurements(KFNav::phi) = imu.orientation()[ImuHandler::roll];
//		measurements(KFNav::theta) = imu.orientation()[ImuHandler::pitch];
//		measurements(KFNav::psi) = imu.orientation()[ImuHandler::yaw];
//
//		ROS_DEBUG("NEW IMU: r=%f, p=%f, y=%f", imu.orientation()[ImuHandler::roll],
//				imu.orientation()[ImuHandler::pitch],
//				imu.orientation()[ImuHandler::yaw]);
//
//
//		if ((newMeas(KFNav::r) = useYawRate))
//		{
//			measurements(KFNav::r) = imu.rate()[ImuHandler::r];
//			//Combine measurement with DVL
//			dvl.current_r(measurements(KFNav::r));
//		}
//	}
//	//DVL measurements
//	//if ((newMeas(KFNav::u) = newMeas(KFNav::v) = newMeas(KFNav::w) = dvl.NewArrived()))
//	if ((newMeas(KFNav::u) = newMeas(KFNav::v) = dvl.newArrived()))
//	{
//		double vx = dvl.body_speeds()[DvlHandler::u];
//		double vy = dvl.body_speeds()[DvlHandler::v];
//		double vxe = nav.getState()(KFNav::u);
//		double vye = nav.getState()(KFNav::v);
//
//		double rvx(10),rvy(10);
//		//Calculate the measured value
//		//This depends on the DVL model actually, but lets assume
//		nav.calculateUVInovationVariance(nav.getStateCovariance(), rvx, rvy);
//
//		double cpsi = cos(nav.getState()(KFNav::psi));
//		double spsi = sin(nav.getState()(KFNav::psi));
//		double xc = nav.getState()(KFNav::xc);
//		double yc = nav.getState()(KFNav::yc);
//		switch (dvl_model)
//		{
//		  case 1:
//		    vxe += xc*cpsi + yc*spsi;
//		    vye += -xc*spsi + yc*cpsi;
//		    break;
//		default: break;
//		}
//
//		if ((fabs((vx - vxe)) > dvl_fp*fabs(rvx)) || (fabs((vy - vye)) > dvl_fp*fabs(rvy)))
//		{
//			ROS_INFO("Outlier rejected: meas=%f, est=%f, tolerance=%f", vx, vxe, dvl_fp*fabs(rvx));
//			ROS_INFO("Outlier rejected: meas=%f, est=%f, tolerance=%f", vy, vye, dvl_fp*fabs(rvy));
//			newMeas(KFNav::u) = false;
//			newMeas(KFNav::v) = false;
//		}
//		measurements(KFNav::u) = vx;
//		measurements(KFNav::v) = vy;
//
//		//Sanity check
//		if ((fabs(vx) > max_dvl) || (fabs(vy) > max_dvl))
//		{
//			ROS_INFO("DVL measurement failed sanity check for maximum speed %f. Got: vx=%f, vy=%f.",max_dvl, vx, vy);
//			newMeas(KFNav::u) = false;
//			newMeas(KFNav::v) = false;
//		}
//
//		if (newMeas(KFNav::u) || newMeas(KFNav::v))
//		{
//			dvl_time = ros::Time::now();
//			std_msgs::Bool data;
//			data.data = false;
//			unsafe_dvl.publish(data);
//		}
//		//measurements(KFNav::w) = dvl.body_speeds()[DvlHandler::w];
//	}
//	else
//	{
//		if ((ros::Time::now() - dvl_time).toSec() > dvl_timeout)
//		{
//			std_msgs::Bool data;
//			data.data = true;
//			unsafe_dvl.publish(data);
//		}
//	}
//	l.unlock();
//
//	//Publish measurements
//	auv_msgs::NavSts::Ptr meas(new auv_msgs::NavSts());
//	meas->body_velocity.x = measurements(KFNav::u);
//	meas->body_velocity.y = measurements(KFNav::v);
//	meas->body_velocity.z = measurements(KFNav::w);
//
//	meas->position.north = measurements(KFNav::xp);
//	meas->position.east = measurements(KFNav::yp);
//	meas->position.depth = measurements(KFNav::zp);
//	meas->altitude = measurements(KFNav::altitude);
//
//	meas->orientation.roll = measurements(KFNav::phi);
//	meas->orientation.pitch = measurements(KFNav::theta);
//	meas->orientation.yaw = labust::math::wrapRad(measurements(KFNav::psi));
//	if (useYawRate)	meas->orientation_rate.yaw = measurements(KFNav::r);
//
//	meas->origin.latitude = gps.origin().first;
//	meas->origin.longitude = gps.origin().second;
//	meas->global_position.latitude = gps.latlon().first;
//	meas->global_position.longitude = gps.latlon().second;
//
//	meas->header.stamp = ros::Time::now();
//	meas->header.frame_id = "local";
//	stateMeas.publish(meas);
}

#endif /* MEASUREMENTHANDLER_HPP_ */
