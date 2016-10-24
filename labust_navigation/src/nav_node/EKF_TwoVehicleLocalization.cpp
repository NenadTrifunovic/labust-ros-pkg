/*********************************************************************
 *  EKF_TwoVehicleLocalization.cpp
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
 *  Copyright (c) 2015-2016, LABUST, UNIZG-FER
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
#include <labust/math/NumberManipulation.hpp>
#include <labust/navigation/KFModelLoader.hpp>
#include <labust/navigation/TwoVehicleLocalization.hpp>
#include <labust/navigation/TwoVehicleLocalizationModel.hpp>
#include <labust/simulation/DynamicsParams.hpp>
#include <labust/tools/DynamicsLoader.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/NavSts.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <navcon_msgs/RelativePosition.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <underwater_msgs/SonarFix.h>
#include <underwater_msgs/USBLFix.h>

#include <ros/ros.h>

#include <math.h>
#include <boost/bind.hpp>

using namespace labust::navigation;

Estimator3D::Estimator3D()
  :

  tauIn(KFNav::vector::Zero(KFNav::inputSize))
  , measurements(KFNav::vector::Zero(KFNav::measSize))
  , newMeas(KFNav::vector::Zero(KFNav::measSize))
  , measDelay(KFNav::vector::Zero(KFNav::measSize))
  , measurement_timeout(ros::Time::now())
  , enableDelay(false)
  , enableRange(true)
  , enableBearing(true)
  , enableElevation(false)
  , enableRejection(false)
  , enable_camera_heading(false)
  , alternate_outlier(false)
  , sonar_offset(0.0)
  , usbl_offset(0.0)
  , camera_offset(0.0)
  , usbl_bearing_offset(0.0)
  , camera_bearing_offset(0.0)
  , sonar_bearing_offset(0.0)
  , diver_camera_heading_offset(0.0)
  , divernet_heading_offset(0.0)
  , depth_offset(0.0)
  , meas_timeout_limit(10.0)
  , cov_limit(50.0)
  , delay_time(0.0)
  , dvl_model(1)
  , OR(3, 0.9)
  , OR_b(2, 0.97)
  , P_rng_bear_relative(Eigen::Matrix2d::Zero())
  , display_counter(0)
{
  this->onInit();
};

void Estimator3D::onInit()
{
  ros::NodeHandle nh, ph("~");

  /** Configure the navigation */
  configureNav(nav, nh);

  /** Publishers */
  pubLocalStateHat = nh.advertise<auv_msgs::NavSts>("localStateHat", 1);
  pubSecondStateHat = nh.advertise<auv_msgs::NavSts>("secondStateHat", 1);

  pubLocalStateMeas = nh.advertise<auv_msgs::NavSts>("localMeasurement", 1);
  pubSecondStateMeas = nh.advertise<auv_msgs::NavSts>("secondMesurement", 1);

  pubBearing = nh.advertise<std_msgs::Float32>("bearing_meas", 1);

  pubCondP = nh.advertise<std_msgs::Float32>("P_trace", 1);
  pubCondPxy = nh.advertise<std_msgs::Float32>("condPxy", 1);
  pubCost = nh.advertise<std_msgs::Float32>("cost", 1);

  pubSecondRelativePosition =
      nh.advertise<navcon_msgs::RelativePosition>("relative_position", 1);

  // pubRangeFiltered = nh.advertise<std_msgs::Float32>("range_filtered",1);
  // pubwk = nh.advertise<std_msgs::Float32>("w_limit",1);

  /** Subscribers */
  subLocalStateHat = nh.subscribe<auv_msgs::NavSts>(
      "stateHat", 1, &Estimator3D::onLocalStateHat, this);
  subSecond_navsts = nh.subscribe<auv_msgs::NavSts>(
        "second_navsts", 1, &Estimator3D::onSecond_navsts, this);
  subSecond_usbl_fix = nh.subscribe<underwater_msgs::USBLFix>(
      "usbl_fix", 1, &Estimator3D::onSecond_usbl_fix, this);
  //subSecond_sonar_fix = nh.subscribe<underwater_msgs::SonarFix>(
  //    "sonar_fix", 1, &Estimator3D::onSecond_sonar_fix, this);
  subSecond_sonar_fix = nh.subscribe<navcon_msgs::RelativePosition>(
      "sonar_fix", 1, &Estimator3D::onSecond_sonar_fix, this);
  subSecond_camera_fix = nh.subscribe<navcon_msgs::RelativePosition>(
      "camera_fix", 1, &Estimator3D::onSecond_camera_fix, this);

  resetTopic = nh.subscribe<std_msgs::Bool>("reset_nav_covariance", 1,
                                            &Estimator3D::onReset, this);
  sub_usbl_bearing_offset = nh.subscribe<std_msgs::Float32>("usbl_bearing_offset", 1,
                                            &Estimator3D::onUSBLbearningOffset, this);
  sub_usbl_range_offset = nh.subscribe<std_msgs::Float32>("usbl_range_offset", 1,
          &Estimator3D::onUSBLrangeOffset, this);
  sub_camera_range_offset = nh.subscribe<std_msgs::Float32>("camera_range_offset", 1,
            &Estimator3D::onCameraRangeOffset, this);
  sub_camera_bearing_offset = nh.subscribe<std_msgs::Float32>("camera_bearing_offset", 1,
                                              &Estimator3D::onCameraBearningOffset, this);

  sub_sonar_range_offset = nh.subscribe<std_msgs::Float32>("sonar_range_offset", 1,
            &Estimator3D::onCameraRangeOffset, this);
  sub_depth_offset = nh.subscribe<std_msgs::Float32>("depth_offset", 1,
                                              &Estimator3D::onDepthOffset, this);

  sub_camera_heading_offset  = nh.subscribe<std_msgs::Float32>("camera_heading_offset", 1,
          &Estimator3D::onCameraHeadingOffset, this);
  sub_divernet_heading_offset  = nh.subscribe<std_msgs::Float32>("divernet_heading_offset", 1,
          &Estimator3D::onDivernetHeadingOffset, this);

  pub_usbl_range = nh.advertise<std_msgs::Float32>("measurement_diver_2/usbl/range", 1);
  pub_usbl_bearing = nh.advertise<std_msgs::Float32>("measurement_diver_2/usbl/bearing", 1);
  pub_sonar_range = nh.advertise<std_msgs::Float32>("measurement_diver_2/sonar/range", 1);
  pub_sonar_bearing = nh.advertise<std_msgs::Float32>("measurement_diver_2/sonar/bearing", 1);
  pub_camera_range = nh.advertise<std_msgs::Float32>("measurement_diver_2/camera/range", 1);
  pub_camera_bearing =nh.advertise<std_msgs::Float32>("measurement_diver_2/camera/bearing", 1);
  pub_diver_course =nh.advertise<std_msgs::Float32>("estimate_diver_2/diver/course", 1);


  /** Enable USBL measurements */
  ph.param("delay", enableDelay, enableDelay);
  ph.param("delay_time", delay_time, delay_time);
  ph.param("range", enableRange, enableRange);
  ph.param("bearing", enableBearing, enableBearing);
  ph.param("elevation", enableElevation, enableElevation);

  /** Enable outlier rejection */
  ph.param("meas_outlier_rejection", enableRejection, enableRejection);
  ph.param("alternate_outlier_rejection", alternate_outlier, alternate_outlier);

  ph.param("sonar_offset", sonar_offset, sonar_offset);
  ph.param("usbl_offset", usbl_offset, usbl_offset);
  ph.param("covariance_limit", cov_limit, cov_limit);
  ph.param("usbl_bearing_offset", usbl_bearing_offset, usbl_bearing_offset);
  ph.param("depth_offset", depth_offset, depth_offset);
  ph.param("measurement_timeout", meas_timeout_limit, meas_timeout_limit);
  ph.param("camera_offset", camera_offset, camera_offset);
  ph.param("camera_bearing_offset", camera_bearing_offset, camera_bearing_offset);
  ph.param("sonar_bearing_offset", sonar_bearing_offset, sonar_bearing_offset);

  ph.param("enable_camera_heading", enable_camera_heading, enable_camera_heading);
  ph.param("camera_heading_offset", diver_camera_heading_offset, diver_camera_heading_offset);
  ph.param("divernet_heading_offset", divernet_heading_offset, divernet_heading_offset);





}

void Estimator3D::onReset(const std_msgs::Bool::ConstPtr& reset)
{
  if (reset->data)
  {
	KFNav::matrix covariance = nav.getStateCovariance();
    covariance(KFNav::xb,KFNav::xb) = 10000;
    covariance(KFNav::xb,KFNav::yb) = 0;
    covariance(KFNav::yb,KFNav::yb) = 10000;
    covariance(KFNav::yb,KFNav::xb) = 0;

    nav.setStateCovariance(covariance);
    ROS_ERROR("Diver filter reset.");
  }

}

void Estimator3D::configureNav(KFNav& nav, ros::NodeHandle& nh)
{
  ROS_INFO("Configure navigation.");

  /** No dynamic params initialization */

  nav.initModel();
  labust::navigation::kfModelLoader(nav, nh, "ekfnav_twl");
}

void Estimator3D::onUSBLbearningOffset(const std_msgs::Float32::ConstPtr& data)
{
  usbl_bearing_offset = data->data;
  ROS_ERROR("USBL bearing offset changed: %f.", usbl_bearing_offset);
}

void Estimator3D::onCameraBearningOffset(const std_msgs::Float32::ConstPtr& data)
{
  camera_bearing_offset = data->data;
  ROS_ERROR("Camera bearing offset changed: %f.", camera_bearing_offset);
}

void Estimator3D::onSonarBearningOffset(const std_msgs::Float32::ConstPtr& data)
{
  sonar_bearing_offset = data->data;
  ROS_ERROR("Sonar bearing offset changed: %f.", sonar_bearing_offset);
}

void Estimator3D::onCameraRangeOffset(const std_msgs::Float32::ConstPtr& data)
{
  camera_offset = data->data;
  ROS_ERROR("Camera range offset changed: %f.", camera_offset);
}

void Estimator3D::onUSBLrangeOffset(const std_msgs::Float32::ConstPtr& data)
{
  usbl_offset = data->data;
  ROS_ERROR("USBL range offset changed: %f.", usbl_offset);
}

void Estimator3D::onSonarRangeOffset(const std_msgs::Float32::ConstPtr& data)
{
  sonar_offset = data->data;
  ROS_ERROR("Sonar range offset changed: %f.", sonar_offset);
}

void Estimator3D::onDepthOffset(const std_msgs::Float32::ConstPtr& data)
{
  depth_offset = data->data;
  ROS_ERROR("Depth offset changed: %f.", depth_offset);
}

void Estimator3D::onDivernetHeadingOffset(const std_msgs::Float32::ConstPtr& data)
{
  divernet_heading_offset = data->data;
  ROS_ERROR("Divernet heading offset changed: %f.", divernet_heading_offset);
}

void Estimator3D::onCameraHeadingOffset(const std_msgs::Float32::ConstPtr& data)
{
  diver_camera_heading_offset = data->data;
  ROS_ERROR("Camera heading offset changed: %f.", diver_camera_heading_offset);
}

/*********************************************************************
 *** Measurement callback
 ********************************************************************/

void Estimator3D::onLocalStateHat(const auv_msgs::NavSts::ConstPtr& data)
{
  measurements(KFNav::xp) = data->position.north;
  newMeas(KFNav::xp) = !std::isnan(data->position.north);
;

  measurements(KFNav::yp) = data->position.east;
  newMeas(KFNav::yp) = !std::isnan(data->position.east);

  measurements(KFNav::zp) = data->position.depth;
  newMeas(KFNav::zp) = !std::isnan(data->position.depth);

  bool use_depth_approx(false);
  if(use_depth_approx)
  {
    measurements(KFNav::zb) = data->position.depth;
    newMeas(KFNav::zb) = 1;
  }
  measurements(KFNav::hdg) = unwrap(data->orientation.yaw);
  newMeas(KFNav::hdg) = !std::isnan(data->orientation.yaw);;

  Eigen::Matrix2d R;
  double yaw = data->orientation.yaw;
  R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  Eigen::Vector2d in, out;
  in << data->gbody_velocity.x, data->gbody_velocity.y;
  out = R * in;

//  measurements(KFNav::psi) = course_unwrap(std::atan2(out(1), out(0)));
  measurements(KFNav::psi) = std::atan2(out(1), out(0));
  newMeas(KFNav::psi) = 1;

  measurements(KFNav::u) =
      std::sqrt(std::pow(out(0), 2) + std::pow(out(1), 2));
  newMeas(KFNav::u) = 1;

  measurements(KFNav::w) = data->gbody_velocity.z;
  newMeas(KFNav::w) = !std::isnan(data->gbody_velocity.z);

  measurements(KFNav::r) = data->orientation_rate.yaw;
  newMeas(KFNav::r) = !std::isnan(data->orientation_rate.yaw);
};

void Estimator3D::onSecond_navsts(const auv_msgs::NavSts::ConstPtr& data)
{
  measurements(KFNav::zb) = data->position.depth + depth_offset;
  newMeas(KFNav::zb) = 1;

  //measurements(KFNav::psib) = data->orientation.yaw;
  //newMeas(KFNav::psib) = 1;
  measurements(KFNav::hdgb) = hdgb_unwrap(data->orientation.yaw + divernet_heading_offset);
  newMeas(KFNav::hdgb) = 1;
  ROS_ERROR("DIVER - ACOUSTIC - DEPTH: %f, HEADING: %f",measurements(KFNav::zb), measurements(KFNav::hdgb));
}

void Estimator3D::onSecond_usbl_fix(
    const underwater_msgs::USBLFix::ConstPtr& data)
{

  measurement_timeout = ros::Time::now();
  /*** Calculate measurement delay ***/
  double delay =
      double(calculateDelaySteps(currentTime - delay_time, currentTime)); // Totalno nepotrebno

//  double bear =
//      data->bearing -
//      180 * nav.getState()(KFNav::hdg) / M_PI + usbl_bearing_offset;  // Buddy pings Videoray

  double bear =
      data->bearing -
      180 * labust::math::wrapRad(nav.getState()(KFNav::hdg)) / M_PI + usbl_bearing_offset;  // Buddy pings Videoray
  double elev = 180 - data->elevation;

  const KFNav::vector& x = nav.getState();

  /*** Get USBL measurements ***/
  measurements(KFNav::range) =  data->range+usbl_offset;
  newMeas(KFNav::range) = enableRange && (data->range > 0.1);
  measDelay(KFNav::range) = delay;

  //measurements(KFNav::bearing) = bearing_unwrap(bear * M_PI / 180);
  measurements(KFNav::bearing) = bear * M_PI / 180;
  newMeas(KFNav::bearing) = enableBearing;
  measDelay(KFNav::bearing) = delay;

  measurements(KFNav::elevation) = elev * M_PI / 180;
  newMeas(KFNav::elevation) = enableElevation;
  measDelay(KFNav::elevation) = delay;

  ROS_ERROR("RANGE: %f, BEARING: %f deg, Time %d %d", data->range,
            labust::math::wrapDeg(bear), data->header.stamp.sec,
            data->header.stamp.nsec);

  /*** Force USBL position ***/
  const KFNav::matrix& covariance = nav.getStateCovariance();
  if(covariance(KFNav::xb, KFNav::xb) > cov_limit || covariance(KFNav::yb, KFNav::yb) > cov_limit)
  {
	measurements(KFNav::xb) = measurements(KFNav::xp)+(measurements(KFNav::range))*cos(measurements(KFNav::bearing)+measurements(KFNav::hdg));
	newMeas(KFNav::xb)  = 1;
	measurements(KFNav::yb) = measurements(KFNav::yp)+(measurements(KFNav::range))*sin(measurements(KFNav::bearing)+measurements(KFNav::hdg));
	newMeas(KFNav::yb)  = 1;
	ROS_ERROR("Forcing USBL position!!");
  }
}

void Estimator3D::onSecond_sonar_fix(
    const navcon_msgs::RelativePosition::ConstPtr& data)
{

  measurement_timeout = ros::Time::now();
  /*** Get sonar measurements ***/
  measurements(KFNav::sonar_range) = data->range+sonar_offset;
  newMeas(KFNav::sonar_range) = data->range > 0.1;

  //measurements(KFNav::sonar_bearing) = bearing_unwrap(data->bearing);
  measurements(KFNav::sonar_bearing) = data->bearing;
  newMeas(KFNav::sonar_bearing) = 1;

  ROS_ERROR("SONAR - RANGE: %f, BEARING: %f deg, TIME: %d %d", measurements(KFNav::sonar_range),
            data->bearing * 180 / M_PI, data->header.stamp.sec,
            data->header.stamp.nsec);
}

void Estimator3D::onSecond_camera_fix(
    const navcon_msgs::RelativePosition::ConstPtr& data)
{
  measurement_timeout = ros::Time::now();
  /*** Get sonar measurements ***/
  measurements(KFNav::camera_range) = data->range + camera_offset;
  newMeas(KFNav::camera_range) = (data->range > 0.1) && !std::isnan(data->range);

  //measurements(KFNav::camera_bearing) = bearing_unwrap(data->bearing + camera_bearing_offset*M_PI/180);
  //measurements(KFNav::camera_bearing) = camera_bearing_unwrap(data->bearing + camera_bearing_offset*M_PI/180);
  measurements(KFNav::camera_bearing) = data->bearing + camera_bearing_offset*M_PI/180;
  newMeas(KFNav::camera_bearing) = !std::isnan(data->bearing);

  measurements(KFNav::camera_hdgb) = hdgb_camera_unwrap(data->heading + diver_camera_heading_offset);
  newMeas(KFNav::camera_hdgb) = enable_camera_heading && (std::abs(data->heading) <= M_PI) && !std::isnan(data->heading);

  ROS_ERROR("CAMERA - RANGE: %f, BEARING: %f deg, TIME: %d %d", measurements(KFNav::camera_range),
            data->bearing * 180 / M_PI, data->header.stamp.sec,
            data->header.stamp.nsec);
  ROS_ERROR("DIVER - CAMERA - HEADING: %f", newMeas(KFNav::camera_hdgb)?measurements(KFNav::camera_hdgb):-9999.0);
}

/*********************************************************************
 *** Helper functions
 ********************************************************************/

void Estimator3D::processMeasurements()
{

  if((ros::Time::now() - measurement_timeout) > ros::Duration(meas_timeout_limit))
  {
	measurements(KFNav::ub) = 0;
	newMeas(KFNav::ub) = 1;
	if(display_counter++%50==0) ROS_ERROR("Measurement timeout");
  }

  /*** Publish local measurements ***/
  auv_msgs::NavSts::Ptr meas(new auv_msgs::NavSts());
  meas->body_velocity.x = measurements(KFNav::u);
  meas->body_velocity.z = measurements(KFNav::w);

  meas->position.north = measurements(KFNav::xp);
  meas->position.east = measurements(KFNav::yp);
  meas->position.depth = measurements(KFNav::zp);

  meas->orientation.yaw = labust::math::wrapRad(measurements(KFNav::hdg));
  meas->orientation_rate.yaw = measurements(KFNav::r);

  meas->header.stamp = ros::Time::now();
  meas->header.frame_id = "local";
  pubLocalStateMeas.publish(meas);

  /*** Publish second vehicle measurements ***/
  auv_msgs::NavSts::Ptr meas2(new auv_msgs::NavSts());
  meas2->body_velocity.x = measurements(KFNav::ub);
  //meas2->body_velocity.z = measurements(KFNav::wb);

  meas2->position.north = measurements(KFNav::xb);
  meas2->position.east = measurements(KFNav::yb);
  meas2->position.depth = measurements(KFNav::zb);

  meas2->orientation.yaw = labust::math::wrapRad(measurements(KFNav::psib));
  //meas2->orientation_rate.yaw = measurements(KFNav::rb);

  meas2->header.stamp = ros::Time::now();
  meas2->header.frame_id = "local";
  pubSecondStateMeas.publish(meas2);

  std_msgs::Float32::Ptr data(new std_msgs::Float32);
  data->data = measurements(KFNav::range);
  if(newMeas(KFNav::range)) pub_usbl_range.publish(data);

  data->data = labust::math::wrapRad(measurements(KFNav::bearing));
  if(newMeas(KFNav::bearing)) pub_usbl_bearing.publish(data);

  data->data = measurements(KFNav::sonar_range);
  if(newMeas(KFNav::sonar_range)) pub_sonar_range.publish(data);

  data->data = labust::math::wrapRad(measurements(KFNav::sonar_bearing));
  if(newMeas(KFNav::sonar_bearing)) pub_sonar_bearing.publish(data);

  data->data = measurements(KFNav::camera_range);
  if(newMeas(KFNav::camera_range)) pub_camera_range.publish(data);

  data->data = labust::math::wrapRad(measurements(KFNav::camera_bearing));
  if(newMeas(KFNav::camera_bearing)) pub_camera_bearing.publish(data);
}

void Estimator3D::publishState()
{
  auv_msgs::NavSts::Ptr state(new auv_msgs::NavSts());
  const KFNav::vector& estimate = nav.getState();
  state->gbody_velocity.x = estimate(KFNav::u);
  state->gbody_velocity.z = estimate(KFNav::w);

  state->orientation.yaw = labust::math::wrapRad(estimate(KFNav::hdg));
  state->orientation_rate.yaw = estimate(KFNav::r);

  state->position.north = estimate(KFNav::xp);
  state->position.east = estimate(KFNav::yp);
  state->position.depth = estimate(KFNav::zp);

  const KFNav::matrix& covariance = nav.getStateCovariance();
  state->position_variance.north = covariance(KFNav::xp, KFNav::xp);
  state->position_variance.east = covariance(KFNav::yp, KFNav::yp);
  state->position_variance.depth = covariance(KFNav::zp, KFNav::zp);
  state->orientation_variance.yaw = covariance(KFNav::psi, KFNav::psi);

  state->header.stamp = ros::Time::now();
  state->header.frame_id = "local";
  pubLocalStateHat.publish(state);

  /*** Publish second vehicle states */
  auv_msgs::NavSts::Ptr state2(new auv_msgs::NavSts());
  state2->gbody_velocity.x = estimate(KFNav::ub);
  //state2->gbody_velocity.z = estimate(KFNav::wb);

  // state2->orientation.yaw = 0;
  //state2->orientation.pitch = 0;
  state2->orientation.yaw = labust::math::wrapRad(estimate(KFNav::hdgb));
  //state2->orientation_rate.yaw = estimate(KFNav::rb);

  state2->position.north = estimate(KFNav::xb);
  state2->position.east = estimate(KFNav::yb);
  state2->position.depth = estimate(KFNav::zb);

  // state2->position.north =
  // measurements(KFNav::xp)+measurements(KFNav::sonar_range)*cos(measurements(KFNav::sonar_bearing)+measurements(KFNav::hdg));
  // state2->position.east =
  // measurements(KFNav::yp)+measurements(KFNav::sonar_range)*sin(measurements(KFNav::sonar_bearing)+measurements(KFNav::hdg));

  state2->position_variance.north = covariance(KFNav::xb, KFNav::xb);
  state2->position_variance.east = covariance(KFNav::yb, KFNav::yb);
  state2->position_variance.depth = covariance(KFNav::zb, KFNav::zb);
  state2->orientation_variance.yaw = covariance(KFNav::psib, KFNav::psib);

  state2->header.stamp = ros::Time::now();
  state2->header.frame_id = "local";
  pubSecondStateHat.publish(state2);

  navcon_msgs::RelativePosition::Ptr rel_pos(
      new navcon_msgs::RelativePosition());

  double delta_x = estimate(KFNav::xb) - estimate(KFNav::xp);
  double delta_y = estimate(KFNav::yb) - estimate(KFNav::yp);

  Eigen::Matrix2d R;
  double yaw = estimate(KFNav::hdg);
  R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  Eigen::Vector2d in, out;
  in << delta_x, delta_y;
  out = R.transpose() * in;

  rel_pos->x = out(0);
  rel_pos->y = out(1);

  Eigen::Matrix2d Pt;
  Eigen::Matrix2d P;
  P << covariance(KFNav::xb, KFNav::xb), covariance(KFNav::xb, KFNav::yb),
      covariance(KFNav::yb, KFNav::xb), covariance(KFNav::yb, KFNav::yb);
  Pt = R.transpose() * P * R;

  rel_pos->x_variance = Pt(0, 0);
  rel_pos->y_variance = Pt(1, 1);

  rel_pos->range = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
  rel_pos->bearing =
      labust::math::wrapRad(atan2(delta_y, delta_x) - estimate(KFNav::hdg));

  Eigen::Matrix2d J;
  J << rel_pos->x / rel_pos->range, rel_pos->y / rel_pos->range,
      -rel_pos->y / pow(rel_pos->range, 2),
      rel_pos->x / pow(rel_pos->range, 2);

  Pt = J * Pt * J.transpose();

  P_rng_bear_relative = Pt;

  rel_pos->range_variance = Pt(0, 0);
  rel_pos->bearing_variance = Pt(1, 1);

  rel_pos->header.stamp = ros::Time::now();
  rel_pos->header.frame_id = "local";
  pubSecondRelativePosition.publish(rel_pos);

  /*** Calculate trace ***/

  double traceP = covariance.trace();
  std_msgs::Float32::Ptr data(new std_msgs::Float32);
  data->data = traceP;
  pubCondP.publish(data);

  data->data = labust::math::wrapRad(estimate(KFNav::psib));
  pub_diver_course.publish(data);

}

int Estimator3D::calculateDelaySteps(double measTime, double arrivalTime)
{
  return floor((arrivalTime - measTime) / nav.Ts);
}

/*********************************************************************
 *** Main loop
 ********************************************************************/
void Estimator3D::start()
{
  ros::NodeHandle ph("~");
  double Ts(0.1);
  ph.param("Ts", Ts, Ts);
  ros::Rate rate(1 / Ts);
  nav.setTs(Ts);

  /*** Initialize time (for delay calculation) ***/
  currentTime = ros::Time::now().toSec();

  while (ros::ok())
  {
    /*** Process measurements ***/
    processMeasurements();

    /*** Store filter data ***/
    FilterState state;
    state.input = tauIn;
    state.meas = measurements;
    state.newMeas = newMeas;

    /*** Check if there are delayed measurements, and disable them in current
     * step ***/
    bool newDelayed(false);
    for (size_t i = 0; i < measDelay.size(); ++i)
    {
      if (measDelay(i))
      {
        state.newMeas(i) = 0;
        newDelayed = true;
      }
    }

    /*** Store x, P, R data ***/
    state.state = nav.getState();
    state.Pcov = nav.getStateCovariance();
    // state.Rcov = ; // In case of time-varying measurement covariance

    // ROS_ERROR_STREAM(state.Pcov);
    /*** Limit queue size ***/
    if (pastStates.size() > 1000)
    {
      pastStates.pop_front();
      // ROS_ERROR("Pop front");
    }
    pastStates.push_back(state);

    if (newDelayed && enableDelay)
    {
      /*** Check for maximum delay ***/
      int delaySteps = measDelay.maxCoeff();

      /*** If delay is bigger then buffer size assume that it is max delay ***/
      if (delaySteps >= pastStates.size())
        delaySteps = pastStates.size() - 1;

      /*** Position delayed measurements in past states container
           and store past states data in temporary stack ***/
      std::stack<FilterState> tmp_stack;
      for (size_t i = 0; i <= delaySteps; i++)
      {
        KFNav::vector tmp_cmp;
        tmp_cmp.setConstant(KFNav::measSize, i);
        if ((measDelay.array() == tmp_cmp.array()).any() && i != 0)
        {
          FilterState tmp_state = pastStates.back();
          for (size_t j = 0; j < measDelay.size(); ++j)
          {
            if (measDelay(j) == i)
            {
              tmp_state.newMeas(j) = 1;
              tmp_state.meas(j) = measurements(j);

              if (enableRejection)
              {
                /////////////////////////////////////////
                /// Outlier test
                /////////////////////////////////////////

            	if(alternate_outlier)
            	{

            		if (j == KFNav::range)
            		{
            			const KFNav::vector& x = tmp_state.state;
            			double rng = sqrt(pow((x(KFNav::xp)-x(KFNav::xb)),2)+pow((x(KFNav::yp)-x(KFNav::yb)),2)+pow((x(KFNav::zp)-x(KFNav::zb)),2));
                		double dist=fabs(rng - measurements(j));
                		newMeas(j) = (dist <= sqrt(P_rng_bear_relative(0,0)) + sqrt(nav.R0(j,j)));
                		if(!newMeas(j)) ROS_ERROR("USBL range outlier!");
            		}
            		if (j == KFNav::bearing)
            		{
            			const KFNav::vector& x = tmp_state.state;
            			double bear = bearing_unwrap(atan2(KFNav::yp-KFNav::yb,KFNav::xp-KFNav::xb) -1*x(KFNav::hdg), false);
                		double dist=fabs(bear - measurements(j));
                		newMeas(j) = (dist <= sqrt(P_rng_bear_relative(1,1)) + sqrt(nav.R0(j,j)));
            		}
            		if (j == KFNav::sonar_range)
            		{
            			const KFNav::vector& x = tmp_state.state;
            			double rng = sqrt(pow((x(KFNav::xp)-x(KFNav::xb)),2)+pow((x(KFNav::yp)-x(KFNav::yb)),2)+pow((x(KFNav::zp)-x(KFNav::zb)),2));
                		double dist=fabs(rng - measurements(j));
                		newMeas(j) = (dist <= sqrt(P_rng_bear_relative(0,0)) + sqrt(nav.R0(j,j)));
            		}
            		if (j == KFNav::sonar_bearing)
            		{
            			const KFNav::vector& x = tmp_state.state;
            			double bear = bearing_unwrap(atan2(KFNav::yp-KFNav::yb,KFNav::xp-KFNav::xb) -1*x(KFNav::hdg), false);
                		double dist=fabs(bear - measurements(j));
                		newMeas(j) = (dist <= sqrt(P_rng_bear_relative(1,1)) + sqrt(nav.R0(j,j)));
            		}


            	} else {
                if (j == KFNav::range)
                {
                  const KFNav::vector& x =
                      tmp_state.state;  ///// Treba li jos predikciju napravit?
                  double range = measurements(j);

                  Eigen::VectorXd input(Eigen::VectorXd::Zero(3));
                  // Eigen::VectorXd output(Eigen::VectorXd::Zero(1));

                  input << x(KFNav::xp) - x(KFNav::xb),
                      x(KFNav::yp) - x(KFNav::yb), x(KFNav::zp) - x(KFNav::zb);
                  // input << x(KFNav::u), x(KFNav::yp)-x(KFNav::yb),
                  // x(KFNav::zp)-x(KFNav::zb);

                  // output << measurements(KFNav::range);
                  double y_filt, sigma, w;
                  OR.step(input, measurements(KFNav::range), &y_filt, &sigma,
                          &w);
                  // ROS_INFO("Finished outlier rejection");

                  std_msgs::Float32 rng_msg;
                  // ROS_ERROR("w: %f",w);

                  double w_limit = 0.3 * std::sqrt(float(sigma));
                  w_limit = (w_limit > 1.0) ? 1.0 : w_limit;
                  if (w > w_limit)
                  {
                    //	rng_msg.data = range;
                    //	pubRangeFiltered.publish(rng_msg);
                    // pubRangeFiltered.publish(rng_msg);
                  }
                  else
                  {
                    tmp_state.newMeas(j) = 0;
                    ROS_ERROR("rng outlier!!!");
                  }
                  // std_msgs::Float32 rng_msg;

                  // rng_msg.data = w_limit;
                  // pubwk.publish(rng_msg);

                  // rng_msg.data = range;
                  // pubRange.publish(rng_msg);
                }

                /*								if(j == KFNav::bearing &&
                   tmp_state.newMeas(j-1) == 0)
                                {
                                  tmp_state.newMeas(j) = 0;

                                }*/

                if (j == KFNav::bearing)
                // if(j == KFNav::bearing || j == KFNav::sonar_bearing)
                {
                  const KFNav::vector& x =
                      tmp_state.state;  ///// Treba li jos predikciju napravit?
                  // double range = measurements(j);

                  Eigen::VectorXd input(Eigen::VectorXd::Zero(2));
                  // Eigen::VectorXd output(Eigen::VectorXd::Zero(1));

                  input << x(KFNav::xb) - x(KFNav::xp),
                      x(KFNav::yb) - x(KFNav::yp);
                  // input << x(KFNav::u), x(KFNav::yp)-x(KFNav::yb),
                  // x(KFNav::zp)-x(KFNav::zb);

                  // output << measurements(KFNav::range);
                  double y_filt, sigma, w;
                  OR_b.step(input, measurements(KFNav::bearing), &y_filt,
                            &sigma, &w);
                  // ROS_INFO("Finished outlier rejection");

                  std_msgs::Float32 rng_msg;
                  // ROS_ERROR("w: %f",w);

                  double w_limit = 0.3 * std::sqrt(float(sigma));
                  w_limit = (w_limit > 1.0) ? 1.0 : w_limit;
                  if (w > w_limit)
                  {
                    // rng_msg.data = range;
                    // pubRangeFiltered.publish(rng_msg);
                    // pubRangeFiltered.publish(rng_msg);
                  }
                  else
                  {
                    tmp_state.newMeas(j) = 0;
                    ROS_ERROR("bearing outlier!!!");
                  }
                  // std_msgs::Float32 rng_msg;

                  // rng_msg.data = w_limit;
                  // pubwk.publish(rng_msg);

                  // rng_msg.data = range;
                  // pubRange.publish(rng_msg);
                }
            	}
              }

              //////////////////////////////////////////

              // ROS_ERROR("Dodano mjerenje. Delay:%d",i);
            }
          }
          tmp_stack.push(tmp_state);
        }
        else
        {
          tmp_stack.push(pastStates.back());
        }
        pastStates.pop_back();
      }

      /*** Load past state and covariance for max delay time instant ***/
      FilterState state_p = tmp_stack.top();
      nav.setStateCovariance(state_p.Pcov);
      nav.setState(state_p.state);

      /*** Pass through stack data and recalculate filter states ***/
      while (!tmp_stack.empty())
      {
        state_p = tmp_stack.top();
        tmp_stack.pop();
        pastStates.push_back(state_p);

        /*** Estimation ***/
        nav.predict(state_p.input);
        bool newArrived(false);
        for (size_t i = 0; i < state_p.newMeas.size(); ++i)
          if ((newArrived = state_p.newMeas(i)))
            break;
        if (newArrived)
          nav.correct(nav.update(state_p.meas, state_p.newMeas));
      }
    }
    else
    {
      /*** Estimation ***/
      nav.predict(tauIn);
      bool newArrived(false);
      for (size_t i = 0; i < newMeas.size(); ++i)
        if ((newArrived = newMeas(i)))
          break;
      if (newArrived)
        nav.correct(nav.update(measurements, newMeas));
    }

    newMeas.setZero();  // razmisli kako ovo srediti
    measDelay.setZero();
    publishState();

    // ROS_ERROR_STREAM(nav.getStateCovariance());

    /*** Send the base-link transform ***/
    /*geometry_msgs::TransformStamped transform;
    transform.transform.translation.x = nav.getState()(KFNav::xp);
    transform.transform.translation.y = nav.getState()(KFNav::yp);
    transform.transform.translation.z = nav.getState()(KFNav::zp);
    labust::tools::quaternionFromEulerZYX(0, 0, nav.getState()(KFNav::psi),
    transform.transform.rotation);
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
  ros::init(argc, argv, "nav_3d");
  Estimator3D nav;
  nav.start();
  return 0;
}
