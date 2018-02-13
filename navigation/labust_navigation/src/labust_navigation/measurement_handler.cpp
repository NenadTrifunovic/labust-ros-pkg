/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, LABUST, UNIZG-FER
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
 *********************************************************************/

#include <labust/navigation/measurement_handler.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

using namespace labust::navigation;

MeasurementHandler::MeasurementHandler()
  : measurements(KFNav::vector::Zero(KFNav::stateNum))
  , newMeas(KFNav::vector::Zero(KFNav::stateNum))
  , status_handler_("Navigation", "navigation")
{
  this->onInit();
};

MeasurementHandler::~MeasurementHandler()
{
}

void MeasurementHandler::onInit()
{
  ros::NodeHandle nh, ph("~");

  // Publishers
  pub_state_hat = nh.advertise<auv_msgs::NavSts>("state_hat", 1);
  pub_state_meas = nh.advertise<auv_msgs::NavSts>("measurement", 1);

  pub_gps_odom =
      nh.advertise<nav_msgs::Odometry>("measurement/gps_odometry", 1);
  pub_imu = nh.advertise<sensor_msgs::Imu>("measurement/imu", 1);
  pub_iusbl = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "measurement/iusbl_pose", 1);
  pub_dvl_twist = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(
      "measurement/dvl_twist", 1);

  // Configure sensor handlers.
  lpos.configure(nh);
  gps.configure(nh);
  dvl.configure(nh);
  imu.configure(nh);
  imu.setGpsHandler(&gps);
  iusbl.configure(nh);

  /*** Diagnostic handler initialization ***/
  status_handler_.addKeyValue("GPS measurement");
  status_handler_.addKeyValue("IMU measurement");
  status_handler_.addKeyValue("DVL measurement");
  status_handler_.addKeyValue("iUSBL measurement");
  status_handler_.addKeyValue("Filter state");
  status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::OK);
  // status_handler_.setEntityMessage("Status handler initialized.");
  status_handler_.setEntityMessage("");
  status_handler_.publishStatus();
}

void MeasurementHandler::processMeasurements()
{
  // boost::mutex::scoped_lock l(meas_mux);

  if (lpos.newArrived())
  {
  }

  if (gps.newArrived())
  {
    measurements(KFNav::xp) = gps.position().first;
    measurements(KFNav::yp) = gps.position().second;
  }

  if (imu.newArrived())
  {
    measurements(KFNav::phi) = imu.orientation()[ImuHandler::roll];
    measurements(KFNav::theta) = imu.orientation()[ImuHandler::pitch];
    measurements(KFNav::psi) = imu.orientation()[ImuHandler::yaw];

    measurements(KFNav::p) = imu.rate()[ImuHandler::p];
    measurements(KFNav::q) = imu.rate()[ImuHandler::q];
    measurements(KFNav::r) = imu.rate()[ImuHandler::r];
  }

  if (dvl.newArrived())
  {
    double vx = dvl.body_speeds()[DvlHandler::u];
    double vy = dvl.body_speeds()[DvlHandler::v];
    double vz = dvl.body_speeds()[DvlHandler::w];
  }

  if (iusbl.newArrived())
  {
    // USBL measurements
    if (!(newMeas(KFNav::xp) || newMeas(KFNav::yp)))
    {
      if ((newMeas(KFNav::xp) = newMeas(KFNav::yp) = iusbl.newArrived()))
      {
        measurements(KFNav::xp) = iusbl.position()[0];
        measurements(KFNav::yp) = iusbl.position()[1];
      }
    }
  }

  bool gps_timeout(false);
  bool lpos_timeout(false);
  bool imu_timeout(false);
  bool dvl_timeout(false);
  bool iusbl_timeout(false);

  gps_timeout = (ros::Time::now() - gps.newArrivedTimestamp()).toSec() >
                measurement_timeout;
  lpos_timeout = (ros::Time::now() - lpos.newArrivedTimestamp()).toSec() >
                 measurement_timeout;
  imu_timeout = (ros::Time::now() - imu.newArrivedTimestamp()).toSec() >
                measurement_timeout;
  dvl_timeout = (ros::Time::now() - dvl.newArrivedTimestamp()).toSec() >
                measurement_timeout;
  iusbl_timeout = (ros::Time::now() - iusbl.newArrivedTimestamp()).toSec() >
                  measurement_timeout;

  if (gps_timeout)
  {
    status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::WARN);
    status_handler_.setEntityMessage("Measurements missing.");
    status_handler_.updateKeyValue("GPS measurement", "No measurement");
  }
  else
  {
    status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::OK);
    status_handler_.setEntityMessage("");
    status_handler_.updateKeyValue("GPS measurement", "OK");
  }

  if (imu_timeout)
  {
    status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::WARN);
    status_handler_.setEntityMessage("Measurements missing.");
    status_handler_.updateKeyValue("IMU measurement", "No measurement");
  }
  else
  {
    status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::OK);
    status_handler_.setEntityMessage("");
    status_handler_.updateKeyValue("IMU measurement", "OK");
  }

  if (dvl_timeout)
  {
    status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::WARN);
    status_handler_.setEntityMessage("Measurements missing.");
    status_handler_.updateKeyValue("DVL measurement", "No measurement");
  }
  else
  {
    status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::OK);
    status_handler_.setEntityMessage("");
    status_handler_.updateKeyValue("DVL measurement", "OK");
  }

  if (iusbl_timeout)
  {
    status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::WARN);
    status_handler_.setEntityMessage("Measurements missing.");
    status_handler_.updateKeyValue("iUSBL measurement", "No measurement");
  }
  else
  {
    status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::OK);
    status_handler_.setEntityMessage("");
    status_handler_.updateKeyValue("USBL measurement", "OK");
  }

  // Publish measurements
  auv_msgs::NavSts::Ptr meas(new auv_msgs::NavSts());
  meas->body_velocity.x = measurements(KFNav::u);
  meas->body_velocity.y = measurements(KFNav::v);
  meas->body_velocity.z = measurements(KFNav::w);

  meas->gbody_velocity.x = measurements(KFNav::u);
  meas->gbody_velocity.y = measurements(KFNav::v);
  meas->gbody_velocity.z = measurements(KFNav::w);

  meas->status = dvl.has_bottom_lock() ?
                     auv_msgs::NavSts::STATUS_GROUND_VELOCITY_OK :
                     auv_msgs::NavSts::STATUS_WATER_VELOCITY_OK;

  meas->position.north = measurements(KFNav::xp);
  meas->position.east = measurements(KFNav::yp);
  meas->position.depth = measurements(KFNav::zp);
  meas->altitude = measurements(KFNav::altitude);

  meas->orientation.roll = measurements(KFNav::phi);
  meas->orientation.pitch = measurements(KFNav::theta);
  meas->orientation.yaw = labust::math::wrapRad(measurements(KFNav::psi));
  if (useYawRate)
  {
    meas->orientation_rate.roll = imu.rate()[ImuHandler::p];
    meas->orientation_rate.pitch = imu.rate()[ImuHandler::q];
    meas->orientation_rate.yaw = measurements(KFNav::r);
  }

  meas->origin.latitude = gps.origin().first;
  meas->origin.longitude = gps.origin().second;
  meas->global_position.latitude = gps.latlon().first;
  meas->global_position.longitude = gps.latlon().second;

  meas->header.stamp = ros::Time::now();
  meas->header.frame_id = "local";
  stateMeas.publish(meas);
}

void MeasurementHandler::publishState()
{
  auv_msgs::NavSts::Ptr state(new auv_msgs::NavSts());
  const KFNav::vector& estimate = nav.getState();
  state->body_velocity.x = estimate(KFNav::u);
  state->body_velocity.y = estimate(KFNav::v);
  state->body_velocity.z = estimate(KFNav::w);

  Eigen::Matrix2d R;
  double yaw = labust::math::wrapRad(estimate(KFNav::psi));
  R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  Eigen::Vector2d in, out;
  in << estimate(KFNav::xc), estimate(KFNav::yc);
  out = R.transpose() * in;

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
  labust::tools::quaternionFromEulerZYX(M_PI, 0, M_PI / 2, qrot);
  Eigen::Vector3d ned;
  ned << state->position.north, state->position.east, state->position.depth;
  double h;
  Eigen::Vector3d enu = qrot.toRotationMatrix().transpose() * ned;
  proj.Reverse(enu(0), enu(1), enu(2), state->global_position.latitude,
               state->global_position.longitude, h);

  const KFNav::matrix& covariance = nav.getStateCovariance();
  state->position_variance.north = covariance(KFNav::xp, KFNav::xp);
  state->position_variance.east = covariance(KFNav::yp, KFNav::yp);
  state->position_variance.depth = covariance(KFNav::zp, KFNav::zp);
  state->orientation_variance.roll = covariance(KFNav::phi, KFNav::phi);
  state->orientation_variance.pitch = covariance(KFNav::theta, KFNav::theta);
  state->orientation_variance.yaw = covariance(KFNav::psi, KFNav::psi);

  state->header.stamp = ros::Time::now();
  state->header.frame_id = "local";
  stateHat.publish(state);

  if (diagnostic_error_flag)
  {
    status_handler_.setEntityMessage("No measurements.");
    status_handler_.updateKeyValue("Filter state", "Error state.");
    status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::ERROR);
  }
  else if (covariance(KFNav::xp, KFNav::xp) > 10 ||
           covariance(KFNav::xp, KFNav::xp) > 10)
  {
    status_handler_.setEntityMessage("Large position covariance.");
    status_handler_.updateKeyValue("Filter state", "Large position "
                                                   "covariance.");
    status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::WARN);
  }
  else
  {
    status_handler_.updateKeyValue("Filter state", "OK.");
  }

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
  turns->data = estimate(KFNav::psi) / (2 * M_PI);
  turns_pub.publish(turns);

  std_msgs::Float32::Ptr altcov(new std_msgs::Float32());
  altcov->data = covariance(KFNav::altitude, KFNav::altitude);
  altitude_cov.publish(altcov);
}

void MeasurementHandler::start()
{
  ros::NodeHandle ph("~");
  double Ts(0.1);
  ph.param("Ts", Ts, Ts);
  ros::Rate rate(1 / Ts);
  nav.setTs(Ts);

  while (ros::ok())
  {
    nav.predict(tauIn);
    processMeasurements();
    bool newArrived(false);
    boost::mutex::scoped_lock l(meas_mux);
    for (size_t i = 0; i < newMeas.size(); ++i)
      if ((newArrived = newMeas(i)))
        break;

    // Update sensor flag
    bool updateDVL = !newMeas(KFNav::r);

    std::ostringstream out;
    out << newMeas;
    ROS_DEBUG("Measurements %d:%s", KFNav::psi, out.str().c_str());

    if (newArrived)
      nav.correct(nav.update(measurements, newMeas));
    KFNav::vector tcstate = nav.getState();
    if (tcstate(KFNav::buoyancy) < -30)
      tcstate(KFNav::buoyancy) = -10;
    if (tcstate(KFNav::buoyancy) > 0)
      tcstate(KFNav::buoyancy) = 0;
    if (tcstate(KFNav::altitude) < 0)
      tcstate(KFNav::altitude) = 0;
    nav.setState(tcstate);
    l.unlock();
    publishState();

    // Send the base-link transform
    geometry_msgs::TransformStamped transform;
    KFNav::vectorref cstate = nav.getState();
    // Update DVL sensor
    if (updateDVL)
      dvl.current_r(cstate(KFNav::r));

    if (enable_base_pose_tf)
    {
      // local -> base_pose
      transform.transform.translation.x = cstate(KFNav::xp);
      transform.transform.translation.y = cstate(KFNav::yp);
      transform.transform.translation.z = cstate(KFNav::zp);
      labust::tools::quaternionFromEulerZYX(0, 0, 0,
                                            transform.transform.rotation);
      if (absoluteEKF)
      {
        transform.child_frame_id = tf_prefix + "base_pose_abs";
      }
      else
      {
        transform.child_frame_id = tf_prefix + "base_pose";
      }
      transform.header.frame_id = tf_prefix + "local";
      transform.header.stamp = ros::Time::now();
      broadcaster.sendTransform(transform);
    }

    if (enable_base_link_tf)
    {
      // base_pose->base_link
      transform.transform.translation.x = 0;
      transform.transform.translation.y = 0;
      transform.transform.translation.z = 0;
      labust::tools::quaternionFromEulerZYX(
          cstate(KFNav::phi), cstate(KFNav::theta), cstate(KFNav::psi),
          transform.transform.rotation);
      if (absoluteEKF)
      {
        transform.child_frame_id = tf_prefix + "base_link_abs";
      }
      else
      {
        transform.child_frame_id = tf_prefix + "base_link";
      }
      transform.header.frame_id = tf_prefix + "base_pose";
      broadcaster.sendTransform(transform);
    }

    /*** Publish diagnostic status ***/
    status_handler_.publishStatus();

    rate.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "measurement_handler");
  MeasurementHandler mh;
  mh.start();
  return 0;
}
