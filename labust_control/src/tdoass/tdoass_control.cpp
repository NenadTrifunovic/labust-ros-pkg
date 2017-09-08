/*********************************************************************
 * tdoass_control.cpp
 *
 *  Created on: Sep 6, 2017
 *      Author: Filip Mandic
 *
 ********************************************************************/

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
 *********************************************************************/
#include <labust/control/tdoass/tdoass_control.h>

using namespace labust::control;

TDOASSControl::TDOASSControl()
  : toa1_old(0)
  , toa2_old(0)
  , toa1(0)
  , toa2(0)
  , tdoa(0)
  , speed_of_sound(1500.0)
  , veh_type(SLAVE)
  , baseline(5.0)
  , m(1.0)
  , epsilon(1.0)
  , es_controller(1, 0.1)
  , eta_filter_state_k0(2, 0)
  , eta_filter_state_k1(2, 0)
  , ts(0.1)
  , w1(1)
  , w2(1)
  , k1(1)
  , center_ref(auv_msgs::BodyVelocityReq())
  , tf_listener(tf_buffer)
  , master_active_flag(false)
{
}

TDOASSControl::~TDOASSControl()
{
}

void TDOASSControl::init()
{
  ros::NodeHandle nh, ph("~");

  bool master_flag;
  ph.getParam("master", master_flag);
  setAsMaster(master_flag);

  state[CENTER] = auv_msgs::NavSts();
  state[MASTER] = auv_msgs::NavSts();
  state[SLAVE] = auv_msgs::NavSts();

  link_names[CENTER] = "center_frame";
  link_names[MASTER] = "master_frame";
  link_names[SLAVE] = "slave_frame";

  sub_veh1_state =
      nh.subscribe("veh1/state", 1, &TDOASSControl::onVeh1State, this);
  sub_veh2_state =
      nh.subscribe("veh2/state", 1, &TDOASSControl::onVeh2State, this);
  sub_veh1_toa = nh.subscribe("veh1/toa", 1, &TDOASSControl::onVeh1Toa, this);
  sub_veh2_toa = nh.subscribe("veh2/toa", 1, &TDOASSControl::onVeh2Toa, this);
  sub_veh2_ref =
      nh.subscribe("veh2/state_ref", 1, &TDOASSControl::onSlaveRef, this);

  if (isMaster())
  {
    pub_veh1_ref = nh.advertise<auv_msgs::NavSts>("veh1/state_ref", 1);
    pub_veh2_ref = nh.advertise<auv_msgs::NavSts>("veh2/state_ref", 1);

    pub_tdoa = nh.advertise<std_msgs::Float64>("tdoa", 1);
    pub_delta = nh.advertise<std_msgs::Float64>("delta", 1);
    pub_master_active = nh.advertise<std_msgs::Bool>("master/active", 1);

    initializeController();
  }
  else
  {
    dp_srv = nh.serviceClient<misc_msgs::DynamicPositioningPrimitiveService>(
        "commander/primitive/dynamic_positioning");
    sub_master_active =
        nh.subscribe("master/active", 1, &TDOASSControl::onMasterActive, this);
    pub_slave_pos_ref =
        nh.advertise<geometry_msgs::PointStamped>("slave/pos_ref", 1);
    pub_slave_hdg_ref = nh.advertise<std_msgs::Float32>("slave/hdg_ref", 1);
  }
}

void TDOASSControl::initializeController()
{
  ROS_INFO("Initializing extremum seeking controller...");

  ros::NodeHandle nh;

  double sin_amp = 0.1;
  double sin_freq = 2 * M_PI / 16;
  double corr_gain = -1;
  double high_pass_pole = 3 / 16;
  double low_pass_pole = 0;
  double comp_zero = 0;
  double comp_pole = 0;
  double sampling_time = 1.0;

  nh.param("esc/sin_amp", sin_amp, sin_amp);
  nh.param("esc/sin_freq", sin_freq, sin_freq);
  nh.param("esc/corr_gain", corr_gain, corr_gain);
  nh.param("esc/high_pass_pole", high_pass_pole, high_pass_pole);
  nh.param("esc/low_pass_pole", low_pass_pole, low_pass_pole);
  nh.param("esc/comp_zero", comp_zero, comp_zero);
  nh.param("esc/comp_pole", comp_pole, comp_pole);
  nh.param("esc/sampling_time", sampling_time, sampling_time);

  // esc_Ts = sampling_time;

  es_controller.initController(sin_amp, sin_freq, corr_gain, high_pass_pole,
                               low_pass_pole, comp_zero, comp_pole,
                               sampling_time);

  disable_axis[x] = 0;
  disable_axis[y] = 0;
  disable_axis[yaw] = 0;

  ROS_ERROR("Extremum seeking controller initialized.");
}

void TDOASSControl::idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
          const auv_msgs::BodyVelocityReq& track)
{
  if (isMaster())
  {
    std_msgs::Bool flag;
    flag.data = false;
    pub_master_active.publish(flag);
  }
}

auv_msgs::BodyVelocityReqPtr TDOASSControl::step(
    const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state_hat)
{
  if (isMaster())
  {
    std_msgs::Bool flag;
    flag.data = true;
    pub_master_active.publish(flag);
    // Publish transforms.
    auv_msgs::NavSts offset;
    offset.position.east = -baseline / 2;
    broadcastTransform(offset, link_names[MASTER], link_names[CENTER]);
    broadcastTransform(offset, link_names[CENTER], link_names[SLAVE]);

    // TODO check if new state and toa measurements are used.
    // TODO decide what executes at higher frequncy.
    if (calcluateTimeDifferenceOfArrival())
    {
      double delta = getNormalizedDifferenceOfArrivalMeters();
      double cost = std::pow(delta, 2);
      double yaw_rate = state[MASTER].orientation_rate.yaw;

      bool publish_flag(true);
      if (publish_flag)
      {
        std_msgs::Float64 data;
        data.data = getTimeDifferenceOfArrival();
        pub_tdoa.publish(data);
        data.data = delta;
        pub_delta.publish(data);
      }
      yawRateControl(center_ref, delta, cost);
      // TODO check at which frequncy perturbation is set. (probably to low. add
      // flag
      // which updates )
      surgeSpeedControl(center_ref, delta, cost,
                        etaFilterStep(delta, yaw_rate));
    }
    // TODO Check if one step estimate is needed.
    // calculateSlaveReference();

    auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());
    *nu = allocateSpeed(center_ref);
    nu->header.stamp = ros::Time::now();
    nu->goal.requester = "tdoass_controller";
    labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);

    return nu;
  }
  else
  {
    // Slave actions.
    ROS_ERROR("DEBUG_slave");
    // TODO check this.
    disable_axis[x] = 1;
    disable_axis[y] = 1;
    disable_axis[yaw] = 1;
    auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());
    nu->header.stamp = ros::Time::now();
    nu->goal.requester = "tdoass_controller";
    labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);
    return nu;
  }
}

bool TDOASSControl::setAsMaster(bool flag)
{
  if (flag)
    veh_type = MASTER;
  else
    veh_type = SLAVE;
  return true;
}

bool TDOASSControl::isMaster()
{
  return (veh_type == MASTER) ? true : false;
}

bool TDOASSControl::calcluateTimeDifferenceOfArrival()
{
  // TODO Add timeout in case one measurement does not arrive.
  if (toa1 != toa1_old && toa2 != toa2_old)
  {
    tdoa = (toa1 - toa2).toSec();
    toa1_old = toa1;
    toa2_old = toa2;
    return true;
  }
  return false;
}

double TDOASSControl::getTimeDifferenceOfArrival()
{
  return tdoa;
}

double TDOASSControl::getDifferenceOfArrivalMeters()
{
  return tdoa * speed_of_sound;
}

double TDOASSControl::getNormalizedDifferenceOfArrivalMeters()
{
  return tdoa * speed_of_sound / baseline;
}

auv_msgs::BodyVelocityReq
TDOASSControl::allocateSpeed(auv_msgs::BodyVelocityReq req)
{
  auv_msgs::BodyVelocityReq master_ref;
  double yaw = state[MASTER].orientation.yaw;
  double u_ref = req.twist.linear.x;
  double yaw_rate_ref = req.twist.angular.z;

  Eigen::Vector2d out, in;
  Eigen::Matrix2d R;

  in[0] = (u_ref - baseline / 2 * yaw_rate_ref) * std::cos(yaw);
  in[1] = (u_ref + baseline / 2 * yaw_rate_ref) * std::sin(yaw);

  R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  out = R.transpose() * in;

  master_ref.twist.linear.x = out[0];
  master_ref.twist.linear.y = out[1];
  master_ref.twist.angular.z = yaw_rate_ref;

  return master_ref;
}

auv_msgs::NavSts TDOASSControl::calculateSlaveReference()
{
  // geometry_msgs::TransformStamped transformStamped;
  // try
  // {
  //   transformStamped = tf_buffer.lookupTransform(
  //       link_names[SLAVE], link_names[MASTER], ros::Time(0));
  // }
  // catch (tf2::TransformException& ex)
  // {
  //   ROS_WARN("%s", ex.what());
  // }
  return auv_msgs::NavSts();
}

void TDOASSControl::surgeSpeedControl(auv_msgs::BodyVelocityReq& req,
                                      double delta, double cost, double eta)
{
  const int n = 3;
  double u_amp, u_zeta, u_dir;
  u_dir = labust::math::sgn(eta);
  u_zeta = std::pow(std::abs(eta), n) / std::pow(std::abs(eta) + epsilon, n);
  u_amp = (1 - std::tanh(m * cost));
  req.twist.linear.x = u_amp * u_zeta * u_dir;
}

void TDOASSControl::yawRateControl(auv_msgs::BodyVelocityReq& req, double delta,
                                   double cost)
{
  req.twist.angular.z = (es_controller.step(cost))[0];
  ROS_ERROR("YAW_REF: %f", req.twist.angular.z);
}

double TDOASSControl::etaFilterStep(double delta, double yaw_rate)
{
  // Calculated using Euler method.
  eta_filter_state_k0 = eta_filter_state_k1;
  eta_filter_state_k1[0] += ts * (-w1 * eta_filter_state_k0[0] + delta);
  eta_filter_state_k1[1] +=
      ts * (-w2 * eta_filter_state_k0[1] +
            k1 * yaw_rate * (delta - w1 * eta_filter_state_k0[0]));
  return eta_filter_state_k1[1];
}

void TDOASSControl::onVeh1State(const auv_msgs::NavSts::ConstPtr& msg)
{
  state[MASTER] = *msg;
  // if (isMaster()){}
  // step();
}

void TDOASSControl::onVeh2State(const auv_msgs::NavSts::ConstPtr& msg)
{
  state[SLAVE] = *msg;
  // if (!isMaster()){}
  // step();
}

void TDOASSControl::onVeh1Toa(const std_msgs::Time::ConstPtr& msg)
{
  toa1 = msg->data;
}

void TDOASSControl::onVeh2Toa(const std_msgs::Time::ConstPtr& msg)
{
  toa2 = msg->data;
}

void TDOASSControl::onSlaveRef(const auv_msgs::NavSts::ConstPtr& msg)
{
  if (!isMaster())
  {
    geometry_msgs::PointStamped pos_ref;
    std_msgs::Float32 hdg_ref;
    pos_ref.point.x = msg->position.north;
    pos_ref.point.y = msg->position.east;
    hdg_ref.data = msg->orientation.yaw;
    pub_slave_pos_ref.publish(pos_ref);
    pub_slave_hdg_ref.publish(hdg_ref);
  }
}

void TDOASSControl::onMasterActive(const std_msgs::Bool::ConstPtr& msg)
{
  if (!isMaster())
  {
    if (master_active_flag != msg->data)
    {
      master_active_flag = msg->data;
      if (master_active_flag)
      {
        misc_msgs::DynamicPositioningPrimitiveService srv;
        srv.request.north_enable = true;
        srv.request.east_enable = true;
        srv.request.heading_enable = true;
        srv.request.target_topic_enable = true;
        srv.request.track_heading_enable = true;
        srv.request.target_topic = "/slave/pos_ref";
        srv.request.heading_topic = "/slave/hdg_ref";
        dp_srv.call(srv);
      }
      else
      {
        std_srvs::Trigger srv;
        ros::service::call("commander/stop_mission", srv);
      }
    }
  }
}

void TDOASSControl::broadcastTransform(auv_msgs::NavSts& state,
                                       std::string& frame_id,
                                       std::string& child_frame_id)
{
  // TODO Switch to static tf frame.
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = frame_id;
  transformStamped.child_frame_id = child_frame_id;
  transformStamped.transform.translation.x = state.position.north;
  transformStamped.transform.translation.y = state.position.east;
  transformStamped.transform.translation.z = state.position.depth;
  tf2::Quaternion q;
  q.setRPY(0, 0, state.orientation.yaw);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  transform_broadcaster.sendTransform(transformStamped);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tdoass_control");

  /***
  template <
      class Controller,
      class Enable = NoEnable,
      class Windup = NoWindup,
      class OutputType = auv_msgs::BodyVelocityReq,
      class InputType = auv_msgs::NavSts,
      class ReferenceType = auv_msgs::NavSts
      >
  ***/

  labust::control::HLControl<
      labust::control::TDOASSControl, labust::control::EnableServicePolicy,
      labust::control::WindupPolicy<auv_msgs::BodyForceReq> >
      controller;
  ros::spin();

  return 0;
}
