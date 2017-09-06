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
{
  init();
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

  sub_veh1_state =
      nh.subscribe("veh1/state", 1, &TDOASSControl::onVeh1State, this);
  sub_veh2_state =
      nh.subscribe("veh2/state", 1, &TDOASSControl::onVeh2State, this);
  sub_veh1_toa = nh.subscribe("veh1/toa", 1, &TDOASSControl::onVeh1Toa, this);
  sub_veh2_toa = nh.subscribe("veh2/toa", 1, &TDOASSControl::onVeh2Toa, this);

  pub_veh1_ref = nh.advertise<auv_msgs::NavSts>("veh1/state_ref", 1);
  pub_veh2_ref = nh.advertise<auv_msgs::NavSts>("veh2/state_ref", 1);

  pub_tdoa = nh.advertise<std_msgs::Float64>("tdoa", 1);
  pub_delta = nh.advertise<std_msgs::Float64>("delta", 1);

  state[CENTER] = auv_msgs::NavSts();
  state[MASTER] = auv_msgs::NavSts();
  state[SLAVE] = auv_msgs::NavSts();

  link_names[CENTER] = "center_frame";
  link_names[MASTER] = "master_frame";
  link_names[SLAVE] = "slave_frame";
}

void TDOASSControl::initializeController()
{
  // ROS_INFO("Initializing extremum seeking controller...");
  // 
  // ros::NodeHandle nh;
  // 
  // double sin_amp = 0.2;
  // double sin_demodulation_amp = 1;
  // double sin_freq = 0.09;
  // double corr_gain = -5;
  // double high_pass_pole = 3;
  // double low_pass_pole = 0;
  // double comp_zero = 0;
  // double comp_pole = 0;
  // double sampling_time = 0.1;
  // 
  // nh.param("esc_sin_amp", sin_amp, sin_amp);
  // nh.param("esc_sin_demodulation_amp", sin_amp, sin_amp);
  // nh.param("esc_sin_freq", sin_freq, sin_freq);
  // nh.param("esc_corr_gain", corr_gain, corr_gain);
  // nh.param("esc_high_pass_pole", high_pass_pole, high_pass_pole);
  // nh.param("esc_low_pass_pole", low_pass_pole, low_pass_pole);
  // nh.param("esc_comp_zero", comp_zero, comp_zero);
  // nh.param("esc_comp_pole", comp_pole, comp_pole);
  // nh.param("esc_sampling_time", sampling_time, sampling_time);

  //esc_Ts = sampling_time;

  // es_controller.initController(sin_amp, sin_demodulation_amp, sin_freq,
  // corr_gain, high_pass_pole, low_pass_pole, comp_zero, comp_pole,
  // sampling_time);

  ROS_INFO("Extremum seeking controller initialized.");
  
    ROS_INFO("Initializing extremum seeking controller...");

    ros::NodeHandle nh;

    double sin_amp = 0.2;
    double sin_freq = 0.09;
    double corr_gain = -5;
    double high_pass_pole = 3;
    double low_pass_pole = 0;
    double comp_zero = 0;
    double comp_pole = 0;
    double sampling_time = 0.1;

    nh.param("esc/sin_amp", sin_amp, sin_amp);
    nh.param("esc/sin_freq", sin_freq, sin_freq);
    nh.param("esc/corr_gain", corr_gain, corr_gain);
    nh.param("esc/high_pass_pole", high_pass_pole, high_pass_pole);
    nh.param("esc/low_pass_pole", low_pass_pole, low_pass_pole);
    nh.param("esc/comp_zero", comp_zero, comp_zero);
    nh.param("esc/comp_pole", comp_pole, comp_pole);
    nh.param("esc/sampling_time", sampling_time, sampling_time);

    //esc_Ts = sampling_time;

    es_controller.initController(sin_amp, sin_freq, corr_gain, high_pass_pole,
                                  low_pass_pole, comp_zero, comp_pole,
                                  sampling_time);

    disable_axis[x] = 0;
    disable_axis[y] = 0;

    ROS_INFO("Extremum seeking controller initialized.");  
}

auv_msgs::BodyVelocityReqPtr TDOASSControl::step(const auv_msgs::NavSts& ref,
                                                 const auv_msgs::NavSts& state_hat)
{
  if (isMaster())
  {
    // Publish transforms.
    auv_msgs::NavSts offset;
    offset.position.east = -baseline / 2;
    broadcastTransform(offset, link_names[MASTER], link_names[CENTER]);
    broadcastTransform(offset, link_names[CENTER], link_names[SLAVE]);

    // TODO check if new state and toa measurements are used.
    // TODO decide what executes at higher frequncy.
    if (calcluateTimeDifferenceOfArrival())
    {
      ROS_ERROR("DEBUG");
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
      // surgeSpeedControl(center_ref, delta, cost,
      //                  etaFilterStep(delta, yaw_rate));
    }
    auv_msgs::BodyVelocityReq vel_req = allocateSpeed(center_ref);
  }
  // if((count++)%20 == 0){
 // int t_step = esc_Ts / Ts;
  // if(newRange){
  //if ((count++) % t_step == 0)
 // {
  //  newRange = false;

    Eigen::Vector2d out, in, tmp;
    Eigen::Matrix2d R;

    // in = esc_controller.step(ref.data*ref.data);
//    tmp = es_controller.step(ref.data);

    // ROS_ERROR("cost:");
    // ROS_ERROR_STREAM(ref.data);
    // ROS_ERROR("control:");
    // ROS_ERROR_STREAM(tmp);

    auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());
    nu->header.stamp = ros::Time::now();
    nu->goal.requester = "tdoass_controller";
    labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);

    in[0] = tmp[1];
    in[1] = tmp[0];

    double yaw = state[MASTER].orientation.yaw;
    R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
    out = R.transpose() * in;

    // Switched axes !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    nu->twist.linear.x = out[x];
    nu->twist.linear.y = out[y];

    //nu_past = nu;
    return nu;
//  }
 // else
 // {
 //   ROS_ERROR("PAST");
 //   return nu_past;
 // }
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
  master_ref.twist.linear.x =
      (u_ref - baseline / 2 * yaw_rate_ref) * std::cos(yaw);
  master_ref.twist.linear.y =
      (u_ref + baseline / 2 * yaw_rate_ref) * std::sin(yaw);
  return master_ref;
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
  req.twist.angular.z = es_controller.step(cost)(0);
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
  //if (isMaster()){}
    //step();
}

void TDOASSControl::onVeh2State(const auv_msgs::NavSts::ConstPtr& msg)
{
  state[SLAVE] = *msg;
  //if (!isMaster()){}
    //step();
}

void TDOASSControl::onVeh1Toa(const std_msgs::Time::ConstPtr& msg)
{
  toa1 = msg->data;
}

void TDOASSControl::onVeh2Toa(const std_msgs::Time::ConstPtr& msg)
{
  toa2 = msg->data;
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

  labust::control::HLControl<labust::control::TDOASSControl,
                             labust::control::EnableServicePolicy>
      controller;
  ros::spin();

  return 0;
}
