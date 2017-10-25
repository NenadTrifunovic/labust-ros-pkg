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
#include <misc_msgs/RosbagControl.h>
#include <navcon_msgs/EnableControl.h>

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
  , m(100.0)
  , epsilon(0.5)
  , es_controller(1, 2.0)
  , eta_filter_state_k0(2, 0)
  , eta_filter_state_k1(2, 0)
  , ts(1.0)
  , w1(1)
  , w2(0.4)
  , k1(1000)
  , u0(0.25)
  , center_ref(auv_msgs::BodyVelocityReq())
  , tf_listener(tf_buffer)
  , master_active_flag(false)
  , slave_active_flag(false)
  , controller_active(false)
  , dp_controller_active(false)
  , logging_flag(false)
  , last_meas_time(ros::Time::now())
  , control_timeout(5.0)
  , update(false)
  , counter(0)
  , use_position_control(true)
  , test_init_flag(false)
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
  link_names[SLAVE_REF] = "slave_ref_frame";
  link_names[NED] = "master/local";

  sub_veh1_state =
      nh.subscribe("veh1/state", 1, &TDOASSControl::onVeh1State, this);
  sub_veh2_state =
      nh.subscribe("veh2/state", 1, &TDOASSControl::onVeh2State, this);
  sub_veh1_toa = nh.subscribe("veh1/toa", 1, &TDOASSControl::onVeh1Toa, this);
  sub_veh2_toa = nh.subscribe("veh2/toa", 1, &TDOASSControl::onVeh2Toa, this);

  dp_srv = nh.serviceClient<misc_msgs::DynamicPositioningPrimitiveService>(
      "commander/primitive/dynamic_positioning");

  if (isMaster())
  {
    pub_veh1_ref = nh.advertise<auv_msgs::NavSts>("veh1/state_ref", 1);
    pub_veh2_ref = nh.advertise<auv_msgs::NavSts>("veh2/state_ref", 1);
    pub_center_state = nh.advertise<auv_msgs::NavSts>("/center/state", 1);
    pub_master_active = nh.advertise<std_msgs::Bool>("master/active", 1);

    pub_master_pos_ref =
        nh.advertise<geometry_msgs::PointStamped>("master/pos_ref", 1);
    pub_master_ff_ref = nh.advertise<auv_msgs::NavSts>("master/ff_ref", 1);
    pub_master_hdg_ref = nh.advertise<std_msgs::Float32>("master/hdg_ref", 1);

    pub_tdoa = nh.advertise<std_msgs::Float64>("tdoa", 1);
    pub_tdoa_range = nh.advertise<std_msgs::Float64>("tdoa_range", 1);
    pub_delta = nh.advertise<std_msgs::Float64>("delta", 1);
    pub_eta = nh.advertise<std_msgs::Float64>("eta", 1);

    pub_baseline = nh.advertise<std_msgs::Float64>("baseline", 1);
    pub_surge_speed_ref = nh.advertise<std_msgs::Float64>("surge_speed_ref", 1);
    pub_yaw_rate_ref = nh.advertise<std_msgs::Float64>("yaw_rate_ref", 1);
    pub_delta_norm = nh.advertise<std_msgs::Float64>("delta_norm", 1);
    
    sub_test_init =
      nh.subscribe("/master/test_init", 1, &TDOASSControl::onTestInit, this);    

    /*** Dynamic reconfigure server ***/
    f = boost::bind(&TDOASSControl::reconfigureCallback, this, _1, _2);
    server.setCallback(f);

    /*** Read params ***/
    // ph.param("max_speed",max_speed, max_speed);

    initializeController();
    config.__fromServer__(ph);
    server.setConfigDefault(config);
    // updateDynRecConfig();
    server.updateConfig(config);
  }
  else
  {
    sub_master_active =
        nh.subscribe("master/active", 1, &TDOASSControl::onMasterActive, this);
    pub_slave_pos_ref =
        nh.advertise<geometry_msgs::PointStamped>("slave/pos_ref", 1);
    pub_slave_ff_ref = nh.advertise<auv_msgs::NavSts>("slave/ff_ref", 1);
    pub_slave_hdg_ref = nh.advertise<std_msgs::Float32>("slave/hdg_ref", 1);
    sub_veh2_ref =
        nh.subscribe("veh2/state_ref", 1, &TDOASSControl::onSlaveRef, this);
  }
}

void TDOASSControl::reconfigureCallback(
    labust_control::TDOASSControlConfig& config, uint32_t level)
{
  this->config = config;
  // config.__toServer__(ph);
  if (isMaster())
  {
    if (logging_flag != config.logging)
    {
      logging_flag = config.logging;
      misc_msgs::RosbagControl srv;
      srv.request.action = logging_flag ? "start" : "stop";
      srv.request.bag_name = "master_" + config.log_name;
      ros::service::call("/master/rosbag_record", srv);
      srv.request.bag_name = "slave_" + config.log_name;
      ros::service::call("/slave/rosbag_record", srv);
      ROS_INFO("Logging %s.", logging_flag ? "enabled" : "disabled");
    }

    if (master_active_flag != config.master_enable)
    {
      // master_active_flag = config.master_enable;
      // navcon_msgs::EnableControl srv;
      // srv.request.enable = master_active_flag;
      // ros::service::call("/usv/TDOASS_master_enable", srv);
      // ROS_INFO("Master %s.", master_active_flag ? "enabled" : "disabled");
    }

    if (slave_active_flag != config.slave_enable)
    {
      slave_active_flag = config.slave_enable;
      navcon_msgs::EnableControl srv;
      srv.request.enable = slave_active_flag;
      ros::service::call("/usv/TDOASS_slave_enable", srv);
      ROS_INFO("Slave %s.", slave_active_flag ? "enabled" : "disabled");
    }
  }

  baseline = config.baseline;
  m = config.m;
  epsilon = config.epsilon;
  w1 = config.w1;
  w2 = config.w2;
  k1 = config.k1;
  u0 = config.u0;
  speed_of_sound = config.speed_of_sound;

  double esc_high_pass_pole = 3 / config.esc_sin_period;
  // double esc_high_pass_pole = 0;
  ts = config.esc_sampling_time;
  double esc_Ts = config.esc_sampling_time;

  ROS_INFO("Reconfigure Request: ");
  ROS_INFO("baseline:%f, m:%f, epsilon:%f, w1:%f, w2:%f, "
           "k1:%f u0:%f",
           config.baseline, config.m, config.epsilon, config.w1, config.w2,
           config.k1, config.u0);
  ROS_INFO("a_pert:%f, a_demod:%f, T:%f, k:%f, hp:%f, "
           "lp:%f zc:%f pc:%f Ts:%f",
           config.esc_sin_amp, config.esc_sin_demodulation_amp,
           config.esc_sin_period, config.esc_corr_gain, esc_high_pass_pole,
           config.esc_low_pass_pole, config.esc_comp_zero, config.esc_comp_pole,
           config.esc_sampling_time);

  es_controller.initController(
      config.esc_sin_amp, config.esc_sin_demodulation_amp,
      1 / config.esc_sin_period, config.esc_corr_gain, esc_high_pass_pole,
      config.esc_low_pass_pole, config.esc_comp_zero, config.esc_comp_pole,
      config.esc_sampling_time);
}

void TDOASSControl::updateDynRecConfig()
{
  ROS_INFO("Updating the dynamic reconfigure parameters.");

  // config.Surge_mode = axis_control[u];
  // config.Sway_mode = axis_control[v];
  // config.Heave_mode = axis_control[w];
  // config.Roll_mode = axis_control[p];
  // config.Pitch_mode = axis_control[q];
  // config.Yaw_mode = axis_control[r];
  //
  // config.High_level_controller="0 - None\n 1 - DP";

  server.updateConfig(config);
}

void TDOASSControl::initializeController()
{
  ROS_INFO("Initializing extremum seeking controller...");

  disable_axis[x] = 0;
  disable_axis[y] = 0;
  disable_axis[yaw] = 0;

  ROS_ERROR("Extremum seeking controller initialized.");
}

void TDOASSControl::idle(const auv_msgs::NavSts& ref,
                         const auv_msgs::NavSts& state,
                         const auv_msgs::BodyVelocityReq& track)
{
  if (controller_active && !isMaster())
  {
    std_srvs::Trigger srv;
    ros::service::call("commander/stop_mission", srv);
  }

  if (controller_active && isMaster())
  {
    std_srvs::Trigger srv;
    ros::service::call("commander/stop_mission", srv);
    dp_controller_active = false;
  }
  
  if(controller_active) test_init_flag = false;

  controller_active = false;
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
  if (!controller_active)
  {
    if(!test_init_flag) initBaselinePos();
  }
  controller_active = true;
  if (isMaster())
  {
    std_msgs::Bool flag;
    flag.data = true;
    pub_master_active.publish(flag);

    // TODO check if new state and toa measurements are used.
    // TODO decide what executes at higher frequncy.
    double delta(0.0);
    double cost(0.0);
    double yaw_rate(0.0);
    if (update = calcluateTimeDifferenceOfArrival())
    {
      counter = 0;
      last_meas_time = ros::Time::now();
      delta = getNormalizedDifferenceOfArrivalMeters();
      cost = std::pow(delta, 2);
      yaw_rate = (use_position_control) ? center_ref.twist.angular.z :
                                          state[MASTER].orientation_rate.yaw;

      bool publish_flag(true);
      if (publish_flag)
      {
        std_msgs::Float64 data;
        data.data = getTimeDifferenceOfArrival();
        pub_tdoa.publish(data);
        data.data = getDifferenceOfArrivalMeters();
        pub_tdoa_range.publish(data);
        data.data = getNormalizedDifferenceOfArrivalMeters();
        pub_delta_norm.publish(data);
        data.data = baseline;
        pub_baseline.publish(data);
      }
    }

    if (++counter % 21 == 0 || update)
    {
      if (update)
        surgeSpeedControl(center_ref, delta, cost,
                          etaFilterStep(delta, yaw_rate));

      yawRateControl(center_ref, delta, cost);
    }

    auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());

    if (use_position_control)
    {
      // Publish transforms.
      baselineStep(center_ref);

      broadcastTransform(state[CENTER], link_names[NED], link_names[CENTER]);

      auv_msgs::NavSts offset;
      offset.position.east = baseline / 2;

      broadcastTransform(offset, link_names[CENTER], link_names[MASTER]);
      offset.position.east = -baseline / 2;
      broadcastTransform(offset, link_names[CENTER], link_names[SLAVE]);

      auv_msgs::NavSts master_ref;
      if (calculateMasterReference(master_ref, center_ref))
      {
        master_ref.header.stamp = ros::Time::now();
        geometry_msgs::PointStamped pos_ref;
        std_msgs::Float32 hdg_ref;
        auv_msgs::NavSts ff_ref;
        pos_ref.point.x = master_ref.position.north;
        pos_ref.point.y = master_ref.position.east;
        hdg_ref.data = master_ref.orientation.yaw;
        ff_ref.body_velocity.x = master_ref.body_velocity.x;
        ff_ref.body_velocity.y = master_ref.body_velocity.y;
        pub_master_pos_ref.publish(pos_ref);
        pub_master_hdg_ref.publish(hdg_ref);
        pub_master_ff_ref.publish(ff_ref);
      }

      if (!dp_controller_active && isMaster())
      {
        misc_msgs::DynamicPositioningPrimitiveService srv;
        srv.request.north_enable = true;
        srv.request.east_enable = true;
        srv.request.heading_enable = true;
        srv.request.target_topic_enable = true;
        srv.request.track_heading_enable = true;
        srv.request.target_topic = "/master/pos_ref";
        srv.request.heading_topic = "/master/hdg_ref";
        dp_srv.call(srv);
        dp_controller_active = true;
      }

      auv_msgs::NavSts slave_ref;
      if (calculateSlaveReference(slave_ref, center_ref))
      {
        slave_ref.header.stamp = ros::Time::now();
        pub_veh2_ref.publish(slave_ref);
      }
      disable_axis[x] = 1;
      disable_axis[y] = 1;
      disable_axis[yaw] = 1;
      nu->header.stamp = ros::Time::now();
      nu->goal.requester = "tdoass_controller";
      labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);
      return nu;
    }
    else
    {
      // Publish transforms.
      auv_msgs::NavSts offset;
      offset.position.east = -baseline / 2;
      broadcastTransform(offset, link_names[MASTER], link_names[CENTER]);
      broadcastTransform(offset, link_names[CENTER], link_names[SLAVE]);
      auv_msgs::NavSts slave_ref;
      if (calculateSlaveReference(slave_ref, center_ref))
      {
        slave_ref.header.stamp = ros::Time::now();
        pub_veh2_ref.publish(slave_ref);
      }
      *nu = allocateSpeed(center_ref);
      if ((ros::Time::now() - last_meas_time).toSec() > control_timeout)
      {
        nu->twist.linear.x = 0;
        nu->twist.linear.y = 0;
        nu->twist.angular.z = 0;
      }
      nu->header.stamp = ros::Time::now();
      nu->goal.requester = "tdoass_controller";
      labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);

      return nu;
    }
  }
  else
  {
    // Slave actions.
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
    long double tout = (long double)baseline / (long double)speed_of_sound;
    if (std::fabs((toa1 - toa2).toSec()) >
        (tout + (long double)config.tdoa_timeout))
    {
      // Discard measurements and wait for the new ones.
      toa1_old = toa1;
      toa2_old = toa2;
      return false;
    }
    tdoa = (toa1 - toa2).toSec() + config.tdoa_offset;
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

void TDOASSControl::onTestInit(const auv_msgs::NavSts::ConstPtr& msg)
{
  state[CENTER].position.north = msg->position.north;
  state[CENTER].position.east = msg->position.east;
  state[CENTER].orientation.yaw = msg->orientation.yaw;  
  test_init_flag = true;
}


void TDOASSControl::initBaselinePos()
{
  // TODO add offset
  state[CENTER].position.north = state[MASTER].position.north;
  state[CENTER].position.east = state[MASTER].position.east;
  state[CENTER].orientation.yaw = state[MASTER].orientation.yaw;
}

void TDOASSControl::baselineStep(auv_msgs::BodyVelocityReq req)
{
  std_msgs::Float64 data;
  data.data = req.twist.linear.x;
  pub_surge_speed_ref.publish(data);
  data.data = req.twist.angular.z;
  pub_yaw_rate_ref.publish(data);

  double yaw = state[CENTER].orientation.yaw;
  double vx_ref = req.twist.linear.x * std::cos(yaw);
  double vy_ref = req.twist.linear.x * std::sin(yaw);
  double Tstep = 0.1;
  state[CENTER].position.north += Tstep * vx_ref;
  state[CENTER].position.east += Tstep * vy_ref;
  state[CENTER].orientation.yaw += Tstep * req.twist.angular.z;

  pub_center_state.publish(state[CENTER]);
}

auv_msgs::BodyVelocityReq
TDOASSControl::allocateSpeed(auv_msgs::BodyVelocityReq req)
{
  auv_msgs::BodyVelocityReq master_ref;
  double yaw = state[MASTER].orientation.yaw;
  double u_ref = req.twist.linear.x;
  bool use_meas(false);
  double yaw_rate_ref = req.twist.angular.z;
  double yaw_rate =
      use_meas ? state[MASTER].orientation_rate.yaw : yaw_rate_ref;

  master_ref.twist.linear.x = u_ref - baseline / 2 * yaw_rate;
  master_ref.twist.linear.y = 0;
  master_ref.twist.angular.z = yaw_rate_ref;

  std_msgs::Float64 data;
  data.data = u_ref;
  pub_surge_speed_ref.publish(data);
  data.data = yaw_rate_ref;
  pub_yaw_rate_ref.publish(data);

  return master_ref;
}

bool TDOASSControl::calculateMasterReference(
    auv_msgs::NavSts& master_ref, const auv_msgs::BodyVelocityReq& center_ref)
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_buffer.lookupTransform(
        link_names[CENTER], link_names[MASTER], ros::Time(0));

    double yaw = state[CENTER].orientation.yaw;

    Eigen::Vector2d out, in;
    Eigen::Matrix2d R;

    // Body frame offset
    in[0] = transformStamped.transform.translation.x;
    in[1] = transformStamped.transform.translation.y;

    R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
    out = R * in;

    master_ref.position.north = state[CENTER].position.north + out[0];
    master_ref.position.east = state[CENTER].position.east + out[1];
    master_ref.position.depth = state[CENTER].position.depth;
    master_ref.orientation.roll = 0;
    master_ref.orientation.pitch = 0;
    master_ref.orientation.yaw = state[CENTER].orientation.yaw;
    // TODO account for yaw rate influence.
    // master_ref.body_velocity.x = center_ref.twist.linear.x;
    // TODO can be done with http://wiki.ros.org/tf2_geometry_msgs.
    // broadcastTransform(slave_ref, link_names[NED], link_names[SLAVE_REF]);

    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
}

bool TDOASSControl::calculateSlaveReference(
    auv_msgs::NavSts& slave_ref, const auv_msgs::BodyVelocityReq& center_ref)
{
  if (use_position_control)
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tf_buffer.lookupTransform(
          link_names[CENTER], link_names[SLAVE], ros::Time(0));

      double yaw = state[CENTER].orientation.yaw;

      Eigen::Vector2d out, in;
      Eigen::Matrix2d R;

      // Body frame offset
      in[0] = transformStamped.transform.translation.x;
      in[1] = transformStamped.transform.translation.y;

      R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
      out = R * in;

      slave_ref.position.north = state[CENTER].position.north + out[0];
      slave_ref.position.east = state[CENTER].position.east + out[1];
      slave_ref.position.depth = state[CENTER].position.depth;
      slave_ref.orientation.roll = 0;
      slave_ref.orientation.pitch = 0;
      slave_ref.orientation.yaw = state[CENTER].orientation.yaw;
      // TODO account for yaw rate influence.
      // slave_ref.body_velocity.x = center_ref.twist.linear.x;
      // TODO can be done with http://wiki.ros.org/tf2_geometry_msgs.
      broadcastTransform(slave_ref, link_names[NED], link_names[SLAVE_REF]);

      return true;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return false;
    }
  }
  else
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tf_buffer.lookupTransform(
          link_names[MASTER], link_names[SLAVE], ros::Time(0));

      double yaw = state[MASTER].orientation.yaw;

      Eigen::Vector2d out, in;
      Eigen::Matrix2d R;

      // Body frame offset
      in[0] = transformStamped.transform.translation.x;
      in[1] = transformStamped.transform.translation.y;

      R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
      out = R * in;

      slave_ref.position.north = state[MASTER].position.north + out[0];
      slave_ref.position.east = state[MASTER].position.east + out[1];
      slave_ref.position.depth = state[MASTER].position.depth;
      slave_ref.orientation.roll = 0;
      slave_ref.orientation.pitch = 0;
      slave_ref.orientation.yaw = state[MASTER].orientation.yaw;
      // TODO account for yaw rate influence.
      // slave_ref.body_velocity.x = center_ref.twist.linear.x;
      // TODO can be done with http://wiki.ros.org/tf2_geometry_msgs.
      broadcastTransform(slave_ref, link_names[NED], link_names[SLAVE_REF]);

      return true;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return false;
    }
  }
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_buffer.lookupTransform(
        link_names[MASTER], link_names[SLAVE], ros::Time(0));

    double yaw = state[MASTER].orientation.yaw;

    Eigen::Vector2d out, in;
    Eigen::Matrix2d R;

    // Body frame offset
    in[0] = transformStamped.transform.translation.x;
    in[1] = transformStamped.transform.translation.y;

    R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
    out = R * in;

    slave_ref.position.north = state[MASTER].position.north + out[0];
    slave_ref.position.east = state[MASTER].position.east + out[1];
    slave_ref.position.depth = state[MASTER].position.depth;
    slave_ref.orientation.roll = 0;
    slave_ref.orientation.pitch = 0;
    slave_ref.orientation.yaw = state[MASTER].orientation.yaw;
    // TODO account for yaw rate influence.
    // slave_ref.body_velocity.x = center_ref.twist.linear.x;
    // TODO can be done with http://wiki.ros.org/tf2_geometry_msgs.
    broadcastTransform(slave_ref, link_names[NED], link_names[SLAVE_REF]);

    return true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
}

void TDOASSControl::surgeSpeedControl(auv_msgs::BodyVelocityReq& req,
                                      double delta, double cost, double eta)
{
  const int n = 3;
  double u_amp, u_zeta, u_dir;
  u_dir = labust::math::sgn(eta);
  u_zeta = std::pow(std::fabs(eta), n) / std::pow(std::fabs(eta) + epsilon, n);
  u_amp = (1 - std::tanh(m * cost));

  ROS_ERROR("delta: %f", delta);
  ROS_ERROR("eta: %f, u_amp: %f, u_zeta: %f, u_dir: %f", eta, u_amp, u_zeta,
            u_dir);

  std_msgs::Float64 data;
  data.data = eta;
  pub_eta.publish(data);

  req.twist.linear.x = u0 * u_amp * u_zeta * u_dir;
}

void TDOASSControl::yawRateControl(auv_msgs::BodyVelocityReq& req, double delta,
                                   double cost)
{
  req.twist.angular.z = (es_controller.step(cost, update))[0];
  // Saturate yaw rate reference.
  if (req.twist.angular.z < -config.max_yaw_rate)
    req.twist.angular.z = -config.max_yaw_rate;
  else if (req.twist.angular.z > config.max_yaw_rate)
    req.twist.angular.z = config.max_yaw_rate;
}

double TDOASSControl::etaFilterStep(double delta, double yaw_rate)
{
  /// TODO scale yaw_Rate
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
    auv_msgs::NavSts ff_ref;
    pos_ref.point.x = msg->position.north;
    pos_ref.point.y = msg->position.east;
    hdg_ref.data = msg->orientation.yaw;
    ff_ref.body_velocity.x = msg->body_velocity.x;
    ff_ref.body_velocity.y = msg->body_velocity.y;
    pub_slave_pos_ref.publish(pos_ref);
    pub_slave_hdg_ref.publish(hdg_ref);
    pub_slave_ff_ref.publish(ff_ref);
  }
}

void TDOASSControl::onMasterActive(const std_msgs::Bool::ConstPtr& msg)
{
  if (!isMaster())
  {
    if (controller_active)
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
