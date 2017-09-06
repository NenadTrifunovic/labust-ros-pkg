/*****************+***************************************************
 * tdoass_control.h
 *
 *  Created on: Aug 28, 2017
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

#ifndef TDOASS_CONTROL_H_
#define TDOASS_CONTROL_H_

#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/NavSts.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/HLControl.hpp>
#include <labust/control/WindupPolicy.hpp>
#include <labust/control/esc/EscClassic.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/math/Signum.hpp>
#include <labust/tools/conversions.hpp>
#include <map>
#include <string>

namespace labust
{
namespace control
{
class TDOASSControl : DisableAxis
{
public:
  TDOASSControl();

  ~TDOASSControl();

  /// Intialize class.
  void init();
  ///
  void initializeController();
  ///
  auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
                                    const auv_msgs::NavSts& state_hat);
  ///
  bool setAsMaster(bool flag);
  ///
  bool isMaster();
  ///
  void windup(const auv_msgs::BodyForceReq& tauAch){};
  ///
  void idle(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state,
            const auv_msgs::BodyVelocityReq& track){};
  ///
  void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state){};

private:
  ///
  enum
  {
    x = 0,
    y
  };
  ///
  enum
  {
    MASTER = 0,
    SLAVE,
    CENTER
  };
  /// Caluculate Time difference of arrival
  bool calcluateTimeDifferenceOfArrival();
  /// Get time difference of arrival
  double getTimeDifferenceOfArrival();
  /// Get difference of arrival in meters
  double getDifferenceOfArrivalMeters();
  ///
  double getNormalizedDifferenceOfArrivalMeters();
  ///
  auv_msgs::BodyVelocityReq allocateSpeed(auv_msgs::BodyVelocityReq req);
  /// Calculate and broadcast trasform.
  void broadcastTransform(auv_msgs::NavSts& state, std::string& frame_id,
                          std::string& child_frame_id);
  ///
  void surgeSpeedControl(auv_msgs::BodyVelocityReq& req, double delta,
                         double cost, double eta);
  ///
  void yawRateControl(auv_msgs::BodyVelocityReq& req, double delta,
                      double cost);
  ///
  double etaFilterStep(double delta, double yaw_rate);
  /// Vehicle 1 state callback.
  void onVeh1State(const auv_msgs::NavSts::ConstPtr& msg);
  /// Vehicle 2 state callback.
  void onVeh2State(const auv_msgs::NavSts::ConstPtr& msg);
  ///
  void onVeh1Toa(const std_msgs::Time::ConstPtr& msg);
  ///
  void onVeh2Toa(const std_msgs::Time::ConstPtr& msg);
  /// Transform broadcaster.
  tf2_ros::TransformBroadcaster transform_broadcaster;
  ///
  labust::control::esc::EscClassic es_controller;
  ///
  ros::Subscriber sub_veh1_state;
  ///
  ros::Subscriber sub_veh2_state;
  ///
  ros::Subscriber sub_veh1_toa;
  ///
  ros::Subscriber sub_veh2_toa;
  ///
  ros::Publisher pub_veh1_ref;
  ///
  ros::Publisher pub_veh2_ref;
  ///
  ros::Publisher pub_tdoa;
  ///
  ros::Publisher pub_delta;
  ///
  std::map<int, auv_msgs::NavSts> state;
  ///
  std::map<int, std::string> link_names;
  ///
  ros::Time toa1, toa1_old;
  ///
  ros::Time toa2, toa2_old;
  ///
  auv_msgs::BodyVelocityReq center_ref;
  ///
  std::vector<double> eta_filter_state_k0, eta_filter_state_k1;
  ///
  double ts;
  ///
  double tdoa;
  ///
  double speed_of_sound;
  ///
  double baseline;
  ///
  double m;
  ///
  double epsilon;
  ///
  double w1, w2, k1;
  ///
  int veh_type;
};
}
}

#endif /* TDOASS_CONTROL_H_ */
