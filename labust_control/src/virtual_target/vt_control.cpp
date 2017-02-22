/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
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
 *  Author: Dula Nad
 *  Created: 01.02.2013.
 *********************************************************************/
#include <labust/control/PIFFController.h>
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/HLControl.hpp>
#include <labust/control/WindupPolicy.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/FSPathInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

namespace labust
{
namespace control
{
/// The virtual target controller
struct VTControl : DisableAxis
{
  enum
  {
    u = 0,
    v,
    w,
    r = 5
  };
  enum
  {
    s = 0,
    e,
    h
  };

  VTControl()
    : Ts(0.1)
    , Kpd(Eigen::Matrix3d::Identity())
    , dr_p(Eigen::Vector3d::Zero())
    , kpi(1.0)
    , kpierr(5)
    , listener(buffer)
    , tf_prefix("")
    , use_pi(false){};

  void init()
  {
    initializeController();
  }

  void idle(const auv_msgs::FSPathInfo& ref, const auv_msgs::NavSts& state,
            const auv_msgs::BodyVelocityReq& track){
    // Tracking external commands while idle (bumpless)
  };

  void reset(const auv_msgs::FSPathInfo& ref, const auv_msgs::NavSts& state){};

  void onDiverState(const auv_msgs::NavSts& diver_state){
    // Recalculate the diver speed into dr_p
  };

  void windup(const auv_msgs::BodyForceReq& tauAch)
  {
    // Copy into controller
    bool joint_windup = tauAch.disable_axis.x || tauAch.disable_axis.y;
    con[s].extWindup = joint_windup;
    con[e].extWindup = joint_windup;
    if (joint_windup)
    {
      ROS_INFO("VTControl: Windup");
    }
  };

  auv_msgs::BodyVelocityReqPtr step(const auv_msgs::FSPathInfo& ref,
                                    const auv_msgs::NavSts& state)
  {
    geometry_msgs::TransformStamped td;
    Eigen::Vector3d d(Eigen::Vector3d::Zero());
    Eigen::Vector3d dr_p(ref.dr_p.x, ref.dr_p.y, ref.dr_p.z);
    Eigen::Matrix3d Rpb(Eigen::Matrix3d::Identity());

    try
    {
      // Get the transform frames
      td = buffer.lookupTransform(tf_prefix + "sf_frame",
                                  tf_prefix + "base_link", ros::Time(0));
      d << td.transform.translation.x, td.transform.translation.y,
          td.transform.translation.z;
      // Isolate horizontal plane
      double rool, pitch, yaw;
      Eigen::Quaternion<double> q(
          td.transform.rotation.w, td.transform.rotation.x,
          td.transform.rotation.y, td.transform.rotation.z);

      labust::tools::eulerZYXFromQuaternion(q, rool, pitch, yaw);
      labust::tools::quaternionFromEulerZYX(0, 0, yaw, q);
      Rpb = q.toRotationMatrix().transpose();
      // Rpb = Eigen::Quaternion<double>(
      // td.transform.rotation.w, td.transform.rotation.x,
      // td.transform.rotation.y, td.transform.rotation.z)
      //.toRotationMatrix()
      //.transpose();
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("VTControl: Missing frame : %s", ex.what());
    }

    // Calculate the path control signal
    double dpi_r = labust::math::coerce(
        (1 + 3 * ref.k) * kpi * (d(s) - tanh(ref.pi_tilda / kpierr)) +
            ref.dxi_r,
        -1, 1);
    // Calculate the velocity control signal
    Eigen::Vector3d nur =
        Rpb * (-Kpd * d + dr_p + ref.dxi_r * Eigen::Vector3d(1, 0, 0));
    Eigen::Vector3d dcon = -Rpb*Kpd*d;
    Eigen::Vector3d dscon = Rpb*dr_p;
    Eigen::Vector3d dxcon = ref.dxi_r * Eigen::Vector3d(1, 0, 0);
    ROS_ERROR("Distance contribution: %f %f %f", dcon(0), dcon(1), dcon(2)); 
    ROS_ERROR("Diver speed contribution: %f %f %f", dscon(0), dscon(1), dscon(2)); 
    ROS_ERROR("Path contribution: %f %f %f", dxcon(0), dxcon(1), dxcon(2)); 
    // Calculate PI stuff
    if (use_pi)
    {
      con[s].desired = 0;
      con[e].desired = 0;
      con[s].state = d(s);
      con[e].state = d(e);
      Eigen::Vector3d out, in;

      in << state.gbody_velocity.x, state.gbody_velocity.y,
          0;  // state.gbody_velocity.z;
      out = Rpb.transpose() * in;
      con[s].track = out(s);
      con[e].track = out(e);
      ROS_INFO("VTControl track: %f, %f", out(s), out(e));
      PIFF_ffStep(&con[s], Ts, dr_p(s) + ref.dxi_r);
      PIFF_ffStep(&con[e], Ts, dr_p(e));
      // Try PI control
      Eigen::Vector3d dcon;
      dcon << con[s].output, con[e].output, -Kpd(2, 2) * d(2);
      nur = Rpb * dcon;
    }

    // TODO: Calculate the surge only orientation (Note: only a 2D case,
    // extend
    // to 3D)
    // double zeta_r = labust::math::wrapRad(ref.orientation.yaw) +
    //        atan2(-Kpd(e,e)*d(e) + dr_p(e), -Kpd(s,s)*d(s) + dr_p(s));
    // zeta_r = labust::math::wrapRad(zeta_r);

    // if (zeta_r < 0) zeta_r += 2*M_PI;
    // double delta_r = ref.delta_r;
    // if (delta_r < 0) delta_r += 2*M_PI;

    // Reference orientatio
    // double psi_r = ref.k*zeta_r + (1-ref.k)*ref.delta_r;
    double psi_r = ref.delta_r;
    // if (ref.k > 0.5) psi_r = zeta_r; else psi_r=ref.delta_r;
    psi_r = labust::math::wrapRad(psi_r);

    // Send the control signal
    auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());
    labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);
    nu->header.stamp = ros::Time::now();
    nu->goal.requester = "vt_controller";

    // ROS_ERROR("Position (s,e,h): (%f, %f, %f)",d(s),d(e),d(h));
    nu->twist.angular.z =
        -labust::math::wrapRad(state.orientation.yaw - psi_r);
    nu->twist.linear.x = nur(u);  //+ nur(v)*nu->twist.angular.z;
    nu->twist.linear.x = labust::math::coerce(nur(u), -0.8, 0.5); //+ nur(v)*nu->twist.angular.z;
    nu->twist.linear.y = nur(v);  //- nur(u)*nu->twist.angular.z;
    nu->twist.linear.z = nur(w);

    geometry_msgs::TwistStamped::Ptr piref_out(
        new geometry_msgs::TwistStamped());
    piref_out->twist.linear.x = dpi_r;
    piref_out->header.stamp = nu->header.stamp;
    piref_out->header.frame_id = tf_prefix + "sf_frame";
    piref.publish(piref_out);

    auv_msgs::NavSts::Ptr psiref_out(new auv_msgs::NavSts());
    psiref_out->orientation.yaw = psi_r;
    psiref_out->header.stamp = nu->header.stamp;
    psiref_out->header.frame_id = tf_prefix + "local";
    psiref.publish(psiref_out);

    return nu;
  }

  void initializeController()
  {
    ros::NodeHandle nh;
    Eigen::Vector3d closedLoopFreq(Eigen::Vector3d::Ones());
    std::vector<double> stateg(3, 1.0), pathg;
    pathg.push_back(kpi);
    pathg.push_back(kpierr);
    nh.param("vt_controller/state_gain", stateg, stateg);
    nh.param("vt_controller/path_gain", pathg, pathg);
    nh.param("vt_controller/sampling", Ts, Ts);
    nh.param("vt_controller/use_pi", use_pi, use_pi);
    labust::tools::getMatrixParam(nh, "vt_controller/closed_loop_freq",
                                  closedLoopFreq);

    std::string key;
    if (nh.searchParam("tf_prefix", key))
      nh.getParam(key, tf_prefix);

    // Setup gains
    for (int i = 0; i < stateg.size(); ++i)
    {
      if (stateg[i] <= 0)
      {
        ROS_WARN("Gains need to be >0.");
        stateg[i] = 1.0;
      }
      Kpd(i, i) = stateg[i];
    }

    kpi = pathg[0];
    if (kpi <= 0)
    {
      ROS_WARN("The gain 'kp' must be >0.");
      kpi = 1.0;
    }
    kpierr = pathg[1];
    if (kpierr <= 0)
    {
      ROS_WARN("The gain 'kpierr' must be >0.");
      kpierr = 1;
    }

    disable_axis[u] = 0;
    disable_axis[v] = 0;
    disable_axis[w] = 0;
    // disable_axis[r] = 0;

    for (size_t i = 0; i < 2; ++i)
    {
      PIDBase_init(&con[i]);
      PIFF_tune(&con[i], float(closedLoopFreq(i)));
      con[i].b = 1.0;
    }

    diver = nh.subscribe("diver_state", 1, &VTControl::onDiverState, this);
    piref = nh.advertise<geometry_msgs::TwistStamped>("dpi_r", 1);
    psiref = nh.advertise<auv_msgs::NavSts>("psi_r", 1);

    ROS_INFO("VT controller initialized.");
  }

private:
  /// Sampling time
  double Ts;
  /// The distance gain
  Eigen::Matrix3d Kpd;
  /// The path speed gain
  double kpi;
  /// The path error gain
  double kpierr;

  // Subscriber for diver messages
  ros::Subscriber diver;
  // The diver speed (in SF-frame
  Eigen::Vector3d dr_p;
  // Publisher for the path progression speed
  ros::Publisher piref, psiref;
  // The frame transform buffer
  tf2_ros::Buffer buffer;
  // The transform frame listener
  tf2_ros::TransformListener listener;
  // The transform frame prefix for multi-vehicle operations
  std::string tf_prefix;

  // PID for distance
  PIDBase con[2];
  bool use_pi;
};
}
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "uvt_control");

  labust::control::HLControl<
      labust::control::VTControl, labust::control::EnableServicePolicy,
      labust::control::WindupPolicy<auv_msgs::BodyForceReq>,
      auv_msgs::BodyVelocityReq, auv_msgs::NavSts, auv_msgs::FSPathInfo>
      controller;
  ros::spin();

  return 0;
}
