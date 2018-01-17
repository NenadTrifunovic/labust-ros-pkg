/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, LABUST, UNIZG-FER
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
#include <auv_msgs/NavigationStatus.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

class TDOASim
{
public:
  TDOASim()
    : speed_of_sound(1500.0)
    , veh1_pos(Eigen::Vector3d::Zero())
    , veh2_pos(Eigen::Vector3d::Zero())
    , source_pos(Eigen::Vector3d::Zero())
  {
    ros::NodeHandle nh;

    sub_veh1_state = nh.subscribe<auv_msgs::NavigationStatus>(
        "veh1/state", 1, &TDOASim::onVeh1State, this);
    sub_veh2_state = nh.subscribe<auv_msgs::NavigationStatus>(
        "veh2/state", 1, &TDOASim::onVeh2State, this);
    sub_source_state = nh.subscribe<auv_msgs::NavigationStatus>(
        "source/state", 1, &TDOASim::onSourceState, this);

    pub_veh1_toa = nh.advertise<std_msgs::Time>("veh1/toa", 1);
    pub_veh2_toa = nh.advertise<std_msgs::Time>("veh2/toa", 1);
    pub_tdoa = nh.advertise<std_msgs::Float64>("tdoa", 1);

    pub_vis = nh.advertise<visualization_msgs::Marker>("source_marker", 0);
  }

  ~TDOASim()
  {
  }

  void start()
  {
    ros::NodeHandle ph("~");
    double Ts(1.0);
    ph.param("Ts", Ts, Ts);
    ros::Rate rate(1 / Ts);

    while (ros::ok())
    {
      Eigen::VectorXd range1s(1);
      Eigen::VectorXd range2s(1);

      range1s(0) = (veh1_pos - source_pos).norm();
      range2s(0) = (veh2_pos - source_pos).norm();

      ros::Time toa_close = ros::Time::now();
      double tdoa = (range1s(0) - range2s(0)) / speed_of_sound;
      ros::Time toa_far = toa_close + ros::Duration(std::abs(tdoa));

      std_msgs::Time timestamp;
      if (range1s(0) <= range2s(0))
      {
        timestamp.data = toa_close;
        pub_veh1_toa.publish(timestamp);
        timestamp.data = toa_far;
        pub_veh2_toa.publish(timestamp);
      }
      else
      {
        timestamp.data = toa_close;
        pub_veh2_toa.publish(timestamp);
        timestamp.data = toa_far;
        pub_veh1_toa.publish(timestamp);
      }
      std_msgs::Float64 duration;
      duration.data = tdoa;
      pub_tdoa.publish(duration);
      rate.sleep();
      ros::spinOnce();
    }
  }

  void onVeh1State(const auv_msgs::NavigationStatus::ConstPtr& data)
  {
    veh1_pos << data->position.north, data->position.east, data->position.depth;
  }

  void onVeh2State(const auv_msgs::NavigationStatus::ConstPtr& data)
  {
    veh2_pos << data->position.north, data->position.east, data->position.depth;
  }

  void onSourceState(const auv_msgs::NavigationStatus::ConstPtr& data)
  {
    source_pos << data->position.north, data->position.east,
        data->position.depth;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "master/local";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = data->position.north;
    marker.pose.position.y = data->position.east;
    marker.pose.position.z = data->position.depth;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    // only if using a MESH_RESOURCE marker type:
    // marker.mesh_resource =
    // "package://pr2_description/meshes/base_v0/base.dae";
    pub_vis.publish(marker);
  }

  ros::Subscriber sub_veh1_state, sub_veh2_state, sub_source_state;
  ros::Publisher pub_veh1_toa, pub_veh2_toa, pub_tdoa, pub_vis;

  Eigen::Vector3d veh1_pos, veh2_pos, source_pos;

  double speed_of_sound;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tdoa_sim");
  TDOASim ts;
  ts.start();
  return 0;
}
