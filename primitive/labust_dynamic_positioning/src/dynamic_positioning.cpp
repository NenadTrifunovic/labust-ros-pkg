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
#include <labust/math/Line.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/primitive/PrimitiveBase.hpp>
#include <labust/tools/conversions.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <labust_msgs/DynamicPositioningAction.h>
#include <labust_msgs/EnableControl.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Dense>

#include <boost/array.hpp>
#include <boost/thread/mutex.hpp>

namespace labust
{
namespace primitive
{
/*************************************************************
 *** Dynamic positioning primitive class
 ************************************************************/
struct DynamicPositionining
    : protected ExecutorBase<labust_msgs::DynamicPositioningAction>
{
  typedef labust_msgs::DynamicPositioningGoal Goal;
  typedef labust_msgs::DynamicPositioningResult Result;
  typedef labust_msgs::DynamicPositioningFeedback Feedback;

  enum
  {
    ualf = 0,
    falf,
    fadp,
    hdg,
    numcnt
  };
  enum
  {
    x = 0,
    y,
    z,
    roll,
    pitch,
    yaw
  };

  DynamicPositionining()
    : ExecutorBase("dynamic_positioning")
    , underactuated(true)
    , headingEnabled(false)
    , processNewGoal(false)
    , heading_reference(0.0){};

  void init()
  {
    ros::NodeHandle ph("~");

    /*** Initialize controller names ***/
    controllers.name.resize(numcnt);
    controllers.state.resize(numcnt, false);
    controllers.name[fadp] = "FADP_enable";
    controllers.name[hdg] = "HDG_enable";
  }

  void onGoal()
  {
    boost::mutex::scoped_lock l(state_mux);
    /*** Set the flag to avoid disabling controllers on preemption ***/
    processNewGoal = true;
    Goal::ConstPtr new_goal = aserver->acceptNewGoal();
    processNewGoal = false;

    /*** Save new goal ***/
    goal = new_goal;

    /*** Subscribe to reference topics (if enabled) ***/
    if (goal->track_heading_enable)
    {
      ros::NodeHandle nh;
      sub_heading = nh.subscribe(goal->heading_topic, 1,
                                 &DynamicPositionining::onHeading, this);
    }

    if (goal->target_topic_enable)
    {
      ros::NodeHandle nh;
      sub_target = nh.subscribe(goal->target_topic, 1,
                                &DynamicPositionining::onTargetPoint, this);
      sub_ff = nh.subscribe("/slave/ff_ref", 1,
                            &DynamicPositionining::onFeedForward, this);
    }

    /*** Update reference ***/
    stateRef.publish(step(lastState));

    /*** Enable controllers ***/
    controllers.state[fadp] =
        true && goal->axis_enable[x] && goal->axis_enable[y];
    controllers.state[hdg] = true && goal->axis_enable[yaw];
    this->updateControllers();
  }

  void onPreempt()
  {
    ROS_WARN("dynamic_positioning: Goal preempted.");
    if (!processNewGoal)
    {
      goal.reset();
      ROS_INFO("dynamic_positioning: Stopping controllers.");
      controllers.state.assign(numcnt, false);
      this->updateControllers();
    }
    else
    {
      // ROS_ERROR("New goal processing.");
    }
    aserver->setPreempted();
  };

  void updateControllers()
  {
    /*** Enable high level controllers ***/
    ros::NodeHandle nh;
    ros::ServiceClient cl;

    cl = nh.serviceClient<labust_msgs::EnableControl>(
        std::string(controllers.name[fadp]));
    labust_msgs::EnableControl a;
    a.request.enable = controllers.state[fadp];
    cl.call(a);

    cl = nh.serviceClient<labust_msgs::EnableControl>(
        std::string(controllers.name[hdg]));
    a.request.enable = controllers.state[fadp];
    cl.call(a);
  }

  void onStateHat(const auv_msgs::NavigationStatus::ConstPtr& estimate)
  {
    boost::mutex::scoped_lock l(state_mux);

    if (aserver->isActive())
    {
      stateRef.publish(step(*estimate));

      /*** Check if goal (victory radius) is achieved ***/
      Eigen::Vector3d distance;
      distance << ((goal->target_topic_enable) ? target_reference.x :
                                                 goal->T1.point.x) -
                      estimate->position.north,
          ((goal->target_topic_enable) ? target_reference.y :
                                         goal->T1.point.y) -
              estimate->position.east,
          0;

      /*** Calculate bearing to endpoint ***/
      Eigen::Vector3d T1, T2;
      labust::math::Line bearing_to_endpoint;
      T1 << estimate->position.north, estimate->position.east, 0;
      T2 << ((goal->target_topic_enable) ? target_reference.x :
                                           goal->T1.point.x),
          ((goal->target_topic_enable) ? target_reference.y : goal->T1.point.y),
          0;
      bearing_to_endpoint.setLine(T1, T2);

      /*** Publish primitive feedback ***/
      Feedback feedback;
      feedback.error.point.x = distance(0);
      feedback.error.point.y = distance(1);
      feedback.distance = distance.norm();
      feedback.bearing = bearing_to_endpoint.gamma();
      aserver->publishFeedback(feedback);
    }
    else if (goal != 0)
    {
      goal.reset();
      ROS_INFO("dynamic_positioning: Stopping controllers.");
      controllers.state.assign(numcnt, false);
      this->updateControllers();
    }
    lastState = *estimate;
  }

  auv_msgs::NavigationStatusPtr step(const auv_msgs::NavigationStatus& state)
  {
    auv_msgs::NavigationStatusPtr ref(new auv_msgs::NavigationStatus());

    ref->position.north =
        (goal->target_topic_enable) ? target_reference.x : goal->T1.point.x;
    ref->position.east =
        (goal->target_topic_enable) ? target_reference.y : goal->T1.point.y;
    ref->orientation.z =
        (goal->track_heading_enable) ? heading_reference : goal->yaw;
    ref->body_velocity.x = feed_forward.body_velocity.x;
    ref->body_velocity.y = feed_forward.body_velocity.y;
    ref->header.frame_id = tf_prefix + "local";
    ref->header.stamp = ros::Time::now();

    return ref;
  }

  void onTargetPoint(const geometry_msgs::PointStamped::ConstPtr& point)
  {
    target_reference = point->point;
  }

  void onFeedForward(const auv_msgs::NavigationStatus::ConstPtr& msg)
  {
    feed_forward.body_velocity.x = msg->body_velocity.x;
    feed_forward.body_velocity.y = msg->body_velocity.y;
  }

  void onHeading(const std_msgs::Float32::ConstPtr& heading)
  {
    heading_reference = heading->data;
  }

private:
  geometry_msgs::Point lastPosition;
  labust::math::Line line;
  tf2_ros::StaticTransformBroadcaster broadcaster;
  bool underactuated;
  bool headingEnabled;
  bool processNewGoal;
  Goal::ConstPtr goal;
  auv_msgs::NavigationStatus lastState;
  boost::mutex state_mux;
  labust_msgs::ControllerSelectRequest controllers;

  geometry_msgs::Point target_reference;
  double heading_reference;
  auv_msgs::NavigationStatus feed_forward;

  ros::Subscriber sub_target, sub_heading, sub_ff;
};
}
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dynamic_positioning");

  labust::primitive::PrimitiveBase<labust::primitive::DynamicPositionining>
      primitive;
  ros::spin();

  return 0;
}
