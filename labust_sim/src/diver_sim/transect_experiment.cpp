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
 *********************************************************************/
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/NavSts.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/conversions.hpp>

int experiment(0);
bool start(false);
double angle_deg(0);
double duration_def(60);
double speed(0.3);
double turnrate(0.3);
auv_msgs::NavSts pos;

void onExperimentType(const std_msgs::Int32::ConstPtr& data)
{
  experiment = data->data;
  start = false;
}

void onAngle(const std_msgs::Float32::ConstPtr& data)
{
  angle_deg = data->data;
}

void onPosition(const auv_msgs::NavSts::ConstPtr& data)
{
  pos = *data;
}

void onSpeed(const std_msgs::Float32::ConstPtr& data)
{
  speed = data->data;
}

void onTurnrate(const std_msgs::Float32::ConstPtr& data)
{
  turnrate = data->data;
}

void onDuration(const std_msgs::Float32::ConstPtr& data)
{
  duration_def = data->data;
}

void onStart(const std_msgs::Bool::ConstPtr& data)
{
  start = data->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transect_experiment");
  ros::NodeHandle nh;
  ros::Publisher pose_pub =
      nh.advertise<auv_msgs::NavSts>("diver_pose_req", 1);
  ros::Publisher buddy_init_pub =
      nh.advertise<auv_msgs::NavSts>("buddy_pose_req", 1);
  ros::Publisher init_pub = nh.advertise<auv_msgs::NavSts>("diver_init", 1);

  ros::Subscriber diver_pos =
      nh.subscribe<auv_msgs::NavSts>("position", 1, &onPosition);
  ros::Subscriber start_sub =
      nh.subscribe<std_msgs::Bool>("experiment_start", 1, &onStart);
  ros::Subscriber experiment_sub =
      nh.subscribe<std_msgs::Int32>("experiment_type", 1, &onExperimentType);

  // Parameters
  double dt(0.05);
  double tspeed(0.3);
  std::vector<double> T1(6, 0), T2(6, 0), T3(6, 0);
  nh.param("T1", T1, T1);
  nh.param("T2", T2, T2);
  nh.param("T3", T3, T3);

  ros::Rate rate(1 / dt);
  auv_msgs::NavSts binitpoint, initpoint, reqpoint;
  while (ros::ok())
  {
    if (start)
    {
      switch (experiment)
      {
        case 0:
          // Approach experiment
          // Start the pointer experiment
          // Wait for vehicle to converge
          experiment = 1;
          break;
        case 1:
          // Rotation experiment
          experiment = 2;
          break;
        case 2:
          // Tracking experiment
          experiment = 3;
          break;
        case 3:
          // Guide experiment
          // Activate guidance
          experiment = 0;
          break;
        default:
          break;
      }
    }
    else
    {
      // Init the states
      experiment = 0;
      // Switch Buddy to DP controller
      // Go to the initial position
      labust::tools::vectorToNED(T1, initpoint.position);
      labust::tools::vectorToRPY(T1, initpoint.orientation, 3);
      labust::tools::vectorToNED(T2, binitpoint.position);
      labust::tools::vectorToRPY(T2, binitpoint.orientation, 3);
      reqpoint = initpoint;
      pose_pub.publish(reqpoint);
      init_pub.publish(initpoint);
      buddy_init_pub.publish(binitpoint);
    }
    rate.sleep();
    ros::spinOnce();
  }

  // ros::Subscriber angle_sub =
  //     nh.subscribe<std_msgs::Float32>("experiment_angle", 1, &onAngle);
  // ros::Subscriber duration_sub =
  //     nh.subscribe<std_msgs::Float32>("experiment_duration", 1,
  //     &onDuration);
  // ros::Subscriber speed_sub =
  //     nh.subscribe<std_msgs::Float32>("experiment_speed", 1, &onSpeed);
  // ros::Subscriber turnrate_sub =
  //     nh.subscribe<std_msgs::Float32>("experiment_turnrate", 1,
  //     &onTurnrate);

  // double dt(0.05), timebase(0), turn_time(0);
  // int it(0);
  // double duration = duration_def;
  // int nit = 6;
  // int sgn = 1;
  // ros::Rate rate(1 / dt);
  // auv_msgs::NavSts::Ptr initpoint(new auv_msgs::NavSts());
  // auv_msgs::BodyVelocityReq::Ptr nuref(new auv_msgs::BodyVelocityReq());
  // while (ros::ok())
  // {
  //   if (start)
  //   {
  //     int rem;
  //     int div;
  //     int sgnl;
  //     timebase += dt;
  //     ROS_INFO("Timebase:%f", timebase);
  //     switch (experiment)
  //     {
  //       case 1:
  //         initpoint->position.north = -9.0;
  //         initpoint->position.east = 14.0;
  //         if (timebase > duration_def)
  //         {
  //           initpoint->orientation.yaw += angle_deg / 180 * M_PI;
  //           initpoint->orientation.yaw =
  //               labust::math::wrapRad(initpoint->orientation.yaw);
  //           timebase = 0;
  //         }
  //         init_pub.publish(initpoint);
  //         break;
  //       case 2:
  //         nuref.reset(new auv_msgs::BodyVelocityReq());
  //         nuref->twist.linear.x = speed;
  //         nuref->twist.angular.z =
  //             0.2 * labust::math::wrapRad(initpoint->orientation.yaw -
  //                                         pos.orientation.yaw);
  //
  //         if (timebase > duration_def)
  //         {
  //           nuref->twist.linear.x = speed;
  //           nuref->twist.angular.z = 0;
  //           initpoint->orientation.yaw += angle_deg / 180 * M_PI;
  //           initpoint->orientation.yaw =
  //               labust::math::wrapRad(initpoint->orientation.yaw);
  //           timebase = 0;
  //           turn_time = (fabs(angle_deg) / 180 * M_PI) / turnrate;
  //         }
  //         nu_pub.publish(nuref);
  //         break;
  //       case 3:
  //         nuref.reset(new auv_msgs::BodyVelocityReq());
  //         nuref->twist.linear.x = speed;
  //         nuref->twist.angular.z =
  //             0.5 * labust::math::wrapRad(
  //                       initpoint->orientation.yaw -
  //                       labust::math::wrapRad(pos.orientation.yaw));
  //         duration = (it % 2 == 0) ? duration_def : 2 * duration_def;
  //         if (timebase > duration)
  //         {
  //           if (it < nit)
  //           {
  //             nuref->twist.linear.x = speed;
  //             nuref->twist.angular.z = 0;
  //             rem = it % 4;
  //             ++it;
  //
  //             if ((rem == 0) || (rem == 3))
  //             {
  //               sgnl = 1;
  //             }
  //             else
  //             {
  //               sgnl = -1;
  //             }
  //
  //             initpoint->orientation.yaw += sgnl * angle_deg / 180 * M_PI;
  //             initpoint->orientation.yaw =
  //                 labust::math::wrapRad(initpoint->orientation.yaw);
  //             timebase = 0;
  //           }
  //           else
  //           {
  //             if (it == nit)
  //             {
  //               it++;
  //               timebase = 0;
  //             }
  //             nuref->twist.linear.x = speed;
  //             initpoint->orientation.yaw = 2 * M_PI / 3;
  //             if (timebase >= it * duration_def)
  //             {
  //               nuref->twist.linear.x = 0;
  //             }
  //           }
  //         }
  //         nu_pub.publish(nuref);
  //         break;
  //       default:
  //         break;
  //     }
  //   }
  //   else
  //   {
  //     // Setup the initial position for each experiment
  //     switch (experiment)
  //     {
  //       case 0:
  //         initpoint.reset(new auv_msgs::NavSts());
  //         initpoint->position.north = -9.0;
  //         initpoint->position.east = 14.0;
  //         initpoint->orientation.yaw = 2 * M_PI / 3;
  //         break;
  //       case 1:
  //         initpoint.reset(new auv_msgs::NavSts());
  //         initpoint->position.north = -9.0;
  //         initpoint->position.east = 14.0;
  //         initpoint->orientation.yaw = 0;
  //         timebase = 0;
  //         break;
  //       case 2:
  //         initpoint.reset(new auv_msgs::NavSts());
  //         initpoint->position.north = -3.0;
  //         initpoint->position.east = 14.0;
  //         initpoint->orientation.yaw = -65 * M_PI / 180.0;
  //         timebase = 0;
  //         break;
  //       case 3:
  //         initpoint.reset(new auv_msgs::NavSts());
  //         initpoint->position.north = -10.0;
  //         initpoint->position.east = 25.0;
  //         initpoint->orientation.yaw = -65 * M_PI / 180.0;
  //         it = 0;
  //         timebase = 0;
  //         break;
  //       default:
  //         break;
  //     }
  //     nuref.reset(new auv_msgs::BodyVelocityReq());
  //     nu_pub.publish(nuref);
  //     init_pub.publish(initpoint);
  //   }
  //   rate.sleep();
  //   ros::spinOnce();
  // }
}
