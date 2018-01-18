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
#include <labust/control/HLControl.hpp>
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/WindupPolicy.hpp>
#include <labust/control/PIFFController.h>
#include <labust/control/IPFFController.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <Eigen/Dense>
#include <labust_msgs/BodyForceReq.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The heading controller
		struct HDGControl : DisableAxis
		{
			enum {N=5};

			HDGControl():Ts(0.1),yawRefPast(0.0),manRefFlag(true){};

			void init()
			{
				ros::NodeHandle nh;
				manRefSub = nh.subscribe<std_msgs::Bool>("manRefHeading",1,&HDGControl::onManRef,this);
				initialize_controller();
			}

			void onManRef(const std_msgs::Bool::ConstPtr& state)
			{
				manRefFlag = state->data;
			}


			void windup(const labust_msgs::BodyForceReq& tauAch)
			{
				//Copy into controller
				con.extWindup = tauAch.windup.yaw;
			};


			void idle(const auv_msgs::NavigationStatus& ref, const auv_msgs::NavigationStatus& state,
					const labust_msgs::BodyVelocityReq& track)
			{
				//Tracking external commands while idle (bumpless)
				con.desired = labust::math::wrapRad(state.orientation.z);
				con.state = unwrap(state.orientation.z);
				con.track = track.twist.angular.z;

				float werror = labust::math::wrapRad(con.desired - con.state);
				float wperror = con.b*werror + (con.b-1)*con.state;
				PIFF_wffIdle(&con, Ts, werror, wperror, ref.orientation_rate.z);
			};

			void reset(const auv_msgs::NavigationStatus& ref, const auv_msgs::NavigationStatus& state)
			{
				//UNUSED
			};

			labust_msgs::BodyVelocityReqPtr step(const auv_msgs::NavigationStatus& ref,
					const auv_msgs::NavigationStatus& state)
			{
				double a = 0;
				//Populate variables
				//con.desired = labust::math::wrapRad(ref.orientation.yaw);
				yawRefPast = labust::math::wrapRad((1-a)*ref.orientation.z + (a)*yawRefPast);
				con.desired = yawRefPast;
				//yawRefPast = ref.orientation.yaw;
				con.state = unwrap(state.orientation.z);
				con.track = state.orientation_rate.z;

				float werror = labust::math::wrapRad(con.desired - con.state);
				float wperror = con.b*werror + (con.b-1)*con.state;

				if(manRefFlag)
				{
					PIFF_wffStep(&con,Ts, werror, wperror, 0*ref.orientation_rate.z);

					//Publish output
					labust_msgs::BodyVelocityReqPtr nu(new labust_msgs::BodyVelocityReq());
					nu->header.stamp = ros::Time::now();
					nu->goal.requester = "hdg_controller";
					labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);
					nu->twist.angular.z = con.output;
					return nu;

				}
				else
				{
					con.track = ref.orientation_rate.z;

					PIFF_wffIdle(&con, Ts, werror, wperror,  ref.orientation_rate.z);

					//Publish output
					labust_msgs::BodyVelocityReqPtr nu(new labust_msgs::BodyVelocityReq());
					nu->header.stamp = ros::Time::now();
					nu->goal.requester = "hdg_controller";
					labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);
					/*** Publish only feed forward speed ***/
					nu->twist.angular.z = ref.orientation_rate.z;
					return nu;
				}
			}

			void initialize_controller()
			{
				ROS_INFO("Initializing heading controller...");

				ros::NodeHandle nh;
				double closedLoopFreq(1);
				nh.param("hdg_controller/closed_loop_freq", closedLoopFreq, closedLoopFreq);
				nh.param("hdg_controller/sampling",Ts,Ts);
				double overshoot(1.5);
				nh.param("hdg_controller/overshoot",overshoot,overshoot);

				disable_axis[N] = 0;

				PIDBase_init(&con);
				PIFF_tune(&con, float(closedLoopFreq), overshoot);

				ROS_INFO("Heading controller initialized.");
			}

		private:
			PIDBase con;
			double Ts;
			labust::math::unwrap unwrap;
			double yawRefPast;
			ros::Subscriber manRefSub;
			bool manRefFlag;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"hdg_control");

	labust::control::HLControl<labust::control::HDGControl,
	labust::control::EnableServicePolicy,
	labust::control::WindupPolicy<labust_msgs::BodyForceReq> > controller;
	ros::spin();

	return 0;
}



