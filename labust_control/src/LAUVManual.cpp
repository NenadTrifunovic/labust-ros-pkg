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
#include <labust/control/HLControl.hpp>
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/ManControl.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <labust/vehicles/LupisAllocation.hpp>

#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/NavSts.h>


namespace labust
{
	namespace control{
		///The LAUV manual controller
		template <class Enable>
		struct LAUVManual : public Enable, ConfigureAxesPolicy
		{
			LAUVManual():
			nu_max(Eigen::Vector6d::Zero()),
			_nu_in(Eigen::Vector6d::Zero())
			{this->init();};

			void init()
			{
				ros::NodeHandle nh;
				_subJoy = nh.subscribe<sensor_msgs::Joy>("joy", 1, &LAUVManual::onJoy,this);
				_subState = nh.subscribe<auv_msgs::NavSts>("stateHat", 1, &LAUVManual::onState,this);
				_pubTauOut = nh.advertise<auv_msgs::BodyForceReq>("tauOut", 1);

				initialize_manual();
			}

			void onJoy(const sensor_msgs::Joy::ConstPtr& joyIn)
			{
				//if (!Enable::enable) return;
				auv_msgs::BodyForceReq::Ptr tau(new auv_msgs::BodyForceReq());
				Eigen::Vector6d mapped;
				mapper.remap(*joyIn, mapped);

				tau->header.stamp = ros::Time::now();
				tau->header.frame_id = "base_link";
				tau->goal.requester = "tau_manual";
				//tau->disable_axis = this->disable_axis;

				//mapped = nu_max.cwiseProduct(mapped);
				double scale = 0.01;

				Eigen::Vector6d tau_out, servo_pos, thruster_act;
				tau_out = Eigen::Vector6d::Zero();
				servo_pos = Eigen::Vector6d::Zero();
				thruster_act = Eigen::Vector6d::Zero();

				thruster_act(0) = mapped(2);

				servo_pos(1) = scale*mapped(0);
				servo_pos(2) = servo_pos(1);

				servo_pos(0) = scale*mapped(1);
				servo_pos(3) = servo_pos(0);

				_allocation.inverseAllocate(thruster_act,_nu_in,servo_pos,tau_out);

				labust::tools::vectorToPoint(tau_out, tau->wrench.force);
				labust::tools::vectorToPoint(tau_out, tau->wrench.torque, 3);

				_pubTauOut.publish(tau);
			}

			void onState(const auv_msgs::NavSts::ConstPtr& state)
			{
				_nu_in << state->body_velocity.x, state->body_velocity.y, state->body_velocity.z,
						state->orientation_rate.roll, state->orientation_rate.pitch, state->orientation_rate.yaw;
			}

			void initialize_manual()
			{
				ROS_INFO("Initializing manual nu controller...");

				Eigen::Vector6d fin_lift = Eigen::Vector6d::Zero();

				ros::NodeHandle nh;
				labust::tools::getMatrixParam(nh,"nu_manual/maximum_speeds", nu_max);
				labust::tools::getMatrixParam(nh,"simulation/fin_lift", fin_lift);

				_allocation.init(7,0,fin_lift);

				ROS_INFO("Manual nu controller initialized.");
			}

		private:
			Eigen::Vector6d nu_max, _nu_in;
			ros::Subscriber _subJoy, _subState;
			ros::Publisher _pubTauOut;
			JoystickMapping mapper;
			labust::vehicles::LupisAllocator _allocation;
		};
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"lauv_manual");

	labust::control::LAUVManual<labust::control::EnableServicePolicy> controller;
	ros::spin();

	return 0;
}



