/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2015, LABUST, UNIZG-FER
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
#ifndef ALLOCATION_THRUSTERCONFIGURATION_H
#define ALLOCATION_THRUSTERCONFIGURATION_H
#include <labust/allocation/ethruster.h>

#include <std_msgs/Float32.h>
#include <ros/ros.h>

#include <boost/array.hpp>
#include <Eigen/Dense>

#include <cmath>
#include <vector>

namespace labust
{
	namespace allocation
	{
		/**
		 * The class implements a object representation of a thruster
		 * configuration.
		 */
		class ThrusterConfiguration
		{
		public:
			///Main constructor
			ThrusterConfiguration();

			///Configure the thruster collection from a private handle
			bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

			///The ideal allocation matrix
			inline const Eigen::MatrixXd& B() const {return _B;};
			///The ideal inverse allocation matrix
			inline const Eigen::MatrixXd& Binv() const {return _Binv;};

			///The achieved tau with the current pwm output.
			inline const Eigen::VectorXd& tauA() const {return T_a;};
			///The pwm output calculation based on the supplied tau vector forces
			inline const std::vector<double>& pwmFromTau(const Eigen::VectorXd& tau){return pwm(_Binv*tau);}

			///The achieved thruster forces with the current pwm output.
			inline const Eigen::VectorXd& Fa() const {return F_a;};
			///The pwm output calculation based on the supplied thruster forces
			const std::vector<double>& pwm(const Eigen::VectorXd& F);

			///The maximum allowable thruster forces
			const Eigen::VectorXd& maxF(){return F_max;};
			///The minimum allowable thruster forces
			const Eigen::VectorXd& minF(){return F_min;};

		private:
			///Handle supply voltage
			void onSupplyVoltage(const std_msgs::Float32::ConstPtr& voltage)
			{
				if (voltage->data <= 0) return;
				this->Us = voltage->data;
				this->updateMinMax();
			}

			///Helper method for initialization of maximum and minimum thruster forces
			void updateMinMax();

			///Supply voltage subscriber
			ros::Subscriber voltage_sub;

			///Allocation matrix
			Eigen::MatrixXd _B;
			///The inverse allocation matrix
			Eigen::MatrixXd _Binv;
			///Thruster collection
			std::vector<EThruster> thrusters;

			///The current supply voltage
			double Us;
			///The nominal motor voltage
			double Un;
			///The pwm adaptation flag
			bool adapt;
			///The pwm output
			std::vector<double> pwm_out;
			///The achieved thrust forces after all coercions
			Eigen::VectorXd T_a;
			///The achieved thrust forces after all coercions
			Eigen::VectorXd F_a;
			///The maximum forces
			Eigen::VectorXd F_max;
			///The minimum forces
			Eigen::VectorXd F_min;
		};
	}
}

/* ALLOCATION_THRUSTERCONFIGURATION_H */
#endif
