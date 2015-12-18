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
 *  Created: 06.03.2013.
 *********************************************************************/
#ifndef LUPISALLOCATION_HPP_
#define LUPISALLOCATION_HPP_
#include <labust/simulation/matrixfwd.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <vector>
#include <ros/ros.h>

namespace labust
{
	namespace vehicles
	{
		/**
		 * Performs thrust allocation based on configuration. Added support for thruster grouping
		 * to support multi-scaling allocation.
		 */
		class LupisAllocator
		{
			enum {NoAlloc=0,SimpleAlloc=1, ScaleAlloc=2};
		public:
			/**
			 * Main constructor.
			 */
			LupisAllocator():
				_max_thrust(1.0),
				_motor_friction(0.0),
				_fin_lift(Eigen::VectorXd::Zero(5)){}

			/**
			 * Performs allocation and necessary conversions.
			 */
			void allocate(const labust::simulation::vector& tauIn,
					labust::simulation::vector& tauOut)
			{

			}

			/**
			 * Performs inverse allocation and necessary conversions.
			 */
			void inverseAllocate(const labust::simulation::vector& thruster_act, labust::simulation::vector&nu_in,
					labust::simulation::vector& servo_pos, labust::simulation::vector& tau_out)
			{
				double speed_u(nu_in(0));

				Eigen::Vector3d deflections(0.0);
				Eigen::Vector3d deflections(0.0);


				deflections(0) = servo_pos(3) - servo_pos(0) + servo_pos(1) - servo_pos(2);
				deflections(1) = servo_pos(1) + servo_pos(2);
				deflections(2) = servo_pos(0) + servo_pos(3);

				tau_out(0) = thruster_act * _max_thrust;
				tau_out(1) = _fin_lift(0) * speed_u * speed_u * deflections(2);
				tau_out(2) = _fin_lift(1) * speed_u * speed_u * deflections(1);
				tau_out(3) = _fin_lift(2) * speed_u * speed_u * deflections(0) + _motor_friction * tau_out(0);
				tau_out(4) = _fin_lift(3) * speed_u * speed_u * deflections(1);
				tau_out(5) = _fin_lift(4) * speed_u * speed_u * deflections(2);

				return;
			}

			/**
			 * Model parameters.
			 */
			double _motor_friction, _max_thrust;
			/**
			 * Fin lift model parameters.
			 */
			Eigen::VectorXd _fin_lift;
		};
	}
}

/* LUPISALLOCATION_HPP_ */
#endif
