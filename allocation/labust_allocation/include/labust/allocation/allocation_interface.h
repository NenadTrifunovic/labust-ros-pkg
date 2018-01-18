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
#ifndef ALLOCATION_ALLOCATION_INTERFACE_H
#define ALLOCATION_ALLOCATION_INTERFACE_H
#include <labust/allocation/thruster_configuration.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <vector>

namespace labust
{
	namespace allocation
	{
		/**
		 * Defines a generic allocation interface for all allocation classes
		 * to inherit in order to implement a plug-in based allocation.
		 */
		class AllocationInterface
		{
		public:
			///The virtual thrust and torque enumerator
			enum {X=0,Y,Z,K,M,N};

			///Pointer typedef
			typedef boost::shared_ptr<AllocationInterface> Ptr;
			///Const pointer typedef
			typedef boost::shared_ptr<AllocationInterface const> ConstPtr;

			///Main constructor
			AllocationInterface(){};
			///Main destructor
			virtual ~AllocationInterface(){};

			/**
			 * The main configuration method. The method load parameters
			 * and initializes the subscribers and publishers of the allocation.
			 */
			virtual bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph) = 0;

			/**
			 * The main allocation function that distributes desired
			 * forces and moments directly to the thrusters.
			 *
			 * \return Returns the vector of PWM values need to achieve the desired tau
			 */
			virtual const std::vector<double>& allocate(const Eigen::VectorXd& tau) = 0;

			/**
			 * The method returns the last achieved tau vector.
			 */
			virtual const Eigen::VectorXd& tauA() const = 0;

			/**
			 * The method returns the windup flags per degree of freedom.
			 */
			virtual const std::vector<bool>& windup() = 0;
		};
	}
}

/* ALLOCATION_ALLOCATION_INTERFACE_H */
#endif
