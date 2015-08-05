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
#ifndef ALLOCATION_ALLOCATION_NODE_H
#define ALLOCATION_ALLOCATION_NODE_H
#include <labust/allocation/allocation_interface.h>
#include <pluginlib/class_loader.h>

#include <auv_msgs/BodyForceReq.h>
#include <ros/ros.h>


namespace labust
{
	namespace allocation
	{
		/**
		 * The class implements the ROS container of the allocation classes.
		 */
		class AllocationNode
		{
		public:
			///Main constructor.
			AllocationNode();
			///General destructor.
			~AllocationNode();

			//Default initialization method.
			void onInit();

		private:
			///The desired force and torque subscriber.
			ros::Subscriber tau_sub;
			///The achieved force and torque publisher.
			ros::Publisher tauach_pub;
			///The achieved PWM commands
			ros::Publisher pwm_pub;

			///Handles the desired force and torque request.
			void onTau(const auv_msgs::BodyForceReq::ConstPtr tau);

			///The safety timer that stops the thrusters.
			ros::Timer safety;
			///The safety timeout
			double timeout;

			///Allocation plug-in loader
			pluginlib::ClassLoader<AllocationInterface> alloc_loader;
			///Allocation handle
			AllocationInterface::Ptr alloc;
		};
	}
}
/* ALLOCATION_ALLOCATION_NODE_H */
#endif
