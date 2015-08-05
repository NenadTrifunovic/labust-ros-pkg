/*********************************************************************
 * EKF_RTT.cpp
 *
 *  Created on: 
 *      Author: Dula Nad
 *      
 *  Modified by: Filip Mandic
 *
 ********************************************************************/
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, LABUST, UNIZG-FER
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
#ifndef EKF_RTT_HPP_
#define EKF_RTT_HPP_
#include <labust/navigation/KFCore.hpp>
#include <labust/navigation/RelativeTrackingModel.hpp>
#include <labust/navigation/SensorHandlers.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <navcon_msgs/ModelParamsUpdate.h>

#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <auv_msgs/BodyForceReq.h>

#include <std_msgs/Bool.h>
#include <auv_msgs/NavSts.h>
#include <underwater_msgs/USBLFix.h>

namespace labust
{
	namespace navigation
	{
		/**
	 	 * The 3D state estimator for ROVs and AUVs. Extended with altitude and pitch estimates.
	 	 *
	 	 * \todo Extract the lat/lon part into the llnode.
	 	 */
		class Estimator3D
		{
			enum{X=0,Y,Z,K,M,N, DoF};
			typedef labust::navigation::KFCore<labust::navigation::RelativeTrackingModel> KFNav;
		public:
			/**
			 * Main constructor.
			 */
			Estimator3D();
			/**
			 * Initialize the estimation filter.
			 */
			void onInit();
			/**
			 * Start the estimation loop.
			 */
			void start();

		private:
			/**
			 * Helper function for navigation configuration.
			 */
			void configureNav(KFNav& nav, ros::NodeHandle& nh);
			/**
			 * On model updates.
			 */
			void onModelUpdate(const navcon_msgs::ModelParamsUpdate::ConstPtr& update);
			/**
			 * Handle target input forces and torques.
			 */
			void onStateHat(const auv_msgs::NavSts::ConstPtr& data);
			/**
			 * Handle target input forces and torques.
			 */
			void onCurrentsHat(const  geometry_msgs::TwistStamped::ConstPtr& data);
			/**
			 * Handle target input forces and torques.
			 */
			void onTargetTau(const auv_msgs::BodyForceReq::ConstPtr& tau);
			/**
			 * Handle the depth measurement.
			 */
			void onTargetDepth(const std_msgs::Float32::ConstPtr& data);
			/**
			 * Handle the depth measurement.
			 */
			void onTargetHeading(const std_msgs::Float32::ConstPtr& data);
			/**
			 * Handle the USBL measurement.
			 */
			void onUSBLfix(const underwater_msgs::USBLFix::ConstPtr& data);
			/**
			 * Handle the range measurement.
			 */
			//void onRange(const std_msgs::Float32::ConstPtr& data);
			/**
			 * Handle the bearing measurement.
			 */
			//void onBearing(const std_msgs::Float32::ConstPtr& data);
			/**
			 * Helper method to process measurements.
			 */
			void processMeasurements();
			/**
			 * Helper method to publish the navigation state.
			 */
			void publishState();
			/**
			 * The navigation filter.
			 */
			KFNav nav;
			/**
			 * The input vector.
			 */
			KFNav::vector inputVec;
			/**
			 * The measurements vector and arrived flag vector.
			 */
			KFNav::vector measurements, newMeas;
			/**
			 * Heading unwrapper.
			 */
			labust::math::unwrap unwrap;
			/**
			 * Estimated and measured state publisher.
			 */
			ros::Publisher pubStateMeas, pubStateHat;
			/**
			 * Sensors and input subscribers.
			 */
			ros::Subscriber subStateHat, subCurrentsHat, subTargetTau, subTargetDepth, subTargetHeading, subUSBLfix, modelUpdate;
			/**
			 * The transform broadcaster.
			 */
			tf2_ros::TransformBroadcaster broadcaster;
			/**
			 * Temporary altitude, NED currents storage.
			 */
			double alt, xc, yc, u, v;
			/**
			 * Model parameters
			 */
			KFNav::ModelParams params[DoF];
		};
	}
}
/* EKF3D_HPP_ */
#endif
