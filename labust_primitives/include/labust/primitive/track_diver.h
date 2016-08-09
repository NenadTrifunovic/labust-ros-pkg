/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, LABUST, UNIZG-FER
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
#ifndef PRIMITIVES_TRACK_DIVER_H
#define PRIMITIVES_TRACK_DIVER_H
#include <labust/primitive/PrimitiveBase.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <navcon_msgs/TrackDiverAction.h>
#include <auv_msgs/NavSts.h>
#include <auv_msgs/FSPathInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ros/ros.h>


#include <boost/thread/mutex.hpp>
#include <limits>

namespace labust
{
	namespace primitive
	{
		/**
		 * The class implements the diver tracking primitive utilizing the virtual target
		 * path following.
		 */
		class TrackDiver : protected ExecutorBase<navcon_msgs::TrackDiverAction>
		{
			///Convenience typedefs
			typedef navcon_msgs::TrackDiverGoal Goal;
			typedef navcon_msgs::TrackDiverResult Result;
			typedef navcon_msgs::TrackDiverFeedback Feedback;
			typedef ExecutorBase<navcon_msgs::TrackDiverAction> Base;

			///Controller enumerator
			enum {vt = 0, numcnt};

			///Minimal safety radius
			const float min_radius;

		public:
			///Main constructor
			TrackDiver();

			///Primitive initialization
			void init();

			///Main action startup method
			void onGoal();

			///Main action preemption handler
			void onPreempt();

		protected:
			///Helper function for controller updates
			void updateControllers(bool on = false);
			///Helper function for goal processing
			void processGoal();
			///Helper function for stopping
			void stop();
			///Helper function for parameter loading
			void boundedParam(ros::NodeHandle& ph, const std::string& name, double* param, double bound, double def);

			///Vehicle state handling method
			void onStateHat(const auv_msgs::NavSts::ConstPtr& estimate);
			///Guidance point handling method
			void onGuidancePoint(const geometry_msgs::PointStamped::ConstPtr& point)
			{
				boost::mutex::scoped_lock l(goal_mux);
				cgoal->guidance_target = point->point;
				//Enable guidance
				cgoal->guidance_enable = true;
			}
			///Variable radius handling method
			void onRadius(const std_msgs::Float32::ConstPtr& radius)
			{
				boost::mutex::scoped_lock l(goal_mux);
				cgoal->radius = labust::math::coerce(
						radius->data, min_radius, std::numeric_limits<double>::quiet_NaN());
			}
			///Diver position update handling
			void onDiverState(const auv_msgs::NavSts::ConstPtr& diver_state);
			///Path speed update handling
			void onPathSpeed(const geometry_msgs::TwistStamped::ConstPtr& dpi_r)
			{
				boost::mutex::scoped_lock l(state_mux);
				this->dpi_r = dpi_r->twist.linear.x;
			}


			///The main worker method
			void step();
			///Frame init method
			void initPath();
			///Frame update method
			void updateFS();
			///Monitoring and guidance position caclulation
			void setDesiredPathPosition();
			///Heading calculation
			void setDesiredOrientation(const auv_msgs::NavSts& estimate);

			///Subscriber for the point guidance
			ros::Subscriber guidance_sub;
			//Subscriber for the variable radius
			ros::Subscriber radius_sub;
			///Path speed frame progression
			ros::Subscriber dpi_r_sub;
			///Diver state
			ros::Subscriber diver_state;
			/// Heading reference publisher
			ros::Publisher heading_pub;
			///Current action goal
			Goal::Ptr cgoal;
			///New goal processing flag
			bool process_new;
			///Goal protection mutex
			boost::mutex goal_mux;
			///State protection mutex
			boost::mutex state_mux;
			///FS frame protector
			boost::mutex fs_mux;
			///Transform publisher
			tf2_ros::TransformBroadcaster broadcaster;

			///Current diver position
			auv_msgs::NavSts diver_pos;
			///Current vehicle position
			auv_msgs::NavSts vehicle_pos;
			///Current desired path speed
			double dpi_r;
			///Current path state
			auv_msgs::FSPathInfo path;
			///Current feedback
			Feedback finfo;

			///Guidance FOV parameter
			double sigma_fov;
			///Guidance gain parameter
			double kxi;
			///Maximum monitoring range
			double sigma_m;
			//Approach gain
			double kpsi;

			///Track the unwrapped angles
			labust::math::unwrap psi_d;
		};
	}
}

/* PRIMITIVES_TRACK_DIVER_H */
#endif
