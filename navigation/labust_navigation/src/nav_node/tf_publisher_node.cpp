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
#include <labust/tools/conversions.hpp>
#include <auv_msgs/NavSts.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

struct TFPublisher
{
	TFPublisher()
	{
		ros::NodeHandle nh,ph("~");
		std::string key;
		if (nh.searchParam("tf_prefix", key)) nh.getParam(key, tf_prefix);

		blstate_sub = nh.subscribe<auv_msgs::NavSts>("base_link_state", 1, &TFPublisher::onBaseLinkState, this);
		bpstate_sub = nh.subscribe<auv_msgs::NavSts>("base_pose_state", 1, &TFPublisher::onBasePoseState, this);
	}

	~TFPublisher(){}

	void onBaseLinkState(const auv_msgs::NavSts::ConstPtr& position)
	{
	    // base_pose and base_link have the same origin
        geometry_msgs::TransformStamped transform;
        transform.transform.translation.x = 0;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 0;
        labust::tools::quaternionFromEulerZYX(
                position->orientation.roll,
                position->orientation.pitch,
                position->orientation.yaw,
                transform.transform.rotation);
        transform.child_frame_id = tf_prefix + "base_link";
        transform.header.frame_id = tf_prefix + "base_pose";
        transform.header.stamp = ros::Time::now();
        broadcaster.sendTransform(transform);
	};

    void onBasePoseState(const auv_msgs::NavSts::ConstPtr& position)
    {
        // local -> base_pose have the same orientation (base_pose = vehicle carried NED)
        geometry_msgs::TransformStamped transform;
        transform.transform.translation.x = position->position.north;
        transform.transform.translation.y = position->position.east;
        transform.transform.translation.z = position->position.depth;
        labust::tools::quaternionFromEulerZYX(
                0,0,0,transform.transform.rotation);
        transform.child_frame_id = tf_prefix + "base_pose";
        transform.header.frame_id = tf_prefix + "local";
        transform.header.stamp = ros::Time::now();
        broadcaster.sendTransform(transform);
    };

private:
	/// Subscriber to the base_pose->base_link information state.
	ros::Subscriber blstate_sub;
	/// Subscriber to the base_pose->base_link information state.
	ros::Subscriber bpstate_sub;
	/// Transform broadcaster
	tf2_ros::TransformBroadcaster broadcaster;
	/// The transfomr publisher frame
	std::string tf_prefix;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"tf_publisher_node");
	TFPublisher tfnode;
	ros::spin();
	return 0;
}


