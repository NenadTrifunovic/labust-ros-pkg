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
#include <labust/tools/GeoUtilities.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

struct LTPNode
{
	enum {NEEDED_FIXES=30};

	LTPNode():
		origin_lat(0),
		origin_lon(0),
		fix_valid(false),
		fix_count(0)
	{
		ros::NodeHandle nh,ph("~");
		nh.param("environment/origin_lat", origin_lat, origin_lat);
		ph.param("origin_lat", origin_lat, origin_lat);
		nh.param("environment/origin_lon", origin_lon, origin_lon);
		ph.param("origin_lon", origin_lon, origin_lon);
		ph.param("use_local_fix",fix_valid, fix_valid);
		ph.param("use_local_fix",fix_valid, fix_valid);

		gps_raw = nh.subscribe<sensor_msgs::NavSatFix>("gps",1,&LTPNode::onGps, this);

		if (fix_valid) this->setupFrame();

		//runner = boost::thread(boost::bind(&LTPNode::publishFrame, this));
	}

	~LTPNode()
	{
		runner.join();
	}

	void onGps(const sensor_msgs::NavSatFix::ConstPtr& fix)
	{
		if (!fix_valid)
		{
			if (++fix_count > NEEDED_FIXES)
			{
				origin_lat = fix->latitude;
				origin_lon = fix->longitude;
				fix_valid = true;
				fix_count = 0;
				this->setupFrame();
			}
		}
	};

	void setupFrame()
	{
		Eigen::Vector3d geo;
		geo<<origin_lon, origin_lat, 0;
		Eigen::Vector3d ecef = labust::tools::geodetic2ecef(geo);
		Eigen::Matrix3d nedrot = labust::tools::nedrot(geo);
		Eigen::Quaternion<double> q(nedrot.transpose());

		geometry_msgs::TransformStamped transform;
		transform.transform.translation.x = ecef(0);
		transform.transform.translation.y = ecef(1);
		transform.transform.translation.z = ecef(2);
		transform.transform.rotation.x = q.x();
		transform.transform.rotation.y = q.y();
		transform.transform.rotation.z = q.z();
		transform.transform.rotation.w = q.w();
		transform.child_frame_id = "local";
		transform.header.frame_id = "ecef";
		transform.header.stamp = ros::Time::now();
		broadcaster.sendTransform(transform);
	}

private:
	/// Subscriber to raw GPS data
	ros::Subscriber gps_raw;
	/// Transform broadcaster
	tf2_ros::StaticTransformBroadcaster broadcaster;
	/// Local origin latitude
	double origin_lat;
	/// Local origin longitude
	double origin_lon;
	/// Valid fix that can be used
	bool fix_valid;
	/// Fix counter
	int fix_count;
	/// Runner thread
	boost::thread runner;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"ltp_node");
	LTPNode llnode;
	ros::spin();
	return 0;
}


