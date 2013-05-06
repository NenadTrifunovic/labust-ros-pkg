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
 *  Created: 06.05.2013.
 *********************************************************************/
#include <labust/control/HLManager.hpp>
#include <labust_uvapp/ConfigureVelocityController.h>
#include <labust_uvapp/EnableControl.h>
#include <labust/math/NumberManipulation.hpp>

using labust::control::HLManager;

HLManager::HLManager():
			nh(ros::NodeHandle()),
			ph(ros::NodeHandle("~")),
			lastEst(ros::Time::now()),
			launchTime(ros::Time::now()),
			launchDetected(false),
			timeout(2),
			mode(HLManager::stop),
			type(HLManager::bArt),
			safetyRadius(2),
			safetyDistance(50),
			safetyTime(30),
			circleRadius(10),
			turnDir(1),
			lookAhead(1)
{this->onInit();}

void HLManager::onInit()
{
	//Fill controller names
	controllers.insert(std::make_pair("LF",false));
	controllers.insert(std::make_pair("DP",false));

	//Initialize publishers
	refPoint = nh.advertise<geometry_msgs::PointStamped>("ref_point", 1);
	refTrack = nh.advertise<auv_msgs::NavSts>("ref_track", 1);

	//Initialze subscribers
	state = nh.subscribe<auv_msgs::NavSts>("stateHat", 1,
			&HLManager::onVehicleEstimates,this);
	launch = nh.subscribe<std_msgs::Bool>("launched", 1,
			&HLManager::onLaunch,this);

	//Configure service
	modeServer = nh.advertiseService("SetHLMode",
			&HLManager::setHLMode, this);

	nh.param("hl_manager/timeout",timeout,timeout);
	nh.param("hl_manager/radius",safetyRadius,safetyRadius);
	nh.param("hl_manager/safetyDistance",safetyDistance,safetyDistance);
	nh.param("hl_manager/safetyTime",safetyTime,safetyTime);
	nh.param("hl_manager/circleRadius", circleRadius, circleRadius);
}

bool HLManager::setHLMode(cart2::SetHLMode::Request& req,
		cart2::SetHLMode::Response& resp)
{
	//Check if the mode is already active
	this->point = req.ref_point;

	if (this->mode == req.mode) return true;
	//Else handle the mode change
	mode = req.mode;

	ros::ServiceClient client =
				nh.serviceClient<labust_uvapp::ConfigureVelocityController>("ConfigureVelocityController");
	labust_uvapp::ConfigureVelocityController srv;
	for (int32_t i=srv.request.u; i<= srv.request.r;++i)
	{
		srv.request.desired_mode[i] = srv.request.DisableAxis;
	}

	srv.request.desired_mode[srv.request.u] = srv.request.ControlAxis;
	srv.request.desired_mode[srv.request.r] = srv.request.ControlAxis;
	point.header.frame_id = "local";
	point.header.stamp = ros::Time::now();
	refPoint.publish(point);

	switch (mode)
	{
	case manual:
		ROS_INFO("Set to manual mode.");
		srv.request.desired_mode[srv.request.u] = srv.request.ManualAxis;
		srv.request.desired_mode[srv.request.r] = srv.request.ManualAxis;
		return client.call(srv);
		break;
	case gotoPoint:
		ROS_INFO("Set to GoTo mode.");
		controllers["LF"] = true;
		controllers["DP"] = false;
		return client.call(srv) && configureControllers();
		break;
	case stationKeeping:
		ROS_INFO("Set to Station keeping mode.");
	case circle:
		if (mode == circle) ROS_INFO("Set to Circle mode.");
		controllers["LF"] = false;
		controllers["DP"] = true;
		return client.call(srv) && configureControllers();
	case stop:
		ROS_INFO("Stopping.");
		return this->fullStop();
		break;
	default:
	  ROS_ERROR("Wrong mode selected:%d",mode);
	  break;
	}

	return true;
}

bool HLManager::fullStop()
{
	labust_uvapp::EnableControl srv;
	srv.request.enable = false;
	ros::ServiceClient client = nh.serviceClient<labust_uvapp::EnableControl>("VelCon_enable");
	bool serviceFlag;
	if (!((serviceFlag=client.call(srv))))
	{
	  ROS_ERROR("Failed to call velocity control configuration service.");
	  return false;
	}

	for (ControllerMap::iterator it=controllers.begin();
			it != controllers.end(); ++it)
	{
		it->second = false;
	}

	if (!configureControllers()) return false;

	ROS_INFO("Stopping all axes.\n");
	return true;
}

bool HLManager::configureControllers()
{
	bool success = true;
	for (ControllerMap::const_iterator it=controllers.begin();
			it != controllers.end(); ++it)
	{
		ros::ServiceClient client = nh.serviceClient<labust_uvapp::EnableControl>(it->first + "_enable");
		labust_uvapp::EnableControl srv;
		srv.request.enable = it->second;
		if (!client.call(srv))
		{
		  ROS_ERROR("Failed to call the %s configuration service.",it->first.c_str());
		  success = false;
		}
	}

	return success;
}

void HLManager::onVehicleEstimates(const auv_msgs::NavSts::ConstPtr& estimate)
{
	boost::mutex::scoped_lock l(dataMux);
	this->stateHat = *estimate;
	lastEst = ros::Time::now();
};

void HLManager::onLaunch(const std_msgs::Bool::ConstPtr& isLaunched)
{
	boost::mutex::scoped_lock l(dataMux);
	launchDetected = isLaunched->data;
	launchTime = ros::Time::now();

	if (launchDetected)
	{
		//Get current heading and calculate the desired point
		point.point.x = stateHat.position.north + safetyDistance*cos(stateHat.orientation.yaw);
		point.point.y = stateHat.position.east + safetyDistance*sin(stateHat.orientation.yaw);
		point.point.z = 0;

		//Set the world and local coordinate frames from this point on
	}
}

void HLManager::safetyTest()
{
	bool estTimeout = (ros::Time::now() - lastEst).toSec() > timeout;

	if (estTimeout)
	{
		ROS_WARN("Timeout on the control channel. Controlled axes will be disabled.");

		//Some action
		//Turn off all control, switch to manual ?
	}
}

void HLManager::step()
{
	boost::mutex::scoped_lock l(dataMux);
	if ((type == bArt) && (mode==stop) && (launchDetected))
	{
		//Check that enough time has passed
		if ((ros::Time::now() - launchTime).sec > safetyTime)
		{
			launchDetected = false;
			//Switch to station keeping
			cart2::SetHLMode srv;
			srv.request.mode = srv.request.GoToPoint;
			srv.request.ref_point = point;
			this->setHLMode(srv.request, srv.response);
		}
	};

	//Check distance to point
	double dx(point.point.x - stateHat.position.north);
	double dy(point.point.y - stateHat.position.east);
	double dist(sqrt(dx*dx+dy*dy));
	double relAngle(atan2(dy,dx));
	double angleDiff(fabs(labust::math::wrapRad(relAngle - stateHat.orientation.yaw)));

	if (mode == gotoPoint)
	{
		//If distance to point is OK or the vehicle missed the point
		if (dist < safetyRadius || (angleDiff > M_PI/2))
		{
			//Switch to station keeping
			cart2::SetHLMode srv;
			srv.request.mode = srv.request.StationKeeping;
			srv.request.ref_point = point;
			this->setHLMode(srv.request, srv.response);
		}
	}

	if (mode == circle)
	{
		//Handle modes that need timing
		relAngle = atan2(-dy,-dx);
		trackPoint.position.north = point.point.x +
				circleRadius*cos(relAngle) +
				lookAhead*cos(relAngle + M_PI/2);
		trackPoint.position.east = point.point.y +
				circleRadius*sin(relAngle) +
				lookAhead*sin(relAngle + M_PI/2);

		refTrack.publish(trackPoint);
	}
}

void HLManager::start()
{
	ros::Rate rate(10);

	ros::AsyncSpinner spinner(1);
	spinner.start();
	while (nh.ok())
	{
		this->safetyTest();
		this->step();
		rate.sleep();
		//ros::spinOnce();
	}
}
