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
#include <labust/navigation/SensorHandlers.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <labust/tools/conversions.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>

using namespace labust::navigation;

void GPSHandler::configure(ros::NodeHandle& nh)
{
	gps = nh.subscribe<sensor_msgs::NavSatFix>("gps", 1,
			&GPSHandler::onGps, this);
}

void GPSHandler::onGps(const sensor_msgs::NavSatFix::ConstPtr& data)
{
	//Calculate to X-Y tangent plane
	geometry_msgs::TransformStamped transformDeg, transformLocal, transformGPS;
	try
	{
		transformLocal = buffer.lookupTransform("local", "gps_frame", ros::Time(0));
		transformGPS = buffer.lookupTransform("base_link", "gps_frame", ros::Time(0));
		transformDeg = buffer.lookupTransform("worldLatLon", "local", ros::Time(0));
		posxy =	labust::tools::deg2meter(data->latitude - transformDeg.transform.translation.y,
					data->longitude - transformDeg.transform.translation.x,
					transformDeg.transform.translation.y);
	/*	Eigen::Quaternion<double> rot(transformLocal.transform.rotation.w,
				transformLocal.transform.rotation.x,
				transformLocal.transform.rotation.y,
				transformLocal.transform.rotation.z);*/
		Eigen::Vector3d offset(transformGPS.transform.translation.x,
				transformGPS.transform.translation.y,
				transformGPS.transform.translation.z);
		Eigen::Vector3d pos_corr = rot.matrix()*offset;

 		posxy.first -= pos_corr(0);
 		posxy.second -= pos_corr(1);
		
		//ROS_ERROR("Corrected position: %f %f", pos_corr(0), pos_corr(1));
		
		originLL.first = transformDeg.transform.translation.y;
		originLL.second = transformDeg.transform.translation.x;

		std::pair<double, double> posxy_corr = labust::tools::meter2deg(posxy.first, posxy.second, transformDeg.transform.translation.y);
		posLL.first = originLL.first + posxy_corr.first;
		posLL.second = originLL.second + posxy_corr.second;

		isNew = true;
	}
	catch(tf2::TransformException& ex)
	{
		ROS_WARN("Unable to decode GPS measurement. Missing frame : %s",ex.what());
	}
};

void ImuHandler::configure(ros::NodeHandle& nh)
{
	imu = nh.subscribe<sensor_msgs::Imu>("imu", 1,
			&ImuHandler::onImu, this);
}

void ImuHandler::onImu(const sensor_msgs::Imu::ConstPtr& data)
{
	geometry_msgs::TransformStamped transform;
	try
	{
		transform = buffer.lookupTransform("base_link", data->header.frame_id, ros::Time(0));
		Eigen::Quaternion<double> meas(data->orientation.w, data->orientation.x,
				data->orientation.y, data->orientation.z);
		Eigen::Quaternion<double> rot(transform.transform.rotation.w,
				transform.transform.rotation.x,
				transform.transform.rotation.y,
				transform.transform.rotation.z);
		Eigen::Quaternion<double> result = meas*rot;
		if (gps != 0) gps->setRotation(result);
		//KDL::Rotation::Quaternion(result.x(),result.y(),result.z(),result.w()).GetEulerZYX
		//		(rpy[yaw],rpy[pitch],rpy[roll]);
		labust::tools::eulerZYXFromQuaternion(result, rpy[roll], rpy[pitch], rpy[yaw]);
		rpy[yaw] = unwrap(rpy[yaw]);

		//Transform angular velocities
		Eigen::Vector3d angvel;
		angvel<<data->angular_velocity.x,
				data->angular_velocity.y,
				data->angular_velocity.z;
		angvel = meas*angvel;

		pqr[p] = angvel(0);
		pqr[q] = angvel(1);
		pqr[r] = angvel(2);

		//Transform the accelerations
		angvel<<data->linear_acceleration.x,
				data->linear_acceleration.y,
				data->linear_acceleration.z;
		angvel = meas*angvel;

		axyz[ax] = angvel(0);
		axyz[ay] = angvel(1);
		axyz[az] = angvel(2);

		isNew = true;
	}
	catch (tf2::TransformException& ex)
	{
		ROS_WARN("Failed converting the IMU data: %s",ex.what());
	}
};

void DvlHandler::configure(ros::NodeHandle& nh)
{
	nu_dvl = nh.subscribe<geometry_msgs::TwistStamped>("dvl", 1,
			&DvlHandler::onDvl, this);
	dvl_bottom = nh.subscribe<std_msgs::Bool>("dvl_bottom", 1,
			&DvlHandler::onBottomLock, this);

	bottom_lock = false;

	uvw[u] = uvw[v] = uvw[w] = 0;
}

void DvlHandler::onBottomLock(const std_msgs::Bool::ConstPtr& data)
{
		this->bottom_lock = data->data;
}

void DvlHandler::onDvl(const geometry_msgs::TwistStamped::ConstPtr& data)
{
	//Ignore water lock data (?)
	if (!bottom_lock)	ROS_WARN("No bottom lock.");

	if (data->header.frame_id == "dvl_frame")
	{
		try
		{
			geometry_msgs::TransformStamped transform;
			transform = buffer.lookupTransform("base_link", "dvl_frame", ros::Time(0));

			Eigen::Vector3d speed(data->twist.linear.x, data->twist.linear.y, data->twist.linear.z);
			Eigen::Quaternion<double> rot(transform.transform.rotation.w,
					transform.transform.rotation.x,
					transform.transform.rotation.y,
					transform.transform.rotation.z);
			//Add compensation for DVL offset
			Eigen::Vector3d offset(transform.transform.translation.x,
					transform.transform.translation.y,
					transform.transform.translation.z);
			//Rotational speed compensation
			Eigen::Matrix3d omega;
			Eigen::Vector3d nur(0,0,r);
			enum {p=0,q,r};
			omega<<0,-nur(r),nur(q),
					nur(r),0,-nur(p),
					-nur(q),nur(p),0;
			Eigen::Vector3d body_speed = rot.matrix()*speed - omega*offset;

			uvw[u] = body_speed.x();
			uvw[v] = body_speed.y();
			uvw[w] = body_speed.z();
		}
		catch (std::exception& ex)
		{
			ROS_WARN("DVL measurement failure:%s",ex.what());
			isNew = false;
			return;
		}
	}
	else if (data->header.frame_id == "base_link")
	{
		uvw[u] = data->twist.linear.x;
		uvw[v] = data->twist.linear.y;
		uvw[w] = data->twist.linear.z;
	}
	else if (data->header.frame_id == "local")
	{
		geometry_msgs::TransformStamped transform;
		Eigen::Vector3d meas(data->twist.linear.x,
				data->twist.linear.y,
				data->twist.linear.z);
		Eigen::Quaternion<double> rot(transform.transform.rotation.w,
				transform.transform.rotation.x,
				transform.transform.rotation.y,
				transform.transform.rotation.z);
		transform = buffer.lookupTransform("local", "base_link", ros::Time(0));
		Eigen::Vector3d result = rot.matrix()*meas;
		uvw[u] = result.x();
		uvw[v] = result.y();
		uvw[w] = result.z();
	}
	else if (data->header.frame_id == "gps_frame")
	{
		try
		{
			geometry_msgs::TransformStamped transform;
			transform = buffer.lookupTransform("base_link", "gps_frame", ros::Time(0));

			Eigen::Vector3d speed(data->twist.linear.x, data->twist.linear.y, data->twist.linear.z);
			Eigen::Quaternion<double> rot(transform.transform.rotation.w,
					transform.transform.rotation.x,
					transform.transform.rotation.y,
					transform.transform.rotation.z);
			Eigen::Vector3d body_speed = rot.matrix()*speed;

			//Add compensation for excentralized GPS
			Eigen::Vector3d origin(transform.transform.translation.x,
					transform.transform.translation.y,
					transform.transform.translation.z);
			if (origin.x() != 0 || origin.y() != 0)
			{

				body_speed -= Eigen::Vector3d(-r*origin.y(),r*origin.x(),0);
			}

			uvw[u] = body_speed.x();
			uvw[v] = body_speed.y();
			uvw[w] = body_speed.z();
		}
		catch (std::exception& ex)
		{
			ROS_WARN("DVL measurement failure:%s",ex.what());
			isNew = false;
			return;
		}
	}
	else
	{
		isNew = false;
		return;
	}

	isNew = true;
}
