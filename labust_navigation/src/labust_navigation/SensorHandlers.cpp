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
#include <labust/tools/conversions.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>

#include <boost/lexical_cast.hpp>

using namespace labust::navigation;

void GPSHandler::configure(ros::NodeHandle& nh)
{
	std::string key;
	if (nh.searchParam("tf_prefix", key)) nh.getParam(key, tf_prefix);

	gps = nh.subscribe<sensor_msgs::NavSatFix>("gps", 1,
			&GPSHandler::onGps, this);

	//status_handler_.addKeyValue("Bottom lock");
	status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::WARN);
	status_handler_.setEntityMessage("Status handler initialized.");
	status_handler_.publishStatus();
}

void GPSHandler::onGps(const sensor_msgs::NavSatFix::ConstPtr& data)
{
	status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::ERROR);

	//Calculate to X-Y tangent plane
	geometry_msgs::TransformStamped transformDeg, transformLocal, transformGPS;
	try
	{
		//Get the ENU coordinates
		transformDeg = buffer.lookupTransform("ecef", tf_prefix + "world", ros::Time(0));
		//Set the projection origin
		double lat0,lon0,h0;
		GeographicLib::Geocentric::WGS84.Reverse(
				transformDeg.transform.translation.x,
				transformDeg.transform.translation.y,
				transformDeg.transform.translation.z,
				lat0, lon0, h0);
		proj.Reset(lat0, lon0, h0);
		double e,n,u;
		proj.Forward(data->latitude,
				data->longitude,
				data->altitude,
				e,n,u);
		//Convert to NED
		Eigen::Quaternion<double> qrot;
		labust::tools::quaternionFromEulerZYX(M_PI,0,M_PI/2,qrot);
		Eigen::Vector3d enu;
		enu<<e,n,u;
		Eigen::Vector3d ned = qrot.toRotationMatrix() * enu;
		//Get rotation to base_link
		try
		{
			//Try to get the transform at the exact time or the latest
			try
			{
				transformLocal = buffer.lookupTransform(tf_prefix + "local", tf_prefix + "base_link", data->header.stamp);
			}
			catch (tf2::TransformException& ex)
			{
				transformLocal = buffer.lookupTransform(tf_prefix + "local", tf_prefix + "base_link", ros::Time(0));
			}
			//Get GPS offset
			transformGPS = buffer.lookupTransform(tf_prefix + "base_link", tf_prefix + "gps_frame", data->header.stamp);
			Eigen::Vector3d gps_b;
			gps_b<<transformGPS.transform.translation.x,
					transformGPS.transform.translation.y,
					transformGPS.transform.translation.z;
			Eigen::Quaternion<double> rot(transformLocal.transform.rotation.w,
					transformLocal.transform.rotation.x,
					transformLocal.transform.rotation.y,
					transformLocal.transform.rotation.z);
			ned -= rot.toRotationMatrix()*gps_b;
			enu = qrot.toRotationMatrix().transpose()*ned;
		}
		catch (tf2::TransformException& ex)
		{
			ROS_WARN("Unable to decode GPS measurement. Missing frame : %s",ex.what());
		}
		
		//Set the data
 		posxy.first = ned(0);
 		posxy.second = ned(1);
		originLL.first = lat0;
		originLL.second = lon0;
		
		double lat, lon;
		proj.Reverse(enu(0), enu(1), enu(2), lat, lon, originh);
		posLL.second = lat;
		posLL.first = lon;
		isNew = true;

		status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::OK);
	}
	catch(tf2::TransformException& ex)
	{
		ROS_WARN("Unable to decode GPS measurement. Missing frame : %s",ex.what());
	}

	status_handler_.publishStatus();
};

void ImuHandler::configure(ros::NodeHandle& nh)
{
	std::string key;
	if (nh.searchParam("tf_prefix", key)) nh.getParam(key, tf_prefix);

	imu = nh.subscribe<sensor_msgs::Imu>("imu", 1,
			&ImuHandler::onImu, this);
	mag_dec = nh.subscribe<std_msgs::Float64>("magnetic_declination",1,
					&ImuHandler::onMagDec, this);

	//status_handler_.addKeyValue("Bottom lock");
	status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::WARN);
	status_handler_.setEntityMessage("Status handler initialized.");
	status_handler_.publishStatus();
}

void ImuHandler::onImu(const sensor_msgs::Imu::ConstPtr& data)
{
	status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::ERROR);

	geometry_msgs::TransformStamped transform;
	try
	{
		transform = buffer.lookupTransform(tf_prefix + "base_link", data->header.frame_id, ros::Time(0));
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
		rpy[yaw] += magdec;
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
		status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::OK);
	}
	catch (tf2::TransformException& ex)
	{
		ROS_WARN("Failed converting the IMU data: %s",ex.what());
	}
	status_handler_.publishStatus();
};

void DvlHandler::configure(ros::NodeHandle& nh)
{
	std::string key;
	if (nh.searchParam("tf_prefix", key)) nh.getParam(key, tf_prefix);

	nu_dvl = nh.subscribe<geometry_msgs::TwistStamped>("dvl", 1,
			&DvlHandler::onDvl, this);
	dvl_bottom = nh.subscribe<std_msgs::Bool>("dvl_bottom", 1,
			&DvlHandler::onBottomLock, this);

	bottom_lock = false;

	uvw[u] = uvw[v] = uvw[w] = 0;

	status_handler_.addKeyValue("Bottom lock");
	status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::WARN);
	status_handler_.setEntityMessage("Status handler initialized.");
	status_handler_.publishStatus();


}

void DvlHandler::onBottomLock(const std_msgs::Bool::ConstPtr& data)
{
		this->bottom_lock = data->data;
		status_handler_.updateKeyValue("Bottom lock", boost::lexical_cast<std::string>(bottom_lock));

}

void DvlHandler::onDvl(const geometry_msgs::TwistStamped::ConstPtr& data)
{
	status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::ERROR);

	//Ignore water lock data (?)
	if (!bottom_lock)
		ROS_WARN("No bottom lock.");

	if (data->header.frame_id.find("dvl_frame") != std::string::npos)
	{
		try
		{
			geometry_msgs::TransformStamped transform;
			transform = buffer.lookupTransform(tf_prefix + "base_link", tf_prefix + "dvl_frame", ros::Time(0));

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
	else if (data->header.frame_id.find("base_link") != std::string::npos)
	{
		uvw[u] = data->twist.linear.x;
		uvw[v] = data->twist.linear.y;
		uvw[w] = data->twist.linear.z;
	}
	else if (data->header.frame_id.find("local") != std::string::npos)
	{
		geometry_msgs::TransformStamped transform;
		Eigen::Vector3d meas(data->twist.linear.x,
				data->twist.linear.y,
				data->twist.linear.z);
		Eigen::Quaternion<double> rot(transform.transform.rotation.w,
				transform.transform.rotation.x,
				transform.transform.rotation.y,
				transform.transform.rotation.z);
		transform = buffer.lookupTransform(tf_prefix + "local", tf_prefix + "base_link", ros::Time(0));
		Eigen::Vector3d result = rot.matrix()*meas;
		uvw[u] = result.x();
		uvw[v] = result.y();
		uvw[w] = result.z();
	}
	else if (data->header.frame_id.find("gps_frame") != std::string::npos)
	{
		try
		{
			geometry_msgs::TransformStamped transform;
			transform = buffer.lookupTransform(tf_prefix + "base_link", tf_prefix + "gps_frame", ros::Time(0));

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
	status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::OK);
	status_handler_.publishStatus();
}

void iUSBLHandler::configure(ros::NodeHandle& nh)
{
	std::string key;
	if (nh.searchParam("tf_prefix", key)) nh.getParam(key, tf_prefix);

	usbl_sub = nh.subscribe<underwater_msgs::USBLFix>("usbl_fix", 1,
			&iUSBLHandler::onUSBL, this);
	remote_pos_sub = nh.subscribe<auv_msgs::NavSts>("remote_position", 1,
			&iUSBLHandler::onSurfacePos, this);

	pos[x] = pos[y] = pos[z] = 0;

	//status_handler_.addKeyValue("Bottom lock");
	status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::WARN);
	status_handler_.setEntityMessage("Status handler initialized.");
	status_handler_.publishStatus();
}

void iUSBLHandler::onSurfacePos(const auv_msgs::NavSts::ConstPtr& data)
{
	remote_position = *data;
	remote_arrived = true;
	if (fix_arrived) merge();
}

void iUSBLHandler::onUSBL(const underwater_msgs::USBLFix::ConstPtr& data)
{
	fix = *data;
	fix_arrived = true;
	if (remote_arrived) merge();
}

void iUSBLHandler::merge()
{
	status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::ERROR);

	//Reset arrivals
	fix_arrived = remote_arrived = false;
	//Offset compensation
	//if (fix.header.frame_id.find("usbl_frame") != std::string::npos)
	{
		try
		{
			geometry_msgs::TransformStamped transform, transformDeg, transformDegUSBL;
			transform = buffer.lookupTransform(tf_prefix + "base_link", tf_prefix + "usbl_frame", ros::Time(0));
			transformDeg = buffer.lookupTransform(tf_prefix + "local", tf_prefix + "base_link", ros::Time(0));
			transformDegUSBL = buffer.lookupTransform(tf_prefix + "local", tf_prefix + "usbl_frame", ros::Time(0));

			Eigen::Vector3d offset_base(transform.transform.translation.x, 
					transform.transform.translation.y, 
					transform.transform.translation.z);
			Eigen::Quaternion<double> rot(transformDeg.transform.rotation.w,
					transformDeg.transform.rotation.x,
					transformDeg.transform.rotation.y,
					transformDeg.transform.rotation.z);
			Eigen::Vector3d offset_ned = rot.matrix()*offset_base;

			Eigen::Vector3d rpos(fix.relative_position.x, 
				fix.relative_position.y, 
				fix.relative_position.z);
			Eigen::Quaternion<double> rrot(transformDegUSBL.transform.rotation.w,
					transformDegUSBL.transform.rotation.x,
					transformDegUSBL.transform.rotation.y,
					transformDegUSBL.transform.rotation.z);
			Eigen::Vector3d pos_ned = rrot.matrix()*rpos;
			pos[0] = remote_position.position.north - pos_ned(0) - offset_ned(0);
			pos[1] = remote_position.position.east - pos_ned(1) - offset_ned(1);

			ROS_INFO("Received new position: %f %f", pos[x], pos[y]);

			isNew = true;
			status_handler_.setEntityStatus(diagnostic_msgs::DiagnosticStatus::OK);
		}
		catch (std::exception& e)
		{
		   ROS_WARN("%s",e.what());
		}
		status_handler_.publishStatus();
	}

	
	//This is actually delayed position
	//TODO add conversion to horizontal range for improved position
	//TODO add depth
	double bearing = fix.bearing*M_PI/180;
	pos[x] = remote_position.position.north - fix.range * cos(bearing);
	pos[y] = remote_position.position.east - fix.range * sin(bearing);
	ROS_INFO("Received new position: %f %f", pos[x], pos[y]);

	isNew = true;
}
