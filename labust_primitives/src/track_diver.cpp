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
#include <labust/primitive/track_diver.h>
#include <labust/tools/conversions.hpp>

#include <auv_msgs/FSPathInfo.h>
#include <navcon_msgs/EnableControl.h>

using namespace labust::primitive;

TrackDiver::TrackDiver():
				Base("track_diver"),
				min_radius(2),
				process_new(false),
				dpi_r(0),
				sigma_fov(M_PI/6),
				kxi(1.0),
				sigma_m(10),
				kpsi(1.0){};


void TrackDiver::boundedParam(ros::NodeHandle& ph, const std::string& name, double* param, double bound, double def)
{
	ph.param(name, *param, def);
	if (*param <= bound)
	{
		ROS_WARN("%s needs to be > %f.", name.c_str(),bound);
		*param = def;
	}
}

void TrackDiver::init()
{
	ros::NodeHandle nh, ph("~");
	//Load parameters
	double radius(min_radius);
	boundedParam(ph, "safety_radius", &radius, min_radius, min_radius);
	boundedParam(ph, "kxi", &kxi, 0.0, kxi);
	boundedParam(ph, "kpsi", &kpsi, 0.0, kpsi);
	boundedParam(ph, "sigma_fov", &sigma_fov, radius, sigma_fov);

	//Set default curvature
	path.curvature = 1/radius;
	path.torsion = 0;
	//Init subscribers
	diver_state = nh.subscribe("diver_state",1,
			&TrackDiver::onDiverState, this);
	dpi_r_sub = nh.subscribe("dpi_r", 1,
			&TrackDiver::onPathSpeed,this);
}

void TrackDiver::initPath()
{
	boost::mutex::scoped_lock lg(goal_mux);
	double radius = cgoal->radius;
	double voff = cgoal->vertical_offset;
	lg.unlock();
	boost::mutex::scoped_lock ls(state_mux);
	double nr = sqrt(radius*radius - voff*voff);
	double dx = vehicle_pos.position.north - diver_pos.position.north;
	double dy = vehicle_pos.position.east - diver_pos.position.east;
	ls.unlock();
	boost::mutex::scoped_lock lp(fs_mux);
	path.pi = nr*atan2(dy,dx);
}

void TrackDiver::onGoal()
{
	ROS_DEBUG("TrackDiver: Goal received");
	//Lock the state
	boost::mutex::scoped_lock l(goal_mux);
	bool active = !(cgoal == 0);
	//Protect from preemption in case it is running
	if (!active) cgoal.reset(new Goal());
	process_new = true;
	*cgoal = *aserver->acceptNewGoal();
	process_new = false;
	//Process goal
	processGoal();
	l.unlock();
	//Init path position
	initPath();
	//Update the frame
	//updateFS();
	//Update reference
	//this->step();

	//If no goal was running
	if (!active)
	{
		//Start the controllers
		this->updateControllers(true);
	};
};

void TrackDiver::processGoal()
{
	ROS_DEBUG("Process goal.");
	//If not empty and the topic name changed
	if (!cgoal->guidance_topic.empty() &&
			(cgoal->guidance_topic != guidance_sub.getTopic()))
	{
		ros::NodeHandle nh;
		guidance_sub.shutdown();
		guidance_sub =  nh.subscribe("guidance_point",1,
				&TrackDiver::onGuidancePoint, this);
	}

	//If not empty and the topic name changed
	if (!cgoal->radius_topic.empty() &&
			(cgoal->radius_topic != radius_sub.getTopic()))
	{
		ros::NodeHandle nh;
		radius_sub.shutdown();
		radius_sub =  nh.subscribe("safety_radius",1,
				&TrackDiver::onRadius, this);
	}

	if (cgoal->radius < min_radius)
		cgoal->radius = min_radius;
}


void TrackDiver::onPreempt()
{
	ROS_DEBUG("Preempted.");
	if (!process_new)
	{
		cgoal.reset();
		this->stop();
	}
	else
	{
		ROS_DEBUG("Goal update.");
	}
	aserver->setPreempted();
};

void TrackDiver::onStateHat(const auv_msgs::NavSts::ConstPtr& estimate)
{
	//Update FS frame position internally
	this->updateFS();

	if (aserver->isActive())
	{
		boost::mutex::scoped_lock lg(goal_mux);
		boost::mutex::scoped_lock ls(state_mux);
		//Update current position
		vehicle_pos = *estimate;
		//Calculate desired position and orientation
		//if on a operational circle
		if (fabs(cgoal->vertical_offset) < cgoal->radius)
		{
			this->setDesiredPathPosition();
			this->setDesiredOrientation(*estimate);
		}
		else
		{
			finfo.operational_radius = 0;
			finfo.mu_r = 0;
			finfo.gamma_r = 0;
			boost::mutex::scoped_lock lf(fs_mux);
			path.pi_tilda = 0;
			path.xi_r = path.pi;
			path.dxi_r = 0;
		}
		this->step();
	}
	else
	{
		boost::mutex::scoped_lock l(goal_mux);
		if (cgoal != 0) this->stop();
		l.unlock();
		boost::mutex::scoped_lock ls(state_mux);
		vehicle_pos = *estimate;
	}
};

void TrackDiver::setDesiredPathPosition()
{
	double nr = sqrt(cgoal->radius*cgoal->radius -
			cgoal->vertical_offset*cgoal->vertical_offset);
	finfo.operational_radius = nr;
	//Find monitoring position
	finfo.mu_r = psi_d(diver_pos.orientation.yaw);
	if (cgoal->wrapping_enable) finfo.mu_r = labust::math::wrapRad(finfo.mu_r);
	finfo.mu_r = nr*finfo.mu_r;

	boost::mutex::scoped_lock lf(fs_mux);
	if (cgoal->guidance_enable)
	{
		double dx = cgoal->guidance_target.x - diver_pos.position.north;
		double dy = cgoal->guidance_target.y - diver_pos.position.east;
		finfo.gamma_r = nr*atan2(dy,dx);

		double diff = (finfo.gamma_r - finfo.mu_r);
		if (cgoal->wrapping_enable) diff = nr*labust::math::wrapRad(diff/nr);
		path.xi_r = finfo.mu_r + sigma_fov*tanh(diff/kxi);
		if (cgoal->wrapping_enable) path.xi_r = nr*labust::math::wrapRad(path.xi_r/nr);
		double dmu_r = nr*diver_pos.orientation_rate.yaw;
		path.dxi_r = dmu_r + dmu_r*sigma_fov/(2*kxi)*(tanh(diff/kxi)*tanh(diff/kxi)-1);
	}
	else
	{
		finfo.gamma_r = finfo.mu_r;
		path.xi_r = finfo.mu_r;
		path.dxi_r = 0*diver_pos.orientation_rate.yaw*nr;
	}

	double dx = diver_pos.position.north - vehicle_pos.position.north;
	double dy = diver_pos.position.east - vehicle_pos.position.east;
	path.pi_tilda = (path.pi - path.xi_r);0/cosh(5*(sqrt(dx*dx+dy*dy) - nr));
	if (cgoal->wrapping_enable) path.pi_tilda = nr*labust::math::wrapRad(path.pi_tilda/nr);
}

void TrackDiver::setDesiredOrientation(const auv_msgs::NavSts& estimate)
{
	double dx = diver_pos.position.north - estimate.position.north;
	double dy = diver_pos.position.east - estimate.position.east;
	boost::mutex::scoped_lock lf(fs_mux);
	path.delta_r = atan2(dy,dx);
	path.k = 0;
	if (cgoal->streamline_orientation)
		path.k = 0.5*(tanh((sqrt(dx*dx+dy*dy) - sigma_m)/kpsi) + 1);
}

void TrackDiver::updateFS()
{
	static ros::Time last_update = ros::Time::now();

	double dT = (ros::Time::now() - last_update).toSec();
	last_update = ros::Time::now();

	boost::mutex::scoped_lock l(fs_mux);
	boost::mutex::scoped_lock lg(goal_mux);
	//Set path info from goal if available
	double voff = 0;
	double radius = 1/path.curvature;
	if (cgoal != 0)
	{
		voff = cgoal->vertical_offset;
		path.curvature = 1/cgoal->radius;
		radius = cgoal->radius;
	}
	lg.unlock();

	//Update frame position in path coordinates
	path.pi += dT*dpi_r;
	//Default guidance is in the current point
	path.pi_tilda = 0;
	path.dxi_r = 0;
	path.xi_r = path.pi;

	//Calcuate the feedforward
	Eigen::Quaternion<double> q;
	labust::tools::quaternionFromEulerZYX(
			diver_pos.orientation.roll - path.orientation.roll,
			diver_pos.orientation.pitch - path.orientation.pitch,
			diver_pos.orientation.yaw - path.orientation.yaw, q);
	Eigen::Vector3d speed;
	speed<<diver_pos.body_velocity.x,
			diver_pos.body_velocity.y,
			diver_pos.body_velocity.z;
	speed = q.toRotationMatrix()*speed;
	path.dr_p.x = speed(0);
	path.dr_p.y = speed(1);
	path.dr_p.z = speed(2);

	if (fabs(voff) < radius)
	{
		double nr = sqrt(radius*radius - voff*voff);
		path.position.x = diver_pos.position.north + nr*cos(path.pi/nr);
		path.position.y = diver_pos.position.east + nr*sin(path.pi/nr);
		path.position.z = diver_pos.position.depth + voff;
		path.orientation.yaw = path.pi/nr + M_PI/2;
	}
	else
	{
		path.position.x = diver_pos.position.north;
		path.position.y = diver_pos.position.east;
		path.position.z = diver_pos.position.depth + voff;
		//\todo What should be the orientation here ?
		path.orientation.yaw = 0;
	}
}

void TrackDiver::step()
{
	boost::mutex::scoped_lock l(fs_mux);

	//Publish transform
	geometry_msgs::TransformStamped tfs;
	tfs.transform.translation.x = path.position.x;
	tfs.transform.translation.y = path.position.y;
	tfs.transform.translation.z = path.position.z;
	labust::tools::quaternionFromEulerZYX(
			path.orientation.roll,
			path.orientation.pitch,
			path.orientation.yaw,
			tfs.transform.rotation);
	tfs.child_frame_id = tf_prefix + "sf_frame";
	tfs.header.frame_id = tf_prefix + "local";
	tfs.header.stamp = ros::Time::now();
	broadcaster.sendTransform(tfs);

	//Publish reference
	auv_msgs::FSPathInfo::Ptr piout(new auv_msgs::FSPathInfo(path));
	piout->header.stamp = tfs.header.stamp;
	piout->header.frame_id = "sf_frame";
	stateRef.publish(piout);
}

void TrackDiver::stop()
{
	ROS_DEBUG("TrackDiver: Stopping.");
	cgoal.reset();
	this->updateControllers();
	//Stop path progression
	dpi_r = 0;
}

void TrackDiver::updateControllers(bool on)
{
	if (!this->control_manager.isValid())
	{
		ros::NodeHandle nh;
		this->control_manager = nh.serviceClient<navcon_msgs::EnableControl>("VT_enable", true);
	}
	navcon_msgs::EnableControl req;
  req.request.enable = on;
	this->control_manager.call(req);
}

///Diver position update handling
void TrackDiver::onDiverState(const auv_msgs::NavSts::ConstPtr& diver_state)
{
	boost::mutex::scoped_lock l(state_mux);
	diver_pos = *diver_state;
	//Publish transform
	geometry_msgs::TransformStamped tfs;
	tfs.transform.translation.x = diver_pos.position.north;
	tfs.transform.translation.y = diver_pos.position.east;
	tfs.transform.translation.z = diver_pos.position.depth;
	labust::tools::quaternionFromEulerZYX(
			diver_pos.orientation.roll,
			diver_pos.orientation.pitch,
			diver_pos.orientation.yaw,
			tfs.transform.rotation);
	tfs.child_frame_id = tf_prefix + "diver_frame";
	tfs.header.frame_id = tf_prefix + "local";
	tfs.header.stamp = ros::Time::now();
	l.unlock();
	broadcaster.sendTransform(tfs);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"track_diver");
	labust::primitive::PrimitiveBase<
		labust::primitive::TrackDiver,
		auv_msgs::FSPathInfo> primitive;
	ros::spin();
	return 0;
}








