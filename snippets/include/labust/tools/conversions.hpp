/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, LABUST, UNIZG-FER
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
 *  Created: 20.05.2013.
 *  Author: Đula Nađ
 *********************************************************************/
#ifndef CONVERSIONS_HPP_
#define CONVERSIONS_HPP_
#include <Eigen/Dense>

#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace labust
{
	namespace tools
	{
		/**
		 * The class offers mapping from a XYZ structure to a vector.
		 */
		template <class Point, class Iterator>
		void pointToVector(const Point& point, Iterator& vec, int offset = 0)
		{
			vec[offset+0] = point.x;
			vec[offset+1] = point.y;
			vec[offset+2] = point.z;
		}

		/**
		 * The class offers mapping from a XYZ structure to a vector.
		 */
		template <class Point, class Iterator>
		void vectorToPoint(const Iterator& vec, Point& point, int offset = 0)
		{
			point.x = vec[offset+0];
			point.y = vec[offset+1];
			point.z = vec[offset+2];
		}

		/**
		 * The function offers mapping from NED structure to a vector.
		 */
		template <class Point, class Iterator>
		void nedToVector(const Point& point, Iterator& vec, int offset = 0)
		{
			vec[offset+0] = point.north;
			vec[offset+1] = point.east;
			vec[offset+2] = point.depth;
		}

		/**
		 * The class offers mapping to a RPY structure from a vector.
		 */
		template <class Point, class Iterator>
		void vectorToNED(const Iterator& vec, Point& point, int offset = 0)
		{
			point.north = vec[offset+0];
			point.east = vec[offset+1];
			point.depth = vec[offset+2];
		}

		/**
		 * The function offers mapping from RPY structure to a vector.
		 */
		template <class Point, class Iterator>
		void rpyToVector(const Point& point, Iterator& vec, int offset = 0)
		{
			vec[offset+0] = point.roll;
			vec[offset+1] = point.pitch;
			vec[offset+2] = point.yaw;
		}

		/**
		 * The class offers mapping to a NED structure from a vector.
		 */
		template <class Point, class Iterator>
		void vectorToRPY(const Iterator& vec, Point& point, int offset = 0)
		{
			point.roll = vec[offset+0];
			point.pitch = vec[offset+1];
			point.yaw = vec[offset+2];
		}

		/**
		 * The class offers mapping from auv_msgs disable_axis structure to a vector.
		 */
		template <class Point, class Iterator>
		void vectorToDisableAxis(const Iterator& vec, Point& point)
		{
			point.x = vec[0];
			point.y = vec[1];
			point.z = vec[2];
			point.roll = vec[3];
			point.pitch = vec[4];
			point.yaw = vec[5];
		}

		template <class Point, class Iterator>
		void disableAxisToVector(Point& point, const Iterator& vec)
		{
			vec[0] = point.x;
			vec[1] = point.y;
			vec[2] = point.z;
			vec[3] = point.roll;
			vec[4] = point.pitch;
			vec[5] = point.yaw;
		}

		template <class T>
		void quaternionFromEulerZYX(double roll, double pitch, double yaw, Eigen::Quaternion<T>& q)
		{
			using namespace Eigen;
			Matrix<T,3,3> Cx,Cy,Cz,m;
			typedef Matrix<T,3,1> Vector3;
			Cx = AngleAxis<T>(roll, Vector3::UnitX());
			Cy = AngleAxis<T>(pitch, Vector3::UnitY());
			Cz = AngleAxis<T>(yaw, Vector3::UnitZ());
			//ZYX convention
			m = (Cz*Cy*Cx); //From child to parent
			//m = (Cz*Cy*Cx).transpose(); //From parent to child
			q = Quaternion<T>(m);
		}

		template <class T>
		void quaternionFromEulerZYX(double roll, double pitch, double yaw, T& q)
		{
			Eigen::Quaternion<double> t;
			quaternionFromEulerZYX(roll, pitch, yaw, t);
			q.x = t.x();
			q.y = t.y();
			q.z = t.z();
			q.w = t.w();
		}

		//\todo Test and document this method
		template <class T>
		void eulerZYXFromQuaternion(const T& q, double& roll, double& pitch, double& yaw)
		{
			using namespace Eigen;
			//From child to parent
			roll = atan2(2*(q.y()*q.z() + q.x()*q.w()),1-2*(q.x()*q.x() + q.y()*q.y()));
			pitch = -asin(2*(q.x()*q.z()-q.y()*q.w()));
			yaw = atan2(2*(q.y()*q.x()+q.w()*q.z()),1-2*(q.y()*q.y()+q.z()*q.z()));
			//From parent to child
			//roll = atan2(2*(q.y()*q.z() - q.x()*q.w()),1-2*(q.x()*q.x() + q.y()*q.y()));
			//pitch = -asin(2*(q.x()*q.z() + q.y()*q.w()));
			//yaw = atan2(2*(q.x()*q.y()-q.w()*q.z()),1-2*(q.y()*q.y()+q.z()*q.z()));
		}

		//\todo Test and document this method
		template <class T, class V>
		void eulerZYXFromQuaternion(const T& q, V& vect)
		{
			using namespace Eigen;
			//From child to parent
			enum {roll=0, pitch, yaw};
			vect(roll) = atan2(2*(q.y()*q.z() + q.x()*q.w()),1-2*(q.x()*q.x() + q.y()*q.y()));
			vect(pitch) = -asin(2*(q.x()*q.z()-q.y()*q.w()));
			vect(yaw) = atan2(2*(q.y()*q.x()+q.w()*q.z()),1-2*(q.y()*q.y()+q.z()*q.z()));
			//From parent to child
			//roll = atan2(2*(q.y()*q.z() - q.x()*q.w()),1-2*(q.x()*q.x() + q.y()*q.y()));
			//pitch = -asin(2*(q.x()*q.z() + q.y()*q.w()));
			//yaw = atan2(2*(q.x()*q.y()-q.w()*q.z()),1-2*(q.y()*q.y()+q.z()*q.z()));
		}

		//\todo Test and document this method
		inline void eulerZYXFromQuaternion(const geometry_msgs::Quaternion& q,
				double& roll, double& pitch, double& yaw)
		{
			using namespace Eigen;
			//From child to parent
			roll = atan2(2*(q.y*q.z + q.x*q.w),1-2*(q.x*q.x + q.y*q.y));
			pitch = -asin(2*(q.x*q.z-q.y*q.w));
			yaw = atan2(2*(q.y*q.x+q.w*q.z),1-2*(q.y*q.y+q.z*q.z));
			//From parent to child
			//roll = atan2(2*(q.y*q.z - q.x*q.w),1-2*(q.x*q.x + q.y*q.y));
			//pitch = -asin(2*(q.x*q.z + q.y*q.w));
			//yaw = atan2(2*(q.x*q.y-q.w*q.z),1-2*(q.y*q.y+q.z*q.z));
		}
		
    /// Apply a geometry_msgs TransformStamped to a sensor_msgs IMU type.		 		 		
	  void doTransform(const sensor_msgs::Imu &in, sensor_msgs::Imu &out, const geometry_msgs::TransformStamped &transform)		
	  {			
      // Transform orientation.			
			geometry_msgs::QuaternionStamped qs_in, qs_out;
			qs_in.header = in.header;
			qs_in.quaternion = in.orientation;
			tf2::doTransform(qs_in, qs_out, transform);    
			
      // Transform angular velocity.
			geometry_msgs::Vector3Stamped av_in, av_out;
			av_in.header = in.header;
			av_in.vector = in.angular_velocity;
			tf2::doTransform(av_in, av_out, transform);  			
	
      // Transform linear acceleration.
			geometry_msgs::Vector3Stamped la_in, la_out;
			la_in.header = in.header;
			la_in.vector = in.linear_acceleration;
			tf2::doTransform(la_in, la_out, transform);
			
			// Fill the output.
			out.orientation = qs_out.quaternion;
			out.angular_velocity = av_out.vector; 
			out.linear_acceleration = la_out.vector;   					  
	  }
		
    /// Apply a geometry_msgs TransformStamped to a geometry_msgs PoseWithCovarianceStamped type.		 		 				
		void doTransform(const geometry_msgs::PoseWithCovarianceStamped &in, geometry_msgs::PoseWithCovarianceStamped &out, const geometry_msgs::TransformStamped &transform)
		{
			// Transform pose.
			geometry_msgs::PoseStamped p_in, p_out;
			p_in.header = in.header;
			p_in.pose = in.pose.pose;
			tf2::doTransform(p_in, p_out, transform);			

			// Fill the output.
			out.pose.pose = p_out.pose;					
		}	
	
    /// Apply a geometry_msgs TransformStamped to a geometry_msgs TwistWithCovarianceStamped type.		 		 				
		void doTransform(const geometry_msgs::TwistWithCovarianceStamped &in, geometry_msgs::TwistWithCovarianceStamped &out, const geometry_msgs::TransformStamped &transform)
		{
			// Transform linear velocity.
			geometry_msgs::Vector3Stamped l_in, l_out;
			l_in.header = in.header;
			l_in.vector = in.twist.twist.linear;
			tf2::doTransform(l_in, l_out, transform);			
			
			// Transform angular velocity.
			geometry_msgs::Vector3Stamped a_in, a_out;
			a_in.header = in.header;
			a_in.vector = in.twist.twist.angular;
			tf2::doTransform(a_in, a_out, transform);			

			// Fill the output.
			out.twist.twist.linear = l_out.vector;		
			out.twist.twist.angular = a_out.vector;										
		}
		
    /// Apply a geometry_msgs TransformStamped to a nav_msgs Odometry type.		 		 				
		void doTransform(const nav_msgs::Odometry &in, nav_msgs::Odometry &out, const geometry_msgs::TransformStamped &transform)
		{
			// Transform pose.
			geometry_msgs::PoseStamped p_in, p_out;
			p_in.header = in.header;
			p_in.pose = in.pose.pose;
			tf2::doTransform(p_in, p_out, transform);			
			
			// Transform twist.
			// Transform linear velocity.
			geometry_msgs::Vector3Stamped l_in, l_out;
			l_in.header = in.header;
			l_in.vector = in.twist.twist.linear;
			tf2::doTransform(l_in, l_out, transform);	
					
			// Transform angular velocity.						
			geometry_msgs::Vector3Stamped a_in, a_out;
			a_in.header = in.header;
			a_in.vector = in.twist.twist.angular;
			tf2::doTransform(a_in, a_out, transform);			

			// Fill the output.
			out.pose.pose = p_out.pose;
			out.twist.twist.linear = l_out.vector;		
			out.twist.twist.angular = a_out.vector;										
		}						
	}
}

/* CONVERSIONS_HPP_ */
#endif
