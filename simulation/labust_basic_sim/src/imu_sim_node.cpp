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
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <labust/simulation/NoiseModel.hpp>
#include <labust/tools/conversions.hpp>

#include <boost/thread/mutex.hpp>

using labust::simulation::NoiseGenerators;

struct ImuSim
{
  ImuSim()
    : last_time(ros::Time::now())
    , last_omega(Eigen::Matrix3d::Zero())
    , magdec(0)
    , tf_prefix("")
    , sigma_rpy(0)
    , sigma_depth(0)
    , tfListener(tfBuffer)
  {
    ros::NodeHandle nh, ph("~");

    std::vector<double> offset(3, 0), orot(3, 0);
    ph.param("offset", offset, offset);
    ph.param("orot", orot, orot);
    ph.param("sigma_rpy", sigma_rpy, sigma_rpy);
    ph.param("sigma_depth", sigma_depth, sigma_depth);
    for (int i = 0; i < 2; ++i)
    {
      gen_rpy.addNew(0, sigma_rpy / 3);
    }
    gen_rpy.addNew(0, sigma_rpy);
    gen_depth.addNew(0, sigma_depth);
    this->offset << offset[0], offset[1], offset[2];
    labust::tools::quaternionFromEulerZYX(M_PI * orot[0] / 180,
                                          M_PI * orot[1] / 180,
                                          M_PI * orot[2] / 180, this->orot);

    std::string key;
    if (nh.searchParam("tf_prefix", key))
      nh.getParam(key, tf_prefix);

    odom = nh.subscribe<nav_msgs::Odometry>("meas_odom", 1, &ImuSim::onOdom,
                                            this);
    acc = nh.subscribe<geometry_msgs::Vector3>("nuacc_ideal", 1,
                                               &ImuSim::onAcc, this);
    mag_dec = nh.subscribe<std_msgs::Float64>(
        "magnetic_declination", 1, &ImuSim::onMagneticDeclination, this);

    imu_flu_pub = nh.advertise<sensor_msgs::Imu>("imu/imu", 1);
    imu_frd_pub = nh.advertise<sensor_msgs::Imu>("imu", 1);
    depth_pub = nh.advertise<std_msgs::Float32>("depth", 1);
    depth_odom_pub = nh.advertise<nav_msgs::Odometry>("odometry/depth", 1);
  }

  void onMagneticDeclination(const typename std_msgs::Float64::ConstPtr& msg)
  {
    magdec = msg->data;
  }

  void onOdom(const typename nav_msgs::Odometry::ConstPtr& msg)
  {
    double dT = (ros::Time::now() - last_time).toSec();
    sensor_msgs::Imu::Ptr imu(new sensor_msgs::Imu());
    imu->header.stamp = ros::Time::now();
    imu->header.frame_id = tf_prefix + "imu_frame_frd";
    // Calculate angular velocities
    Eigen::Vector3d nua;
    labust::tools::pointToVector(msg->twist.twist.angular, nua);
    Eigen::Vector3d cnua = orot.toRotationMatrix().transpose() * nua;
    labust::tools::vectorToPoint(cnua, imu->angular_velocity);
    // Calculate orientation
    Eigen::Quaternion<double> meas(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    double roll, pitch, yaw;
    labust::tools::eulerZYXFromQuaternion(meas, roll, pitch, yaw);
    roll += gen_rpy(0);
    pitch += gen_rpy(1);
    yaw += gen_rpy(2);
    yaw -= magdec;
    labust::tools::quaternionFromEulerZYX(roll, pitch, yaw, meas);
    Eigen::Quaternion<double> sim_meas = meas * orot.inverse();
    imu->orientation.x = sim_meas.x();
    imu->orientation.y = sim_meas.y();
    imu->orientation.z = sim_meas.z();
    imu->orientation.w = sim_meas.w();

    {
      boost::mutex::scoped_lock l(acc_mux);
      Eigen::Matrix3d omegac;
      enum
      {
        p = 0,
        q,
        r
      };
      omegac << 0, -nua(r), nua(q), nua(r), 0, -nua(p), -nua(q), nua(p), 0;
      // Calculate derivative
      Eigen::Matrix3d omegad = (omegac - last_omega) / dT;
      last_omega = omegac;
      Eigen::Vector3d accr;
      accr << nuacc.x, nuacc.y, nuacc.z;
      Eigen::Vector3d accm =
          orot.toRotationMatrix().transpose() * (accr + omegad * offset);
      //labust::tools::vectorToPoint(-1*accm, imu->linear_acceleration); // -1* due to ROS convention.
      labust::tools::vectorToPoint(1*accm, imu->linear_acceleration); 
      
      /*ROS_ERROR("Offset: %f %f %f",offset(0),offset(1),offset(2));
      std::stringstream os;
      os<<omegad;
      ROS_ERROR("Omegad: %s",os.str().c_str());
      os.str("");
      os<<omegac;
      ROS_ERROR("Omegac: %s",os.str().c_str());*/
    }



    imu_frd_pub.publish(imu);
    //sensor_msgs::Imu::Ptr imu_flu(new sensor_msgs::Imu());
    //convertFRDtoFLD(imu, imu_flu);
    //imu_flu->header.frame_id = tf_prefix + "imu_frame"; 
    //imu_flu_pub.publish(imu_flu);
    
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform(tf_prefix + "imu_frame", tf_prefix + "imu_frame_frd",
                               ros::Time(0), ros::Duration(0.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    } 
    
    imu->linear_acceleration.z = -imu->linear_acceleration.z;
    
    sensor_msgs::Imu imu_flu;
    labust::tools::doTransform(*imu, imu_flu, transformStamped);
    
    Eigen::Quaternion<double> meas2(
        imu_flu.orientation.w, imu_flu.orientation.x,
        imu_flu.orientation.y, imu_flu.orientation.z);
    //double roll, pitch, yaw;    
    labust::tools::eulerZYXFromQuaternion(meas2, roll, pitch, yaw);
    yaw += M_PI/2 - magdec; // East equals 0. Due to ROS convention.
    roll += M_PI; // Sensor is simulated in NED frame.
    labust::tools::quaternionFromEulerZYX(roll, pitch, yaw, meas2);
    imu_flu.orientation.x = meas2.x();
    imu_flu.orientation.y = meas2.y();
    imu_flu.orientation.z = meas2.z();
    imu_flu.orientation.w = meas2.w();    
        
    imu_flu.orientation_covariance[0] = 0.01;
    imu_flu.orientation_covariance[4] = 0.01;
    imu_flu.orientation_covariance[8] = 0.01;
    
    imu_flu.angular_velocity_covariance[0] = 0.01;
    imu_flu.angular_velocity_covariance[4] = 0.01;
    imu_flu.angular_velocity_covariance[8] = 0.01;
    
    imu_flu.linear_acceleration_covariance[0] = 1.0;
    imu_flu.linear_acceleration_covariance[4] = 1.0;
    imu_flu.linear_acceleration_covariance[8] = 1.0;
        
    imu_flu.header = imu->header;
    imu_flu.header.frame_id = tf_prefix + "imu_frame";
    imu_flu_pub.publish(imu_flu);
       

    std_msgs::Float32::Ptr depth(new std_msgs::Float32());
    depth->data = msg->pose.pose.position.z + gen_depth(0);
    depth_pub.publish(depth);
    
    nav_msgs::Odometry depth_odom;
    depth_odom.header.stamp = ros::Time::now();
    depth_odom.header.frame_id = tf_prefix + "map_ned";
    depth_odom.pose.pose.position.z = depth->data;
    depth_odom.pose.covariance[14] = 0.02;
    depth_odom_pub.publish(depth_odom);
    
    
  }
  
  void onAcc(const typename geometry_msgs::Vector3::ConstPtr& msg)
  {
    boost::mutex::scoped_lock l(acc_mux);
    nuacc = *msg;
  }

private:
  ros::Subscriber odom, acc, mag_dec;
  ros::Publisher imu_flu_pub, imu_frd_pub, depth_pub, depth_odom_pub;
  Eigen::Vector3d offset;
  Eigen::Quaternion<double> orot;
  Eigen::Matrix3d last_omega;
  boost::mutex acc_mux;
  ros::Time last_time;
  geometry_msgs::Vector3 nuacc;
  tf2_ros::TransformBroadcaster broadcaster;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;  
  double magdec;
  NoiseGenerators gen_rpy, gen_depth;
  double sigma_rpy, sigma_depth;
  std::string tf_prefix;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "imu_sim");
  ros::NodeHandle nh;
  ImuSim imu;
  ros::spin();
  return 0;
}
