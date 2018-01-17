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
 *********************************************************************/
#include <labust/tools/conversions.hpp>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MagneticModel.hpp>

#include <boost/date_time.hpp>
#include <boost/thread.hpp>

struct TFPublishNode
{
  TFPublishNode()
    : originLat(0)
    , originLon(0)
    , originH(0)
    , proj(originLat, originLon, originH)
    , fixValidated(false)
    , fixCount(0)
    , use_mag(false)
    , tf_prefix("")
    , tfListener(tfBuffer)

  {
    ros::NodeHandle nh, ph("~");
    nh.param("LocalOriginLat", originLat, originLat);
    nh.param("LocalOriginLon", originLon, originLon);
    //TODO Check if this is correct.
		nh.param("LocalOriginH", originH, originH);		
    ph.param("LocalFixSim", fixValidated, fixValidated);

    // Get magnetic data
    std::string magnetic_model("wmm2015");
    std::string magnetic_path("/usr/share/geographiclib/magnetic");
    ph.param("magnetic_data_path", magnetic_path, magnetic_path);
    ph.param("magnetic_model", magnetic_model, magnetic_model);

    std::string key;
    if (nh.searchParam("tf_prefix", key))
      nh.getParam(key, tf_prefix);

    try
    {
      mag.reset(
          new GeographicLib::MagneticModel(magnetic_model, magnetic_path));
      use_mag = (mag != 0);
    }
    catch (std::exception& e)
    {
      ROS_ERROR("%s", e.what());
    }

    // Setup the local projection
    if (fixValidated)
      proj.Reset(originLat, originLon, originH);

    gps_raw =
        nh.subscribe<sensor_msgs::NavSatFix>("gps", 1, &TFPublishNode::onGps, this);
    gps_ned = nh.advertise<geometry_msgs::Vector3Stamped>("gps_raw", 1);
    mag_dec = nh.advertise<std_msgs::Float64>("magnetic_declination", 1, true);

    runner = boost::thread(boost::bind(&TFPublishNode::publishFrame, this));
  }

  ~TFPublishNode()
  {
    runner.join();
  }

  void onGps(const sensor_msgs::NavSatFix::ConstPtr& fix)
  {
    // In case we didn't have a fix on launch, but now we got 10 fixes in 15
    // sec.
    if (!fixValidated)
    {
      originLat = fix->latitude;
      originLon = fix->longitude;
      proj.Reset(originLat, originLon, originH);
      fixValidated = true;
    }
    else
    {
      // Publish projection
      geometry_msgs::Vector3Stamped gpsout;
      gpsout.header.frame_id = "map_ned";
      gpsout.header.stamp = fix->header.stamp;
      Eigen::Quaternion<double> q;
      labust::tools::quaternionFromEulerZYX(M_PI, 0, M_PI / 2, q);
      double x, y, z;
      proj.Forward(fix->latitude, fix->longitude, fix->altitude, x, y, z);
      Eigen::Vector3d enu;
      enu << x, y, z;
      Eigen::Vector3d ned = q.toRotationMatrix() * enu;
      labust::tools::vectorToPoint(ned, gpsout.vector);
      gps_ned.publish(gpsout);

      // Magnetic declination
      if (use_mag)
      {
        double Bx, By, Bz;
        using namespace boost::posix_time;
        double days = second_clock::local_time().date().day_of_year();
        double year = second_clock::local_time().date().year() + days / 356.25;
        (*mag)(year, fix->latitude, fix->longitude, fix->altitude, Bx, By, Bz);
        double H, F, D, I;
        GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);
        std_msgs::Float64 dec;
        dec.data = M_PI * D / 180;
        mag_dec.publish(dec);
      }
      else
      {
        ROS_WARN("Magnetic model unavailable.");
      }
    }
  };
  
  void publishFrame()
  {
    ros::Rate rate(10);
    while (ros::ok())
    {
      ros::Time ct = ros::Time::now();
      geometry_msgs::TransformStamped transform;
      if (fixValidated)
      {
        // Setup the geocentric <-> world frame
        geometry_msgs::TransformStamped transform;
        GeographicLib::Geocentric::WGS84().Forward(
            originLat, originLon, originH, transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);

        labust::tools::quaternionFromEulerZYX(
            M_PI / 2 - M_PI * originLat / 180, 0,
            M_PI / 2 + M_PI * originLon / 180, transform.transform.rotation);
        transform.child_frame_id = tf_prefix + "map";
        transform.header.frame_id = "earth";
        transform.header.stamp = ct;
        broadcaster.sendTransform(transform);
      }
          
      try
      {
        transform = tfBuffer.lookupTransform(tf_prefix + "map_ned", tf_prefix + "base_link",
                                 ros::Time(0), ros::Duration(0));                                 
         
        geometry_msgs::Quaternion rot(transform.transform.rotation);                                                        

        // Broadcast map_ned->base_pose_ned frame.                           
        labust::tools::quaternionFromEulerZYX(0, 0, 0,
                                              transform.transform.rotation);
        transform.child_frame_id = tf_prefix + "base_pose_ned";
        transform.header.frame_id = tf_prefix + "map_ned";
        transform.header.stamp = ct;
        broadcaster.sendTransform(transform);      

        // Boradcast base_pose_ned -> base_link_frd frame.
        transform.transform.translation.x = 0;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 0;
        
        double roll, pitch, yaw;
        labust::tools::eulerZYXFromQuaternion(rot, roll, pitch, yaw);
        roll += M_PI;
        //yaw -= M_PI/2; // East equals 0. No operation needed. Due to ROS convention.
        labust::tools::quaternionFromEulerZYX(roll, pitch, yaw, rot);        
             
        transform.transform.rotation = rot;
        transform.child_frame_id = tf_prefix + "base_link_frd";
        transform.header.frame_id = tf_prefix + "base_pose_ned";
        transform.header.stamp = ct;
        broadcaster.sendTransform(transform);                                   
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
      } 
      
      rate.sleep();
    }
  }

private:
  ros::Subscriber gps_raw;
  ros::Publisher gps_ned, mag_dec;
  tf2_ros::TransformBroadcaster broadcaster;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;    
  double originLat, originLon, originH;
  bool fixValidated;
  int fixCount;
  boost::thread runner;
  // The ENU frame
  GeographicLib::LocalCartesian proj;
  // The magnetic model
  boost::shared_ptr<GeographicLib::MagneticModel> mag;
  bool use_mag;
  std::string tf_prefix;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tf_publish_node");
  ros::NodeHandle nh;
  TFPublishNode tf_publish_node;
  ros::spin();
  return 0;
}
