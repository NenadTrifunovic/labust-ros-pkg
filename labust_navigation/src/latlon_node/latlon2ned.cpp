/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, LABUST, UNIZG-FER
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

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <auv_msgs/NavSts.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MagneticModel.hpp>

#include <boost/date_time.hpp>
#include <boost/thread.hpp>

struct LatLon2NED
{
  LatLon2NED()
    : originLat(0)
    , originLon(0)
    , originH(0)
    , proj(originLat, originLon, originH)
    , use_mag(false)
  {
    ros::NodeHandle nh, ph("~");
    ph.param("LocalOriginLat", originLat, originLat);
    ph.param("LocalOriginLon", originLon, originLon);

    // Get magnetic data
    std::string magnetic_model("wmm2015");
    std::string magnetic_path("/usr/share/geographiclib/magnetic");
    ph.param("magnetic_data_path", magnetic_path, magnetic_path);
    ph.param("magnetic_model", magnetic_model, magnetic_model);

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
    proj.Reset(originLat, originLon, originH);


    sub_pos_latlon = nh.subscribe<auv_msgs::NavSts>("position_in", 1, &LatLon2NED::onPosIn, this);
    pub_pos_ned =  nh.advertise<auv_msgs::NavSts>("position_out", 1);
  }

  ~LatLon2NED()
  {
  }

  void onPosIn(const auv_msgs::NavSts::ConstPtr& pos_in)
  {

      // Publish projection
      auv_msgs::NavSts pos_out;
      pos_out.header.frame_id = "local";
      pos_out.header.stamp = pos_in->header.stamp;
      pos_out.origin.latitude = originLat;
      pos_out.origin.longitude = originLon;
      pos_out.origin.altitude = originH;

      pos_out.global_position.latitude = pos_in->global_position.latitude;
      pos_out.global_position.longitude = pos_in->global_position.longitude;
      pos_out.global_position.altitude = pos_in->global_position.altitude;

      pos_out.orientation = pos_in->orientation;

      Eigen::Quaternion<double> q;
		labust::tools::quaternionFromEulerZYX(M_PI, 0, M_PI / 2, q);
		double x, y, z;
		proj.Forward(pos_in->global_position.latitude, pos_in->global_position.longitude, pos_in->global_position.altitude, x, y, z);
		Eigen::Vector3d enu;
		enu << x, y, z;
		Eigen::Vector3d ned = q.toRotationMatrix() * enu;
		labust::tools::vectorToNED(ned, pos_out.position);
		pub_pos_ned.publish(pos_out);

      // Magnetic declination
      if (use_mag)
      {
//        double Bx, By, Bz;
//        using namespace boost::posix_time;
//        double days = second_clock::local_time().date().day_of_year();
//        double year = second_clock::local_time().date().year() + days / 356.25;
//        (*mag)(year, fix->latitude, fix->longitude, fix->altitude, Bx, By, Bz);
//        double H, F, D, I;
//        GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);
//        std_msgs::Float64 dec;
//        dec.data = M_PI * D / 180;
//        mag_dec.publish(dec);
      }
      else
      {
        ROS_WARN("Magnetic model unavailable.");
      }
    }




private:
  ros::Subscriber sub_pos_latlon;
  ros::Publisher pub_pos_ned;
  double originLat, originLon, originH;
  // The ENU frame
  GeographicLib::LocalCartesian proj;
  // The magnetic model
  boost::shared_ptr<GeographicLib::MagneticModel> mag;
  bool use_mag;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "latlon2ned");
  ros::NodeHandle nh;
  LatLon2NED latlon2ned;
  ros::spin();
  return 0;
}
