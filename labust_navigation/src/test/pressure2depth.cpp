/*
 * eigen_test.cpp
 *
 *  Created on: Oct 16, 2013
 *      Author: dnad
 */
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

double a(1.0),b(0.0);
bool do_zero(false);
ros::Publisher depthpub;

void onZeroing(const std_msgs::Bool::ConstPtr& zero)
{
  do_zero = zero->data;
}

void onPressure(const std_msgs::Float32::ConstPtr& pressure)
{
  if (do_zero)
  {
     b = -a*pressure->data;
     do_zero = false;
     ROS_INFO("Pressure offset set to %f.", b);
  }

  std_msgs::Float32 depth;
  depth.data = a*pressure->data + b;
  depthpub.publish(depth);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pressure2depth");

    ros::NodeHandle nh,ph("~");

    ph.param("scaling",a,a);
    ph.param("offset",b,b);

    ros::Subscriber pressure = nh.subscribe("pressure", 1, &onPressure);
    ros::Subscriber reset = nh.subscribe("depth_reset", 1, &onZeroing);
    depthpub = nh.advertise<std_msgs::Float32>("depth",1);

    ros::spin();
	return 0;
}





