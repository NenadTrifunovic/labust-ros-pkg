#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <labust/tools/GeoUtilities.hpp>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

tf2_ros::Buffer buffer;
tf2_ros::TransformListener listener(buffer);

ros::Publisher out_pub;
geometry_msgs::Vector3Stamped msg_out;

void GPSCallback(const sensor_msgs::NavSatFix& msg)
try
{
    msg_out.header.stamp = msg.header.stamp;
    geometry_msgs::TransformStamped transformLocal, transformGPS, transformDeg;
    transformLocal = buffer.lookupTransform("local", "gps_frame", ros::Time(0));
    transformGPS = buffer.lookupTransform("base_link", "gps_frame", ros::Time(0));
    transformDeg = buffer.lookupTransform("worldLatLon", "local", ros::Time(0));
    std::pair<double, double> posxy = labust::tools::deg2meter(msg.latitude - transformDeg.transform.translation.y,
		msg.longitude - transformDeg.transform.translation.x,
		transformDeg.transform.translation.y);
  	Eigen::Quaternion<double> rot(transformLocal.transform.rotation.w,
  				transformLocal.transform.rotation.x,
  				transformLocal.transform.rotation.y,
  				transformLocal.transform.rotation.z);
    Eigen::Vector3d offset(transformGPS.transform.translation.x,
 			transformGPS.transform.translation.y,
			transformGPS.transform.translation.z);
    Eigen::Vector3d pos_corr = rot.matrix()*offset;

    posxy.first -= pos_corr(0);
    posxy.second -= pos_corr(1);
    
    msg_out.vector.x = posxy.first;
    msg_out.vector.y = posxy.second;		
    out_pub.publish(msg_out);
}
catch(tf2::TransformException& ex)
{
    ROS_WARN("Unable to decode GPS measurement. Missing frame : %s",ex.what());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "navstsfix_vector3");
    ros::NodeHandle n, nh("~");

    std::string gps_topic, out_topic;
    //Parameters
    if(!nh.getParamCached("gps_topic",gps_topic)) { ROS_ERROR("No Parameter: gps_topic"); ros::shutdown(); exit(-1);}
    if(!nh.getParamCached("out_topic",out_topic)) { ROS_ERROR("No Parameter: out_topic"); ros::shutdown(); exit(-1);}

    //Subscribing Topics
    ros::Subscriber sub1 = n.subscribe(gps_topic, 10, GPSCallback);

    //Publishing
    out_pub = n.advertise<geometry_msgs::Vector3Stamped>(out_topic, 10);

    ros::spin();
}

