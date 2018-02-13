#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <labust/tools/conversions.hpp>

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"tf_test_comp");
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener listener(buffer);
	tf2_ros::TransformBroadcaster broadcast;
	ros::Rate rate(10);
	ros::NodeHandle nh;

	double yaw(0);

	while (ros::ok())
	{
		rate.sleep();
		ros::spinOnce();

		try
		{
			geometry_msgs::TransformStamped out;
			out.header.frame_id = "local";
			out.child_frame_id = "base_link";
			out.transform.translation.x = 10;
			out.transform.translation.y = -17;
			out.transform.translation.z = 0;
			labust::tools::quaternionFromEulerZYX(0,0,yaw, out.transform.rotation);
			yaw += 0.3;
			out.header.stamp =  ros::Time::now();
			broadcast.sendTransform(out);

			geometry_msgs::TransformStamped transformLocal, transformGPS;
			ros::Time past = ros::Time::now() - ros::Duration(5.0);
			transformLocal = buffer.lookupTransform("local", "gps_frame", past);
			transformGPS = buffer.lookupTransform("base_link", "gps_frame", ros::Time(0));

			std::cout<<"GPS position: "<<transformLocal.transform.translation.x<<", "<<transformLocal.transform.translation.y<<std::endl;
			std::cout<<"GPS offset: "<<transformGPS.transform.translation.x<<","<<transformGPS.transform.translation.y<<","<<transformGPS.transform.translation.z<<std::endl;

			Eigen::Quaternion<double> rot(transformLocal.transform.rotation.w,
					transformLocal.transform.rotation.x,
					transformLocal.transform.rotation.y,
					transformLocal.transform.rotation.z);

			Eigen::Vector3d offset(transformGPS.transform.translation.x,
					transformGPS.transform.translation.y,
					transformGPS.transform.translation.z);
			Eigen::Vector3d pos_corr = rot.matrix()*offset;


			std::cout<<"correction:"<<pos_corr(0)<<","<<pos_corr(1)<<std::endl;
			std::cout<<"Base_link position:"<<(transformLocal.transform.translation.x - pos_corr(0))<<","<<(transformLocal.transform.translation.y - pos_corr(1))<<std::endl;
		}
		catch(tf2::TransformException& ex)
		{
			ROS_WARN("Unable to decode GPS measurement. Missing frame : %s",ex.what());
		}
	}

	return 0;
}



