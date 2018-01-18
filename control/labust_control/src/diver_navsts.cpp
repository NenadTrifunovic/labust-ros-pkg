#include <ros/ros.h>
#include <cmath>
#include <auv_msgs/NavigationStatus.h>
//#include <labust_msgs/NavStsReq.h>

ros::Publisher out_pub;

double orientation(0);

void onPoseReq(const auv_msgs::NavigationStatus::ConstPtr& req){

    auv_msgs::NavigationStatus::Ptr navsts(new auv_msgs::NavigationStatus());
    navsts->global_position = req->global_position;
    navsts->origin = req->origin;
    navsts->altitude = req->altitude;
    navsts->body_velocity = req->body_velocity;
    navsts->seafloor_velocity = req->seafloor_velocity;
    navsts->global_position = req->global_position;
    navsts->orientation = req->orientation;
    navsts->orientation.z = orientation;
    //navsts->orientation_rate = req->orientation_rate;
    navsts->header.stamp = req->header.stamp;
    navsts->header.frame_id = req->header.frame_id;
    out_pub.publish(navsts);
}

void onOrientation(const auv_msgs::NavigationStatus::ConstPtr& req)
{
	orientation = req->orientation.z;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "navstsreq_navsts");
    ros::NodeHandle n;

    //Subscribing Topics
    ros::Subscriber sub = n.subscribe("diver_position", 1, onPoseReq);
    ros::Subscriber nsub = n.subscribe("diver_orientation", 1, onOrientation);

    //Publishing
    out_pub = n.advertise<auv_msgs::NavigationStatus>("stateRef", 1);

    ros::spin();
}

