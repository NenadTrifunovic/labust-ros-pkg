#include <ros/ros.h>
#include <cmath>
#include <auv_msgs/NavSts.h>
#include <auv_msgs/NavStsReq.h>

ros::Publisher out_pub;

double orientation(0);

void onPoseReq(const auv_msgs::NavSts::ConstPtr& req){

    auv_msgs::NavSts::Ptr navsts(new auv_msgs::NavSts());
    navsts->global_position = req->global_position;
    navsts->origin = req->origin;
    navsts->altitude = req->altitude;
    navsts->body_velocity = req->body_velocity;
    navsts->gbody_velocity = req->gbody_velocity;
    navsts->global_position = req->global_position;
    navsts->orientation = req->orientation;
    navsts->orientation.yaw = orientation;
    //navsts->orientation_rate = req->orientation_rate;
    navsts->header.stamp = req->header.stamp;
    navsts->header.frame_id = req->header.frame_id;
    out_pub.publish(navsts);
}

void onOrientation(const auv_msgs::NavSts::ConstPtr& req)
{
	orientation = req->orientation.yaw;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "navstsreq_navsts");
    ros::NodeHandle n;

    //Subscribing Topics
    ros::Subscriber sub = n.subscribe("diver_position", 1, onPoseReq);
    ros::Subscriber nsub = n.subscribe("diver_orientation", 1, onOrientation);

    //Publishing
    out_pub = n.advertise<auv_msgs::NavSts>("stateRef", 1);

    ros::spin();
}

