#include <ros/ros.h>
#include <cmath>
#include <auv_msgs/NavigationStatus.h>
#include <labust_msgs/NavStsReq.h>

ros::Publisher out_pub;

void onPoseReq(const labust_msgs::NavStsReq::ConstPtr& req){

    auv_msgs::NavigationStatus::Ptr navsts(new auv_msgs::NavigationStatus());
    navsts->global_position = req->global_position;
    navsts->origin = req->origin;
    navsts->altitude = req->altitude;
    navsts->position = req->position;
    navsts->body_velocity = req->body_velocity;
    navsts->seafloor_velocity = req->gbody_velocity;
    navsts->global_position = req->global_position;
    navsts->orientation.x = req->orientation.roll;
    navsts->orientation.y = req->orientation.pitch;
    navsts->orientation.z = req->orientation.yaw; 
    navsts->orientation_rate.x = req->orientation_rate.roll;
    navsts->orientation_rate.y = req->orientation_rate.pitch;
    navsts->orientation_rate.z = req->orientation_rate.yaw;    
    navsts->header.stamp = req->header.stamp;
    navsts->header.frame_id = req->header.frame_id;
    out_pub.publish(navsts);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "navstsreq_navsts");
    ros::NodeHandle n;

    //Subscribing Topics
    ros::Subscriber sub = n.subscribe("pose_req", 3, onPoseReq);

    //Publishing
    out_pub = n.advertise<auv_msgs::NavigationStatus>("stateRef", 1);

    ros::spin();
}

