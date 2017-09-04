/*********************************************************************
 * tdoass_node.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: Filip Mandic
 *
 ********************************************************************/

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
*********************************************************************/

#include <labust/control/tdoass/tdoass_node.h>

using namespace labust::control::tdoass;

TDOASSNode::TDOASSNode() : toa1_old(0)
                         , toa2_old(0)
                         , toa1(0)
                         , toa2(0)
                         , tdoa(0)
                         , speed_of_sound(1500.0)
                         , veh_type(SLAVE)
                         , baseline(5.0)
                         , m(1.0)
                         , epsilon(1.0)
                         , es_controller(1,0.1)
                         , eta_filter_state_k0(2,0)
                         , eta_filter_state_k1(2,0)                         
                         , ts(0.1)
                         , w1(1)
                         , w2(1)
                         , k1(1)
{
  init();
}

TDOASSNode::~TDOASSNode()
{
}

void TDOASSNode::init()
{
  ros::NodeHandle nh, ph("~");  
  sub_veh1_state = nh.subscribe("veh1/state", 1, &TDOASSNode::onVeh1State, this);
  sub_veh2_state = nh.subscribe("veh2/state", 1, &TDOASSNode::onVeh2State, this);
  sub_veh1_toa = nh.subscribe("veh1/toa", 1, &TDOASSNode::onVeh1Toa, this);
  sub_veh2_toa = nh.subscribe("veh2/toa", 1, &TDOASSNode::onVeh2Toa, this);
  
  pub_veh1_ref = nh.advertise<auv_msgs::NavSts>("veh1/state_ref", 1);  
  pub_veh2_ref = nh.advertise<auv_msgs::NavSts>("veh2/state_ref", 1); 
  
  state[CENTER] = auv_msgs::NavSts();
  state[MASTER] = auv_msgs::NavSts();
  state[SLAVE] = auv_msgs::NavSts();   
  
  link_names[CENTER] = "center_link";
  link_names[MASTER] = "master_link";
  link_names[SLAVE] = "slave_link";   
}

void TDOASSNode::step()
{
  if (isMaster())
  {
    // Publish transforms.
    auv_msgs::NavSts offset;
    offset.position.east = -baseline/2;
    broadcastTransform(offset,link_names[MASTER],link_names[CENTER]);
    broadcastTransform(offset,link_names[CENTER],link_names[SLAVE]);     
    
    // TODO decide what executes at higher frequncy.
    if (calcluateTimeDifferenceOfArrival())
    {
      double delta = getNormalizedDifferenceOfArrivalMeters();
      double cost = std::pow(delta, 2);
      double yaw_rate = state[MASTER].orientation_rate.yaw;
      yawRateControl(center_ref, delta, cost);
      // check at which frequncy perturbation is set. (probably to low. add flag which updates )
      surgeSpeedControl(center_ref, delta, cost, etaFilterStep(delta, yaw_rate));
    } 
    auv_msgs::BodyVelocityReq vel_req = allocateSpeed(center_ref);      
  }
}

bool TDOASSNode::setAsMaster(bool flag)
{
  if (flag)
    veh_type = MASTER;
  else
    veh_type = SLAVE;
  return true;  
}

bool TDOASSNode::isMaster()
{
  return (veh_type == MASTER)?true:false;  
}

bool TDOASSNode::calcluateTimeDifferenceOfArrival()
{
  //TODO Add timeout in case one measurement does not arrive.
  if (toa1 != toa1_old && toa2 != toa2_old)
  {
    tdoa = (toa1 - toa2).toSec();
    toa1_old = toa1;
    toa2_old = toa2;
  }
}

double TDOASSNode::getTimeDifferenceOfArrival()
{
  return tdoa;
}

double TDOASSNode::getDifferenceOfArrivalMeters()	
{
  return tdoa*speed_of_sound;
}

double TDOASSNode::getNormalizedDifferenceOfArrivalMeters()	
{
  return tdoa*speed_of_sound/baseline;
}


auv_msgs::BodyVelocityReq TDOASSNode::allocateSpeed(auv_msgs::BodyVelocityReq req)
{
  auv_msgs::BodyVelocityReq master_ref;
  double yaw = state[MASTER].orientation.yaw;
  double u_ref = req.twist.linear.x;
  double yaw_rate_ref = req.twist.angular.z;
  master_ref.twist.linear.x = (u_ref - baseline/2 *yaw_rate_ref)*std::cos(yaw);
  master_ref.twist.linear.y = (u_ref + baseline/2 *yaw_rate_ref)*std::sin(yaw);
  return master_ref;
}

void TDOASSNode::surgeSpeedControl(auv_msgs::BodyVelocityReq& req, double delta, double cost, double eta)
{
  const int n = 3;
  double u_amp, u_zeta, u_dir;
  u_dir = labust::math::sgn(eta);
  u_zeta = std::pow(std::abs(eta),n)/std::pow(std::abs(eta)+epsilon, n);
  u_amp = (1-std::tanh(m*cost));
  req.twist.linear.x = u_amp*u_zeta*u_dir;
}

void TDOASSNode::yawRateControl(auv_msgs::BodyVelocityReq& req, double delta, double cost)
{
  req.twist.angular.z = es_controller.step(cost)(0);  
}	

double TDOASSNode::etaFilterStep(double delta, double yaw_rate)
{
  // Calculated using Euler method.
  eta_filter_state_k0 = eta_filter_state_k1;
  eta_filter_state_k1[0] += ts*(-w1*eta_filter_state_k0[0] + delta);
  eta_filter_state_k1[1] += ts*(-w2*eta_filter_state_k0[1] + 
                            k1*yaw_rate*(delta - w1*eta_filter_state_k0[0])); 
  return eta_filter_state_k1[1]; 
}	

void TDOASSNode::onVeh1State(const auv_msgs::NavSts::ConstPtr& msg)
{
  //if (isMaster())
  state[MASTER] = *msg;
}

void TDOASSNode::onVeh2State(const auv_msgs::NavSts::ConstPtr& msg)
{
  state[SLAVE] = *msg; 
}

void TDOASSNode::onVeh1Toa(const std_msgs::Time::ConstPtr& msg)
{
  toa1 = msg->data;
}

void TDOASSNode::onVeh2Toa(const std_msgs::Time::ConstPtr& msg)
{
  toa2 = msg->data;
}

void TDOASSNode::broadcastTransform(auv_msgs::NavSts& state, std::string& frame_id, std::string& child_frame_id)
{
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = frame_id;
  transformStamped.child_frame_id = child_frame_id;
  transformStamped.transform.translation.x = state.position.north;
  transformStamped.transform.translation.y = state.position.east;
  transformStamped.transform.translation.z = state.position.depth;
  tf2::Quaternion q;
  q.setRPY(0, 0, state.orientation.yaw);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  transform_broadcaster.sendTransform(transformStamped);
}

/*********************************************************************
 ***  Main function
 ********************************************************************/

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tdoass_node");
	ros::NodeHandle nh,ph("~");

  //setAsMaster()
  
  
	// std::string primitive_definitions_xml;
	// if(!nh.getParam("primitive_definitions_path",primitive_definitions_xml))
	// {
	// 	ROS_FATAL("Commander: NO PRIMITIVE DEFINITION XML PATH DEFINED.");
	// 	exit (EXIT_FAILURE);
	// }
	// labust::mission::Commander CMD(primitive_definitions_xml);
	
	ros::spin();
	return 0;
}