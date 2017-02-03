#!/usr/bin/env python
'''
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
 *********************************************************************/
 
Created on Dec 15, 2015
@author: Filip Mandic
'''
import rospy
import numpy
import math
from auv_msgs.msg import NavSts, BodyForceReq
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from geometry_msgs.msg import TransformStamped, Vector3Stamped, PoseStamped
import tf

class MessageTransformer:
    def __init__(self):
        
        ''' Get parameters '''
        self.model_name = rospy.get_param('~model_name','lupis')
        ''' Initialize services '''
        rospy.wait_for_service('/gazebo/set_model_state')
        self.setState = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState, True)
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.applyWrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench, True)
        ''' Setup listener '''
        self.listener = tf.TransformListener()
        ''' Subscribers '''
        rospy.Subscriber("stateHat", NavSts, self.onNavSts)
        rospy.Subscriber("tauOut", BodyForceReq, self.onTauOut)
    
    def onNavSts(self,data):

        req = SetModelStateRequest()
        req.model_state.model_name = self.model_name

        try:

            data.header.frame_id="ned"
            req.model_state.reference_frame = "world"
            ''' Set desired frame for coversion '''
            end_frame= "world"
            ''' Set initial frame pose '''
            pose = PoseStamped()
            pose.header.frame_id = data.header.frame_id
            pose.header.stamp = self.listener.getLatestCommonTime(end_frame,pose.header.frame_id)
            pose.pose.position.x = data.position.north
            pose.pose.position.y = data.position.east
            pose.pose.position.z = data.position.depth
            
            '''Set pitch to zero (fix surface behavior)'''
            data.orientation.pitch = 0

            quat = tf.transformations.quaternion_from_euler(data.orientation.roll,data.orientation.pitch,data.orientation.yaw)
    
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            ''' Get desired frame pose '''
            pose2 = self.listener.transformPose(end_frame, pose)
            req.model_state.pose.position = pose2.pose.position
            req.model_state.pose.orientation = pose2.pose.orientation
            ''' Set model state service call '''
            self.setState(req)
            
            #req.model_state.reference_frame = "base_link"
            #req.model_state.twist.linear = data.body_velocity
            #req.model_state.twist.angular = pose2.pose.orientation
            ''' Set model state service call '''
            #self.setState(req)
               
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Error on frame conversion.")
                              
    def onTauOut(self,data):
        pass

if __name__ == "__main__":
    rospy.init_node("navsts2gazebo");
    tr = MessageTransformer();
    rospy.spin();
