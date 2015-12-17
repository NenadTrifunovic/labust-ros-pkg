#!/usr/bin/env python
'''
Created on Dec 15, 2015
 \todo Add license information here
@author: Filip Mandic
'''
import rospy
import numpy
import math
from auv_msgs.msg import NavSts, BodyForceReq
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest

from geometry_msgs.msg import TransformStamped, Vector3Stamped, PoseStamped
import tf2_ros
import tf


     
class MessageTransformer:
    def __init__(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        self.setState = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState, True)
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.applyWrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench, True)
        
        self.model_name = rospy.get_param('~model_name','lupis')
        #self.buffer = 
        self.listener = tf.TransformListener()
        
        rospy.Subscriber("stateHat", NavSts, self.onNavSts)
        rospy.Subscriber("tauOut", BodyForceReq, self.onTauOut)

           
    def onNavSts(self,data):
        pass   
#         req = SetModelStateRequest()
#         req.model_state.model_name = self.model_name
#         
#         try:      
#             #trans = self.buffer.lookup_transform("gazebo_frame", "base_link", rospy.Time(0))
#             req.model_state.pose.position.x = data.position.north
#             req.model_state.pose.position.y = data.position.east
#             req.model_state.pose.position.z = data.position.depth*0
#             req.model_state.pose.orientation = trans.transform.rotation
#             self.setState(req)
#         except (tf2_ros.LookupException, 
#                 tf2_ros.ConnectivityException, 
#                 tf2_ros.ExtrapolationException):
#             rospy.logerr("Error on frame conversion.")
#         
#         yaw = data.orientation.yaw    
#         R = numpy.array([[math.cos(yaw),-math.sin(yaw)], [math.sin(yaw),math.cos(yaw)]],numpy.float32)
#         u = numpy.dot(R, numpy.array([data.body_velocity.x, data.body_velocity.y], dtype=numpy.float32));

        req = SetModelStateRequest()
        req.model_state.model_name = self.model_name
        try:
            pass
            pose = PoseStamped()
            #pose.header.frame_id = "local"
            pose.header.frame_id = data.header.frame_id
            pose.header.stamp = data.header.stamp
            pose.pose.position.x = data.position.north
            pose.pose.position.y = data.position.east
            pose.pose.position.z = data.position.depth
            
            quat = tf.transformations.quaternion_from_euler(data.orientation.roll,data.orientation.pitch,data.orientation.yaw,'rxyz')
    
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            
            pose2 = self.listener.transformPose("gazebo_world", pose)
            
            req.model_state.pose.position = pose2.pose.position
            req.model_state.pose.orientation = pose2.pose.orientation
            self.setState(req)

            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Error on frame conversion.")
                     
             
    def onTauOut(self,data):
        return
        vrijeme = rospy.Time.now()
        try:      
            #trans = self.buffer.lookup_transform("base_link_sim", "lupis__base_link", rospy.Time(0))
            #trans = self.buffer.lookup_transform("base_link_sim","lupis__base_link", rospy.Time(0))
            
        
            #(trans,rot) = self.listener.lookupTransform("base_link","lupis__base_link", rospy.Time(0))
   
          
           
     
            #quat = trans.transform.rotation
            #rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
            
            #tf.transformations.quaternion_from_euler(ai, aj, ak)
        
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.header.stamp = rospy.Time(0)
            pose.pose.position.x = data.wrench.force.x
            pose.pose.position.y = data.wrench.force.y
            pose.pose.position.z = data.wrench.force.z
            
            quat = tf.transformations.quaternion_from_euler(data.wrench.torque.x,data.wrench.torque.y,data.wrench.torque.z)
    
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            tf.transformations.quaternion_about_axis(3.14, (1,0,0))
            
            pose2 = self.listener.transformPose("lupis__base_link", pose)
            
            vec = Vector3Stamped()
            vec.header.frame_id = "base_link"
            vec.header.stamp = rospy.Time(0)
            vec.vector = data.wrench.force
    
            vec2 = self.listener.transformVector3("lupis__base_link", vec)
    
            print vec  
            print pose2
            
            req = ApplyBodyWrenchRequest()
            req.body_name = self.model_name+'::base_link'
            req.wrench.force = vec2.vector
            #req.wrench.torque = pose2.pose.orientation
            req.wrench.torque=data.wrench.torque
            req.duration = rospy.Duration(0.1)
    #              
            resp = self.applyWrench(req)
#             
#             #print rot[0:3,0:3] 
#             
#             #req.model_state.pose.position.x = data.position.north
#             #req.model_state.pose.position.y = data.position.east
#             #req.model_state.pose.position.z = data.position.depth*0
#             #req.model_state.pose.orientation = trans.transform.rotation
#             
#             tmp= numpy.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z], dtype=numpy.float32).transpose()
#             
#             #print numpy.transpose(tmp)
# 
#             out = numpy.dot(rot[0:3,0:3],numpy.transpose(tmp))
#             #rospy.wait_for_service('add_two_ints')
#             req = ApplyBodyWrenchRequest()
#             req.body_name = self.model_name+'::base_link'
#             req.wrench.force.x = out[0]
#             req.wrench.force.y = out[1] 
#             req.wrench.force.z = out[2]  
#             #req.wrench.torque = numpy.dot(rot,data.wrench.torque, dtype=numpy.float32)
#             req.wrench.force = data.wrench.force
#             req.wrench.torque = data.wrench.torque
#             req.duration = rospy.Duration(0.1)
#              
#             resp = self.applyWrench(req)
#             print resp
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Error on frame conversion.")



if __name__ == "__main__":
    rospy.init_node("navsts2gazebo");
    tr = MessageTransformer();
    rospy.spin();
