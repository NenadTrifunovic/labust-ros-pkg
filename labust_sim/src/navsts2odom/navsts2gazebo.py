#!/usr/bin/env python
'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
import rospy
import numpy
import math
from auv_msgs.msg import NavSts
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros
     
class MessageTransformer:
    def __init__(self):
        #rospy.wait_for_service('gazebo/SetModelState')
        self.setState = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState, True)
        self.model_name = rospy.get_param('~model_name','pladypos')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        rospy.Subscriber("stateHat", NavSts, self.onNavSts)
        self.ff = rospy.Publisher("usv_vel", Twist)
           
    def onNavSts(self,data):   
        req = SetModelStateRequest()
        req.model_state.model_name = self.model_name
        
        try:      
            trans = self.buffer.lookup_transform("gazebo_frame", "base_link", rospy.Time(0))
            req.model_state.pose.position.x = data.position.north
            req.model_state.pose.position.y = data.position.east
            req.model_state.pose.position.z = data.position.depth*0
            req.model_state.pose.orientation = trans.transform.rotation
            self.setState(req)
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException):
            rospy.logerr("Error on frame conversion.")
        
        yaw = data.orientation.yaw    
        R = numpy.array([[math.cos(yaw),-math.sin(yaw)], [math.sin(yaw),math.cos(yaw)]],numpy.float32)
        u = numpy.dot(R, numpy.array([data.body_velocity.x, data.body_velocity.y], dtype=numpy.float32));
        
        ff = Twist()
        ff.linear.x = u[0]
        ff.linear.y = u[1]
        self.ff.publish(ff);

if __name__ == "__main__":
    rospy.init_node("navsts2gazebo");
    tr = MessageTransformer();
    rospy.spin();
