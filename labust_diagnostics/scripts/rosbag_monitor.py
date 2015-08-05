#!/usr/bin/env python
import rospy, subprocess
import time
from std_msgs.msg import Bool
from std_msgs.msg import String
from diagnostic_msgs.msg import *
from misc_msgs.srv import RosbagControl, RosbagControlResponse

class RosbagMonitor():
     
    def __init__(self):
        self.setup()
        pass
    
    def setup(self):
            
        rospy.init_node('rosbag_monitor', anonymous=True)
        save_directory = rospy.get_param('~save_directory')  
        self.rosbagActions = {'start': 'rosbag record -a -o '+str(save_directory)+'rosbag', 'split': ''}
        self.loggerActive = False;
        ''' Publishers and subscribers ''' 
        self.subRosbagAction = rospy.Subscriber("rosbagAction", String, self.onRosbagActionCallback)  
        ''' Service '''
        self.srvRosbag = rospy.Service('rosbagAction',RosbagControl,self.rosbagService)
        rospy.spin()
        pass
    
    def rosbagControl(self,action):
        if action=='stop':
            if self.loggerActive == True:
                rospy.logerr("Stopping Rosbag logger...")
                self.proc.terminate()
                self.proc.wait()
                self.loggerActive = False
            else:
                rospy.logerr("No Active log...")
        elif action=='start':
            if self.loggerActive == True:
                self.proc.terminate()
                self.proc.wait()
            rospy.logerr("Starting Rosbag logger...")
            self.proc = subprocess.Popen(self.rosbagActions['start'].split(" "))
            self.loggerActive = True
       
    def rosbagService(self,req):
        self.rosbagControl(req.action)
        return RosbagControlResponse(self.loggerActive)
      
    def onRosbagActionCallback(self, msg):
        self.rosbagControl(msg.data)

if __name__ == '__main__':
    try:
        RBM = RosbagMonitor()
    except rospy.ROSInterruptException: pass


