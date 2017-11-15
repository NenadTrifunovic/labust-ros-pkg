#!/usr/bin/env python
import rospy
import subprocess
import time
import signal
import os
import math
from std_msgs.msg import Bool
from std_msgs.msg import String
from auv_msgs.msg import NavSts
from diagnostic_msgs.msg import *
from misc_msgs.srv import RosbagControl, RosbagControlResponse
from navcon_msgs.srv import EnableControlRequest, EnableControl


class TDOASSTest():

  def __init__(self):
    self.setup()

  def setup(self):

    self.center_position = NavSts()
    self.target_position = NavSts()
    self.target_position.position.north = 0
    self.target_position.position.east = 0
    self.approach_bearing = 0
    self.initial_alpha = 0
    self.initial_distance = 20

    self.pub_test_init = rospy.Publisher(
        "/master/test_init", NavSts, queue_size=1, latch=True)

    rospy.init_node('tdoass_test', anonymous=True)

    # rospy.spin()

  def startTest(self):
    self.center_position.position.north = self.target_position.position.north + \
        self.initial_distance * math.cos(self.approach_bearing)

    self.center_position.position.east = self.target_position.position.east + \
        self.initial_distance * math.sin(self.approach_bearing)

    self.center_position.orientation.yaw = self.initial_alpha + \
        self.approach_bearing - math.pi

    self.pub_test_init.publish(self.center_position)
    rospy.sleep(3)
    self.enableSlaveController(True)
    self.enableMasterController(True)

  def stopTest(self):
    try:
      self.enableMasterController(False)
    except:
      pass
    try:
      self.enableSlaveController(False)
    except:
      pass

  def enableMasterController(self, flag):
    req = EnableControlRequest()
    req.enable = flag
    rospy.ServiceProxy("/usv/TDOASS_master_enable", EnableControl).call(req)

  def enableSlaveController(self, flag):
    req = EnableControlRequest()
    req.enable = flag
    rospy.ServiceProxy("/usv/TDOASS_slave_enable", EnableControl).call(req)


if __name__ == '__main__':
  try:
    TT = TDOASSTest()        
    TT.startTest()
    rospy.loginfo("Test started.")
    rospy.spin()
    rospy.loginfo("Interrupt.")  
    TT = TDOASSTest()    
    TT.stopTest()
    rospy.loginfo("Test stopped.") 
    
  except rospy.ROSInterruptException:
    rospy.loginfo("Interrupt 2.")          
    pass
