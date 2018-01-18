#!/usr/bin/env python
import rospy
import subprocess
import time
import signal
import os
from std_msgs.msg import Bool
from std_msgs.msg import String
from diagnostic_msgs.msg import *
from misc_msgs.srv import RosbagControl, RosbagControlResponse


class RosbagMonitor():

  def __init__(self):
    self.setup()

  def setup(self):

    rospy.init_node('rosbag_monitor', anonymous=True)
    save_directory = rospy.get_param('~save_directory')
    self.default_bag_name = 'rosbag'
    self.rosbagActions = {
        'start': 'rosbag record -a -o ' + str(save_directory) + '/%s', 'split': ''}
    self.loggerActive = False
    ''' Publishers and subscribers '''
    self.subRosbagAction = rospy.Subscriber(
        "rosbagAction", String, self.onRosbagActionCallback)
    ''' Service '''
    self.srvRosbag = rospy.Service(
        'rosbagAction', RosbagControl, self.rosbagService)
    rospy.spin()

  def rosbagControl(self, req):
    action = req.action
    if action == 'stop':
      if self.loggerActive == True:
        rospy.loginfo("Stopping Rosbag logger...")
        os.killpg(self.proc.pid, signal.SIGINT)
        self.proc.wait()
        self.loggerActive = False
      else:
        rospy.loginfo("No Active log...")
    elif action == 'start':
      if self.loggerActive == True:
        os.killpg(self.proc.pid, signal.SIGINT)
        self.proc.wait()
      rospy.loginfo("Starting Rosbag logger...")
      bag_name = self.default_bag_name
      if req.bag_name != '':
        bag_name = req.bag_name
      self.proc = subprocess.Popen(
          (self.rosbagActions['start'] % bag_name).split(" "), preexec_fn=os.setsid)
      self.loggerActive = True

  def rosbagService(self, req):
    self.rosbagControl(req)
    return RosbagControlResponse(self.loggerActive)

  def onRosbagActionCallback(self, msg):
    #self.rosbagControl(msg.data)
    rospy.logerr("Rosbag control using topic is not implemented.")
   


if __name__ == '__main__':
  try:
    RBM = RosbagMonitor()
  except rospy.ROSInterruptException:
    pass
