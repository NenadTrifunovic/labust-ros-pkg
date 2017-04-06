#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, LABUST.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import roslib
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from labust_diagnostics.StatusHandler import StatusHandler
from std_msgs.msg import Int16MultiArray

import sys
from subprocess import Popen, PIPE

class BatteryMonitor:
    def __init__(self):
        ''' Status handler intialization '''
        self.status_handler_= StatusHandler("Battery","battery")
        self.status_handler_.addKeyValue("Percentage");
        self.status_handler_.addKeyValue("Voltage");
        self.status_handler_.addKeyValue("Current");
        self.status_handler_.setEntityStatus(DiagnosticStatus.OK);
        self.status_handler_.setEntityMessage("Status handler initialized.");
        self.status_handler_.publishStatus();
        
        self.battery_low_threshold = rospy.get_param('~battery_low_threshold', 30)
        self.gain_k = rospy.get_param('~gain_k', 0.05)
        self.battery_voltage_minimum = rospy.get_param('~battery_minimum_voltage', 11.7)
        self.battery_volatge_maximum = rospy.get_param('~battery_maximum_voltage', 12.6)

        self.sub_telemetry = rospy.Subscriber("telemetry", Int16MultiArray, self.onTelemetry)
        self.last_measurement_timestamp = rospy.Time.now() 
        
        self.battery_status = 0
        self.battery_voltage = 0
        self.battery_current = 0
        
    def checkBatteryStatus(self):
        
        self.battery_status = 100*(self.battery_voltage-self.battery_voltage_minimum+self.gain_k*self.battery_current)/(self.battery_volatge_maximum-self.battery_voltage_minimum) 
        
        self.status_handler_.updateKeyValue("Percentage",str(self.battery_status))
        self.status_handler_.updateKeyValue("Voltage",str(self.battery_voltage))
        self.status_handler_.updateKeyValue("Current",str(self.battery_current))

        if (rospy.Time.now()-self.last_measurement_timestamp).to_sec > 10: 
            self.status_handler_.setEntityStatus(DiagnosticStatus.ERROR);
            self.status_handler_.setEntityMessage("No measurements.");
        elif self.battery_status > self.battery_low_threshold:
            self.status_handler_.setEntityStatus(DiagnosticStatus.OK);
            self.status_handler_.setEntityMessage("Status normal.");
        else:
            self.status_handler_.setEntityStatus(DiagnosticStatus.WARN);
            self.status_handler_.setEntityMessage("Battery low.");
            
        self.status_handler_.publishStatus();
        
    def onTelemetry(self,msg):
        self.last_measurement_timestamp = rospy.Time.now()
        self.battery_voltage = msg.data[0]
        self.battery_current = msg.data[1]
    
if __name__ == "__main__":
    try:
        rospy.init_node('battery_monitor_node', anonymous=True)
        update_rate = rospy.get_param('~update_rate', 1)
        bm = BatteryMonitor()
        rate = rospy.Rate(update_rate) # In Hz
        while not rospy.is_shutdown():
            bm.checkBatteryStatus()
            rate.sleep()
    except KeyboardInterrupt: pass
    except SystemExit: pass
    except:
        import traceback
        traceback.print_exc()
