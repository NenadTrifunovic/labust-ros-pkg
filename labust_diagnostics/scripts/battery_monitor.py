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
#roslib.load_manifest('labust_diagnostics')
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from labust_diagnostics.StatusHandler import StatusHandler

import sys
from subprocess import Popen, PIPE

class BatteryMonitor:
    def __init__(self):
        ''' Status handler intialization '''
        self.status_handler_= StatusHandler("Battery monitor","battery_monitor")
        self.status_handler_.addKeyValue("Status");
        self.status_handler_.addKeyValue("Voltage");
        self.status_handler_.addKeyValue("Current");
        self.status_handler_.setEntityStatus(DiagnosticStatus.OK);
        self.status_handler_.setEntityMessage("Status handler initialized.");
        self.status_handler_.publishStatus();
        
        self.battery_low_threshold = rospy.get_param('~battery_low_threshold', 30)

    def checkBatteryStatus(self):
        battery_status  = 57;
        battery_voltage = 11.67;
        battery_current = 6.78;
        
        self.status_handler_.updateKeyValue("Percentage",str(battery_status))
        self.status_handler_.updateKeyValue("Voltage",str(battery_voltage))
        self.status_handler_.updateKeyValue("Current",str(battery_current))
        
        if battery_status > self.battery_low_threshold:
            self.status_handler_.setEntityStatus(DiagnosticStatus.OK);
            self.status_handler_.setEntityMessage("Status normal.");
        else:
            self.status_handler_.setEntityStatus(DiagnosticStatus.WARN);
            self.status_handler_.setEntityMessage("Battery low.");
            
        self.status_handler_.publishStatus();
    
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
