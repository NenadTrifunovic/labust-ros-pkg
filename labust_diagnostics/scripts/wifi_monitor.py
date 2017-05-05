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
from std_msgs.msg import Bool

import sys
import subprocess
import netifaces as ni
import ipaddress


class WiFiMonitor:
    def __init__(self):
        ''' Status handler intialization '''
        self.status_handler_ = StatusHandler("WiFi", "wifi")
        self.status_handler_.addKeyValue("Link quality")
        self.status_handler_.setEntityStatus(DiagnosticStatus.OK)
        self.status_handler_.setEntityMessage("Status handler initialized.")
        self.status_handler_.publishStatus()

        self.wifi_low_threshold = rospy.get_param('~wifi_low_threshold', 20)
        self.wifi_communication_timeout = rospy.get_param(
            '~wifi_communication_timeout', 30)

        self.wifi_communication_active = False
        self.wifi_communication_last = rospy.Time.now()

        self.wifi_active_pub = rospy.Publisher(
            'wifi_active', Bool, queue_size=1)

        # ni.ifaddresses('eno1')
        self.local_ip = ni.ifaddresses('br0')[2][0]['addr']
        rospy.loginfo("wifi_monitor:: Local ip address: %s", self.local_ip)
        self.picostation_address = str(
            ipaddress.ip_address(self.local_ip.decode('utf-8')) + 1)
        rospy.loginfo("wifi_monitor:: Picostation ip address: %s",
                      self.picostation_address)

    def checkWiFiStatus(self):
        # p = subprocess.Popen('ssh -T ubnt@10.0.103.1 "awk \'NR==4 {print $3 0}\'\'\' /proc/net/wireless"',
        #                     stdout = subprocess.PIPE,
        #                     stderr = subprocess.PIPE, shell = True)
        p = subprocess.Popen('cat /home/stdops/ros/src/labust-ros-pkg/labust_diagnostics/scripts/read_wifi_info.sh | ssh -T ubnt@' + self.picostation_address,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, shell=True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if stdout != '':
            if float(stdout) > self.wifi_low_threshold:
                self.status_handler_.setEntityStatus(DiagnosticStatus.OK)
                self.status_handler_.setEntityMessage("Status normal.")
            else:
                self.status_handler_.setEntityStatus(DiagnosticStatus.WARN)
                self.status_handler_.setEntityMessage("Low signal quality.")
            self.status_handler_.updateKeyValue("Link quality", str(stdout))

            self.wifi_communication_active = True
            self.wifi_communication_last = rospy.Time.now()

        else:
            self.status_handler_.setEntityStatus(DiagnosticStatus.ERROR)
            self.status_handler_.setEntityMessage("No WiFi connection.")
            self.status_handler_.updateKeyValue("Link quality", str(0))

            if (rospy.Time.now() - self.wifi_communication_last).to_sec() > self.wifi_communication_timeout:
                self.wifi_communication_active = False

        self.status_handler_.publishStatus()
        self.wifi_active_pub.publish(Bool(self.wifi_communication_active))


if __name__ == "__main__":
    try:
        rospy.init_node('wifi_monitor_node', anonymous=True)
        update_rate = rospy.get_param('~update_rate', 1)
        dm = WiFiMonitor()
        rate = rospy.Rate(update_rate)  # In Hz
        while not rospy.is_shutdown():
            dm.checkWiFiStatus()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except:
        import traceback
        traceback.print_exc()
