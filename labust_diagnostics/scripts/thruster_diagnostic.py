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
#from diagnostic_msgs.msg import DiagnosticStatus
from labust_diagnostics.StatusHandler import StatusHandler
from std_msgs.msg import Float32MultiArray, Int32


import sys
from subprocess import Popen, PIPE


class ThrusterDiagnostic():

    def __init__(self):

        self.sub_diagnostic_request = rospy.Subscriber(
            "thruster_diagnostic_request", Float32MultiArray, self.onDiagnosticRequest)

        self.pub_allocation_mode = rospy.Publisher(
            "allocation_mode", Int32, queue_size=1)

        self.pub_pwm_out = rospy.Publisher(
            "pwm_out", Float32MultiArray, queue_size=1)

        self.pwm_request = Float32MultiArray()

        self.pwm_request_flag = False

        self.test_active_flag = False

        self.start_timestamp = rospy.Time.now()

    def loop(self):
        """ ... """
        if self.pwm_request_flag:
            msg = Int32()
            msg.data = 0
            self.pub_allocation_mode.publish(msg)
            self.pwm_request_flag = False
            self.test_active_flag = True
            self.start_timestamp = rospy.Time.now()

        elif self.test_active_flag:
            self.pub_pwm_out.publish(self.pwm_request)

        if (rospy.Time.now() - self.start_timestamp).to_sec() > 10:
            self.test_active_flag = False
            msg = Int32()
            msg.data = 1
            self.pub_allocation_mode.publish(msg)

    def onDiagnosticRequest(self, msg):
        self.pwm_request = msg
        self.pwm_request_flag = True


if __name__ == "__main__":
    try:
        rospy.init_node('thruster_diagnostic_node', anonymous=True)
        update_rate = rospy.get_param('~update_rate', 10)
        td = ThrusterDiagnostic()
        rate = rospy.Rate(update_rate)  # In Hz
        while not rospy.is_shutdown():
            td.loop()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except:
        import traceback
        traceback.print_exc()
