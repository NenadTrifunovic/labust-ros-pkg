#!/usr/bin/env python

#/*********************************************************************
 #* topic_transform.py
 #*
 #*  Created on: Aug 29, 2014
 #*      Author: Filip Mandic
 #*
 #********************************************************************/

#/*********************************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2014, LABUST, UNIZG-FER
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the LABUST nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#*********************************************************************/

#!/usr/bin/env python
import rospy

from misc_msgs.msg import ExternalEvent

from std_msgs.msg import *
from auv_msgs.msg import *

class TopicTransform():
    
    def __init__(self):
        self.setup()

    def callback(self,data):
   
        tmpList = self.msg_field.split("/")
        
        parentData = data

        for field in tmpList:
            parentData = getattr(parentData,field)
        
        eventData = ExternalEvent()
        eventData.id = self.eventID
        eventData.value = parentData
        
        self.pubExternalEvent.publish(eventData)
        
    def setup(self):
        
        rospy.init_node('topic_transform', anonymous=True)
        
        topic_name = rospy.get_param('~topic_name')
        self.eventID = rospy.get_param('~event')
       
        tmp = ""
        tmp = rospy.get_param('~msg_type')  
        tmpList = tmp.split("/")
        msg_package = tmpList[0]
        msg_type = tmpList[1]
 
        self.msg_field = ""
        self.msg_field = rospy.get_param('~msg_field')
                
        if msg_package == "std_msgs":
            msgType = getattr(std_msgs.msg, msg_type)
            
        elif msg_package == "auv_msgs":
            msgType = getattr(auv_msgs.msg, msg_type)
                   
        self.pubExternalEvent = rospy.Publisher("externalEvent", ExternalEvent, queue_size=20)
        
        rospy.Subscriber(topic_name, msgType, self.callback)
    
        rospy.spin()
        
if __name__ == '__main__':
    try:
        TT = TopicTransform()
    except rospy.ROSInterruptException: pass
    
    
  
