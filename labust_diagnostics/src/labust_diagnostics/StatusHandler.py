#!/usr/bin/env python
'''
/*
 * StatusHandler.py
 *
 *  Created on: Feb 16, 2017
 *      Author: filip
 */
'''
import rospy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray

class StatusHandlerBase(object):
    def __init__(self):
        self.entity_status_ = DiagnosticStatus()
        self.setEntityName("No name");
        self.setEntityId("No ID");
        self.setEntityStatus(DiagnosticStatus.STALE);
        self.setEntityMessage("No message.");
        pass
    
    def setEntityName(self, entity_name):
        self.entity_status_.name = entity_name
        
    def setEntityMessage(self,entity_message):
        self.entity_status_.message = entity_message

    def setEntityId(self,entity_id):
        self.entity_status_.hardware_id = entity_id

    def setEntityStatus(self,status):
        self.entity_status_.level = status

    def addKeyValue(self,key_value_name):
        rospy.loginfo("Adding key value %s...", key_value_name)
        new_value = KeyValue()
        new_value.key = key_value_name
        new_value.value = "None"
        self.entity_status_.values.append(new_value)
        rospy.loginfo("Key value %s added.", key_value_name)
    

    def updateKeyValue(self,key_value_name,value):
        rospy.loginfo("Updating key value %s...", key_value_name)
        for i,it in enumerate(self.entity_status_.values):
            if key_value_name == it.key:
                self.entity_status_.values[i].value = value
                rospy.loginfo("Key value %s updated.", key_value_name)
                return
        rospy.logerr("No key value %s found.", key_value_name)
        

    def deleteKeyValue(key_value_name):
        rospy.loginfo("Deleting key value %s...", key_value_name)
        for i,it in enumerate(self.entity_status_.values):
            if key_value_name == it.key:
                #self.entity_status_.values.remove(key_value_name)
                del self.entity_status_.values[i]   
                rospy.loginfo("Key value %s deleted.", key_value_name)
                return
        rospy.logerr("No key value %s found.", key_value_name)

        
class StatusHandler(StatusHandlerBase):
    def __init__(self, entity_name, entity_id):
        super(StatusHandler, self).__init__()
        self.setEntityName(entity_name)
        self.setEntityId(entity_id)
        self.pub_status_ = rospy.Publisher('/diagnostics/'+entity_id,DiagnosticArray, latch=True, queue_size=1)
        
    def publishStatus(self):
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = rospy.Time.now();
        diagnostic_array.status.append(self.entity_status_);
        self.pub_status_.publish(diagnostic_array);
   
        
    




