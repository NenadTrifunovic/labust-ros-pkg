#!/usr/bin/env python
'''
Created on Mar 16, 2015
 \todo Add license information here
@author: dnad
'''
from __future__ import print_function

import rospy
from message_parser import MessageParser
from functools import partial
         
class AsciiSerializator:
    def __init__(self):     
        #self.typemap = ['test_topic','ROS1','auv_msgs/NavSts','position.north','position.east']
        #self.typemap2 = ['test_topic_pub','ROS1','geometry_msgs/Vector3','x','y']
        
        msgs = rospy.get_param('morus_messages')      
        self.desc = MessageParser(msgs)
        
        self._typemap = {}
        self._topic2type = {}
        self._topic2callback= {}
        self.import_types(self.desc.messages)
        self.make_sub(self._topic2type, self._typemap)
        #self.make_pub(self.typemap2)
          
    def import_types(self, desc):
        for message in desc:
            for name, type in message.topic2type.iteritems():
                if name in self._topic2type and self._topic2type[name] != type:
                    rospy.logerr('Topic with name {0} has two distinct types')
                else:
                    self._topic2type[name] = type
                    # Load type
                    pkg, typename = type.split('/')
                    rospy.loginfo('From pkg {0} importing {1}'.format(pkg,typename))
                    mod = __import__(pkg+'.msg', fromlist=[typename])
                    self._typemap[type] = getattr(mod,typename)
                    if not name in self._topic2callback:
                        self._topic2callback[name] = []
                
                    self._topic2callback[name].append(message.on_ros_data)
            
    def make_sub(self, topic2type, typemap):
        self._subs = []
        for topic, type in topic2type.iteritems():
            self._subs.append(rospy.Subscriber(topic,
                                               typemap[type],
                                               partial(self.onData, 
                                                       topic)))
        
    def make_pub(self, typemap):
        type = typemap[2]
        pkg = type.split('/')
        print('From pkg {0} importing {1}'.format(pkg[0],pkg[1]))
        mod = __import__(pkg[0]+'.msg', fromlist=[pkg[1]])
        klass = getattr(mod,pkg[1])
        self.pub1 = rospy.Publisher(typemap[0],klass,queue_size=10)
                
    def get_value(self, data, text):
        textl = text.split('.')
        val = getattr(data, textl[0])
        textl.pop(0)
        for obj in textl:
            val = getattr(val,obj)
            
        return val   
    
    def set_value(self, data, text, value):
        textl = text.split('.')
        val = data
        for obj in textl[:-1]:
            val = getattr(val,obj)
        setattr(val,textl[-1],value)
    
    def onData(self, name, data):
        rospy.loginfo('Received topic {0} with type: {1}'.format(name,data.__class__.__name__))
        for handler in self._topic2callback[name]:
           handler(name, data)
        
        for message in self.desc.messages:
            rospy.loginfo("Data: {0}".format(message.get_fields()))
            
if __name__ == '__main__':
    rospy.init_node("ascii_serializator")
    srv = AsciiSerializator()
    rospy.spin() 
   