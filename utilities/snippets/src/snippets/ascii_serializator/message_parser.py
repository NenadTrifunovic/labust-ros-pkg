#!/usr/bin/env python
'''
Created on Mar 16, 2015
 \todo Add license information here
@author: dnad
'''
from __future__ import print_function
from message_descriptor import MessageDescriptor
import rospy
       
class MessageParser:
    """
    The class implements the message description loaded from the YAML definitions.
    """
    def __init__(self):
        self._init_containers()
        pass
        
    def __init__(self, messages):
        self._init_containers()
        self.parse_messages(messages)
        
    def _init_containers(self):
        self.messages = []
        self.topics = []
        self.topic_type = dict()
        self.subscriptions = []
                
    def parse_messages(self, messages):
        # Process messages
        for message in messages:
            # Test for unique name
            if message['name'] in self.messages:
                rospy.logerr('Multiple ASCII messages',
                             ' with name {0} found.'.format(message['name']))
            else:
                self.messages.append(MessageDescriptor(message))

   