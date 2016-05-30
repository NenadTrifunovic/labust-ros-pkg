#!/usr/bin/env python
'''
Created on Mar 16, 2015
 \todo Add license information here
@author: dnad
'''
from __future__ import print_function

import rospy

class ElementDescriptor:
    """
    The class implements the topic elements description.
    """
    def __init__(self, element):
        self.element = element['element']
        self.field_location = element['location']
        self.format = element['format']
        self.modifier = element['modifier']
       
class MessageDescriptor:
    """
    The class implements the ASCII message representation.
    """       
    def __init__(self, message):
        self.name = message['name']
        self.subscribe = message['subscribe']
        self.rate = message['rate']
        
        # Populate topic association
        self.topic2type = dict()
        self.topic_elements = dict()
        nfields = 0
        for topic in message['topics']:
            name = topic['name']
            type = topic['type']
            if name in self.topic2type and self.topic2type[name] != type:
                rospy.logerr('Topic with name {0} has two distinct types')
            else:
                self.topic2type[name] = type
                self.topic_elements[name] = []
                for element in topic['fields']:
                    el = ElementDescriptor(element)
                    self.topic_elements[name].append(el)
                    if el.field_location > nfields:
                        nfields = el.field_location
                
        # Populate the field name
        self.fields = [''] * nfields
        print(self.fields)
    
    def on_ros_data(self, name, data):
        for el in self.topic_elements[name]:
            val = self._get_value(data, el.element)
            # TODO: add modificator
            # TODO: add formating
            self.fields[el.field_location-1] = str(val)
    
    def on_field_data(self, field):
        pass
    
    def get_fields(self):
        return self.fields
    
    def _get_value(self, data, text):
        textl = text.split('.')
        val = getattr(data, textl[0])
        textl.pop(0)
        for obj in textl:
            val = getattr(val,obj)
                  
        return val   
    
    def _set_value(self, data, text, value):
        textl = text.split('.')
        val = data
        for obj in textl[:-1]:
            val = getattr(val,obj)
        setattr(val,textl[-1],value)