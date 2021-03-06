"""
Created on Nov 22, 2014

Software License Agreement (BSD License)
 Copyright (c) 2014, LABUST, UNIZG-FER
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:
 
 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of the LABUST nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

@author: Dula Nad
"""

class Enum:
    """
    Represents the C++ data enumeration in the object form.
    
    Class attributes:
    _XMLTAG -- constant string with the xml tag description
    """
    _XMLTAG = 'enum'

    def __init__(self, xmlnode = None):
        """
        Populates the variable from a XML node info.
        When the XML node is not defined, does nothing.
        
        xmlnode -- the XML node with the member definition
            
        Instance attributes:
        ename -- name of the enumerator
        evalue -- value of the enumerator
        """
        self.ename = 'unnamed'
        self.evalue = '0x00'
        
        if xmlnode != None: self.from_xml(xmlnode)
        
    def from_xml(self, xmlnode):
        """
        Extracts the enum attributes from the XML node.
        
        xmlnode -- the XML node with the member definition
        """
        
        if xmlnode.tag != self._XMLTAG:
            raise NameError(self.__class__.__name__ + 
                            ' expected XML tag: "' + 
                            self._XMLTAG + 
                            '"')
        
        self.ename = xmlnode.get('name',self.ename)
        self.evalue = xmlnode.get('value',self.evalue)  
        
    def gen_code(self, bindent = '', indent = '  '):
        """
        Generates the full member definition C++ code and
        returns it as a string
        
        bindent -- the current indent in the document
        indent -- the usual minimum indentation
        """
        return (bindent + indent + 'enum {' + self.ename + ' = ' + 
                self.evalue + '};') 
        
    def __str__(self):
        """
        Overriden string specifier returns the C++ code snippet 
        without indentation.
        """
        return self.gen_code(indent = '')
        
        
        