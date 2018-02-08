#!/usr/bin/env python
from snippets.generators.elements.structure import Structure
from snippets.generators.elements.variable import Variable
from snippets.generators.elements.function import Function
import xml.etree.ElementTree as ET

class CppGen():
    def __init__(self):        
        self.type_equiv = {
        'bool': 'byte',
        'int8_t': 'byte',
        'uint8_t': 'byte',
        'int16_t': 'short',
        'uint16_t': 'short',
        'int32_t': 'int',
        'uint32_t': 'int',
        'int64_t': 'long',
        'uint64_t': 'long'
        }
        
        self.type_size = {
         'byte':1,
         'short':2,
         'int':4,
         'long':8,
         'float':4,
         'double':8}
               
        self.primitive_types = self.type_equiv.keys()
        self.packable_types = []
        self.special_types = {}
        self.array_len_type = 'int'
    
    def gen_variable(self, var, indent = ''):
        atype = None
        type = var.type
        cinit = var.default
                
        if self.is_array_type(var.type):
            len = self.get_array_len(var.type)
            type = self.get_array_type(var.type)
            if len == '':
                type = 'std::vector< ' + type + ' >'
            else:
                atype = '[' + len + ']'
                                                             
        cvar = indent + type + ' ' + var.name
        
        if atype != None:
            cvar = cvar + atype
            
        if cinit != None:
            cvar = cvar + ' = ' + cinit
                
        return cvar + ';'
    
    def gen_enum(self, enum, indent = ''):
        return indent + 'enum { ' + enum.name + ' = ' + enum.value + '};'
    
    def gen_method_def(self, method, indent):
        code = indent + method.ret + ' '
        code = code + method.name + '(' 
        
        for arg in method.args:
            code = code + arg.type + ' ' + arg.name + ',' 
       
        if len(method.args): code = code[:-1]
        code = code + ')' 
        
        if method.qual != None:
            code = code + ' ' + method.qual
                      
        if method.inline:
            code = code + '\n{\n' + indent + method.body + '\n}'
                
        return code + ";"
           
    def gen_method(self, name, method, indent):
        if method.inline: return ''
        
        code = indent + method.ret + ' '
        code = code + name + '::' + method.name + '(' 
        
        for arg in method.args:
            code = code + arg.type + ' ' + arg.name + ',' 
       
        if len(method.args): code = code[:-1]
        code = code + ')' 
        
        if method.qual != None:
            code = code + ' ' + method.qual
            
        code = code + '\n{\n' + indent + method.body + '\n}'
            
        return code + ";"
    
    def gen_header(self, struct, includes = [], serialization = True):
        #Header guard
        #Includes
                            
        #Class definition
        code = 'struct ' + struct.name
            
        if struct.inherit != None:
            code = code + ' : public ' + struct.inherit 
        code = code + ' {\n'     
        code = code + 'public:\n'
    
        #Generate typedefs
        for t in struct.typedefs:
            code = code + '  typedef ' + t[0] + ' ' + t[1] + ';\n'
                
        #Generate enums as final static int
        for enum in struct.enums:
            code = code + self.gen_enum(enum, '  ') + '\n'
        code = code + "\n"
        
        #Methods
        if serialization: self.add_serializers(struct, '  ')          
        for method in struct.methods:
            code = code + self.gen_method_def(method, '  ') + '\n\n'
        
        for var in struct.variables:
            code = code + self.gen_variable(var, '  ') + '\n'
        
        code = code + "\n"
                                
        #End class definition
        code = code + '};\n' 
    
        return code
    
    def gen_impl(self, struct, includes = [], serialization = True):
        #Includes
                            
        #Class implementation                
        #Methods
        if serialization: self.add_serializers(struct, '  ')     
        code = ''     
        for method in struct.methods:
            code = code + self.gen_method(struct.name, method, '  ') + '\n\n'
        
        code = code + "\n"
    
        return code
    
    def add_serializers(self, struct, indent):
        # Adhers to the packer interface
        fn = Function()
        fn.name = 'pack' 
        fn.inline = None
        fn.ret = 'void'
        fn.qual = 'const'
        arg = Variable()
        arg.name = 'out'
        arg.type = 'boost::archive::binary_oarchive&'
        fn.args.append(arg)
        fn.body = self.gen_serializer_body(struct, indent)
        struct.methods.append(fn)    
        
        fn = Function()
        fn.name = 'unpack' 
        fn.ret = 'void'
        fn.qual = ''
        arg = Variable()
        arg.name = 'in'
        arg.type = 'boost::archive::binary_iarchive&'
        fn.args.append(arg)
        fn.body = self.gen_deserializer_body(struct, indent)
        struct.methods.append(fn)            

    def gen_bitfield_serializer(self, struct, indent):
        store_type = ''
        sz = int(struct.assert_size)
        if sz == 1:
            store_type = 'uint8_t'
        if sz == 2:
            store_type = 'uint16_t'
        elif (sz > 2) and (sz <= 4):
            store_type = 'uint32_t'
        elif (sz > 4) and (sz <= 8):
            store_type = 'uint64_t'
        elif sz != 1:
            print('Cannot create a serializer for size '+ str(sz))
            return '' 
        
        code = indent + store_type + ' storage = 0;\n'
        shift = 0  
        for var in struct.variables:
            if var.bits == None: 
                print('All variables need a bit size in a bitfield.')
                return ''
            code = code + indent + 'storage |= (' 
            code = code + store_type + '(' + var.name
            code = code + ') & ((1<<' + var.bits + ')-1))'
            code = code + ' << ' + str(shift) + ';\n'
            shift += int(var.bits)
        
        #Minimum byte rounding to avoid having zeros
        code = code + indent + 'uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));\n'
        code = code + indent + 'for(int i=0; i<' + str(sz) + '; ++i,++pt) out << *pt;\n'
        
        return code
    
    def gen_bitfield2_serializer(self, struct, indent):
        code = ''
        code = indent + 'labust::tools::BitStorage st;\n'
        shift = 0  
        for var in struct.variables:
            if var.bits == None: 
                print('All variables need a bit size in a bitfield.')
                return ''
            if self.is_array_type(var.type):
                code = code + indent + 'for(int i=0; i<' + self.get_array_len(var.type)
                code = code + '; ++i) st.put(' + var.name + '[i]'
            else:
                code = code + indent + 'st.put(' + var.name 
                
            code = code + ',' + var.min + ',' + var.max + ','
            code = code + var.bits + ');\n'
        
        #Minimum byte rounding to avoid having zeros
        code = code + indent + 'for(int i=0; i<st.storage().size(); ++i) out << st.storage()[i];\n'
        
        return code
    
    def gen_bitfield3_serializer(self, struct, indent):
        code = ''
        code = indent + 'labust::tools::BitStorage st;\n'
        shift = 0  
        for var in struct.variables:
            if var.bits == None: 
                print('All variables need a bit size in a bitfield.')
                return ''
            has_cond = (var.gcond != None) or (var.cond != None)
            if has_cond:
                code = code + indent + 'if ('   
                
                if var.gcond != None:
                    code = code + '(' + var.gcond + ')'
                    if var.cond != None:
                        code = code + ' && '  
                
                if var.cond != None:
                    code = code + '(' + var.cond + ')'
                
                code = code + '){\n' 
                                
            # Debugging output
            #code = code + indent + 'std::cout<<"' + var.name + ' output:";'
            if self.is_array_type(var.type):
                len = self.get_array_len(var.type)
                               
                if len == '':
                    # WARNING LENGTH IS NOT SAVED, IT IS ASSUMED THIS IS AT THE END OF BITFIELD 
                    code = code + indent + 'for(int i=0; i<' + var.name + '.size(); ++i) '
                else:
                    code = code + indent + 'for(int i=0; i<' + len + '; ++i) '
                    
                code = code + 'st.put(' + var.name + '[i]'
            else:
                code = code + indent + 'st.put(' + var.name 
                
            code = code + ',' + var.min + ',' + var.max + ','
            code = code + var.bits + ');\n'
        
            if has_cond:
                code = code + indent + '}\n'
        
        #Minimum byte rounding to avoid having zeros
        code = code + indent + 'for(int i=0; i<st.storage().size(); ++i) out << st.storage()[i];\n'
        
        return code
    
    def gen_bitfield_deserializer(self, struct, indent):
        store_type = ''
        sz = int(struct.assert_size)
        if sz == 1:
            store_type = 'uint8_t'
        if sz == 2:
            store_type = 'uint16_t'
        elif (sz > 2) and (sz <= 4):
            store_type = 'uint32_t'
        elif (sz > 4) and (sz <= 8):
            store_type = 'uint64_t'
        elif sz != 1:
            print('Cannot create a serializer for size '+ str(sz))
            return '' 
        
        code = ''
        code = code + indent + store_type + ' storage(0);\n'
        code = code + indent + 'uint8_t* pt(reinterpret_cast<uint8_t*>(&storage));\n'
        code = code + indent + 'for(int i=0; i<' + str(sz) + '; ++i,++pt) in >> *pt;\n'
        for var in struct.variables:
            if var.bits == None: 
                print('All variables need a bit size in a bitfield.')
                return ''
            code = code + indent + var.name + ' = static_cast<'
            code = code + var.type + '>'
            code = code + '(storage & ((1<<' + var.bits + ')-1));\n' 
            code = code + indent + 'storage >>= ' + str(var.bits) + ';\n'
        
        return code
    
    def gen_bitfield2_deserializer(self, struct, indent):
        sz = int(struct.assert_size)
        
        code = ''
        code = code + indent + 'std::vector<uint8_t> data;\n'
        code = code + indent + 'for(int i=0; i<' + str(sz) + '; ++i)\n'
        code = code + indent + '{\n'
        code = code + indent + 'uint8_t temp;\n'
        code = code + indent + 'in >> temp;\n'
        code = code + indent + 'data.push_back(temp);\n'
        code = code + indent + '}\n'
                
        code = code + indent + 'labust::tools::BitStorage st(data);\n'
        shift = 0  
        for var in struct.variables:
            if var.bits == None: 
                print('All variables need a bit size in a bitfield.')
                return ''
            if self.is_array_type(var.type):
                code = code + indent + 'for(int i=0; i<' + self.get_array_len(var.type)
                code = code + '; ++i) st.get(' + var.name + '[i]'
            else:
                code = code + indent + 'st.get(' + var.name 
            
            code = code + ',' + var.min + ',' + var.max + ','
            code = code + var.bits + ');\n'
                
        return code
    
    def gen_bitfield3_deserializer(self, struct, indent):
       
        code = ''
        code = code + indent + 'std::vector<uint8_t> data;\n'
        code = code + indent + 'try{\n'
        code = code + indent + 'while(true)\n'
        #code = code + indent + 'for(int i=0; i<' + str(sz) + '; ++i)\n'
        code = code + indent + '{\n'
        code = code + indent + 'uint8_t temp;\n'
        code = code + indent + 'in >> temp;\n'
        code = code + indent + 'data.push_back(temp);\n'
        code = code + indent + '}\n'
        code = code + indent + '} catch (std::exception& e){};\n'
                
        code = code + indent + 'labust::tools::BitStorage st(data);\n'
        shift = 0  
        for var in struct.variables:
            if var.bits == None: 
                print('All variables need a bit size in a bitfield.')
                return ''
            
            has_cond = (var.gcond != None) or (var.cond != None)
            if has_cond:
                code = code + indent + 'if ('   
                
                if var.gcond != None:
                    code = code + '(' + var.gcond + ')'
                    if var.cond != None:
                        code = code + ' && '  
                
                if var.cond != None:
                    code = code + '(' + var.cond + ')'
                
                code = code + '){\n' 
            
            #code = code + indent + 'std::cout<<"' + var.name + ' input:";'
                        
            if self.is_array_type(var.type):
                len = self.get_array_len(var.type)
                
                if len == '':
                    # WARNING LENGTH IS NOT SAVED, IT IS ASSUMED THIS IS AT THE END OF BITFIELD
                    code = code + indent + var.name + '.resize(st.remaining());\n'
                    code = code + indent + 'for(int i=0; i<st.remaining()'
                else:
                    code = code + indent + 'for(int i=0; i<' + len
                    
                code = code + '; ++i) st.get(' + var.name + '[i]'
            else:
                code = code + indent + 'st.get(' + var.name 
            
            code = code + ',' + var.min + ',' + var.max + ','
            code = code + var.bits + ');\n'
            
            if has_cond:
                code = code + indent + '}\n'
                
        return code
    
    
    def is_array_type(self, type):
        return ((type.find('[') != -1) and
            (type.find(']') != -1))
    
    def get_array_len(self, type):
        sidx = type.find('[')
        eidx = type.find(']')
        return type[sidx+1:eidx]
    
    def get_array_type(self, type):
        sidx = type.find('[')
        return type[:sidx]
            
    def gen_array_serializer(self, var, indent):
        code = indent
        len = self.get_array_len(var.type)
               
        if len == '':
            code = code + self.array_len_type + ' ' + var.name + '_len'
            code = code + '(' + var.name +'.size());\n'
            code = code + indent + 'out << ' + var.name + '_len;\n'
            code = code + indent + 'for(int i=0; i<' + var.name + '.size(); ++i) '
        else:
            code = code + indent + 'for(int i=0; i<' + len + '; ++i) '
        
        code = code + 'out << '
        code = code + var.name + '[i];\n'
                    
        return code
    
    def gen_array_deserializer(self, var, indent):
        code = indent
        len = self.get_array_len(var.type)
               
        if len == '':
            code = code + self.array_len_type + ' ' + var.name + '_len;\n'
            code = code +  'in >> ' + var.name + '_len;\n'
            code = code + indent + var.name + '.resize(' + var.name + '_len);\n'
            code = code + indent + 'for(int i=0; i<' + var.name + '.size(); ++i) '
        else:
            code = code + indent + 'for(int i=0; i<' + len + '; ++i) '
            
        code = code + 'in >> '
        code = code + var.name + '[i];\n'
            
        return code
            
    def gen_serializer_body(self, struct, indent):
        # Specially handle bitfields          
        if struct.bitfield: return self.gen_bitfield_serializer(struct, indent)
        if struct.bitfield2: return self.gen_bitfield2_serializer(struct, indent)
        if struct.bitfield3: return self.gen_bitfield3_serializer(struct, indent)
                        
        # Serialize each memeber variable
        code = ''
        for var in struct.variables:  
            if var.cond != None:
                code = code + indent + 'if (' + var.cond + '){\n'
                          
            if self.is_array_type(var.type):
                code = code + indent + self.gen_array_serializer(var, indent)
            elif var.type in self.primitive_types:
                code = code + indent + 'out <<' + var.name + ';\n'
            elif var.type in self.packable_types:
                code = code + indent + var.name + '.pack(out);\n'
            elif var.type in self.special_types.keys():
                code = code + indent + self.special_types[var.type](var, "serializer")
            else:
                print('Cannot create a serializer for type: ' + var.type)
                
            if var.cond != None:
                code = code + indent + '}\n'
            
        return code
    
    def gen_deserializer_body(self, struct, indent):
        # Specially handle bitfields          
        if struct.bitfield: return self.gen_bitfield_deserializer(struct, indent)
        if struct.bitfield2: return self.gen_bitfield2_deserializer(struct, indent)
        if struct.bitfield3: return self.gen_bitfield3_deserializer(struct, indent)
                        
                # Serialize each memeber variable
        code = ''
        for var in struct.variables:            
            if var.cond != None:
                code = code + indent + 'if (' + var.cond + '){\n'    
            if self.is_array_type(var.type):
                code = code + indent + self.gen_array_deserializer(var, indent)
            elif var.type in self.primitive_types:
                code = code + indent + 'in >> ' + var.name + ';\n'
            elif var.type in self.packable_types:
                code = code + indent + var.name + '.unpack(in);\n'
            elif var.type in self.special_types.keys():
                code = code + indent + self.special_types[var.type](var, "deserializer")
            else:
                print('Cannot create a serializer for type: ' + var.type)
            
            if var.cond != None:
                code = code + indent + '}\n'
        return code
