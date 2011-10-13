"""autogenerated by genmsg_py from SRS_Action.msg. Do not edit."""
import roslib.message
import struct

import srs_msgs.msg

class SRS_Action(roslib.message.Message):
  _md5sum = "78f876cadd0a50a227e671e96dc7e09b"
  _type = "srs_msgs/SRS_Action"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# this message contains information to define a SRS action 

uint32 objectID #targeted object

string objectName #object name for easy reading

uint32 actionID #action to be applied

string actionName #action name for easy reading

bool component_is_required #a component is required or not

srs_msgs/Component[] components   #components required

bool symbolic_is_required #a component is required or not

uint32 symbolicID # ID of the symbolic

string symbolicName #name of the symbolic

string description  #a short description on the current action set to null if no description
================================================================================
MSG: srs_msgs/Component
# this message contains information to define a list of required components for an action 
uint32 componentID   #ID
string componentName   #Name

"""
  __slots__ = ['objectID','objectName','actionID','actionName','component_is_required','components','symbolic_is_required','symbolicID','symbolicName','description']
  _slot_types = ['uint32','string','uint32','string','bool','srs_msgs/Component[]','bool','uint32','string','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       objectID,objectName,actionID,actionName,component_is_required,components,symbolic_is_required,symbolicID,symbolicName,description
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(SRS_Action, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.objectID is None:
        self.objectID = 0
      if self.objectName is None:
        self.objectName = ''
      if self.actionID is None:
        self.actionID = 0
      if self.actionName is None:
        self.actionName = ''
      if self.component_is_required is None:
        self.component_is_required = False
      if self.components is None:
        self.components = []
      if self.symbolic_is_required is None:
        self.symbolic_is_required = False
      if self.symbolicID is None:
        self.symbolicID = 0
      if self.symbolicName is None:
        self.symbolicName = ''
      if self.description is None:
        self.description = ''
    else:
      self.objectID = 0
      self.objectName = ''
      self.actionID = 0
      self.actionName = ''
      self.component_is_required = False
      self.components = []
      self.symbolic_is_required = False
      self.symbolicID = 0
      self.symbolicName = ''
      self.description = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      buff.write(_struct_I.pack(self.objectID))
      _x = self.objectName
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      buff.write(_struct_I.pack(self.actionID))
      _x = self.actionName
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      buff.write(_struct_B.pack(self.component_is_required))
      length = len(self.components)
      buff.write(_struct_I.pack(length))
      for val1 in self.components:
        buff.write(_struct_I.pack(val1.componentID))
        _x = val1.componentName
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      _x = self
      buff.write(_struct_BI.pack(_x.symbolic_is_required, _x.symbolicID))
      _x = self.symbolicName
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      _x = self.description
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 4
      (self.objectID,) = _struct_I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.objectName = str[start:end]
      start = end
      end += 4
      (self.actionID,) = _struct_I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.actionName = str[start:end]
      start = end
      end += 1
      (self.component_is_required,) = _struct_B.unpack(str[start:end])
      self.component_is_required = bool(self.component_is_required)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.components = []
      for i in range(0, length):
        val1 = srs_msgs.msg.Component()
        start = end
        end += 4
        (val1.componentID,) = _struct_I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.componentName = str[start:end]
        self.components.append(val1)
      _x = self
      start = end
      end += 5
      (_x.symbolic_is_required, _x.symbolicID,) = _struct_BI.unpack(str[start:end])
      self.symbolic_is_required = bool(self.symbolic_is_required)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.symbolicName = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.description = str[start:end]
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      buff.write(_struct_I.pack(self.objectID))
      _x = self.objectName
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      buff.write(_struct_I.pack(self.actionID))
      _x = self.actionName
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      buff.write(_struct_B.pack(self.component_is_required))
      length = len(self.components)
      buff.write(_struct_I.pack(length))
      for val1 in self.components:
        buff.write(_struct_I.pack(val1.componentID))
        _x = val1.componentName
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      _x = self
      buff.write(_struct_BI.pack(_x.symbolic_is_required, _x.symbolicID))
      _x = self.symbolicName
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
      _x = self.description
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x.encode()))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      start = end
      end += 4
      (self.objectID,) = _struct_I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.objectName = str[start:end]
      start = end
      end += 4
      (self.actionID,) = _struct_I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.actionName = str[start:end]
      start = end
      end += 1
      (self.component_is_required,) = _struct_B.unpack(str[start:end])
      self.component_is_required = bool(self.component_is_required)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.components = []
      for i in range(0, length):
        val1 = srs_msgs.msg.Component()
        start = end
        end += 4
        (val1.componentID,) = _struct_I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1.componentName = str[start:end]
        self.components.append(val1)
      _x = self
      start = end
      end += 5
      (_x.symbolic_is_required, _x.symbolicID,) = _struct_BI.unpack(str[start:end])
      self.symbolic_is_required = bool(self.symbolic_is_required)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.symbolicName = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.description = str[start:end]
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B = struct.Struct("<B")
_struct_BI = struct.Struct("<BI")
