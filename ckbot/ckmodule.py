"""
The ckmodule class defines the abstract superclass of all ckbot modules
and those Module subclasses that are entirely hardware independent,
such as GenericModule and MissingModule. In addition, it defines the
AbstractNodeAdaptor and AbstractProtocol classes, which are used for
organizational purposes to allow all Protocol-s and 
ProtocolNodeAdaptor-s to inherit from a single superclass

These classes were separated out of logical.py to allow non-CAN module
classes to inherit functionality from the Module base class.

The users of ckmodule are logical.py, which import *-s everything,
and python software modules which define Module subclasses.
"""

from sys import platform as SYS_PLATFORM, stdout
from time import time as now
from warnings import warn
from glob import glob as GLOB_GLOB
from os import sep, getenv

# Support functions ###################################################

_T0 = now()
def progress(msg):
  """Display a progress message"""
  stdout.write("%7.3f %s\n" % (now()-_T0,msg))
  stdout.flush()

# Library location specified by PYCKBOTPATH environment var
PYCKBOTPATH = getenv('PYCKBOTPATH','.')
if PYCKBOTPATH[-1:] != sep:
  PYCKBOTPATH=PYCKBOTPATH+sep
if not GLOB_GLOB(PYCKBOTPATH+"py%sckbot%sckmodule.py" % (sep,sep)):
  progress("WARNING: PYCKBOTPATH='%s' does not point to the right place" % PYCKBOTPATH )

# Organizational superclasses #########################################

class AbstractProtocol( object ):
  """abstract superclass of all Protocol classes
  
  Protocol classes are responsible for generating the Protocol Node 
  Adaptors for each module on the bus, when the bus is scanned. 
  Protocols implement the transactions that the underlying communication
  infrastructure supports, and typically maintain state representing any
  incomplete transactions. 
  
  AbstractProtocol subclasses must implement the following methods:
    p.update()
    p.hintNodes(nodes)
    p.generatePNA( nid )
  
  AbstractProtocol instances must have the following data attributes:
    p.heartbeats -- dict -- nid to last heartbeat
  """
  def __init__(self,bus=None):
    """
    Allow a bus parameter to be passed to all AbstractProtocol subclass
    constructors
    """
    self.heartbeats = {}

  def update( self ):
    """
    *PURE* perform Protocol housekeeping operations
    """
    raise TypeError,"pure method called"
    
  def hintNodes( self, nodes ):
    """
    *PURE* use hint that specified nodes are available
    """
    raise TypeError,"pure method called"
    
  def generatePNA( self, nid ):
    """
    *PURE* Generate a ProtocolNodeAdaptor for the specified nid
    """
    raise TypeError,"pure method called"    
  
class AbstractBus( object ):
  """abstract superclass of all Bus classes
  
     Bus classes are responsible for wrapping OS hardware drivers and
     presenting a pythonically correct, human readable interface.
     
     Bus instances should be (mostly) stateless, and bus methods should
     return as fast as possible.
  """
  def __init__(self,port=None):
    """
    Allow a port parameter to be passed to all AbstractBus subclass
    constructors
    """
    pass

class AbstractNodeAdaptor( object ):
  """abstract superclass of all ProtocolNodeAdaptor classes
  
  AbstractNodeAdaptor subclasses must implement the get_typecode()
  method, returning the module's type identification string
  """
  pass

class PermissionError( RuntimeError ):
  "(organizational) abstract superclass of all PermissionError classes"
  def __init__(self,*arg,**kw):
    RuntimeError.__init__(self,*arg,**kw)

class AbstractBusError( StandardError ):
  "(organizational) abstract superclass of all BusError classes"
  def __init__(self,*arg,**kw):
    StandardError.__init__(self,*arg,**kw)

class AbstractProtocolError( StandardError ):
  "(organizational) abstract superclass of all ProtocolError classes"
  def __init__(self,*arg,**kw):
    StandardError.__init__(self,*arg,**kw)

class AttributeGetter( object ):
  """
  Callable wrapper for providing access to object properties via
  a getter function
  
  AttributeGetter(obj,attr)() is getattr(obj,attr) 
  """
  def __init__(self,obj,attr):
    self.obj = obj
    self.attr = attr
  
  def __repr__( self ):
    return "<%s at 0x%x for %s of %s>" % (
      self.__class__.__name__, id(self), self.attr, repr(self.obj) )
    
  def __call__(self):
    return getattr(self.obj,self.attr)

class AttributeSetter( object ):
  """
  Callable wrapper for providing access to object properties via
  a setter function
  
  AttributeSetter(obj,attr)(value) is setattr(obj,attr,value) 
  """
  def __init__(self,obj,attr):
    self.obj = obj
    self.attr = attr
  
  def __repr__( self ):
    return "<%s at 0x%x for %s of %s>" % (
      self.__class__.__name__, id(self), self.attr, repr(self.obj) )
    
  def __call__(self,value):
    setattr(self.obj,self.attr,value)

class Module(object):
  """
  Abstract superclass representing a CKBot module.
  Cluster creates the appropriate Module subclass by calling
  Module.newFromDiscovery
  """
  # dictionary mapping module type-codes (as read from CAN via object 
  # dictionary to the appropriate Module subclass
  Types = {}

  @classmethod
  def newFromDiscovery(cls, nid, typecode, pna):
    """
    Factory method for instantiating Module subclasses given a node id and typecode
    INPUTS:
      nid -- int -- the node id
      typecode -- string -- the typecode string, which is looked up in
         the Module.Types dictionary
      pna -- a ProtocolNodeAdaptor -- typically created using a 
         Protocol instance's .generatePNA(nid) factory method
    """
    subclass = cls.Types.get(typecode,GenericModule)
    m = subclass(nid, typecode, pna)
    m.code_version = typecode
    return m

  def __init__(self, node_id, typecode, pna ):
    """
    Concrete constructor. 

    ATTRIBUTES:
      node_id -- 7 bit number to address a module uniquely
      typecode -- version number of module code 
      pna -- ProtocolNodeAdaptor -- specialized for this node_id
    """
    self.node_id = int(node_id)
    self.od = None
    self.code_version = None    
    assert pna is None or isinstance(pna,AbstractNodeAdaptor)
    assert pna is None or pna.nid == node_id 
    self.pna = pna
    self.name = None
    self._attr = {}
    #these lines added from mem classes
    self.mcu = None
    self.mem = None
    self.od = None
  
  def get_od(self):
    """
    This method creates an Object dictionary for this module. 
    """
    if self.od is None:
        self.od = self.pna.get_od(progress=progress)

  def iterhwaddr(self):
    """
    Iterator for all Object Dictionary index addresses in this module
    
    The addresses are returned as integers 
    """
    if self.od is None:
      return iter([])
    return self.od.index_table.iterkeys()
 
  def iterprop(self, perm=''):
    """
    Iterator for Object Dictionary properties exposed by this module
    
    INPUTS:
      perm -- string -- ''(default) all properties; 
            'R' readable/gettable
            'W' writable/settable
    OUTPUT:
      property names returned as strings
    """
    if self.od is None or perm not in 'RW':
      return iter([])
    idx = self.od.name_table.iterkeys()
    if perm=='R':
      return (nm for nm in idx if self.od.name_table[nm].isReadable())
    elif perm=='W':
      return (nm for nm in idx if self.od.name_table[nm].isWritable())    
    return idx
  
  def iterattr(self,perm=''):
    """
    Iterator for module attributes exposed by this module class
    
    INPUTS:
      perm -- string -- ''(default) all properties; 'R' readable/gettable
                'W' writable/settable
    OUTPUT:
      property names returned as strings
    """
    plan = self._attr.iteritems()
    if perm:
      return (nm for nm,acc in plan if perm in acc)
    return (nm for nm,_ in plan)
  
  def _getAttrProperty( self, prop, req ):
    """(private)
    Access python attributes exposed as properties 
    ('/@' property names)
    """
    def boolambda( b ):
      if b:
        return lambda : True
      return lambda : False
    # Check what access is provided to this attribute
    perm = self._attr.get(prop,None)
    if perm is None:
      raise KeyError("Property '@%s' was not found in class %s" 
                     % (prop,self.__class__.__name__) )        
    # Try to access attribute; generates error on failure
    val = getattr( self, prop )
    ## Attribute permissions:
    #   R -- readable; by default using AttributeGetter
    #   W -- writeable; by default using AttributeSetter
    #   1 -- method with no parameters, treated as a getter
    #   2 -- method with 1 parameter, treated as a setter
    if req == 'isReadable': return boolambda( "R" in perm )
    elif req == 'isWritable': return boolambda( "W" in perm )
    elif req == 'get_sync':
      if "1" in perm: return val
      if "R" in perm: return AttributeGetter(self,prop)
    elif req == 'set':
      if "2" in perm: return val
      if "W" in perm: return AttributeSetter(self,prop)
    raise TypeError("Property '@%s' does not provide '%s'" % (prop,req)) 
    
  def _getModAttrOfClp( self, clp, attr ):
    """
    Obtain a module attribute from a clp
    
    At the module level, the clp is expected to start with '/'
    
    This is the Module superclass implementation.
    clp-s starting with /@ expose module attrbiutes
    """
    assert clp[0]=="/", "Logical clp-s only"
    if clp[:2]=="/@":
      return self._getAttrProperty(clp[2:],attr) 
    # Must be an OD property access
    if self.od is None:
      raise KeyError('ObjectDictionary was not initialized; use .get_od() or populate(...,walk=1)')
    # Look for OD name
    odo = self.od.name_table[clp[1:]]
    return getattr( odo, attr )

  def start(self):
    """
    This method sends a CAN Message to start this module.
    """
    self.pna.start()

  def stop(self):
    """
    This method sends a CAN Message to stop this module.
    """
    self.pna.stop()

  def reset(self):
    """
    This method sends a CAN Message to reset this module.
    """
    self.pna.reset()

class AbstractServoModule( Module ):
  """
  Abstract superclass of servo modules. These support additional functionality 
  associated with position servos:

  .set_pos -- sets position of the module. -- units in 100s of degrees between 
    -9000 and 9000. This is a "safe" version, which only accepts legal values
  .set_pos_UNSAFE -- sets position of module, without any validity checking
  .get_pos -- reads current position (may be at offset from set_pos, even without load)
  .go_slack -- makes the servo go limp
  .is_slack -- return True if and only if module is slack
  """

  # Upper limit for positions
  POS_UPPER = 9000
  
  # Lower limit for positions
  POS_LOWER = -9000
    
  def __init__(self, *argv, **kwarg):
    Module.__init__(self, *argv, **kwarg)
    self._attr=dict(
      go_slack="1R",
      is_slack="1R",
      get_pos="1R",
      set_pos="2W",
      set_pos_UNSAFE="2W"      
    )

  def go_slack(self):
    """
    Makes the servo go limp
    """
    raise RuntimeError("PURE")

  def set_pos_UNSAFE(self, val):
    """
    Sets position of the module, without any validity checking
    
    INPUT:
      val -- units in 100s of degrees between -9000 and 9000
    """
    raise RuntimeError("PURE")
  
  def set_pos(self,val):
    """
    Sets position of the module, with safety checks.
    
    INPUT:
      val -- units in 100s of degrees between -9000 and 9000
    """
    raise RuntimeError("PURE")
  
   
  def get_pos(self):
    """
    Gets the actual position of the module
    """
    raise RuntimeError("PURE")

  def get_pos_async(self):
    """
    Asynchronously gets the actual position of the module
    """
    raise RuntimeError("PURE")

  def is_slack(self):
    """
    Gets the actual position of the module
    """
    raise RuntimeError("PURE")

class MemInterface( object ):
  """
  Implementation of the .mem sub-object for modules that provide the generic IO memory interface
  
  Instances of this class are created by the module subclass constructor, for those module software
  versions that actually implement the interface.

  Typical usage, assuming m is a module:
  >>> m.mem[0xBAD] = 0xC0ED
  >>> print m.mem[0xDEAD]
  >>> m.mem[[0xBAD, 0xD00D]] = [0xBE, 0xEF]
  >>> print m.mem[[0xC0ED, 0xBABE]]
  >>> m.mem[0xF00] |= (1<<5) # set bit 5
  >>> m.mem[0xF00] &= ~(1<<5) # clear bit 5
  >>> m.mem[0xF00] ^= (1<<5) # toggle bit 5
  
  If the module also has a .mcu sub-object, it will typically provide the constants needed instead of
  the literals in the example above.
  """
  def __init__(self, module):
    "Attach memory sub-object to this module"
    self.set = module.mem_write
    self.get = module.mem_read

  def __getitem__( self, key ):
    """
    Read memory from microcontroller.
    
    key can either be an integer memory address, or a list of such addresses.
    When a list is used, returns a list of the results
    """
    if type(key) is list:
      return [ self.get(k) for k in key ]
    #elif type(key) is tuple:
    #  addr, bit = key
    #  return (self.get(addr) & (1<<bit)) != 0
    return self.get(key)

  def __setitem__(self, key, val):
    """
    Write memory values in microcontroller
    
    key can be an address and val a byte value, or key is a list of addresses, and val a sequence of
    values (i.e. val can also be a tuple, or any other sequence type).
    """
    if type(key) is list:
      for k,v in zip(key,val):
        self[k]=v
    else:
      self.set(key,val)
    
class MemIxMixin( object ):
  """
  Module mixin class implementing memory interface

  This adds mem_read and mem_write methods, and a get_mem 
  """
       
  def mem_write( self, addr, val ):
     "Write a byte to a memory address in the module's microcontroller"
     self.pna.set( self.GIO_W_VAL, "B", val )
     self.pna.set( self.GIO_W_ADDR, "H", addr )

  def mem_read( self, addr ):
     "Read a memory address from the module's microncontroller"
     self.pna.set( self.GIO_R_ADDR, "H", addr )
     return self.pna.get_sync( self.GIO_R_VAL, "B" )
     
  def mem_getterOf( self, addr ):
     "Return a getter function for a memory address"
     # Create the closure
     def getter():
       return self.mem_read(addr)
     return getter
     
  def mem_setterOf( self, addr ):
     "Return a setter function for a memory address"
     # Create the closure
     def setter(val):
       return self.mem_write(addr,val)
     return setter

class GenericModule( Module ):
  """
  Concrete subclass of Module used for modules whose typecode was not 
  recognized
  """
  def __init__(self, nid, typecode , pna):
    Module.__init__(self,nid,typecode,pna)
    warn( 
      RuntimeWarning("CAN node 0x%02X reported unknown typecode '%s'. Falling back on GenericModule class"% (nid, str(typecode))) 
    )
    
class MissingModule( Module ):
  """
  Concrete class representing required modules that were not discovered during
  the call to Cluster.populate
  
  Instances of this class "fake" servo and motor modules, and implement the 
  MemIx interface.
  
  The .msg attribute may contain a callable. If present, this callable
  will be called with messages describing any API calls made on the 
  MissingModule. Typically, one may set: m.msg = ckbot.logical.progress
  
  """
  TYPECODE = "<<missing>>"
  def __init__(self, nid, name, pna=None):
    Module.__init__(self, nid, MissingModule.TYPECODE, None)
    self.mem = MemInterface(self)
    self.mcu = object()
    self._mem = {}
    self.od = None
    self.msg = None
    self.nid = nid
    self.name = name
    self.slack = True
    self.pos = 0
    self.speed = 0
    self._attr=dict(
      go_slack="1R",
      is_slack="1R",
      get_pos="1R",
      set_pos="2W",
      get_speed="1R",
      set_speed="2W",
    )
    
  def get_od( self ):
    """get_od is not supported on MissingModule-s"""
    pass
  
  def mem_read( self, addr ):
    val = self._mem.get(addr,None)
    if val is None:
      self._mem[addr] = 0
      val = 0
    if self.msg is not None:
      self.msg("%s.mem[0x%04X] --> %d" % (self.name,addr,val))  
    return val
  
  def mem_write( self, addr, val ):
    self._mem[addr] = val
    if self.msg is not None:
      self.msg("%s.mem[0x%04X] <-- %d" % (self.name,addr,val))  
  
  def go_slack(self):
    self.slack = True
    self.speed = 0
    if self.msg is not None:
      self.msg("%s.go_slack()" % self.name )
      
  def is_slack(self):
    if self.msg is not None:
      self.msg("%s.is_slack() --> %s" % (self.name,str(self.slack)))  
    return self.slack
 
  def set_pos( self, val ):
    self.slack = False
    self.pos = val
    if self.msg is not None:
      self.msg("%s.set_pos(%d)" % (self.name,val))  
  
  def set_speed( self, val ):
    self.slack = False
    self.speed = val
    if self.msg is not None:
      self.msg("%s.set_speed(%d)" % (self.name,val))  

  def set_torque( self, val ):
    self.slack = False
    self.speed = val
    if self.msg is not None:
      self.msg("%s.set_torque(%d) [alias for speed]" % (self.name,val))  
  
  def get_pos(self):
    if self.msg is not None:
      self.msg("%s.get_pos() --> %d" % (self.name,self.pos))  
    return self.pos
    
  def get_speed(self):
    if self.msg is not None:
      self.msg("%s.get_speed() --> %d" % (self.name,self.speed))  
    return self.speed

