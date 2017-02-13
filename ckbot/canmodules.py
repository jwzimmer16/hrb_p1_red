from ckmodule import Module, MemInterface, MemIxMixin

import P18F2680

def crop( val, lower, upper ):
  return max(min(val,upper),lower)

class MemAt0x1010( object ):
  GIO_R_ADDR = 0x1011
  GIO_R_VAL = 0x1012
  GIO_W_ADDR = 0x1021
  GIO_W_VAL = 0x1022

class MemAt0x1000( object ):
  GIO_W_ADDR = 0x1001
  GIO_R_VAL = 0x1004
  GIO_R_ADDR = 0x1003
  GIO_W_VAL = 0x1002

    
class SensorModule( Module ):
  """Abstract superclass for all sensor module classes
  
  Implements the .pins member, used to coordinate between ports regarding 
  pin usage.
  
  """
  def __init__(self, nid, typecode, pna):
      Module.__init__(self, nid, typecode, pna)
      self.pins = {}

class MCU_Port( object ):
  """Abstract superclass representing all MCU ports -- used or unused
  """
  def __init__(self,pins,**kw):
    """
    INPUTS:
      keyword parameters are assigned as attributes
    """
    for k,v in kw.iteritems():
      if hasattr(self,k):
        raise AttributeError("Attribute '%s' already exists" % k)
      setattr(self,k,v)
    self.need_pins = set(pins)
    self.active = False
  
  def use(self, pinass, pins=None):
    """
    INPUT:
      pinass -- dict -- table of pin assignments mapping pins to the port 
        object owning them
      pins -- set or None -- set of pins to actually take over; uses port 
        defaults if not specified
    OUTPUT:
      updates the value of pinass to reflect newly used pins
    """
    if pins is None:
      pins = self.need_pins
    used = set(pinass.keys())
    if not used.isdisjoint(pins):
      raise IndexError("Needed pins %s are not free" 
        % (used.intersection(pins) ) )
    for p in pins:
      pinass[p] = self
    self.active = True
    
class Pic18GPIO( MCU_Port ):
  """Concrete class representing a GPIO port on a PIC18 MCU
  
  Usage: assume m.gpio_A is a Pic18GPIO instance, and m has .mcu, .mem, .pins
  >>> m.gpio_A.use( m.pins )
  >>> m.gpio_A.set_inputs( 0xF0 )
  >>> m.gpio_A.put( 3 )
  >>> print m.gpio_A.get()
  """
  def __init__(self,pins,**kw):
    """
    INPUTS:
      keyword parameters are assigned as attributes
    """
    MCU_Port.__init__(self,pins,**kw)
    self.imask = "<<GENERATE ERROR FOR UNINITIALIZED MASK>>"

  def set_inputs( self, mask ):
    """
    INPUT:
      mask -- byte -- bit-mask of bits to be used as inputs
    """
    assert self.active,"Must call use() to activate this port first"
    mask = int(mask)
    if mask<0 or mask>255:
      raise ValueError("Mask 0x%x out of range -- must be a byte" % mask)
    self.imask = mask
    self._set_tris(mask)
        
  def get(self):
    """
    Reads GPIO pins and returns the value.
    
    The value of non-input pins is undefined
    """
    assert self.active,"Must call use() to activate this port first"
    return self._get_port()

  def put(self,val):
    """
    Writes a byte to a GPIO port.
    
    Only bits that were declared as non-inputs may be used.
    """
    assert self.active,"Must call use() to activate this port first"
    return self._set_latch(val & ~self.imask)
  
  def setterForMask(self,mask):
    """
    Returns a setter function that writes only specified bits
    """
    def setter(val):
      try:
        val = (val & mask) | (self._get_port() & ~mask)
        self._set_latch(val & ~self.imask)
      except TypeError,err:
        raise TypeError("%s : did you forget to call set_input()?" % str(err))
    return setter
    
class Sensor_v16( SensorModule, MemIxMixin ):
  """ Concrete class implementing a sensor module based on the Pic18 mcu
  
  Use the .gpio_A .gpio_B .gpio_C members to use the ports in GPIO modes.
  
  Typical usage:
  >>> m.gpio_A.use(m.pins)
  >>> m.set_input(0)
  >>> import time
  >>> while True:
  >>>   m.gpio_A.put(255)
  >>>   time.sleep(0.2)
  >>>   m.gpio_A.put(0)
  >>>   time.sleep(0.2)
  """
  def __init__(self, nid, typecode, pna):
     SensorModule.__init__(self, nid, typecode, pna)
     MemIxMixin.__init__(self)
     self.mcu = P18F2680
     self.mem = MemInterface( self )
     self.gpio_A = Pic18GPIO([2,3,4,5,6,7,9,10],
       _get_port = self.mem_getterOf(self.mcu.PORTA),
       _set_latch = self.mem_setterOf(self.mcu.LATA),
       _set_tris = self.mem_setterOf(self.mcu.TRISA)    
       )
     self.gpio_B = Pic18GPIO(xrange(21,29),
       _get_port = self.mem_getterOf(self.mcu.PORTB),
       _set_latch = self.mem_setterOf(self.mcu.LATB),
       _set_tris = self.mem_setterOf(self.mcu.TRISB)
       )
     self.gpio_C = Pic18GPIO(xrange(11,19),
       _get_port = self.mem_getterOf(self.mcu.PORTC),
       _set_latch = self.mem_setterOf(self.mcu.LATC),
       _set_tris = self.mem_setterOf(self.mcu.TRISC)        
       )            

class GenericIO( Sensor_v16, MemAt0x1000):
  pass

class SensorNode_v06( Sensor_v16, MemAt0x1010):
  pass

class ServoModule( Module ):
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

  # (pure) Process message ID for set_pos process messages
  PM_ID_POS = None 

  # (pure) Magic value that makes the servo go slack
  POS_SLACK = 9001
  
  # (pure) Object Dictionary index of current "encoder" position 
  ENC_POS_INDEX = None
  
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
    self.set_pos_UNSAFE(self.POS_SLACK)

  def set_pos_UNSAFE(self, val):
    """
    Sets position of the module, without any validity checking
    
    INPUT:
      val -- units in 100s of degrees between -9000 and 9000
    """
    val = int(val)
    self.pna.send_pm(self.PM_ID_POS, 'h', val)
  
  def set_pos(self,val):
    """
    Sets position of the module, with safety checks.
    
    INPUT:
      val -- units in 100s of degrees between -9000 and 9000
    """
    self.set_pos_UNSAFE(crop(val,self.POS_LOWER,self.POS_UPPER))
  
    
  def get_pos_PM(self):
    """
    Gets the actual position of the module via process messages.

    OUTPUT:
      val -- list of positions -- units in 100s of degrees between -9000 and 9000
      time -- list of timestamps --
    """
    pass
    #TODO

  def get_pos(self):
    """
    Gets the actual position of the module
    """
    return self.pna.get_sync(self.ENC_POS_INDEX, 'h')

  def get_pos_async(self):
    """
    Asynchronously gets the actual position of the module
    """
    return self.pna.get_async(self.ENC_POS_INDEX)

  def is_slack(self):
    """
    Gets the actual position of the module
    """
    pos = self.pna.get_sync(self.POS_INDEX, 'h')
    return pos == self.POS_SLACK
    
class V1_3Module( ServoModule ):
  """
  Concrete subclass of Module for V1.3 modules.
  """
  PM_ID_POS = 0x100
  PM_ID_ENC_POS = 0x200
  ENC_POS_INDEX = 0x1051
  POS_INDEX = 0x1050

class V1_4Module( ServoModule ):
  """
  Concrete subclass of Module for V1.4 modules.
  """
  PM_ID_POS = 0x100
  PM_ID_ENC_POS = 0x200
  ENC_POS_INDEX = 0x1051
  POS_INDEX = 0x1050
    
class MotorModule( Module ):
  """
  Abstract superclass of motor modules. These support additional functionality 
  associated with Motor Modules:

  .set_speed -- sets speed of the module -- units are RPM, range is +/-200
  """

  # (pure) Process message ID for set_speed process messages
  PM_ID_SPEED = None 

  # (pure) Upper limit for speed
  SPEED_UPPER = None
  
  # (pure) Lower limit for speed
  SPEED_LOWER = None
  
  # RPM to module speak conversion factor (x * RPM = -9000:9000)
  RPM_CONVERSION = 45

  def __init__(self, *argv, **kwarg):
    Module.__init__(self, *argv, **kwarg)
    self._attr=dict(
      set_speed="2W",
      go_slack="1R",
      set_speed_UNSAFE="2W"
    )

  def set_speed(self,val):
    """
    Sets speed of the module, with safety checks.
    
    INPUT:
      val -- units in between SPEED_LOWER and SPEED_UPPER
    """
    self.set_speed_UNSAFE(crop(val,self.SPEED_LOWER,self.SPEED_UPPER))
    
  def set_speed_UNSAFE(self, val):
    """
    Sets speed of the module, without any validity checking
    
    INPUT:
      val -- units in RPM  
    
    Do not use values outside the range SPEED_LOWER to SPEED_UPPER
    """
    # the 45 converts from RPM to +/- 9000 range the module expects
    val = int(self.RPM_CONVERSION*val)
    self.pna.send_pm(self.PM_ID_SPEED, 'h', val)

  def go_slack(self):
    """
    Makes the module go "slack": power down the motor.
    """
    # We send message directly because don't want RPM_CONVERSION to multiply it
    self.pna.send_pm(self.PM_ID_SPEED, 'h', self.SPEED_SLACK)
    
class ICRA_Motor_Module( MotorModule ):
  """
  Concrete subclass of Motor modules.
  """
  PM_ID_SPEED = 0x100
  SPEED_UPPER = 200
  SPEED_LOWER = -200
  SPEED_SLACK = 0
  
class Motor_Module_V1_0_MM( MotorModule ):
  """
  Concrete subclass of Motor modules.
  """
  PM_ID_SPEED = 0x100
  SPEED_UPPER = 9000/36
  SPEED_LOWER = -9000/36
  SPEED_SLACK = 9001
  RPM_CONVERSION = 36
  VEL_COMMAND_INDEX = 0x1030
  VEL_FEEDBACK_INDEX = 0x1031
  REL_POS_COMMAND_INDEX = 0x1032
  BRAKE_COMMAND_INDEX = 0x1033
  REL_POS_VELOCITY_CAP_INDEX = 0x1034
  RPM_FEEDBACK_INDEX = 0x1035
  
  def set_rel_pos(self, val):
    """
    Commands a relative position.
    
    INPUT:
        val -- units in between -32767 and 32767 (degrees * 100)
    """
    self.pna.set(self.REL_POS_COMMAND_INDEX, 'h', crop(val,-32767,32767))
    
  def set_servo_speed_cap_RPM(self, val):
    """
    Sets the maximum servo speed (speed when given a position command)
    
    INPUT:
        val -- RPM in between SPEED_LOWER and SPEED_UPPER
    """
    self.pna.set(self.REL_POS_VELOCITY_CAP_INDEX, 'h', crop(val*self.RPM_CONVERSION,self.SPEED_LOWER,self.SPEED_UPPER))
  
  def set_brake(self, val):
    """
    Sets the amount that the motor leads are tied together, therefore, braking
    
    INPUT:
      val -- number between 0 and 9000 where 9000 is always braking and 0 is never
    """
    self.pna.set(self.BRAKE_COMMAND_INDEX, 'h', crop(val,0,9000))
    
  def get_speed(self):
    """
    Gets the actual speed of the module in RPM
    """
    return self.pna.get_sync(self.RPM_FEEDBACK_INDEX, 'h')

  def get_speed_async(self):
    """
    Asynchronously gets the actual speed of the module in RPM
    """
    return self.pna.get_async(self.RPM_FEEDBACK_INDEX)
    
class IR_Node_Atx( Module, MemAt0x1010, MemIxMixin):
  """
    - IR Module Subclass. Has basic 'Module' functionality
        and should at some point have full memory access.
    - Will hopefully eventually have functions to deal with IR 
        communication and topology mapping. 
  """
  def __init__(self, nid, typecode, pna):
     Module.__init__(self, nid, typecode, pna)
     MemIxMixin.__init__(self)
     self.mcu = None
     self.mem = MemInterface( self )

def _register_types(tbl):
  #Register module types:
  tbl['V1.3'] = V1_3Module
  tbl['V1.4'] = V1_4Module
  tbl['mm'] =ICRA_Motor_Module
  tbl['V1.0-MM'] =Motor_Module_V1_0_MM
  tbl['GenericIO'] =GenericIO
  tbl['Sensor0.6'] =SensorNode_v06
  tbl['Sensor0.2'] =SensorNode_v06
  tbl['V0.1-ATX'] = IR_Node_Atx

