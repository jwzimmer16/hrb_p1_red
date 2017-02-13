"""
The CKBot.hitec python module provides classes used to communicate with CKBot robot modules connected the hitec HMI serial bus (rather than CAN) using a protocol similar to the Robotics Bus Protocol. These classes are primarily used to interface with the CKBot.logical python module found in the Modlab CKBot repository. For more information on
the low-level interface, please refer to the Hitec HMI Manual found at #J TODO.


Main uses of this module:
(*) control CKBot robot modules (specifically the servos) connected to a the hitec HMI serial bus
(*) mimics the behaviour of the CAN Bus, except using the hitec hitec HMI serial bus  as a communications channel

Example 1 - hitec robot module only: 
  #>>> import logical
  #>>> nodes = [0x01, 0x06]
  #>>> p = hitec.Protocol(nodes = nodes)
  #>>> p.send_cmd([0x02,7,0])

Example 2 - Integrate with CKBot.logical python module:
  >>> import logical
  >>> nodes = [0x01, 0x06, 0x03]
  >>> p = hitec.Protocol(nodes = nodes)
  >>> c = logical.Cluster(p)
  >>> c.populate(3,{0x01:'head', 0x06:'mid', 0x03:'tail'})
  >>> c.at.head.set_pos(1000)

Both examples sets the position of the 0xD1 robot module, to 1000 (10 degrees from the  neutral position)
"""

from time import time, sleep
from sys import platform as SYS_PLATFORM, stdout
from struct import pack, unpack
from array import array
from serial import Serial
from subprocess import Popen, PIPE
from random import uniform

from ckmodule import Module, AbstractNodeAdaptor, AbstractProtocol, AbstractProtocolError, AbstractBusError, AbstractBus, AbstractServoModule, progress
from port2port import newConnection

def crop( val, lower, upper ):
  return max(min(val,upper),lower)

class HiTec( object ):
  """Namespace containing useful constants and functions"""
  CMD_READ_EEPROM = 0xE1
  CMD_WRITE_EEPROM = 0xE2
  CMD_READ_MEMORY = 0XE3
  CMD_WRITE_MEMORY = 0XE4
  CMD_READ_POSITION = 0XE5
  CMD_SET_POSITION = 0XE6
  CMD_GET_ID = 0XE7
  CMD_GET_PULSEWITDTH_VOLTAGE = 0XE8
  CMD_SET_SERVO_SPEED = 0XE9
  CMD_SELECT_CNTRL_PARAMETER = 0XEA
  CMD_GO_STOP = 0XEB
  CMD_RELEASE = 0XEF
  CMD_SPEED = 0XFF
  ID_EADDR = 0x29
  EEPROM_LEN = 0x2D
  SYNC = 0x80
  #  Scale and offset for converting CKBot angles to and from hitec units 
  MAX_POS = 2450
  MIN_POS = 550
  MIN_ANG = -9000
  MAX_ANG = 9000
  SCL = float(MAX_POS - MIN_POS)/(MAX_ANG - MIN_ANG)
  OFS = (MAX_POS - MIN_POS)/2 + MIN_POS       
  SYNC_CHR = pack('B',SYNC)
  FRAME_SIZE = 7

  @classmethod
  def ang2hitec(cls, ang ):
    """Convert CKBot angle values to hitec units"""
    return int(ang * cls.SCL + cls.OFS)  
    
  @classmethod
  def hitec2ang(cls, hitec ):
    """Convert hitec units to CKBot angles"""
    return int((hitec-cls.OFS) / cls.SCL)  

class BusError( AbstractBusError ):
  def __init__(self,msg=""):
    AbstractBusError.__init__(self,"HITEC bus error : " + msg)

class ProtocolError( AbstractProtocolError ):
  def __init__(self,msg=""):
    AbstractProtocolError.__init__(self,"HITEC Bus error : "+msg)

class Bus(AbstractBus):
    """
    Concrete class that provides the functionality
    needed to send messages to a Hitec servos
    over their serial connection. 

    It is responsible for correctly formatting certain inputs
    to hitec-understandable formats as documented in the Hitec
    datasheet
    """
    """
    Utilizes the protocol along with nid to create 
    an interface for a specific module
    """
    def __init__(self, port = 'tty={ "baudrate":19200, "stopbits":2, "parity":"N", "timeout":0.5 }',*args,**kw):
      """ 
      Initialize hitec bus
      
      INPUT: 
      port -- serial port string, or None to autoconfig  
      
      ATTRIBUTES:
      ser -- serial handle 
      """
      AbstractBus.__init__(self,*args,**kw)
      self.ser = newConnection(port)
      # Initialize receive buffer
      self.rxbuf = ''
      ###self.stats = dict(out=0,inp=0,sync=0,retry=0,frame=0,fail=0,empty=0)

    def close(self):
      """
      Close communication with servo bus
      """
      self.ser.close()
      self.rxbuf = ''

    def reset(self):
      """
      Reset communication with servo bus
      ... note: this can be useful when serial fails after a power reset
      """
      self.ser.close()
      self.ser.open()
      self.rxbuf = ''
        
    def read( self ):
      """ read a single frame off the serial or returns None,None
      
      Makes sure that the framing and sync byte are in place.
      If a fragment is received -- stores it for future use, and 
      returns None,None
      """
      b = self.rxbuf
      SZ = HiTec.FRAME_SIZE
      # Precondition: len(b) < SZ
      while self.ser.inWaiting():
        b = b + self.ser.read( SZ - len(b) )
        # If not in sync --> skip a byte
        if not b.startswith(HiTec.SYNC_CHR):
          b = b[1:]
          ###self.stats['frame'] += 1
          continue
        # If message was not completed --> failed
        if len(b) < SZ:
          self.rxbuf = b
          ###self.stats['fail'] += 1
          return None, None
        assert len(b) == SZ
        break
      # Return the message
      self.rxbuf = ''
      if not b:
        ###self.stats['empty'] += 1
        return None, None
      ###self.stats['inp'] += 1
      return unpack('3B',b[1:4]), unpack('>1H',b[-2:])[0]
           
    def write(self, cmd, tx0, tx1 ):
      """Formats a message, sends it and returns a copy of header"""
      S = HiTec.SYNC
      chk = (256 - (S + cmd + tx0 + tx1)) & 0xFF
      msg =  pack('7B',S,cmd,tx0,tx1,chk,0,0)
      self.ser.write(msg)
      ###self.stats['out'] += 1
      return (cmd,tx0,tx1)
    
    def sync_write_read( self, pkt ):
      """Synchronously send packet and read (same length) response"""
      self.ser.flushInput()
      self.ser.flushOutput()
      N = len(pkt)
      self.ser.write(pkt)
      return self.ser.read(N)
      
    def send_cmd_sync( self, cmd, tx0, tx1, pause=0.01, retries=4 ):
      """Send a command in synchronous form, waiting for reply
      
      If reply times out -- retries until retry count runs out
      """
      for k in xrange(retries):
        hdr0 = self.write(cmd,tx0,tx1)
        sleep(pause)
        hdr1 = True
        while hdr1:
          hdr1, dat = self.read()
          if hdr1 == hdr0:
            ###self.stats['sync'] += 1
            return dat
        # fall through if no data in serial
        ###self.stats['retry'] += 1          
      # fall through if no more retries
      return None

class EEPROM( object ):
  """Concrete class representing the EEPROM in a servo"""
  
  def __init__(self, bus):
    self.mem = None
    self.bus = bus
    
  def readAll(self):
    if self.mem is None:
      self.mem = array('B',[0]*HiTec.EEPROM_LEN)  
    for addr in xrange(len(self.mem)):
      val = self._bus_get(addr)
      self.mem[addr] = val
    
  def _bus_put(self, addr, data):
    """(private)
    Write a value to servo EEPROM location.
    
    WARNING: does not update checksum!!! 
    INPUT:
      addr -- EEPROM address
      data -- Desired EEPROM value
    """
    self.bus.write( HiTec.CMD_WRITE_EEPROM ,addr, data )
  
  def _bus_get(self,addr, **kw):
    """(private)
    Read a value from servo EEPROM location.
    
    WARNING: this is a synchronous operation that may take many 
      milliseconds to complete
      
    INPUT:
      addr -- EEPROM address
      **kw -- passed on to Bus.send_cmd_sync
    """
    res = self.bus.send_cmd_sync( 
      HiTec.CMD_READ_EEPROM, addr, 0, **kw )
    if res is None:
      raise ProtocolError('Failed to read EEPROM address 0x%02x' % addr )
    if (res & 0xFF) != 0x03:
      print "!!! > expected 0x03 in 2nd byte, got 0x%02x at 0x%02x" % (res & 0xFF,addr)
    return (res >> 8) & 0xFF

  def __setitem__(self,addr,value):
    # Make sure we have cached EEPROM
    if self.mem is None:
      self.readAll()
    # perform operation
    self.mem[addr] = value
  
  def __getitem__(self,addr):
    # Make sure we have cached EEPROM
    if self.mem is None:
      self.readAll()
    # perform operation
    return self.mem[addr]
    
  def writeAll( self ):
    """
    Write EEPROM out into servo, validating as we go
    """
    # Make sure we have cached EEPROM
    if self.mem is None:
      self.readAll()
    # Compute checksum
    self.mem[-1] = 256 - (sum(self.mem[:-1]) & 0xFF)
    # Write
    for addr,val in enumerate(self.mem):
      self._bus_put( addr, val )
      if self._bus_get(addr) != val:
        raise BusError('Failed to write 0x%02x to EEPROM address 0x%02x' % (val, addr))
  


class IncompleteMessage(object):
  """ internal concrete class representing incomplete messages
  """
  def __init__(self, nid, msg, ts=None, retries=0 ):
    if ts is None:
      ts = time()
    self.nid = nid
    self.msg = msg
    self.ts = ts
    self.retries = retries
    self.promise = []
    
class Protocol(AbstractProtocol):
  """
  TBD
  """
  def __init__(self, bus=None, nodes=None,*args,**kw):
    """
    Initialize a hitec.Protocol
    
    INPUT:
    bus -- hitec.Bus -- Serial bus used to communicate with Hitec servos
    nodes -- list -- list of hitec ids
      
    ATTRIBUTES:
    heartbeats -- dictionary -- nid : (timestamp, last message)
    pnas -- dictionary -- table of NodeID to ProtocolNodeAdaptor mappings
    """
    AbstractProtocol.__init__(self,*args,**kw)
    if bus is None:
      self.bus = Bus()
    else:
      self.bus = bus

    self.heartbeats = {} # Gets populated by update
    self._incm = []
    self.pnas = {}
    self.ping_period = 3
    if nodes is None:
      progress("Scanning bus for nodes\n")
      nodes = self.scan()
    for nid in nodes:
      self.generatePNA(nid)
    progress("HiTec nodes: %s\n" % repr(list(nodes)))      
  
  def scan( self ):
    """Do a low-level scan for all nodes"""
    # Build a "scan all" packet
    def ping(nid):
      msg = [HiTec.SYNC,HiTec.CMD_SET_SERVO_SPEED,nid,255,0,0,0]
      msg[4] = 256 - (0xFF & sum(msg))
      return pack('7B',*msg)
    pall = "".join([ ping(x) for x in xrange(128) ])  

    # do the ping-all
    while True:
      res = self.bus.sync_write_read(pall)
      if len(res) == len(pall):
        break
      progress('Ping packet response was truncated (got %d not %d)-- retrying' % (len(res),len(pall)))
    #
    found = set()
    for k in xrange(128):
      if ord(res[k*7+5]) or ord(res[k*7+6]):
        found.add(k)
    return found
    
  def hintNodes( self, nodes ):
    n1 = set(nodes)
    n0 = set(self.pnas.keys())
    for nid in n0-n1:
      # corrupt the pna, just in case someone has access to it
      self.pnas[nid].nid = "<<INVALID>>"
      del self.pnas[nid]
    for nid in n1-n0:
      self.generatePNA(nid)
    
  def write( self, *args, **kw ):
    """Write out a command --> redirected to Bus.write"""
    return self.bus.write( *args, **kw )
  
  def read( self ):
    """Read a response; redirected to Bus.read """
    return self.bus.read()
  
  def send_cmd_sync( self, *argv, **kw ):
    return self.bus.send_cmd_sync( *argv, **kw )

  def request( self, cmd, tx0, tx1, nid=None, **kw ):
    """Write out a command and record incomplete reply
    
    return promise
    """
    hdr = self.bus.write( cmd, tx0, tx1, **kw )
    inc = IncompleteMessage( nid, hdr )
    self._incm.append( inc )
    return inc.promise
    
  def _completeResponses(self, now, timeout = 0.05, retries = 4):
    """(private)
    Retries another request of incomplete message if timed out, and
    completes any request for which a response arrived.
    
    After several retries puts a ProtocolError in the result

    INPUT:
      now -- current time
      timeout -- retry rate in seconds
      retries -- number of retries
    OUTPUT:
      num_failed -- number of failed requests
    """
    num_failed = 0
    next = []
    while self._incm:
      # Get the first incomplete msg
      inc = self._incm.pop(0)
      # If hasn't timed out --> keep
      if now - inc.ts < timeout:
        # find heartbeat; if not found -- put in the future
        ts,dat = self.heartbeats.get(inc.nid,[now+1,None])
        # if response is newer than request --> fulfill promise
        if now>ts:
          inc.promise[:] = (dat,)
        else: # otherwise --> leave request for later
          next.append(inc)
        continue
      # If we are out of retry attempts --> emit error
      if inc.retries >= retries:
        inc.promise[:] = [ProtocolError("Timeout on node 0x%x" % inc.nid)]
        num_failed += 1
        continue
      # assert: we should retry 
      self.bus.write( *inc.msg )
      inc.ts = now
      inc.retries += 1
      next.append(inc)
    # Store new list of incomplete messages
    self._incm = next
    return num_failed

  def _pingAsNeeded( self, now ):
    for nid,pna in self.pnas.iteritems():
      ts = self.heartbeats.get(nid,[0])[0] # we assume 0 is so far in past that timeout always occurs
      if (now - ts) > self.ping_period * uniform(0.75,1.0):
        pna.ping()
    
  def update(self):
    """
    Complete any outstanding async requests and collect heartbeats
    """
    now = time()
    
    # Only process for a timeslice at the longest
    timeslice = 0.05
    while time() - now < timeslice:
      msg, dat = self.read() 
      if msg is None:
        break
      # If was a set_speed --> may be a heartbeat / response
      cmd,nid,spd = msg
      if cmd == HiTec.CMD_SET_SERVO_SPEED and dat:
        self.heartbeats[nid] = (now, dat)
        
    # Clean / process any incomplete messages that can be completed
    self._completeResponses(now)
    self._pingAsNeeded(now)
    return len(self._incm)

  def generatePNA(self, nid):
    """
    Generates a pololu.ProtocolNodeAdaptor, associating a pololu protocol with 
    a specific node id and returns it
    """ 
    pna = ProtocolNodeAdaptor(self, nid)
    self.pnas[nid] = pna
    return pna

class ProtocolNodeAdaptor(AbstractNodeAdaptor):
  """
  Utilizes the protocol along with nid to create 
  an interface for a specific module
  """
  def __init__(self, protocol, nid = None):
    AbstractNodeAdaptor.__init__(self)
    self.p = protocol
    self.nid = nid
    self.eeprom = None
    self._speed = 0xFF
    
  def set_pos(self, tgt): #J: Need to modify this
    """
    Sends a position command
    """ 
    self.p.write(self.nid
      , (tgt >> 8) & 0xFF
      , tgt & 0xFF
    )

  def get_typecode( self ):  # Change this!!! ... looks at nid and returns string accordingly
    if self.nid < 64:
      return "HitecServoModule"
    else:
      return "HitecMotorModule"

  def _onlyOne(self):
    """(private) make sure there is only one servo on the bus
    """
    # Check if more than on module is on the bus
    if len(self.p.heartbeats) > 1:
      raise BusError('Operation is not allowed if more than one module is on the Bus')
    
  def getID(self):
    """
    Get current module ID ... note: only works with single module bus
    
    OUTPUT: 
      ret -- module ID
    """
    self._onlyOne()
    # Get current version and id        
    vid = self.p.send_cmd_sync(HiTec.CMD_GET_ID, 0x00, 0x00)
    ID = vid & 0xFF
    if self.nid is not None and ID != self.nid:
      raise BusError('getID returned ID 0x%02x, not 0x%02x' %(ID, self.nid))
    return ID

  def getEEPROM( self ):
    self._onlyOne()
    if self.eeprom is None:
      self.eeprom = EEPROM( self.p.bus )
      self.eeprom.readAll()
    return self.eeprom
      
  def setID(self, new_id):
    """
    Sets module ID ... note: only works with single module bus

    INPUT:
      new_id -- desired servo ID
    """
    new_id = int(new_id) & 0xFF
    ep = self.getEEPROM()
    ep[HiTec.ID_EADDR] = new_id
    ep.writeAll()
    self.nid = new_id

  def ping(self):
    """
    Send query to servo without having protocol expect a reply
    """
    self.p.write(HiTec.CMD_SET_SERVO_SPEED, self.nid, self._speed)

  def set_speed(self, speed):
    """
    Set speed of for servo and get position

    INPUT: 
      speed -- set speed

    """
    self._speed = speed
    self.p.write(HiTec.CMD_SET_SERVO_SPEED, self.nid, speed)

  def get_speed(self):
    """
    Get speed value for servo ... this just returns the last speed set,
       if that has not been set, it returns the default value: 0xFF
    
    OUTPUT: 
      ret -- servo speed between [0 - 255]
    """
    return self._speed
      
  def get_sync(self, tic=0.01):
    """
    Synchronous wrapper on top of get_async
    
    Returns the get_async result
    """
    promise = self.get_async()
    while not promise:
      sleep(tic)
      self.p.update()
    return self.async_parse(promise)

  @classmethod
  def async_parse(cls,promise,barf=True):
    """
    Parse the result of an asynchronous get.
    
    When barf is True, this may raise a deferred exception if the
    operation timed out or did not complete. Otherwise the exception
    object will be returned.
    """
    if not promise:
      raise ProtocolError("Asynchronous operation did not complete")
    dat = promise[0]
    if barf and isinstance(dat,Exception):
      raise dat
    return dat
    
  def get_async(self):
    """
    Send out an asynchronous get request

    OUTPUT:
    ret -- promise that will hold result after update()
    """
    return self.p.request( HiTec.CMD_SET_SERVO_SPEED
      , self.nid
      , self._speed
      , nid=self.nid
    )
            
  def go(self):
    """
    Set servo to go state 
      ... if module is in stop state it will immediately move to 
          position commanded
    """
    self.p.write( HiTec.CMD_GO_STOP, 0, 1 )
      
  def stop(self):
    """
    Set servo to stop state
       ... will freeze module motion until go state set or until power reset
    """
    self.p.write( HiTec.CMD_GO_STOP, 0, 0 )
      
  def release(self):
    """
    Release servo
      ... will turn on PID and make servo go slack
    """
    self.p.write(HiTec.CMD_RELEASE, 0x00, 0x00)
    
## Safety/range values goes in here

class HitecModule( Module ):
  def __init__(self, node_id, typecode, pna, *argv, **kwarg):
    Module.__init__(self, node_id, typecode, pna)    
    self._attr.update(
      go_slack="1R",
      is_slack="1R"
      )
    self.slack = False
    self.eeprom = None
    
  def getEEPROM( self ):
    """
    Access the module's EEPROM.
    
    WARNING: can only be used when there is a SINGLE servo on bus
    
    WARNING: data is only written when the .write() method is called
      on the .eeprom object. Without that, changes will be lost      
    """
    self.eeprom = self.pna.getEEPROM()

  def is_slack(self):
    """
    Returns true if the module is slack, none if go_slack has not been called yet.    
    """
    return self.slack

  def go_slack(self):
    """
    Sets *ALL* servos on the Bus slack
    """            
    # Send the command via the PNA
    self.pna.release()
    # Module should now be slack
    self.slack = True

class ServoModule( AbstractServoModule, HitecModule ):
  """
  TBD
  """
  def __init__(self, node_id, typecode, pna, *argv, **kwarg):
    AbstractServoModule.__init__(self, node_id, typecode, pna) 
    HitecModule.__init__(self, node_id, typecode, pna)    
    self._attr.update(
      set_pos="2W",
      set_pos_RAW="2W",
      set_speed="2W",
      get_speed="1R",
      get_pos="1R",
    )
    self.zero_ofs = 0.0
    self.gain = 1.0

  def set_pos(self, pos):
    """
    Set module position
    """
    self.set_pos_RAW(crop(HiTec.ang2hitec(self.zero_ofs+self.gain*pos),HiTec.MIN_POS,HiTec.MAX_POS))

  def set_pos_RAW(self, pos):
    """
    Set module position in native (HiTec) units
    
    WARNING: no checks!
    """
    # Send Position Command
    self.slack = False
    self.pna.set_pos(pos)

  def get_pos(self):
    """
    Get module position
    """
    return HiTec.hitec2ang(self.pna.get_sync())

  def set_speed(self, speed):
    """ 
    Set module speed
    """
    self.pna.set_speed(speed)

  def get_speed(self):
    """
    Return latest speed set to module, else return default speed
    """
    return self.pna.get_speed()

class MotorModule( HitecModule ):
  """
  TBD
  """
  SPEED_LOWER = -450 
  SPEED_UPPER = 450 
  RPM_CONVERSION = 1
  
  def __init__(self, node_id, typecode, pna, *argv, **kwarg):  
    HitecModule.__init__(self, node_id, typecode, pna)    
    self._attr.update(
      set_speed="2W",
      get_speed="1R",
      set_speed_UNSAFE="2W"
    )
    self._speed = 0;
    self.zero_pos = None
    self.promise = None

  def set_speed(self,val):
    """
    Sets speed of the module, with safety checks.
    
    INPUT:
      val -- units in between SPEED_LOWER and SPEED_UPPER
    """
    self.set_speed_UNSAFE(crop(val,self.SPEED_LOWER,self.SPEED_UPPER))

  def _ensure_zero_pos(self):
    """(private)
    
    Asynchronously request the zero position from the servo,
    and parse the response if it appeared.
    
    """
    # If no pot position was ever requested --> make async request
    if self.promise is None:
      self.promise = self.pna.get_async()
      return # without setting a zero_pos

    # If async request for pot position has not completed --> warn
    if not self.promise:
      progress("WARNING: 0x%02x get zero_pos has not completed\n" % self.node_id )
      return # without setting a zero_pos
    # Parse response
    pos = self.pna.async_parse(self.promise, False)
    # If failed --> retry
    if isinstance(pos,Exception):
      progress("WARNING: 0x%02x get zero_pos failed. Retrying...\n" % self.node_id )
      self.promise = self.pna.get_async()
      return # without setting a zero_pos
    #
    self.zero_pos = pos        
  
  def set_speed_UNSAFE(self, val):
    """
    Sets speed of the module, without any validity checking
    
    INPUT:
      val -- units in RPM  
    
    Do not use values outside the range SPEED_LOWER to SPEED_UPPER,
    otherwise the command may be ignored
    """
    # If motor is uncalibrated --> try to calibrate pot zero value
    if self.zero_pos is None:
      self._ensure_zero_pos()
    # If calibration isn't done --> skip this call, doing nothing
    if self.zero_pos is None:
      return  
    ofs = int(self.RPM_CONVERSION*val)
    self.pna.set_pos(self.zero_pos + ofs)

