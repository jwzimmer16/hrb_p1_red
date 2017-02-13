"""
-- The ckbot.dynamixel python module implements classes that communicate with CKBot nodes connected
   to a ( Robotis ) Dynamixel Single-Line Serial or Half-Duplex RS485 Bus. 

-- ckbot.dynamixel utilizes Bus and Protocol classes as an interface between the dynamixel hardware
   and protocol communication layers ( As specified in the Robotis E-manual http://support.robotis.com/en/ )

   ... talk about meminterface, pna, and module classes ...
-- Typical usage via ckbot.logical python module:
    >>> from ckbot import logical, dynamixel
    >>> nodes = {0x01:'head', 0x06:'mid', 0x03:'tail'}
    >>> c = logical.Cluster( dynamixel.Protocol(nodes = nodes.keys()) )
    >>> c.populate( names=nodes, required=nodes.keys() )
    >>> c.at.head.set_pos(1000)

-- Subclass/extend??

-- CKbot.dynamixel depends upon the following:
 - ckbot.ckmodule
   - Module
   - AbstractBus
   - AbstractNodeAdaptor
   - AbstractProtocol
   - AbstractServoModule
   - progress
 - ckbot.port2port
   - newConnection        
"""

from time import time as now, sleep
from sys import platform as SYS_PLATFORM, stdout
from struct import pack, unpack, calcsize
from array import array
from serial import Serial
from subprocess import Popen, PIPE
from glob import glob
from random import uniform
from collections import deque

from ckmodule import Module, AbstractNodeAdaptor, AbstractProtocol, AbstractBus, progress, AbstractServoModule, AbstractProtocolError, AbstractBusError, MemInterface
from port2port import newConnection

DEFAULT_PORT = dict(TYPE='tty', baudrate=115200, timeout=0.01)

class DynamixelServoError( AbstractBusError ):
  "(organizational) Error for Dynamixel Servo """
  def __init__(self,*arg,**kw):
    StandardError.__init__(self,*arg,**kw)

def crop( val, lower, upper ):
  return max(min(val,upper),lower)

DEBUG = []

class Dynamixel( object ):
  """ 
  DESCRIPTION: 
    -- The Dynamixel class is a namespace containing useful constants and functions 
  RESPONSIBILITIES: 
    -- Serve as a utility class with dynamixel commands and useful functions
  OWNERSHIP:
    -- owned by the dynamixel module
  THEORY: 
    -- none
  CONSTRAINTS: 
    -- none ... so far
  """
  CMD_PING = 0x01 #: from command table EX-106 section 3-2
  CMD_READ_DATA = 0x02 #: from command table EX-106 section 3-2
  CMD_WRITE_DATA = 0x03 #: from command table EX-106 section 3-2
  CMD_REG_WRITE = 0x04 #: from command table EX-106 section 3-2
  CMD_ACTION = 0x05 #: from command table EX-106 section 3-2
  CMD_RESET = 0x06 #: from command table EX-106 section 3-2
  CMD_SYNC_WRITE = 0x83 #: from command table EX-106 section 3-2
  MEM_LEN = 0x39 #: length of control table EX-106 section 3-4
  SYNC = '\xff\xff' #: synchronization pattern at start of packets EX-106 section 3-2
  MAX_ID = 0xFD #: maximal value of ID field EX-106 section 3-2
  BROADCAST_ID = 0xFE #: broadcast address EX-106 section 3-2
  
 
class DynamixelMemMap:
    """
    DESCRIPTION:
      Abstract superclass of all DynamixelMemMap classes. Provides
      shared functionality, while the concrete subclasses provide the
      actual memory map details.

      NOTE: this is intentionally an "old style class", to ensure that tab completion
        does not list a gazzilion irrelevant builtin methods.

    RESPONSIBILITIES: 
      -- Provide human readable names for addresses in the dynamixel control table
    OWNERSHIP:
      -- owned by the dynamixel module
    THEORY: 
      -- create a dictionary _ADDR_DCR_ that holds the memory location, name, and type (B, >H)
         generate a set of properties with the commmand name and it equivalent location
    """
    _ADDR_DCR = {
    '\x00' : ("model", "<H") # superclass only knows where to find model number
    }    
    MODEL_ADDR = pack('B',0) #: Address of model number, shared for all models
    MODEL_ADDR_LEN = 2 #: Length of model number

    @classmethod
    def _prepare( cls ):
      """ (protected)
      Set up all the _ADDR_DCR names as class attributes
      This classmethod must be called on every subclass after it is declared
      """
      for adr,(nm,fmt) in cls._ADDR_DCR.iteritems():
          setattr(cls,nm,adr)
DynamixelMemMap._prepare()

class MemMapOpsMixin:
    """
    Mixin class providing utility functions for operating on a MemMap.

    The .mm members of PNA-s use this mixin, whereas the .mcu members of modules do not
    """

    @classmethod
    def pkt2val( cls, addr, val ):
      """ Parse the message bytes returned when reading address addr """
      if isinstance(val,Exception):
        raise val
      return unpack( cls._ADDR_DCR[addr][1], val )[0]
    
    @classmethod
    def val2pkt( cls, addr, val ):
      """ Build the message bytes sent when writing address addr with value val"""
      return pack( cls._ADDR_DCR[addr][1], val )
    
    @classmethod
    def val2len( cls, addr ):
      """ Number of message bytes returned when writing address addr """
      return calcsize( cls._ADDR_DCR[addr][1] )
      
    @classmethod
    def show( cls, addr, val ):
      """ Provide human readable representation of a value from address addr"""
      nm,fmt = cls._ADDR_DCR[addr]
      return "%s = %d" % (nm, unpack(fmt, val)[0])

class MX64Mem( DynamixelMemMap):
    """ 
    DESCRIPTION: 
      -- The MX64Mem class provides a mapping of the dynamixel control table: 
         as per the MX-64 manual pp. 2-3, section title "Control Table"
    """
    _ADDR_DCR = {
    '\x00' : ("model", "<H"),
    '\x02' : ("version", "B"),
    '\x03' : ("ID", "B"),
    '\x04' : ("baud", "B"),
    '\x05' : ("ret_delay", "B"),
    '\x06' : ("cw_angle_limit", "<H"),
    '\x08' : ("ccw_angle_limit", "<H"),
    '\x0b' : ("max_temp", "B"),
    '\x0c' : ("min_voltage", "B"),
    '\x0d' : ("max_voltage", "B"),
    '\x0e' : ("max_torque", "<H"),
    '\x10' : ("status", "B"),
    '\x11' : ("alarm_LED", "B"),
    '\x12' : ("alarm_shutdown", "B"),
    '\x18' : ("torque_en", "B"),
    '\x19' : ("LED", "B"),
    '\x1a' : ("D_gain", "B"),
    '\x1b' : ("I_gain", "B"),
    '\x1c' : ("P_gain", "B"),
    '\x1e' : ("goal_position", "<H"),
    '\x20' : ("moving_speed", "<H"),
    '\x22' : ("torque_limit", "<H"),
    '\x24' : ("present_position", "<H"),
    '\x26' : ("present_speed", "<H"),
    '\x28' : ("present_load", "<H"),
    '\x2a' : ("present_voltage", "B"),
    '\x2b' : ("present_temperature", "B"),
    '\x2c' : ("registered_instruction", "B"),
    '\x2e' : ("moving", "B"),
    '\x2f' : ("lock", "B"),
    '\x30' : ("punch", "<H"),
    '\x44' : ("current", "<H"),
    '\x46' : ("torque_control_mode_enable", "B"),
    '\x47' : ("goal_torque", "<H"),
    '\x49': ("goal_acceleration", "B"),
    }
MX64Mem._prepare()

class MX28Mem( DynamixelMemMap):
    """ 
    DESCRIPTION: 
      -- The MX28Mem class provides a mapping of the dynamixel control table: 
         http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm
    """
    _ADDR_DCR = {
    '\x00' : ("model", "<H"),
    '\x02' : ("version", "B"),
    '\x03' : ("ID", "B"),
    '\x04' : ("baud", "B"),
    '\x05' : ("ret_delay", "B"),
    '\x06' : ("cw_angle_limit", "<H"),
    '\x08' : ("ccw_angle_limit", "<H"),
    '\x0b' : ("max_temp", "B"),
    '\x0c' : ("min_voltage", "B"),
    '\x0d' : ("max_voltage", "B"),
    '\x0e' : ("max_torque", "<H"),
    '\x10' : ("status", "B"),
    '\x11' : ("alarm_LED", "B"),
    '\x12' : ("alarm_shutdown", "B"),
    '\x14' : ("multi_turn_offset","<H"),
    '\x16' : ("resolution_divider","B"),
    '\x18' : ("torque_en", "B"),
    '\x19' : ("LED", "B"),
    '\x1a' : ("D_gain", "B"),
    '\x1b' : ("I_gain", "B"),
    '\x1c' : ("P_gain", "B"),
    '\x1e' : ("goal_position", "<H"),
    '\x20' : ("moving_speed", "<H"),
    '\x22' : ("torque_limit", "<H"),
    '\x24' : ("present_position", "<H"),
    '\x26' : ("present_speed", "<H"),
    '\x28' : ("present_load", "<H"),
    '\x2a' : ("present_voltage", "B"),
    '\x2b' : ("present_temperature", "B"),
    '\x2c' : ("registered", "B"),
    '\x2e' : ("moving", "B"),
    '\x2f' : ("lock", "B"),
    '\x30' : ("punch", "<H"),
    '\x49' : ("goal_acceleration", "B"),
    }
MX28Mem._prepare()
    
class RX64Mem(DynamixelMemMap):
    """ 
    DESCRIPTION: 
      -- The RX64Mem class provides a mapping of the dynamixel control table: 
         as per the RX-64 manual pp. 1-2, section title "Control Table"
    NOTES:
      -- Similar to EX106+ but lacks drive mode (for dual module joints) and current sensing.
    """
    _ADDR_DCR = {
    '\x00' : ("model", "<H"),
    '\x02' : ("version", "B"),
    '\x03' : ("ID", "B"),
    '\x04' : ("baud", "B"),
    '\x05' : ("ret_delay", "B"),
    '\x06' : ("cw_angle_limit", "<H"),
    '\x08' : ("ccw_angle_limit", "<H"),
    '\x0b' : ("max_temp", "B"),
    '\x0c' : ("min_voltage", "B"),
    '\x0d' : ("max_voltage", "B"),
    '\x0e' : ("max_torque", "<H"),
    '\x10' : ("status", "B"),
    '\x11' : ("alarm_LED", "B"),
    '\x12' : ("alarm_shutdown", "B"),
    '\x18' : ("torque_en", "B"),
    '\x19' : ("LED", "B"),
    '\x1a' : ("cw_compliance_margin", "B"),
    '\x1b' : ("ccw_compliance_margin", "B"),
    '\x1c' : ("cw_compliance_slope", "B"),
    '\x1d' : ("ccw_compliance_slope", "B"),
    '\x1e' : ("goal_position", "<H"),
    '\x20' : ("moving_speed", "<H"),
    '\x22' : ("torque_limit", "<H"),
    '\x24' : ("present_position", "<H"),
    '\x26' : ("present_speed", "<H"),
    '\x28' : ("present_load", "<H"),
    '\x2a' : ("present_voltage", "B"),
    '\x2b' : ("present_temperature", "B"),
    '\x2c' : ("registered_instruction", "B"),
    '\x2e' : ("moving", "B"),
    '\x2f' : ("lock", "B"),
    '\x30' : ("punch", "<H"),
    }
RX64Mem._prepare()

class EX106Mem( DynamixelMemMap ):
    """ 
    DESCRIPTION: 
      -- The EX106Mem class provides a mapping of the dynamixel control table: 
         as per EX-106 section 3-4 pp. 20
    CONSTRAINTS: 
      -- none???
    """
    _ADDR_DCR = {
    '\x00' : ("model", "<H"),
    '\x02' : ("version", "B"),
    '\x03' : ("ID", "B"),
    '\x04' : ("baud", "B"),
    '\x05' : ("ret_delay", "B"),
    '\x06' : ("cw_angle_limit", "<H"),
    '\x08' : ("ccw_angle_limit", "<H"),
    '\x0a' : ("drive_mode", "B"),
    '\x0b' : ("max_temp", "B"),
    '\x0c' : ("min_voltage", "B"),
    '\x0d' : ("max_voltage", "B"),
    '\x0e' : ("max_torque", "<H"),
    '\x10' : ("status", "B"),
    '\x11' : ("alarm_LED", "B"),
    '\x12' : ("alarm_shutdown", "B"),
    '\x18' : ("torque_en", "B"),
    '\x19' : ("LED", "B"),
    '\x1a' : ("cw_compliance_margin", "B"),
    '\x1b' : ("ccw_compliance_margin", "B"),
    '\x1c' : ("cw_compliance_slope", "B"),
    '\x1d' : ("ccw_compliance_slope", "B"),
    '\x1e' : ("goal_position", "<H"),
    '\x20' : ("moving_speed", "<H"),
    '\x22' : ("torque_limit", "<H"),
    '\x24' : ("present_position", "<H"),
    '\x26' : ("present_speed", "<H"),
    '\x28' : ("present_load", "<H"),
    '\x2a' : ("present_voltage", "B"),
    '\x2b' : ("present_temperature", "B"),
    '\x2c' : ("registered_instruction", "B"),
    '\x2e' : ("moving", "B"),
    '\x2f' : ("lock", "B"),
    '\x30' : ("punch", "<H"),
    '\x38' : ("sense_current","<H"),
    }
EX106Mem._prepare()

class DynamixelMemWithOps( DynamixelMemMap, MemMapOpsMixin ):
  memMapParent = DynamixelMemMap

class EX106MemWithOps( EX106Mem, MemMapOpsMixin ):
  memMapParent = EX106Mem
  
class RX64MemWithOps( RX64Mem, MemMapOpsMixin ):
  memMapParent = RX64Mem
  
class MX64MemWithOps( MX64Mem, MemMapOpsMixin ):
  memMapParent = MX64Mem

class MX28MemWithOps( MX28Mem, MemMapOpsMixin ):
  memMapParent = MX28Mem

class AX12MemWithOps( RX64Mem, MemMapOpsMixin ):
  """ 
    -- The AX-12 control table in
       http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm
       is identical to that of the RX64, so we re-use that implementation          
  """
  # NOTE: the table is identical to that of the RX64
  memMapParent = RX64Mem

class Bus( AbstractBus ):
    """ ( concrete )
    DESCRIPTION: 
      -- The Bus class serves as an interface between the dynamixel RS485 bus and Protocol
    RESPONSIBILITIES: 
      -- intialize a serial newConnection at the desired baudrate. default: 1000000
      -- format packets into dynamixel understanble messages
      -- write packets
      -- read in valid packets, throw out others
         - maintain error counts from read attempts
      -- provide syncronous write then read capability
    OWNERSHIP:
      -- owned by the dynamixel module
    THEORY: 
      -- builds and writes Dynamixel EX packets as per EX-106 section 3-2 pp. 16
      -- reads packets and parses for valid ones as per EX-106 section 3-3 pp. 17
    CONSTRAINTS: 
      -- requires valid baudrate setting for communication
      -- writes and reads limited by pyserial->termios 
    """
    def __init__(self, port=None,*args,**kw):
        """
        Initialize Dynamixel Bus
      
        INPUT: 
          port -- serial port string or None to autoconfig
 
        ATTRIBUTES:
          ser -- serial handle
          buf -- buffer string '
          expect -- number of expected bytes. default: 6
          eSync -- number of SYNC errors
          eChksum -- number of checksum errors  
          eLen -- number of length errors   
          eID -- number of ID errors 
          count -- a count of bytes received  
          rxPkts -- a count of valid packets received
          txPkts -- a count of packets sent
        """
        AbstractBus.__init__(self,*args,**kw)
        if port is None:
          port = DEFAULT_PORT
        self.ser = newConnection(port)
        self.DEBUG = DEBUG
        self.reset()

    def reconnect( self, **changes ):
        # Close the connection and try the new baudrate
        self.ser.close()
        spec = self.ser.newConnection_spec
        spec.update( changes )
        self.ser = newConnection( spec )
        if not self.ser.isOpen():
          raise ValueError('Could not open newly configured connection')

    def getSupportedBaudrates(self):
        """
        List of supported baud rates
        """
        return self.ser.getSupportedBaudrates()

    def reset( self ):
        """
        The purpose of this method is to reset the state of the Bus to initial values
        """
        self.buf = ''
        self.expect = 6
        self.eSync = 0
        self.eChksum = 0
        self.eLen = 0
        self.eID = 0
        self.count = 0
        self.rxPkts = 0
        self.rxEcho = 0
        self.txPkts = 0
        self.txBytes = 0
        self.suppress = {}
        self.ser.flush()

    def flush( self ):
        """
        Flush software and hardware buffers 
        """
        self.buf = ''
        self.ser.flush()
        if 'x' in self.DEBUG:
          progress('[Dynamixel] flush bus\n')

    def statsMsg( self ):
        """ 
        returns the current Bus statistics 

        OUTPUTS:
          -- list -- strings indicating current counts, error, etc ...
        """
        return [
            'bytes in %d' % self.count,
            'bytes out %d' % self.txBytes,
            'packets out %d' % self.txPkts,
            'packets in %d' % self.rxPkts,
            'echos in %d' % self.rxEcho,
            'sync errors %d' % self.eSync,
            'id errors %d' % self.eID,            
            'length errors %d' % self.eLen, 
            'crc errors %d' % self.eChksum,
            'buffer %d expecting %d' % (len(self.buf),self.expect)
            ]
    
    def reconnect( self, **changes ):
        # Close the connection and try the new baudrate
        self.ser.close()
        spec = self.ser.newConnection_spec
        spec.update( changes )
        self.ser = newConnection( spec )
        if not self.ser.isOpen():
          raise ValueError('Could not open newly configured connection')
    
    @classmethod
    def _chksum( cls, dat ):
        """(private)
        Compute checksum as per EX-106 section 3-2 pp. 17  
        
        INPUTS:
          dat -- string -- data from which to compute checksum
        OUTPUTS:
          return -- int -- value of checksum between 0 and 0xFF
        
        THEORY OF OPERATION:
          Compute checksum using not bit operator for only the lowest
          byte of the sum
        """
        return 0xFF ^ (0xFF & sum([ ord(c) for c in dat ]))

    @classmethod
    def _parseErr( cls, pkt ):
        """ (private)
        Parse and output error returned from dynamixel servos as per 3-4-1 pp. 25
        """
        nid = ord(pkt[0])
        err = ord(pkt[2])
        msg = ["ID 0x%02x" % nid]
        if err & 0x01:
          msg.append("Voltage out of operating range")
        if err & 0x02:
          msg.append("Angular position out of range")
        if err & 0x04:
          msg.append("Overheating")
        if err & 0x08:
          msg.append("Command out of range")
        if err & 0x10:
          msg.append("Received message with checksum error")
        if err & 0x20:
          msg.append("Current load cannot be controlled with set maximum torque")
        if err & 0x40:
          msg.append("Unknown instruction in message")
        if len(msg)>1:
          raise DynamixelServoError("; ".join(msg))
        return 

    def _dropByte( self, error=None ):
        """ (private)
        Drop a single byte from the input buffer .buf and reset .expect
        If error is provided, increment that error counter
        """
        self.buf = self.buf[1:]
        self.expect = 6
        if error is not None:
          setattr(self,error,getattr(self,error)+1)
    
    @classmethod
    def dump( cls, msg ):
        """
        Parses a message into human readable form
        INPUT:
          msg -- one or more message in a string
        OUTPUT:
          the messages in human-readable form
        """
        out = []
        b = [ ord(m) for m in msg ]
        while len(b)>6:
          if b[0] != 0xff or b[1] != 0xff:
            out.append("<<bad SYNC %02X %02X>>" % (b[0],b[1]))
          else:
            out.append("<SYNC>")
          b = b[2:]
          out.append("nid = 0x%02x" % b[0])
          l = b[1]
          if l+1>len(b):
            out.append("<<bad len = %d total %d>>" % (l,len(b)))
            break
          out.append("len = %d" % l)
          out.append({ 0x02:"read", 0x03:"write", 0x01:"ping",
                       0x04:"REG WRITE", 0x05:"ACTION", 0x06:"RESET",
                       0x83:"SYNC WRITE" 
              }.get(b[2],"<<CMD 0x%02x>>" % b[2]))
          # Sync write is parsed differently
          if b[2]!=0x83:
            out.append(" ".join("%02X" % bb for bb in b[3:l+1]) )
          else:
            out.append('addr = 0x%02X for %d' % (b[3],b[4]))
            for k in xrange(5,l,3):
              out.append('nid %02X %02X %02X' % tuple(b[k:k+3]))
          chk = sum(b[:l+1])
          if 0xFF & (b[l+1]+chk) != 0xFF:
            out.append("<<bad CHK %02X, need %02X>>" %(b[l+1],0xFF^chk))
          else:
            out.append("<CHK>")
          # go to next message
          b=b[l+2:]
        return " ".join(out)
          
    def _testForEcho( self, pkt ):
        SWEEP_EVERY = 100
        tbl = self.suppress
        n = self.txPkts
        # Is it time to sweep out old packets from the echo suppression?
        if (n % SWEEP_EVERY) == 0:
          # If so, find packets that are older than SWEEP_EVERY
          #   and remove them from the table
          for key in tbl.keys():
            if tbl[key] < n - SWEEP_EVERY:
              del tbl[key]
        # should packet be suppressed?
        if tbl.has_key(pkt):
          # yes; don't suppress if seen again
          del tbl[pkt]
          return True
        return False
        
    def recv( self, maxlen=10 ):
        """
        reads a single packet off of the RS485 or serial bus 

        INPUTS:
          None
        OUTPUTS:
          pkt -- string -- valid packet from Dynamixel hardware bus
        PRECONDITIONS:
          existence of serial object
          reset() has been called at least once
        POSTCONDITIONS:
          pkt is returned given that all values of packet check OUTPUTS
        
        THEORY OF OPERATION:
          Parse for valid packets as per EX-106 section 3-3 pp. 17
          - While there are bytes to be read from serial buffer or from buf
            - Read in expected number of bytes if available
            - Check for start frame, otherwise drop byte
            - Check for valid ID, otherwise drop byte
            - Check for valid Length, otherwise drop byte
            - Once Length bytes arrive, check if CRC is valid, otherwise drop byte
          - If there are no more bytes to read or parse then return None
        """
        SYNC = Dynamixel.SYNC
        MAX_ID = Dynamixel.MAX_ID        
        assert self.expect >= 6
        while len(self.buf)+self.ser.inWaiting()>=self.expect:
            # If we don't have at least self.expect bytes --> nothing to do
            # if len(self.buf)+self.ser.inWaiting()<self.expect:
            #  return None
            rd = self.ser.read( self.ser.inWaiting() )
            self.buf += rd
            self.count += len(rd)
            # Expecting sync byte
            if not self.buf.startswith(SYNC):
                # --> didn't find sync; drop the first byte
                self._dropByte('eSync')
                continue
            ## assert self.buf.startswith(SYNC)
            # Make sure that our nid makes sense
            ID = ord(self.buf[2])
            if ID > MAX_ID: # Where 0xFD is the maximum ID 
                self._dropByte('eID')
                continue
            # Pull out the length byte and compute total length
            L = 4+ord(self.buf[3])
            # Ensure that length is within valid range. 6 is minimum possible packet length
            if L > maxlen or L < 6: 
              self._dropByte('eLen')
              continue
            if len(self.buf)<L:
              self.expect = L
              continue
            assert len(self.buf)>=L
            chk = self._chksum( self.buf[2:L-1] )
            #print ">>"," ".join([ "%02x" % (ord(b)) for b in self.buf[:6] ]),"chk",hex(chk),hex(ord(self.buf[L-1])),L
            if ord(self.buf[L-1]) != chk:
              self._dropByte('eChksum')
              continue
            # At this point we have a packet that passed the checksum in 
            #   positions buf[:L+1]. We peel the wrapper and return the 
            #   payload portion
            pkt = self.buf[2:L-1]
            fl_pkt = self.buf[:L]
            self.buf = self.buf[L:]
            # Check if this is an echo
            if self._testForEcho(fl_pkt):
              self.rxEcho += 1
              if 'x' in self.DEBUG:
                progress('[Dynamixel] recv echo --> %s\n' % repr(fl_pkt))
              continue
            self.rxPkts += 1
            if 'x' in self.DEBUG:
              progress('[Dynamixel] recv --> %s\n' % repr(fl_pkt))
            # Run error check
            self._parseErr(pkt)
            return pkt
        # ends parsing loop
        # Function terminates returning a valid packet payload or None
        return None 

    def send( self, nid, cmd, pars = '' ):
        """
        Build a message and  transmit, update rxPkts, and return message string
        
        INPUTS:
          nid -- int -- node ID of module
          cmd -- int -- command value. ie: CMD_PING, CMD_WRITE_DATA, etc ...
          pars -- string -- parameter string
        OUTPUTS: 
          msg -- string -- transmitted packet minus sync          
        PRECONDITIONS:
          existance of serial object        
        POSTCONDITIONS:
          None
        
        THEORY OF OPERATION:
          Assemble packet as per EX-106 section 3-2 pp. 16,
          Compute checksum, transmit and then return string

        """
        body = pack('BBB',nid,len(pars)+2,cmd)+pars  
        msg = Dynamixel.SYNC+body+pack("B",self._chksum(body))
        if 'x' in self.DEBUG:
          progress('[Dynamixel] send --> %s\n' % repr(msg))
        self.ser.write(msg)
        self.txPkts+=1
        self.txBytes+=len(msg)
        self.suppress[msg] = self.txPkts
        return msg[2:]

    def send_sync_write( self, nid, addr, pars ):
        """
        Build a SYNC_WRITE message writing a value
        
        INPUTS:
          nid -- int -- node ID of module
          addr -- string -- address
          pars -- string -- value (after marshalling)
        OUTPUTS: 
          msg -- string -- transmitted packet minus sync          
        PRECONDITIONS:
          existance of serial object        
        POSTCONDITIONS:
          None
        
        THEORY OF OPERATION:
          Set a value remotely, without expecting a reply
        """
        body = ( pack('BBB', Dynamixel.BROADCAST_ID, len(pars)+5, Dynamixel.CMD_SYNC_WRITE)
                +addr
                +pack('BB', len(pars),nid)
                +pars
           ) 
        msg = Dynamixel.SYNC+body+pack("B",self._chksum(body))
        if 'x' in self.DEBUG:
          progress('[Dynamixel] sync_write --> %s\n' % repr(msg))
        self.ser.write(msg)
        self.suppress[msg] = self.txPkts
        self.txPkts+=1
        self.txBytes+=len(msg)
        return msg[2:]

    def ping( self, nid ):
        """ 
        Send a low level ping command to a given nid 

        INPUTS:
          nid -- int -- node ID of module
        OUTPUTS: 
          None 
        PRECONDITIONS:
          instantiation of dynamixel.Bus
        POSTCONDITIONS:
          No response occurs 
        
        THEORY OF OPERATION:
          Send CMD_PING for given nid as per section 3-5-5 pp. 37
        """
        return self.send(nid, Dynamixel.CMD_PING)

    def reset_nid( self, nid ):
        """ 
        Send a low level reset command to a given nid 

        INPUTS:
          nid -- int -- node ID of servo to reset 
        
        THEORY OF OPERATION:
          Send CMD_RESET for given nid as per section 3-5-6 pp. 38
          Resets all values in the servo's control table to factory
          defaults, INCLUDING the ID, which is reset to 0x01. 

          By convention, we reserve ID 0x01 for configuration purposes
        """
        return self.write(nid, Dynamixel.CMD_RESET)
    
    def send_cmd_sync( self, nid, cmd, pars, timeout=0.1, retries=5 ):
        """
        Send a command in synchronous form, waiting for reply
      
        INPUTS:
          nid, cmd, pars -- same as for .send()
          timeout -- float-- maximal wait per retry
          retries -- number of allowed retries until giving up
          rate -- wait time between retries
        OUTPUTS: 
          pkt -- string -- response packet's payload          

        THEORY OF OPERATION:
          Plan out sending times for all retries. When a retry goal time is passed
          sends out the message. In between, poll for a reply for that message at a
          hard coded rate (currently 100Hz)
        """
        if nid==Dynamixel.BROADCAST_ID:
          raise ValueError('Broadcasts get no replies -- cannot send_cmd_sync')
        for k in xrange(retries+1):
          hdr0 = self.send(nid, cmd, pars)[0]
          t0 = now()
          # Put a sensible amount of time to wait for a response in
          sleep( 0.003 )
          while now()-t0<timeout:
            pkt = self.recv()
            # If read() got nothing --> we sleep a little
            if pkt is None:
              sleep( 0.001 )
            # If got reply --> done
            elif pkt[0]==hdr0:
              if 't' in self.DEBUG:
                progress("[Dynamixel] send_cmd_sync dt: %2.5f " % (now()-t0))
                progress("[Dynamixel] send_cmd_sync send attempts: %d " % (k+1))                 
              return pkt
            # if neither condition was true, we read as fast as we can
        return None
    
    @staticmethod
    def splitReply( reply ):
      """
      TODO
      """
      return unpack('BBB',reply[:3])+(reply[3:],)
      
class ProtocolError( AbstractProtocolError ):
  def __init__(self,*arg, **kw):
    AbstractProtocolError.__init__(self,*arg, **kw)
    
class ProtocolNodeAdaptor( AbstractNodeAdaptor ):
    def __init__(self, protocol, nid = None, mm=DynamixelMemWithOps):
        AbstractNodeAdaptor.__init__(self)        
        self.p = protocol
        self.nid = nid
        self.mm = mm
        self.model = None
        self._adaptMem()
    
    def _adaptMem( self ):
        """
        Read the typecode and update the mm sub-object to one of the appropriate type
        """
        tc = self.get_typecode()
        try:
          self.mm = (MODELS[tc][0])
        except KeyError, ke:
          raise KeyError('Unknown module typecode "%s"' % tc)  
            
    def mem_write_fast( self, addr, val ):
        """ 
        Send a memory write command expecting no response. 
        The write occurs immediately once called. 
  
        INPUTS:
          addr -- char -- address
          val -- int -- value
        OUTPUTS: 
          msg -- string -- transmitted packet minus SYNC
        
        THEORY OF OPERATION:
          Call write command with BROADCAST_ID as id, SYNC_WRITE as cmd, and params 
          as ( start_address, length_of_data, nid, data1, data2, ... ) as per 
          section 3-5-7 pp. 39
        """
        return self.p.mem_write( self.nid, addr, self.mm.val2pkt( addr, val ) )

    def mem_write_sync( self, addr, val ):
        """ 
        Send a memory write command and wait for response, returning it
  
        INPUTS:
          addr -- char -- address
          val -- int -- value
        OUTPUTS: 
          msg -- string -- transmitted packet minus SYNC        
        """
        return self.p.mem_write_sync( self.nid, addr, self.mm.val2pkt( addr, val ))

    def mem_read_sync( self, addr ):
        """
        TODO
        """
        reply = self.p.mem_read_sync( self.nid, addr, self.mm.val2len(addr) )
        return self.mm.pkt2val( addr, reply )

    def get_typecode( self ):
        """
        TODO
        """
        if self.model is not None:
          return self.model
        mdl = self.mem_read_sync( self.mm.model )
        if isinstance(mdl,Exception):
          raise mdl
        self.model = "Dynamixel-%04x" % mdl
        return self.model

    def mem_read_async( self, addr ):
        """
        Send a request memory read command expecting response later after p.upate() is called
  
        INPUTS:
          addr -- char -- address

        OUTPUTS: 
          promise -- list -- promise returned from request
        """
        return self.p.request( self.nid, Dynamixel.CMD_READ_DATA, addr+pack('B',self.mm.val2len(addr)))

    def mem_write_async( self, addr, val ):
        """
        Send a request memory write command expecting response later after p.update() is called
  
        INPUTS:
          addr -- char -- address
          val -- int -- value to write 

        OUTPUTS: 
          promise -- list -- promise returned from request
        """        
        return self.p.request( self.nid, Dynamixel.CMD_WRITE_DATA, addr+self.mm.val2pkt( addr, val ))

    @classmethod
    def async_parse(cls,promise,barf=True):
        """
        Parse the result of an asynchronous get.
        INPUTS:
          promise -- list -- promise to parse for
          barf -- bool -- throw exception or not

        OUTPUTs:
          dat -- string -- data, ProtocolError, or None returned from promise

        THEORY OF OPERATION:
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

    def get_voltage( self ):
        """
        Get present voltage on bus as read by servo

        OUTPUTS:
        -- voltage -- int -- volts
        THEORY OF OPERATION:
        -- mem_read the present_voltage register and convert
        """      
        return self.dynamixel2voltage(self.mem_read(self.mm.present_voltage))

    def get_temperature( self ):
        """
        Get present temperature of servo

        OUTPUTS:
        -- temperature -- int -- degrees celsius
        THEORY OF OPERATION:
        -- mem_read the present_temperature register
        """
        return self.mem_read(self.mm.present_temperature)

    def get_load( self ):
        """
        Get present load

        OUTPUTS:
        -- load -- int -- a ratio of max torque, where max torque is 0x3FF, needs to be determined!!! -U
        THEORY OF OPERATION:
        -- mem_read the present_load register
        """      
        return self.dynamixel2voltage(self.mem_read(self.mm.present_load))
               
class Request(object):
    """ ( internal concrete )
    DESCRIPTION:
    -- The Request message class represents holds information about a message
       request
    RESPONSIBILITIES:
    -- hold nid, cmd, and params of message
    -- keep track of retries remaining
    -- hold a timestamp 
    -- hold a promise to be updated after dynamixel.Protocol.update()
    OWNERSHIP:
    -- owned by the dynamixel module
    THEORY:
    -- An Request object serves a way to keep track of requests. In the case
       of the dynamixel.Protocol it is appended to a queue and handled at update
    CONSTRAINTS:
    -- Request objects are specific to those messages handled by the 
       dynamixel.Protocol and dynamixel.Bus
    """
    def __init__(self, nid, cmd, pars='',  ts=None, tout=0.05, attempts=4 ):
        if ts is None:
            ts = now()
        self.nid = nid
        self.cmd = cmd
        self.pars = pars 
        self.ts = ts
        self.tout = tout
        self.attempts = attempts
        self.promise = []

    def setError( self, msg ):
      self.promise[:] = [ProtocolError("%s on node 0x%x" % (msg,self.nid))]
    
    def setResponse( self, dat ):
      self.promise[:] = [dat]

    def sendArgs( self ):
      return self.nid, self.cmd, self.pars
      
class Protocol( AbstractProtocol ):
    """ ( concrete )
    DESCRIPTION: 
      -- The Protocol class provides the functionality needed to send messages using the
         Dynamixel Protocol as per section 3 of the EX-106 document
    RESPONSIBILITIES: 
      -- detect modules on the bus and generate NodeAdaptors for each one
      -- ping modules periodically to ensure that they are still on the Bus??
      -- store a queue of pending messages and handle them for during each update 
         for a given timeslice
      -- act as a abstraction layer for basic Bus functionality?
    OWNERSHIP:
      -- owned by the Cluster
    THEORY: 
      -- The protocol owns a dictionary of the NodeAdaptor of each module 
         It initially checks for modules by performing a scan for all nodes
         that the user states should be on the Bus and for each module
         that exists it updates the heartbeat dictionary entry for that node
         
         A heartbeat, is a simple indicator of node existing, holding only a 
         timestamp and the error received.
 
         Message transmission and reception is based upon the following ideas:
         - If asked, Dynamixel servos always send a response
         - Dynamixel servos have no Bus arbitration, therefore the response
           to a request always follows the request
         
         - There are three types of requests 
           - Writes that require an acknowledgement         
           - Reads that require an acknowledgement
           - Writes that do not require an acknowledgement

         The aformentioned ideas led to the following decisions: 
         - Messages should be placed in a queue and handled in order of requests
         - Message that do not require a response should be sent using the 
           "SYNC_WRITE" command as per section 3-5-7 pp. 39 in the EX-106 document
           and no response should be expected.
         - Messages that require a response should be sent and listened for. If a 
           response is not found, a number of retries should occur to accomodate 
           for errors

         A basic representation of the update() algorithm follows:

           get the allowed timeslice
           while len queue > 0: 
             pop message from queue
             if message has timed out:
               declare message promise as Err
             if allowed timeslice is used up: 
               append message to front of queue and exit
             if the message is a broadcast message:
               if ping:
                 warn that broadcast pings are not allowed outside of scan             
               else:
                 send message through Bus, and continue to next message
             else:
               for number of retries remaining in message:
                 if allowed timeslice is used up:
                   append message to front of queue and exit 
                 send message through Bus
                 while have not reached timeout or timeslice is used up:
                   read in packet
                   if packet exists:
                     fulfill message promise, and continue to next message
                   sleep so that we don't overwork the OS
               if no more retries remaining: 
                 declare message promise as Err

    CONSTRAINTS:
      -- a single write when handled by update must have only a single response
      -- it is expected that a Dynamixel Bus has been instantiated as shown in the
         Dynamixel Module Examples   
    """
    def __init__(self, bus=None, nodes=None, *args,**kw):
        """ 
        Initialize a Dynamixel Protocol
      
        INPUTS:
          bus -- object -- dynamixel bus object used for basic communication with dynamixel servos
          nodes -- list -- list of expected dynamixel servo IDs 

        ATTRIBUTES:
          pnas -- dictionary -- dict of NodeAdaptor objects, used for interfacing with Dynamixel Module
          heartbeats -- dictionary -- dict of node heartbeats. nid : (timestamp, err)
          requests -- deque object -- queue of pending requests <<< maybe should just be a list?  
          ping_period -- float -- expected ping period 
      
        """
        AbstractProtocol.__init__(self,*args,**kw)
        if bus is None:
            self.bus = Bus()
        else:
            self.bus = bus
        self.reset(nodes)
        
    def reset( self, nodes=None, ping_rate=1.0 ):
        """
        Reset the Protocol object 
        
        INPUTS:
          ping_rate -- float -- rate at which to ping nodes 
        OUTPUTS: 
          None
        THEORY OF OPERATION:
          scan for nodes, add each node to the pollRing and generate a ProtocolNodeAdaptor
        """
        self.pnas = {}
        self.heartbeats = {}
        self.requests = deque()
        self.pollRing = deque()
        self.ping_rate = ping_rate
        if nodes is None:
            progress("Scanning bus for nodes \n")
            nodes = self.scan()
        for nid in nodes:
            self.pollRing.append( nid )
            self.generatePNA( nid )
        progress("Dynamixel nodes: %s\n" % repr(list(nodes)))

    def scan( self, timeout=1.0, retries=1, get_model=True ):
        """
        Build a broadcast ping message and then read for responses, and if 
          the get_model flag is set, then get model numbers for all existing 
          nodes
        
        INPUTS:
          timeout -- float -- amount of time allowed to scan
          retries -- int -- number of times allowed to retry scan  
          get_model -- bool -- flag determining if node model numbers 
                               should be read to indicate the servo
                               type                                
        OUTPUTS: 
          found -- dict -- map from servo IDs that were discovered
            to their model numbers, or None if get_model is False 
         
        THEORY OF OPERATION:
          Assemble packet as per EX-106 section 3-2 pp. 16, using 
          PING Command as per section 3-5-5 pp. 37 and broadcast ID
          Send this packet, then read responses and add to set  until 
          timeout and number of retries.
        """
        found = []
        for retry in xrange(0, retries):
            self.bus.flush()
            self.bus.send(Dynamixel.BROADCAST_ID, Dynamixel.CMD_PING)
            t0 = now()
            while now()-t0 < float(timeout)/retries:
                pkt = self.bus.recv()
                if pkt is not None:
                    nid = unpack('B',pkt[0])[0]
                    found.append(nid)
                sleep(0.01)
        res={}
        for nid in found:
          if get_model:
            res[nid] = self.mem_read_sync( nid, DynamixelMemMap.MODEL_ADDR, DynamixelMemMap.MODEL_ADDR_LEN)
          else:
            res[nid] = None
        return res

    def hintNodes( self, nodes ):
        """
        Generate NodeAdaptors for nodes that are not already in pnas 
        
        INPUTS:
          nodes -- list -- list of expected nodes 
        
        THEORY OF OPERATION:
          Get all node IDs that are in nodes but not in pnas and 
          generate a NodeAdaptors for those nodes
        """
        n1 = set(nodes)
        n0 = set(self.pnas.keys())
        for nid in n0-n1:
            # corrupt the pna, just in case someone has access to it
            self.pnas[nid].nid = "<<INVALID>>"
            del self.pnas[nid]
        for nid in n1-n0:
            self.generatePNA(nid)

    def generatePNA( self, nid):
        """
        Generate NodeAdaptors for given Node ID
        
        INPUTS:
          nid -- int -- ID of node to generate ProtocolNodeAdaptor object for
        OUTPUTS: 
          pna -- object -- ProtocolNodeAdaptor object for given nid 
        
        THEORY OF OPERATION:
          Instantiate ProtocolNodeAdaptor add to pnas dictionary 
        """
        pna = ProtocolNodeAdaptor(self, nid)
        self.pnas[nid] = pna
        return pna                   

    def off( self ):
      """
      Turn all servos off ... see Bus.off
      """
      self.bus.off()

    def mem_write( self, nid, addr, pars ):
        """ 
        Send a memory write command and don't wait for a response
  
        INPUTS:
          addr -- char -- address
          val -- int -- value
          pars -- string -- parameters
        OUTPUTS: 
          msg -- string -- transmitted packet minus SYNC        
        """
        return self.bus.send_sync_write( nid, addr, pars ) 
        
    def mem_write_sync( self, nid, addr, pars ):
        """ 
        Send a memory write command and wait for response, returning it
  
        INPUTS:
          addr -- char -- address
          val -- int -- value
          pars -- string -- parameters
        OUTPUTS: 
          msg -- string -- transmitted packet minus SYNC        
        """
        return self.bus.send_cmd_sync( nid, Dynamixel.CMD_WRITE_DATA, addr+pars ) 
  
    def mem_read_sync( self, nid, addr, length, retries=4 ):
      """
        A Protocol level syncronous memory read wrapper around bus.send_cmd_sync
            
        INPUTS:
          nid -- int -- node ID
          addr -- string hex -- address location to read from
          length -- int -- number of bytes to read
          retries -- int -- number of times to retry if there exist protocol errors
      """
      for retry in xrange(0,retries):
          reply = self.bus.send_cmd_sync( nid, Dynamixel.CMD_READ_DATA, addr+pack('B', length))
          if reply is None:
            return ProtocolError("NID 0x%02x mem_read[0x%02x] timed out" 
              % (nid, ord(addr)))
          res = self.bus.splitReply(reply)[-1]
          if len(res) != length:
            if retry < retries:
                  continue
            return ProtocolError("NID 0x%02x mem_read[0x%02x] result length mismatch. Reply was %s" % (nid,ord(addr),repr(reply)))
          return res

    def request( self, nid, cmd, pars='', lifetime=0.05,  **kw ):
        """
        Send a response request by creating an incomplete messsage
        and appending it to the queue for handling by update

        INPUTS:
          nid -- int -- node ID of module
          cmd -- int -- command value. ie: CMD_PING, CMD_WRITE_DATA, etc ...
          pars -- string -- parameter string
        OUTPUTS: 
          promise -- list -- promise of incomplete message, will be fulfilled upon 
                             successful read
        PRECONDITIONS:
          None?
        POSTCONDITIONS:
          None?
        
        THEORY OF OPERATION:
          Create an incomplete message for a given request that holds the 
          packet to be written and has a number of other attributes ( see 
          Request ) Then append to Protocols queue for later handling
          and return a promise that will be fulfilled by a successful read 
          during Protocol.update()
        """
        inc = Request( nid, cmd, pars, tout=lifetime)
        self.requests.append( inc )
        return inc.promise

    def _get_heartbeats( self, now ):
        """ 
        Use pollRing to poll all nodes that haven't been seen for
        self.ping_rate seconds with ping packets.
        
        INPUTS:
          now -- float -- current time
        """
        nxt = deque()
        while self.pollRing:
          nid = self.pollRing.popleft()
          nxt.append(nid)
          if now-self.heartbeats.get( nid, (0,) )[0]>self.ping_rate:
            self.request( nid, Dynamixel.CMD_PING )
            break
        self.pollRing.extend(nxt)
        
    def update( self, timeout=0.01 ):
        """ 
        The update method handles current incomplete message requests and ensures that 
        nodes are pinged to check for their existance

        INPUTS:
          None
        OUTPUTS: 
          num_requests -- int -- length of request queue 
        PRECONDITIONS:
          instantiation of dynamixel.Bus, dynamixel.Protocol
        POSTCONDITIONS:
          No response occurs 
        
        THEORY OF OPERATION:
          get the allowed timeslice
          while timeslice not done:
            if queue empty --> return
            pop 1st message on queue
            if message has timed out: 
              fill promise with ProtocolError(timeout)
              continue
            if message ID is BROADCAST_ID:
              if message cmd not CMD_SYNC_WRITE:
                bus.send the message
                fill promise with None
              else:
                fill promise with ProtocolError(not CMD_SYNC_WRITE bcast not allowed) 
              continue
            else:
              bus.send_cmd_sync the message
              store reply in promise 
        """
        t0 = now()
        self._get_heartbeats(t0)
        t1 = t0
        while (t1-t0 < timeout) and self.requests:
            t1 = now()
            self._doRequest( t0, self.requests.popleft() )
        # Parse any remaining messages in buffer, generating
        #   exceptions as needed
        while (t1-t0 < timeout):
            if not self.bus.recv():
              break
            
    
    def _doRequest(self, now, inc ):
        """
        Process a single incomplete request
        If it requires a response -- complete it or time-out; if not,
        complete with a None reply
        
        TODO finish comment
        """
        if inc.nid == Dynamixel.BROADCAST_ID:
            if cmd != Dynamixel.CMD_SYNC_WRITE:
                self.bus.send(*inc.sendArgs())
                inc.setResponse(None)
            else:
                inc.setError("broadcast allowed only for CMD_SYNC_WRITE")
        else:
            reply = self.bus.send_cmd_sync(*inc.sendArgs())
            if reply is not None and not isinstance(reply,Exception):
                self.heartbeats[inc.nid] = (now, reply)
            inc.setResponse(reply)

class DynamixelModule( AbstractServoModule ):
    """ concrete class DynamixelModule provides shared capabilities of 
    Dynamixel modules. It is usually used as the base class in a mixin,
    with the model specific details provided by the mixin classes.
    """
    
    MX_POS =  10000 #: maximal position value for servos
    MN_POS = -10000 #: minimal position value for servos
            
    @classmethod
    def ang2dynamixel(cls, ang):
        """ Convert angle values to Dynamixel units """
        return int(ang * cls.SCL + cls.OFS)
  
    @classmethod 
    def dynamixel2ang(cls, dynamixel ):
        """Convert dynamixel units to Dynamixel angles"""
        return int((dynamixel-cls.OFS) / cls.SCL)  
    
    @classmethod
    def torque2dynamixel(cls, torque ):
        """ Convert "torque" commands to Dynamixel commands """
        if torque < 0:
            return int(abs(torque)*cls.TORQUE_SCL) | cls.DIRECTION_BIT
        else:
            return int(torque*cls.TORQUE_SCL)
    
    @classmethod
    def dynamixel2rpm(cls, dynamixel ):
        """Convert dynamixel units to rotations per minute"""
        # There is some sort of weird nonesense going on in the readings
        direction = dynamixel >> 10
        if direction == 0:
          return (dynamixel & 0x3FF)*cls.SPEED_SCL
        return -(dynamixel & 0x3FF)*cls.SPEED_SCL
  
    @classmethod
    def rpm2dynamixel(cls, rpm ):
        """Convert rpm to dynamixel units"""
        return int(rpm/cls.SPEED_SCL)+1
  
    @classmethod
    def dynamixel2voltage(cls, dynamixel ):
        """Convert dynamixel units to volts """
        return dynamixel*cls.VOLTAGE_SCL
      
    def __init__( self, node_id, typecode, pna ):
        """
        Concrete constructor. 

        ATTRIBUTES:
        node_id -- int -- address for a unique  module 
        typecode -- version name of module  
        pna -- ProtocolNodeAdaptor -- specialized for this node_id
        """
        AbstractServoModule.__init__(self, node_id, typecode, pna) 
        self.mcu = self.pna.mm.memMapParent
        self.mem = MemInterface( self )
        self._attr.update(
          set_pos_sync="2W",
          set_trim="2W",
          get_trim="1R",
          set_speed="2W",
          set_speed_sync="2W",
          set_torque="2W",
          get_speed="1R",
          get_voltage="1R",
          get_temperature="1R",
          get_load="1R",
          mem_write="2W",
          mem_read="1R",
          get_mode="1R",
          set_mode="2W",
          go_slack="2W"
        )
        self.get_mode()

    def set( self, **args ):
        """Syntactic sugar for setting a bunch of addresses in the dynamixel control table to new values

        Keyword arguments may be any of the addresses defined in the .mcu member, e.g. for EX106
        
        TODO: example
        """
        for nm,val in args.iteritems():
            if hasattr( self.mcu,nm ):
                self.mem_write( getattr( self.mcu, nm ), val )
            else:
                raise KeyError("Unknown module address '%s'" % nm)

    def mem_write( self, addr, val ):
        "Write a byte to a memory address in the module's microcontroller"
        self.pna.mem_write_sync(addr, val)

    def mem_read( self, addr ):
        "Read a memory address from the module's microncontroller"
        return self.pna.mem_read_sync( addr )
     
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

    def get_mode( self ):
        """
        Get the current mode of the dynamixel servo, either Motor or Servo

        OUTPUTS: 
          mode -- string -- 1 if Motor; 0 if Servo 
        """      
        cw_limit = self.pna.mem_read_sync(self.mcu.cw_angle_limit)
        ccw_limit = self.pna.mem_read_sync(self.mcu.ccw_angle_limit)
        if cw_limit==0 and ccw_limit==0:
            self.mode = 1
        else:
            self.mode = 0
        return self.mode

    def set_mode( self, mode, min_pos = MN_POS, max_pos = MX_POS ):
        """ ( MAYBE )
        Set the current mode of the servo, either Motor or Servo 

        INPUTS:
          mode -- string -- either Motor or Servo 
          pos_upper -- int -- upper range of servo 
          pos_lower -- int -- lower range of servo          
        """
        if mode in [0,'Servo']:
            pos_upper = self.ang2dynamixel(self.MX_POS)
            pos_lower = self.ang2dynamixel(self.MN_POS)
            self.mode = 0
        elif mode in [1,'Motor']:
            pos_upper = 0
            pos_lower = 0
            self.mode = 1
        else:
            raise ValueError("Unknown mode %s requested" % repr(mode)) 
        self.mem_write(self.mcu.ccw_angle_limit, pos_upper)
        self.mem_write(self.mcu.cw_angle_limit, pos_lower)

    def start( self, null=None ):
        """
        Enable module actuation
        
        THEORY OF OPERATION:
        -- set torque_en  as per section 3-4-2 pp. 27
        """
        return self.mem_write(self.mcu.torque_en, 1)

    def stop( self ):
        """
        Disable module actuation
        
        THEORY OF OPERATION:
        -- clear torque_en. as per section 3-4-2 pp. 27
        """
        return self.mem_write(self.mcu.torque_en, 0)
    
    def go_slack( self , null=None):
        """
        Disable module actuation
        
        THEORY OF OPERATION:
        -- clear torque_en. as per section 3-4-2 pp. 27
        """
        return self.mem_write(self.mcu.torque_en, 0)

    def get_pos(self):
        """
        Gets the actual position of the module
        """
        dxl = self.mem_read(self.mcu.present_position)
        #TODO: convert to units of deg/100
        return self.dynamixel2ang(dxl)

    def get_pos_async(self):
        """
        Returns None, indicating that this functionality is not supported for this module class
        """
        return None 
        
    def set_pos(self,val):
        """
        Sets position of the module, with safety checks.
    
        INPUT:
          val -- units in 1/100s of degrees between -10000 and 10000
        """
        return self.pna.mem_write_fast(self.mcu.goal_position, 
          self.ang2dynamixel(val))

    def set_pos_sync(self,val):
        """
        Sets position of the module, with safety checks.
    
        INPUT:
          val -- units in 1/100s of degrees between -10000 and 10000
        """
        return self.mem_write(self.mcu.goal_position, 
          self.ang2dynamixel(val))


    def get_speed( self ):
        """
        Get present speed of servo

        OUTPUTS:
        -- speed -- float -- speed in rpm
        THEORY OF OPERATION:
        -- mem_read the present_speed register and convert
        """
        spd = self.mem_read(self.mcu.present_speed)
        rpm = self.dynamixel2rpm(spd)
        return rpm 

    def get_moving_speed( self ):
        """
        Get the moving speed (set in set_speed) of servo

        OUTPUTS:
        -- speed -- float -- speed in rpm
        THEORY OF OPERATION:
        -- mem_read the moving_speed register and convert
        """
        spd = self.mem_read(self.mcu.moving_speed)
        rpm = self.dynamixel2rpm(spd)
        return rpm 

    def set_torque(self,val):
        """
        Sets torque of the module, with safety checks.
    
        INPUT:
          val -- units in between -1.0 - 1.0
        """
        if self.mode == 0:
          self.get_mode()
          if self.mode == 0:
            raise TypeError('set_torque not allowed for modules in Servo')
        val = self.torque2dynamixel(val)
        return self.pna.mem_write_fast(self.mcu.moving_speed, val)

    def set_torque_limit( self,val ):
      """
      Sets the maximum torque limit of the module
      INPUT:
          val -- unit from 0.0 to 1.0 where 1.0 is the maximum torque
      """

      if val > 1.0:
        val = 1.0
      elif val < 0.0:
        val = 0.0
      val = int(val*1023)
      return self.mem_write( self.mcu.torque_limit, val )
      

    def set_speed(self,val):
        """
        Sets speed of the module, with safety checks.
    
        INPUT:
          val -- units in rpm from -114 to 114
        """
        if self.mode == 1:
            raise TypeError('set_speed not allowed for modules in CR mode')
        val = self.rpm2dynamixel(val)
        return self.mem_write(self.mcu.moving_speed, val)

    def get_voltage( self ):
        """
        Get present voltage on bus as read by servo

        OUTPUTS:
        -- voltage -- int -- volts
        THEORY OF OPERATION:
        -- mem_read the present_voltage register and convert
        """      
        return self.dynamixel2voltage(self.mem_read(self.mcu.present_voltage))

class MX64Module(DynamixelModule ):
    """
    DESCRIPTION: 
    -- MX64 Specific constants
    RESPONSIBILITIES: 
    -- ...
    """
    #  Scale and offset for converting CKBot angles to and from dynamixel as per MX-64 e-Manual
    MAX_POS = 0xFFF
    MIN_POS = 0
    MIN_ANG = 0 #Min, max angle are listed in centidegrees based on values from the manual
    MAX_ANG = 36000
       
    # Scaling for Dynamixel continuous turn torque 
    MAX_TORQUE = 0x3FF
    DIRECTION_BIT = 1<<10
    TORQUE_SCL = float(MAX_TORQUE/1.0)
    
    # Scaling for Dynamixel speed 
    SPEED_SCL = 0.114
    # Scaling for Dynamixel voltage
    VOLTAGE_SCL = 0.1
    
    SCL = float(MAX_POS - MIN_POS)/(MAX_ANG - MIN_ANG)
    OFS = (MAX_POS - MIN_POS)/2 + MIN_POS


    def __init__( self, node_id, typecode, pna ): 
        """
        """
        DynamixelModule.__init__( self, node_id, typecode, pna )
        self.reset_xpos()
  
    def reset_xpos(self):
        """
        Reset rotation count for the extended position read by get_xpos()
        """
        self.lastPos = 0
        self.xrot = 0
      
    def get_xpos(self):
        """
        Get extended position 
        
        Returns position as floating point in units of rotation.
        This can count an indefinite number of rotations, and may be
        used in Motor mode to get position
        """
        pos = float(self.get_pos()-self.MIN_ANG)/self.MAX_ANG
        if abs(pos-self.lastPos)>0.5:
          if pos<self.lastPos:
            self.xrot += 1
          else:
            self.xrot -= 1
        self.lastPos = pos
        return self.xrot + pos

class MX28Module(DynamixelModule ):
    """
    DESCRIPTION: 
    -- MX28 Specific constants
    
    NOTE: some features are NOT supported
      (*) baudrates above 2.25Mbps
      (*) multi-turn mode
      (*) resolution divider
    """
    #  Scale and offset for converting CKBot angles to and from dynamixel as per MX-64 e-Manual
    MAX_POS = 0xFFF
    MIN_POS = 0
    MIN_ANG = 0 #Min, max angle are listed in centidegrees based on values from the manual
    MAX_ANG = 36000
       
    # Scaling for Dynamixel continuous turn torque 
    MAX_TORQUE = 0x3FF
    DIRECTION_BIT = 1<<10
    TORQUE_SCL = float(MAX_TORQUE/1.0)
    
    # Scaling for Dynamixel speed 
    SPEED_SCL = 0.114
    # Scaling for Dynamixel voltage
    VOLTAGE_SCL = 0.1
    # Scaling for loads (unused)
    LOAD_SCL = 0.001
    
    SCL = float(MAX_POS - MIN_POS)/(MAX_ANG - MIN_ANG)
    OFS = (MAX_POS - MIN_POS)/2 + MIN_POS


    def __init__( self, node_id, typecode, pna ): 
        """
        """
        DynamixelModule.__init__( self, node_id, typecode, pna )
          
class EX106Module( DynamixelModule ):
    """
    DESCRIPTION: 
    -- EX106 Specific constants
    RESPONSIBILITIES: 
    -- ...
    """
    
    #  Scale and offset for converting CKBot angles to and from dynamixel as per EX106+ manual section 3-4-2 pp. 29
    MAX_POS = 0xFFF
    MIN_POS = 0
    MIN_ANG = 0 #Min, max angle are listed in centidegrees based on values from the manual
    MAX_ANG = 25092
       
    # Scaling for Dynamixel continuous turn torque 
    MAX_TORQUE = 0x3FF
    DIRECTION_BIT = 1<<10
    TORQUE_SCL = float(MAX_TORQUE/1.0)
    
    # Scaling for Dynamixel speed 
    SPEED_SCL = 0.111
    # Scaling for Dynamixel voltage
    VOLTAGE_SCL = 0.1
    
    SCL = float(MAX_POS - MIN_POS)/(MAX_ANG - MIN_ANG)
    OFS = (MAX_POS - MIN_POS)/2 + MIN_POS
    
    def __init__( self, node_id, typecode, pna ): 
        """
        """
        DynamixelModule.__init__( self, node_id, typecode, pna )

class AX12Module( DynamixelModule ):    
    """
    DESCRIPTION: 
    -- AX12 Specific constants
    RESPONSIBILITIES: 
    -- ...
    """
    #  Scale and offset for converting CKBot angles to and from dynamixel as per MX-64 e-Manual
    MAX_POS = 0x3FF
    MIN_POS = 0
    MIN_ANG = 0 #Min, max angle are listed in centidegrees based on values from the manual
    MAX_ANG = 30000
       
    # Scaling for Dynamixel continuous turn torque 
    MAX_TORQUE = 0x3FF
    DIRECTION_BIT = 1<<10
    TORQUE_SCL = float(MAX_TORQUE/1.0)
    
    # Scaling for Dynamixel speed 
    SPEED_SCL = 0.114
    # Scaling for Dynamixel voltage
    VOLTAGE_SCL = 0.1
    
    SCL = float(MAX_POS - MIN_POS)/(MAX_ANG - MIN_ANG)
    OFS = (MAX_POS - MIN_POS)/2 + MIN_POS
    def __init__( self, node_id, typecode, pna ): 
        """
        """
        DynamixelModule.__init__( self, node_id, typecode, pna )

class RX64Module( DynamixelModule ):
    """
    DESCRIPTION: 
    -- RX64 Specific constants
    RESPONSIBILITIES: 
    -- ...
    """
    
    #  Scale and offset for converting CKBot angles to and from dynamixel as per RX64 manual section 3-4-2 pp. 28
    MAX_POS = 0x3FF
    MIN_POS = 0
    MIN_ANG = 0 #Min, max angle are listed in centidegrees based on values from the manual
    MAX_ANG = 30000
       
    # Scaling for Dynamixel continous turn torque 
    MAX_TORQUE = 0x3FF
    DIRECTION_BIT = 1<<10
    TORQUE_SCL = float(MAX_TORQUE/1.0)
    # Scaling for Dynamixel speed 
    SPEED_SCL = 0.111
    # Scaling for Dynamixel voltage
    VOLTAGE_SCL = 0.1
    
    SCL = float(MAX_POS - MIN_POS)/(MAX_ANG - MIN_ANG)
    OFS = (MAX_POS - MIN_POS)/2 + MIN_POS
    
    def __init__( self, node_id, typecode, pna ): 
        """
        """
        DynamixelModule.__init__( self, node_id, typecode, pna )

MODELS = {
 'Dynamixel-006b' : (EX106MemWithOps,EX106Module),
 'Dynamixel-0040' : (RX64MemWithOps,RX64Module),
 'Dynamixel-0200' : (RX64MemWithOps,RX64Module),
 'Dynamixel-000c' : (AX12MemWithOps,AX12Module),
 'Dynamixel-001d' : (MX28MemWithOps,MX28Module),
 'Dynamixel-0136' : (MX64MemWithOps,MX64Module),
}
