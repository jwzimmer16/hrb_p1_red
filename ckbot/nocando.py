"""
Module nocando implements a simulated CAN Bus and simulated node classes to go
with it (Simmer).

Main classes:
  Simmer nodes fake a simple object dictionary.
  SimmerWithMem also fakes mcu memory
  SimmerServo fakes a simple servo module interface (get_pos and set_pos)
  
Theory of operation:
  The Bus class uses time.time (via Bus.now) to follow the progress of time,
  and call the Simmer.runToTime() methods of all simulated nodes. These calls
  occur whenever Bus.read() or Bus.write() are called.

Debugging features:
  A global DEBUG list is inherited by all instances. Use slice assignment to
  modify globally, or assign in individual objects to make local changes.
  
  Topics are:
  'r' : Bus.read() calls
  'w' : Bus.write() calls
  'd' : Bus.writeFromNode() calls
  'o' : SimDic object dictionary get and set
  'm' : SimDic attempts to access missing OD entries
  'p' : SimDic access permission errors on OD entries
  'M' : SimmerWithMem memory read and write
  
Typical usage:
  >>> import ckbot.logical
  >>> nodes = [
  >>>   SimmerWithMem(ckbot.logical.MemAt0x1000,0x10,'GenericIO'),
  >>>   Simmer(0x22,'V1.4')
  >>> ]
  >>> DEBUG[:]=['o']
  >>> bus=Bus(nodes)
  >>> clu = ckbot.logical.Cluster( bus=bus ); clu.populate(2)
  >>> m = clu.at.Nx10; m.get_od()
  >>> m.mem[m.mcu.TRISC] = 5
  >>> print m.mem[m.mcu.TRISC] 
  
"""
import time
import struct
try:
  from ckbot.pcan import Msg as pcan_Msg
except ImportError:
  def pcan_Msg(ID,code,length,data):
    return dict(ID=ID,code=code,length=length,data=data)

import ckbot.rb as rb
from ckmodule import progress, AbstractBus

DEBUG=[]

class Bus( AbstractBus ):
  """
  This concrete class provides a simulated bus interface which can be used
  to develop software using Protocol classes without a physical CAN bus
  interface. 
  
  For simplicity, the communication is duplex -- i.e. message from host are
  only seen by nodes, and messages by nodes are only seen by the host.
  
  DEBUG topics:
  'r' : display .read() calls
  'w' : display .write() calls
  'd' : display .writeFromNode() calls
  
  Typical usage consists of:
  >>> import logical, nocando
  >>> nodes = [nocando.Simmer(0x10,'GenericIO')]
  >>> p = logical.Protocol(bus=nocando.Bus( nodes ) )
  >>> p.bus.DEBUG='d'
  
  NOTE: The Bus becomes the owner of the Simmer instances. Simmer instances
    cannot be shared between multiple Bus-s.  
  """
  def __init__(self,nodes=[]):
    self.DEBUG = DEBUG
    self.in_q = []
    self.nodes = nodes
    for n in nodes:
      n.setXmit( self.writeFromNode )
    self.now = time.time
    self.t0 = self.now()
    
  def write(self,ID,data):
    """Emulate a PCAN write operation"""
    self.runOnce()
    if 'w' in self.DEBUG:
      progress("#w TX 0x%04X %s" % (
        ID, " ".join([ "%02X" % d for d in data ])))
    # CAN is a bus -- all nodes receive all messages
    for node in self.nodes:
      node.receiveFromCAN( ID, data )   
    self.runOnce()
    
  def _readQ(self):
    """(private) Read pcan.Msg from queue"""
    ID,DATA = self.in_q.pop(0)
    msg = pcan.Msg(ID,0,len(DATA),DATA)
    if 'r' in self.DEBUG:
      progress("#r RX 0x%04X %s" % (
        ID, " ".join([ "%02X" % d for d in DATA ]))) 
    return msg
  
  def runOnce(self):
    t = self.now()
    for node in self.nodes:
      node.runToTime(t)        
  
  def read(self):
    """Emulate a PCAN read after updating all nodes"""
    self.runOnce()
    if self.in_q:
      return self._readQ()
    return None
    
  def writeFromNode(self,ID,data):
    """Emulate a PCAN write operation for remote side (simulated module)"""
    self.in_q.append((ID,data))
    if 'd' in self.DEBUG:
      progress("#d TX 0x%04X %s" % (
        ID, " ".join(( "%02X" % d for d in data )) ))

class SimDic( object ):
  def __init__(self):
    self.tbl = {}
    self.val = {}
    self.idx = []
    self.DEBUG = DEBUG
    
  def add( self, idx, perm, tc, val, desc ):
    """
    Add a simulated object dictionary entry
    INPUT:
      idx -- int -- index
      perm -- int -- mask composed of rb.RB_PERM_xxx bits
      tc -- int -- typecode; one of the rb.RB_xxxx types
      val -- scalar type -- value appropriate for typecode tc
      desc -- str -- description string
    """
    assert rb.types.has_key( tc ), "a valid typecode"
    assert (perm | rb.RB_PERM_READ | rb.RB_PERM_WRITE) == (rb.RB_PERM_READ | rb.RB_PERM_WRITE), "a valid set of permission bits"
    # Prepare description fragments
    if len(desc)>32:
      desc = desc[:32]
    else:
      desc = desc + '\x00'*(32-len(desc))
    frag = [ desc[k:k+4] for k in xrange(0,32,4) ]    
    upd = { (idx,0) : (perm,tc) }
    for k in xrange(8):
      upd[(idx,0xF7+k)]= tuple(( ord(c) for c in desc[(k*4):(k*4+4)] ))
    self.tbl.update(upd)
    self.idx = []
    self[idx] = val
    
  def _reindex(self):
    """Recompute the linking sub-index fields"""
    if self.idx or not self.tbl:
      return
    idx = [ k[0] for k in self.tbl.iterkeys() if k[1]==0 ]
    idx.sort()
    for cur,nxt in zip( idx[:-1], idx[1:] ):
      val = tuple(( ord(c) for c in struct.pack('<H',nxt) ))
      self.tbl[(cur,0xFF)] = val
    self.tbl[(idx[-1],0xFF)] = (0,0)
    self.idx = idx
  
  def __getitem__(self,idx):
    """Get the value at a dictionary index / None"""
    self._reindex()
    return self.val.get(idx,None)
    
  def __setitem__(self,idx,val):
    """Set the value at a dictionary index"""
    _,tc = self.tbl.get( (idx,0), (None,None) )
    if tc is None:
      raise IndexError('0x%04x not found' % idx)
    v = tuple(( ord(c) for c in struct.pack(rb.DTYPES[tc][1],val) ))    
    self.tbl[(idx,1)] = v
    self.val[idx] = val
    
  def _setFromMsg(self,idx,sidx,perm,tc,payload):
    """(private) set dictionary entry from message"""
    if perm & rb.RB_PERM_WRITE == 0:
      if 'p' in self.DEBUG:
        progress('#p OD %04X %02X <-- is not writable' % (idx,sidx) )
      return False  
    v = tuple( payload )
    self.tbl[ (idx,sidx) ]=v
    s = "".join( ( chr(vi) for vi in v ) )    
    val = struct.unpack(rb.DTYPES[tc][1],s)[0]
    self.val[ idx ] = val
    if 'o' in self.DEBUG:
      progress('#o OD %04X %02X <-- %s' % (idx,sidx,hex(val)) )
    return True

  def _getFromMsg(self,idx,sidx,perm,tc):
    """(private) set dictionary entry from message"""
    if sidx==1 and (perm & rb.RB_PERM_READ == 0):
      if 'p' in self.DEBUG:
        progress('#p OD %04X %02X --> not readable' % (idx,sidx) )
      return None
    self._reindex()
    bytes = self.tbl.get( (idx,sidx), None )
    assert type(bytes)==tuple
    if bytes is None:
      if 'm' in self.DEBUG: 
        progress('#m OD %04X %02X --> subindex not found' % (idx,sidx) )
    elif 'o' in self.DEBUG:
      progress('#o OD %04X %02X --> %s' % (idx,sidx,
        " ".join(("%02X" % b for b in bytes)) ))
    return bytes     

  def doMsg( self, DATA ):
    """Handle incoming dictionary message"""
    cs,sidx,l,h = DATA[:4]
    idx = l | (h<<8)
    perm,tc = self.tbl.get( (idx,0), (None,None) )
    if perm is None:
      if 'm' in self.DEBUG:
        progress('#m OD %04X %02X index not found' % (idx,sidx) )
    elif cs == rb.RB_CS_WRITE:
      if self._setFromMsg(idx,sidx,perm,tc,DATA[4:]):
        return DATA
    elif cs == rb.RB_CS_READ:
      bytes = self._getFromMsg(idx,sidx,perm,tc)
      if bytes:
        return tuple(DATA[:4])+bytes
    elif 'm' in self.DEBUG:
      progress('#m OD %04X %02X unknown CS value %02X' % (idx,sidx,cs) )
    return None
    
class Simmer( object ):
  """
  Abstract superclass of all simulated modules
  
  The Simmer class implements the common traits of simulated modules.
  In particular, Simmer instances implement receiveFromCAN() to process
  incoming CAN messages and runToTime() to execute their own simulation
  steps.
  
  Simmer implements an object dictionary using a SimDic, and contains the
  0x1000 dictionary entry, whose description (the module version string) is
  set by the constructor.  
  """
  def __init__(self, nid, desc ):
    """
    INPUTS:
      nid -- byte -- node id for simulated node
      desc -- str -- software version string to report (up to 16 chars)
    ATTRIBUTES:
      .nid -- node id
      .od -- SimDic with the object dictionary      
    """
    self.nid = nid
    self.od = SimDic()
    self.od.add( 0x1000, rb.RB_PERM_READ, rb.RB_UINT16, 0, desc[:16] )
    self.t0 = -1
    def barf( *args ):
      raise RuntimeError("Simmer.xmit was not initialized")
    self.xmit = barf

  def setXmit( self, fun ):
    """
    Set the transmit callback
    
    INPUT:
      fun -- function -- with signature fun(ID,DATA)
    """
    assert callable(fun)
    self.xmit = fun
    
  def receiveFromCAN( self, ID, DATA ):
    """Simulate receive a CAN message"""
    if ID & 0xFF != self.nid:
      return
    if ID & 0xFF00 == rb.RB_DICT_REQ:
      reply = self.od.doMsg( DATA )
      if reply is not None:
        self.xmit( 0x500 | self.nid, reply )
      
  def runToTime( self, t ):
    """
    Simulate node activities: transmit a heartbeat every second 
    """
    # Emit heartbeat message as needed
    if t < self.t0 + 1:
      return
    ID = rb.RB_HEARTBEAT | self.nid
    DATA = (0,)*8
    self.xmit( ID, DATA )
    self.t0 = t

class SimmerWithMem( Simmer ):
  """
  Concrete class simulating a node with a memory interface
  
  The memory interface OD entry numbers can be selected at initialization.
  The OD entry description strings are 'rAdr','rVal','wAdr','wVal'
  
  !!!TODO: typical usage (copy from wheelarm.py)
  """
  def __init__(self,mx,*args,**kw):
    """    
    INPUT:
      mx -- object specifying the OD indices of the memory interface.
            Use logical.MemAt0x1010 or logical.MemAt0x1000
      *args -- parameters for Simmer.__init__
    ATTRIBUTES:
      .mem -- dictionary containing simulate memory of node
      .DEBUG -- debug topics to display
    """
    Simmer.__init__(self,*args,**kw)
    self.DEBUG = DEBUG
    self.mem = {}
    for args in [
        # idx, perm, tc, val, desc
        ( mx.GIO_R_ADDR, rb.RB_PERM_WRITE, rb.RB_UINT16, 0, 'rAdr' ),
        ( mx.GIO_R_VAL, rb.RB_PERM_READ, rb.RB_UINT8, 0, 'rVal' ),
        ( mx.GIO_W_ADDR, rb.RB_PERM_WRITE | rb.RB_PERM_READ, rb.RB_UINT16, 0, 'rAdr' ),
        ( mx.GIO_W_VAL, rb.RB_PERM_WRITE, rb.RB_UINT8, 0, 'wVal' ) 
    ]: self.od.add( *args )
    self.mx = mx
    
  def runToTime( self, t ):
    addr = self.od[ self.mx.GIO_W_ADDR ]
    if addr != 0:
      val = self.od[ self.mx.GIO_W_VAL ]
      self.mem[ addr ] = val 
      self.od[ self.mx.GIO_W_ADDR ] = 0
      if 'M' in self.DEBUG:
        progress('#M AT %04x <-- %02X' % (addr, val) )
    Simmer.runToTime( self, t )
    addr = self.od[ self.mx.GIO_R_ADDR ]
    if addr != 0:
      val = self.mem.get(addr,0)
      self.od[ self.mx.GIO_R_VAL ] = val
      self.od[ self.mx.GIO_R_ADDR ] = 0
      if 'M' in self.DEBUG:
        progress('#M AT %04x --> %02X' % (addr, val) )

class SimmerServo( Simmer ):
  """
  Concrete class simulating a node with a servo interface.
  
  Implements the 'pos' and 'encpos' OD entries, simulates motion of encpos
  towards pos, and accepts process message position commands.  
  """
  def __init__(self,sx,*args,**kw):
    """
    INPUT:
      sx -- object specifying the OD indices of the servo interface
            Use logical.V1_3Module, logical.V1_4Module, etc. 
    """
    Simmer.__init__(self,*args,**kw)
    self.DEBUG = DEBUG
    for args in [
        # idx, perm, tc, val, desc
        ( sx.PM_ID_POS, rb.RB_PERM_WRITE, rb.RB_SINT16, 0, 'pos' ),
        ( sx.ENC_POS_INDEX, rb.RB_PERM_READ, rb.RB_SINT16, 0, 'encpos' ),
    ]: self.od.add( *args )
    self.sx = sx
    
  def runToTime( self, t ):
    Simmer.runToTime( self, t )
    # Simulate servo motion towards desired position
    des = self.od[ self.sx.PM_ID_POS ]
    cur = self.od[ self.sx.ENC_POS_INDEX ]
    if des != self.sx.POS_SLACK:
      if cur>des+5 and des>self.sx.POS_LOWER:
        self.od[ self.sx.ENC_POS_INDEX ] -= 7
      elif cur<des-5 and des<self.sx.POS_UPPER:
        self.od[ self.sx.ENC_POS_INDEX ] += 7

  def receiveFromCAN( self, ID, DATA ):
    """Simulate receive a CAN message"""
    if ID & 0xFF != self.nid:
      return
    if ID & 0xFF00 == rb.RB_DICT_REQ:
      reply = self.od.doMsg( DATA )
      if reply is not None:
        self.xmit( 0x500 | self.nid, reply )
    elif ID & 0xFF00 == rb.RB_COMMAND:
      # Got a process message with new position --> store in OD
      pos = struct.unpack( '<h', chr(DATA[0])+chr(DATA[1]))[0]
      self.od[ self.sx.PM_ID_POS ] = pos

if __name__=="__main__":
  import ckbot.logical
  nodes = [
    SimmerWithMem(ckbot.logical.MemAt0x1000,0x10,'GenericIO'),
    Simmer(0x22,'V1.4')
  ]
  #DEBUG[:]=[ c for c in 'Mmprdwo' ]
  #DEBUG[:]=['o','M']
  bus=Bus(nodes)
  clu = ckbot.logical.Cluster( bus=bus )
  clu.populate(2)
  m = clu.at.Nx10
  m.get_od()
  m.mem[m.mcu.TRISC] = 5
  assert m.mem[m.mcu.TRISC] == 5
