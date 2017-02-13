
"""
File joy.remote contains the remote.Source and remote.Sink Plan 
subclasses. These Plan-s may be used to relay remote JoyApp events 
between processes that may even be running on different machines

"""

from plans import Plan
from joy.events import JoyEvent, describeEvt
from pygix import TIMEREVENT, KEYUP, KEYDOWN, JOYBUTTONUP, JOYBUTTONDOWN, postEvent
from socket import socket, AF_INET, SOCK_DGRAM, error as SocketError
from errno import EAGAIN
from json import loads as json_loads, dumps as json_dumps
from loggit import progress

# Default UDP port used for communications
DEFAULT_PORT = 0xBAA

class Sink( Plan ):
  """
  Concrete class remote.Sink
  
  This Plan subclass may be used to relay via UDP JoyApp events (including all pygame events) from a remote.Source running in 
  another JoyApp, potentially on another machine.
  
  Events are serialized into a JSON string and sent over separate UDP 
  packets. TIMEREVENTS are ignored.
  
  Additionally, any JSON packets that contain dictionaries without 
  the 'type' key may be shunted to a separate queue, for application 
  specific processing. This queue is windowed in time, i.e. will hold 
  at most X seconds worth of packets, as specified by the allowMisc
  constructor parameter. Its contents are accessible via queueIter(). 
  """ 
  DEFAULT_BINDING = ('0.0.0.0',DEFAULT_PORT)
  
  def __init__( self, app, bnd=DEFAULT_BINDING, rate=100, convert=None, allowMisc = None ):
    """
    Attributes:
      bnd -- 2-tuple -- binding address for socket, in socket library format
      rate -- integer -- maximal number of events processed each time-slice
      sock -- socket / None -- socket only exists while plan is running
      convert -- callable -- convert incoming event dictionaries.
        Allows remote joysticks/keys to be remapped. Returns None if
        event should be ignored
        By default, only KEYUP and KEYDOWN events are allowed
      allowMisc -- float / None -- number of seconds of "misc" packets 
        to store in self.queue or None to disallow non-event packets
    """
    Plan.__init__(self,app)
    self.bnd = bnd
    self.sock = None
    self.rate = rate
    self.flushMisc()
    self.setAllowMisc(allowMisc)
    if convert is None:
      def default_convert( dic ):
        tc = dic['type_code']
        if tc in {KEYUP,KEYDOWN,JOYBUTTONUP,JOYBUTTONDOWN} :
          return dic
        return None
      self.convert = default_convert
    else:
      assert callable(convert)
      self.convert = convert
  
  def onStart( self ):
    self.sock = socket( AF_INET, SOCK_DGRAM )
    self.sock.bind(self.bnd)
    self.sock.setblocking(False)
  
  def onStop( self ):
    if self.sock:
      self.sock.close()
    self.sock = None
  
  def setAllowMisc( self, allow ):
    """
    (re)set the depth of the misc event queue
    """
    self.allow = allow
    self._clearQueue()
  
  def flushMisc( self ):
    """flush the misc event queue"""
    self.queue = []
    
  def setSink( self, bnd ):
    """
    Set the socket binding address. 
    
    Will take effect next time the plan is started
    """
    self.bnd = bnd

  def onEvent( self, evt ):
    if evt.type != TIMEREVENT:
      return
    for k in xrange(self.rate):
      try:
        # Create event from the next packet
        pkt = self.sock.recv(1024)
        dic = json_loads(pkt)
      #
      except SocketError,err:
        # If socket is out of data --> we're done
        if err.errno == EAGAIN:
          break
        raise
      #
      except ValueError,ve:
        # Value errors come from JSON decoding problems. 
        # --> Log and continue
        progress('Received bad UDP packet: %s' % repr(pkt))
        continue
      #
      # Process the event packet
      #
      dic = self.convert(dic)
      # If converter dropped event --> next
      if not dic:
        continue
      # If has a 'type' --> JoyEvent
      if type(dic) is dict and dic.has_key('type'):
        # Put event on event queue
        nev = JoyEvent( **dic )
        #DBG progress('Remote event:'+str(nev))
        postEvent(nev)
        continue
      # If custom events are allowed --> add to queue
      if self.allow:
        now = self.app.now
        self._clearQueue()
        # Store new timestamp and packet
        self.queue.append( (now,dic) )
    return False
  
  def _clearQueue(self):
    """(private) clear old events from misc queue"""
    # Drop any out-of-date packets from queue
    now = self.app.now
    while self.queue and (now-self.queue[0][0] > self.allow):
      self.queue.pop(0)
      
  def queueIter( self ):
    """Iterate over the custom packets in the queue
       Yields pairs (ts, pkt)
         ts -- float -- arrival time
         pkt -- dictionary -- packet contents
    """
    if self.allow is None:
      raise IndexError,"No custom packets allowed -- use allowMisc parameter to Sink constructor"
    while self.queue:
      if (self.app.now-self.queue[0][0]) < self.allow:
        yield self.queue[0]
      self.queue.pop(0)

class Source( Plan ):
  """
  Concrete class remote.Source
  
  This Plan subclass may be used to relay JoyApp events (including all
  pygame events) to a remote.Sink running in another JoyApp via UDP.
  This allows a controller in one JoyApp to control a client in a 
  remote JoyApp connected via an IP network, potentially running on 
  another host altogether.
  
  Events are serialized into a JSON string and sent over separate UDP 
  packets. TIMEREVENTS are ignored.
  """ 
  DEFAULT_SINK = ('localhost',DEFAULT_PORT)
  
  def __init__( self, app, dst=DEFAULT_SINK ):
    """
    Attributes:
      dst -- 2-tuple -- destination address for packets, in socket library format
      sock -- socket / None -- socket only exists while plan is running 
    """
    Plan.__init__(self,app)
    self.dst = dst
    self.sock = None
  
  def onStart( self ):
    self.sock = socket( AF_INET, SOCK_DGRAM )
    self.sock.bind(('0.0.0.0',0))
  
  def onStop( self ):
    if self.sock:
      self.sock.close()
    self.sock = None
      
  def setSink( self, dst ):
    """
    Set the sink address. Will take effect immediately
    """
    self.dst = dst

  def onEvent( self, evt ):
    if evt.type == TIMEREVENT:
      return False
    dic = describeEvt( evt, parseOnly = True )
    dic['t'] = self.app.now
    jsn = json_dumps(dic, ensure_ascii = True )
    self.sock.sendto( jsn, self.dst )
    progress( "Sending '%s' to %s:%d" % ((jsn,)+self.dst) )
    return False

