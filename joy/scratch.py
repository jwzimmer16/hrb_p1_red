import struct
import socket
import time
import sys
import select
import re
from warnings import warn

class FramedSock( object ):
  def __init__(self, host="127.0.0.1", port=42001):
    self.addr = (host,port)
    self.iterLimit = 32
    self.sock = None
  
  def isConnected( self ):
    return self.sock is not None
    
  def connect( self ):
    if self.sock != None:
      raise ValueError("Socket is already open")
    self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    self.sock.connect(self.addr)
  
  def close( self ):
    if self.sock is not None:
      self.sock.close()
      self.sock = None
    
  def recv( self ):
    hdr = self.sock.recv(4)
    if len(hdr) != 4:
      raise IOError("Failed to read header from socket")
    H = struct.unpack(">L",hdr)[0]
    N = 0
    msg = []
    while N<H:
      msg.append( self.sock.recv(H-N) )
      N += len(msg[-1])
    return "".join(msg)
  
  def send( self, msg ):
    hdr = struct.pack(">L",len(msg))
    self.sock.send( hdr+msg )
  
  def recv_async( self, timeout=0.01 ):
    if self.sock:
      r,_,_ = select.select( [self.sock],[],[],timeout )
      if r:
        return self.recv()
    return None

  def __iter__(self):
    for k in xrange(self.iterLimit):
      msg = self.recv_async()
      if msg is None:
        break
      yield msg
    
class Board( object ):
  def __init__(self, host="127.0.0.1", port=42001 ):
    self._fsk = FramedSock( host, port )
    
  REX_STR = re.compile('\s*(?:"(([^"]|"")*)")\s*')
  REX_VAL = re.compile('\s*((?:-?\d*(.\d+)?)|(?:\w[-A-Za-z0-9_]*))\s*')
  REX_CMD = re.compile('\s*((?:sensor-update)|(?:broadcast))\s*')  
  @classmethod
  def _tokenize( cls, msg ):
    tok = []
    # Tokenize the command portion
    m = cls.REX_CMD.match(msg)
    if not m: return tok
    tok.append( m.group(1) )
    msg = msg[len(m.group(0)):]
    # Tokenize the remainder
    while msg:
      m = cls.REX_STR.match(msg)
      if m:
        tok.append( m.group(1).replace('""','"') )
        msg = msg[len(m.group(0)):]
        continue
      m = cls.REX_VAL.match(msg)
      if m:
        tok.append( m.group(1) )
        msg = msg[len(m.group(0)):]
        continue
      raise SyntaxError("Cannot parse '%s'" % msg)
    return tok
  
  def _doSensorUpdate( self, tok, upd ):
    if len(tok) & 1:
      warn('Sensor-update with odd number of tokens; skipping last')
      tok.pop()
    while tok:
      nm = tok.pop(0)
      val = tok.pop(0)
      if val=="true": val = True
      elif val=="false": val = False
      elif val[0] in "-0123456789": val = float(val)
      upd[nm] = val
  
  def close( self ):
    self._fsk.close()
  
  def _autoConnect( self ):
    if self._fsk.isConnected(): return True
    try:
      self._fsk.connect()
      return True
    except IOError, ioe:
      sys.stderr.write( "Failed to connect: %s\n" % ioe )
      self._fsk.close()
    return False
    
  def poll( self ):
    if not self._autoConnect(): return None,None
    # Poll for messages on socket
    upds = {}
    evts = set()
    for msg in self._fsk:
      tok = self._tokenize(msg)
      if tok is None:
        warn("Failed to parse Scratch message '%s'" % msg)
        continue
      # If 'sensor-update' message
      if tok[0][0]=='s':
        self._doSensorUpdate(tok[1:],upds)    
      elif tok[0][0]=='b':
        for nm in tok[1:]:
          evts.add(nm)
    return upds, evts
  
  def sensorUpdate(self, **kw):
    if not self._autoConnect(): return
    msg = ['sensor-update']
    for k,v in kw.iteritems():
      msg.append('"%s"' % k)
      msg.append(str(v))
    self._fsk.send( " ".join(msg) )
  
  def broadcast(self,*arg):
    if not self._autoConnect(): return
    for txt in arg:
      txt = txt.replace('"','""')
      self._fsk.send( 'broadcast "%s"' % txt )
    
    
if __name__=="__main__":
  print "Connecting to Scratch at default host:port"
  try:
    sk = FramedSock()
    sk.connect()
    t0 = time.time()
    while True:
      msg = sk.recv()
      sys.stdout.write("%6.2f %s\n" % (time.time()-t0,msg))
      sys.stdout.flush()
  finally:
    sk.close()
