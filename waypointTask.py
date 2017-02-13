if __name__ != "__main__":
    raise RuntimeError("Run this as a script")

from socket import socket, AF_INET, SOCK_DGRAM, SOCK_STREAM, error as SocketError
from numpy import (
  array,asarray,zeros, exp,linspace, 
  zeros_like,kron, pi, empty_like, nan, isnan,
  concatenate, mean, dot, inf
  )
from numpy.linalg import lstsq, svd, inv
from numpy.random import rand, randn
from gzip import open as gzip_open
from json import loads as json_loads, dumps as json_dumps
from time import time as now
from joy import JoyApp, speak, progress
from joy.plans import AnimatorPlan
from joy.decl import KEYDOWN

execfile('waypointShared.py')

#### CONFIGURATION ##################################################

# Host & Port for sending sensor readings
#ROBOT_INET_ADDR = ("172.16.16.16",0xBAA)
ROBOT_INET_ADDR = ("127.0.0.1",0xBAA)

# Tag ID for robot
ROBOT_TAGID = [ 4 ]

# Rate (seconds) at which to send waypoint updates
# NOTE: waypoints are sent as integer coordinates. Set ref accordingly
WAY_RATE = 10
# Size of zoomed in robot region
ZOOM_SCALE = 10

## April tags associations ##########################################

# Port for receiving data from TagStreamer
APRIL_DATA_PORT = 0xB00

# Corners of the arena, in order
corners = [26,23,27,22,29,24,28,25]

try:
  assert ref.ndim == 2
except NameError:
  # If a ref location was not defined in waypointShared
  # Reference locations of corners, with 1 in the last coordinate
  ref = array([
    [-1,0,1,1,1,0,-1,-1],
    [1,1,1,0,-1,-1,-1,0],
    [1.0/100]*8]).T * 100

# Tag IDs for waypoints
waypoints = range(4)

# EMA coefficient for static tag locations
alpha = 0.05

# Format of log filenames
logfmt = "log-%.0d-%s.txt.gz"
### ------------------------- CODE BEGINS --------------------------

# Clean up the socket if already open
try:
  s.close()
except Exception:
  pass
# Call this before socket is created so that child won't inherit socket
speak.say("")

def skew( v ):
  """
  Convert a 3-vector to a skew matrix such that 
    dot(skew(x),y) = cross(x,y)
  
  The function is vectorized, such that:
  INPUT:
    v -- N... x 3 -- input vectors
  OUTPUT:
    N... x 3 x 3  
    
  For example:
  >>> skew([[1,2,3],[0,0,1]])
  array([[[ 0,  3, -2],
        [-3,  0,  1],
        [ 2, -1,  0]],
  <BLANKLINE>
       [[ 0,  1,  0],
        [-1,  0,  0],
        [ 0,  0,  0]]])
  """
  v = asarray(v).T
  z = zeros_like(v[0,...])
  return array([
      [ z, -v[2,...], v[1,...]],
      [v[2,...], z, -v[0,...] ],
      [-v[1,...], v[0,...], z ] ]).T

def fitHomography( x, y ):
  """Fit a homography mapping points x to points y"""
  x = asarray(x)
  assert x.shape == (len(x),3)
  y = asarray(y)
  assert y.shape == (len(y),3)
  S = skew(y)
  plan = [ kron(s,xi) for s,xi in zip(S,x) ]
  #plan.append([[0]*8+[1]])
  A = concatenate( plan, axis=0 )
  U,s,V = svd(A)
  res = V[-1,:].reshape(3,3)
  return res.T

class Sensor( object ):
  def __init__(self, *lineargs, **linekw):
    self.lineargs = lineargs
    self.linekw = linekw
    self.radius = None
    self.noise = 0.01

  def sense( self, ax, a, b, c, scale=0.2 ):
    """Compute sensor measurement for line from a to b, given
       sensor location c and a scale factor
    """
    # Rigid transform and rescaling that
    # takes (a,b) to (0,1), applied to c
    z = (c-a)/(b-a)
    if (z.real<0) or (z.real>1):
      return lineSensorResponse( inf, self.noise )
    x = z.real * (b-a) + a
    d = z.imag * abs(b-a)
    res = lineSensorResponse( d/scale, self.noise )
    ax.plot( [c.real, x.real], [c.imag, x.imag], 
      *self.lineargs, **self.linekw )
    ax.text( (c.real+x.real)/2, (c.imag+x.imag)/2, 
      "%d" % res, ha='center',va='center' )
    return int(res)

def _animation(f1):
  global s, app
  # Open socket
  try:
    s = socket(AF_INET, SOCK_DGRAM )
    s.bind(("",APRIL_DATA_PORT))
    s.setblocking(0)
    # Server socket
    srv = socket(AF_INET, SOCK_STREAM )
    srv.bind(("0.0.0.0",8080))
    srv.listen(2)
  except:
    app.stop()
    raise
  client = None
  logfile = None
  # Axes for arena display
  ax = array([
      min(ref[:,0]),max(ref[:,0]),
      min(ref[:,1]),max(ref[:,1])
  ])*1.2
  # Allowed tag IDS
  allow = set(corners + waypoints + ROBOT_TAGID)
  # Array size; must store all allowed tags
  N = max(allow)+1
  # Array holding all point locations
  pts = zeros((N,4,3),dtype=float)
  # Indicator array for point that are assumed static
  #   and whose locations will be lowpass filtered
  statics = zeros(N,dtype=bool)
  statics[corners + waypoints] = True
  # Legend for point update indicator strings
  lbl = array(list(".+ld:*LD"))
  # (CONST) Reference locations for tag corners
  ang0 = array([-1+1j,1+1j,1-1j,-1-1j]) * -1j
  # (CONST) Point on a circle 
  circ = exp(1j*linspace(0,2*pi,16))
  ### Initial values for variables
  # Configure sensor line-types
  sensorF = Sensor( ':om', lw=2 )
  sensorB = Sensor( ':oc', lw=2 )
  # Last message received from TagStreamer
  msg = None
  # Last waypoint visited
  M = 0
  # Dynamic zoom scale for robot view
  zoom = None
  # Last homography
  prj = None
  # Start time
  T0 = now()
  # Time last waypoint message was sent
  lastWay = T0-WAY_RATE-1
  # Number of messages received
  nmsg = 0
  #
  ### MAIN LOOP ###
  #
  while len(waypoints)>M+1: # continue until goal is reached
    #
    ### Read data from April
    #
    try:
      while True:
        # read data as fast as possible
        msg = s.recv(1<<16)
        nmsg += 1
    except SocketError, se:
      # until we've run out; last message remains in m
      pass
    # make sure we got something
    if not msg:
      continue
    # Parse tag information from UDP packet
    dat = json_loads(msg)
    msg = ''
    # Collect allowed tags
    h = empty_like(pts)
    h[:] = nan
    for d in dat:
      nm = d['i']
      if not nm in allow:
        continue
      #if ~isnan(h[nm,0,0]):
      #  print '(dup)',
      p = asarray(d['p'])/100
      h[nm,:,:2] = p
      h[nm,:,2] = 1
    #
    # at this point, all observed tag locations are in the dictionary h
    #
    ### Update pts array
    #
    # Tags seen in this frame 
    fidx = ~isnan(h[:,0,0])
    # Tags previously unseen
    uidx = isnan(pts[:,0,0])
    # Tags to update directly: non static, or static and first time seen
    didx = (uidx & fidx) | ~statics
    if any(didx):
      pts[didx,...] = h[didx,...]
    # Tags to update with lowpass: static, seen and previously seen
    lidx = fidx & statics & ~uidx
    if any(lidx):
      pts[lidx,...] *= (1-alpha)
      pts[lidx,...] += alpha * h[lidx,...]
    # Print indicator telling operator what we did
    progress( "%7.2f %5d  "% (now()-T0,nmsg) + lbl[didx+2*lidx+4*fidx].tostring()
             , sameLine = True )
    #
    # Collect the corner tags and estimate homography
    nprj = None
    try:
      roi = array( [ mean(pts[nm],0) for nm in corners ] )
      # Homography mapping roi to ref
      nprj = fitHomography( roi, ref )
    except KeyError, ck:
      progress("-- missing corner %s" % str(c))
    #
    # If no previous homography --> try again
    if prj is None:
      # If no homography found
      if nprj is None:
        yield
        continue
      progress("(say) Homography initialized")
    prj = nprj
    #
    # Apply homography to all the points
    uvs = dot(pts,prj)
    z = uvs[...,0] + 1j*uvs[...,1] 
    nz = ~isnan(z[:,0])
    nz &= asarray(uvs[:,0,-1],dtype=bool)
    z[nz,...] /= uvs[nz,...,[-1]]
    # Centroids of tags
    zc = mean(z,1)
    #
    ### At this point, z has all tag corner points; zc centroids
    ###   the nz index indicated tags for which we have locations
    #
    f1.clf()
    a1 = f1.add_subplot(121)
    # Mark the corner tags
    c = zc[corners]
    vc = ~isnan(c)
    a1.plot( c[vc].real, c[vc].imag,'sy',ms=15)
    # Indicate all tags
    znz = z[nz,...]
    assert ~any(isnan(znz).flatten())
    a1.plot( znz[:,[0,1,2,3,0]].real.T, znz[:,[0,1,2,3,0]].imag.T, '.-b' )
    a1.plot( [znz[:,0].real], [znz[:,0].imag], 'og' )
    for nm,p in enumerate(zc):
      if not isnan(p):
        a1.text( p.real,p.imag, "%d" % nm, ha='center',va='center' )
    # Mark the waypoints
    c = zc[waypoints]
    vc = ~isnan(c)
    a1.plot( c[vc].real, c[vc].imag,'-k',lw=3,alpha=0.3)
    a1.plot( c[[M,M+1]].real, c[[M,M+1]].imag, '--r', lw=4)
    a1.axis('equal')
    a1.axis(ax)
    #
    # If robot not seen --> we're done
    if isnan(zc[ROBOT_TAGID]):
      yield
      continue
    #
    ### robot tag related updates
    # 
    # robot tag corners
    rbt = z[ROBOT_TAGID,...]
    # robot heading angle phasor
    ang = mean((rbt-zc[ROBOT_TAGID])/ang0)
    ang /= abs(ang)
    # If logging data --> put into log
    if logfile is not None:
      lo = [ now(), zc[ROBOT_TAGID].real, zc[ROBOT_TAGID].imag, ang,
             int(zc[waypoints[M]].real), int(zc[waypoints[M]].imag) ]
      logfile.write(", ".join(["%.3f" % x for x in lo])+"\n")      
    # indicate robot
    a1.plot( zc[ROBOT_TAGID].real, zc[ROBOT_TAGID].imag, '*r' ,ms=15)
    #
    ### robot relative view
    #
    a2 = f1.add_subplot(122)
    # 
    # Show the waypoints
    c = zc[waypoints]
    vc = ~isnan(c)
    cr = (c - zc[ROBOT_TAGID])/ang
    a2.plot( cr[vc].real, cr[vc].imag,'-k',lw=3,alpha=0.3)
    a2.plot( cr[[M,M+1]].real, cr[[M,M+1]].imag, '--r', lw=4)
    #
    # Show the tags
    znr = (znz - zc[ROBOT_TAGID])/ang
    a2.plot( znr[:,[0,1,2,3,0]].real.T, znr[:,[0,1,2,3,0]].imag.T, '.-b' )
    a2.plot( [znr[:,0].real], [znr[:,0].imag], 'og' )
    for nm,p in enumerate(znr):
      if not isnan(p[0]):
        q = mean(p)
        a2.text( q.real,q.imag, "%d" % nm, ha='center',va='center' )
    #
    ### Check for waypoint contact
    #
    # Tag scale
    r = max(abs(znr[ROBOT_TAGID]).flat)
    # Indicate goal circle
    tgt = circ*r
    tgth = [
      a2.plot( tgt.real, tgt.imag, '-k' )[0],
      a2.plot( tgt.real/1.5, tgt.imag/1.5, '--k' )[0]
    ]
    # Update dynamic zoom
    if zoom is not None:
      zoom = zoom * 0.95 + r * 0.05
    else:
      zoom = r
    # Check distance of both sensors to the line
    a,b = cr[[M,M+1]]
    # Build into packet
    pkt = {
      'f' : sensorF.sense( a2, a, b, znr[ROBOT_TAGID,0]+znr[ROBOT_TAGID,1], r ),
      'b' : sensorB.sense( a2, a, b, znr[ROBOT_TAGID,2]+znr[ROBOT_TAGID,3], r ),
    }
    # Check for waypoint capture
    if (r<50) and ( abs(b)<r ):
      for h in tgth:
        h.set_linewidth(3)
        h.set_color([0,0,1])
      progress("(say) Reached waypoint %d" % (M+1))
      lastWay = now()-WAY_RATE-1
      M += 1
      if (M+1)>=len(waypoints):
        app.stop()
        return
    # Add waypoint update to packet, as needed
    if now()-lastWay>WAY_RATE:
      pkt['w'] = zip( 
        [ int(x) for x in zc[waypoints[M:]].real], 
        [ int(x) for x in zc[waypoints[M:]].imag] 
      )
      lastWay = now()
    # If we don't have a client -- listen
    if client is None:
      progress( "Waiting for client connection" )
      client,addr = srv.accept()
      progress( "Connection from "+str(addr) )
      if logfmt:
        logfile = gzip_open(logfmt % (now(),addr[0]),'w')
    # Send sensor reading out
    #s.sendto( json_dumps(pkt), ROBOT_INET_ADDR )
    try:
      client.send( json_dumps(pkt) )
      progress( "%s %s       " % (pkt['b'],pkt['f']),sameLine=True)
    except SocketError, er:
      progress( str(er) )
      progress( "Connection dropped" )
      if logfile:
        logfile.close()
        logfile = None
      client = None
    #
    a2.axis('equal')
    a2.axis( array([-1,1,-1,1])*ZOOM_SCALE*zoom)
    yield


class App(JoyApp):
  def onStart(self):
    AnimatorPlan(self,_animation).start()

  def onEvent(self,evt):
      if evt.type == KEYDOWN:
        return JoyApp.onEvent(self,evt)
        
if __name__=="__main__":
  app = App(cfg={'windowSize' : [1200,600]})
  app.run()

# TODO: auto activation of subprocess tagstreamer
# TODO: Cleanup into class format
# TODO: yaml / json for arena config

