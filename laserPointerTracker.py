from numpy import *
from SimpleCV import *
from gzip import open as opengz
from time import time as now


class LaserTracker( object ):
  def __init__(self, fn=None):
    self.cam = JpegStreamCamera("http://admin:admin@192.168.254.125/video.mjpg")
    self.win = Display((800,600))
    self.clearBG()
    if not fn:
      self.out = None
    else:
      self.out = opengz(fn,"w")
      img = self.cam.getImage()
      img.save(fn+'first.png')
 
  def clearBG( self ):
    self.bgN = 5
    self.bgQ = None
    self.bg = None
    self.tsQ = []
    self.blob = []
    self.qi = -1
    self.trr = 0
    self.rrRate = 2
    self.guard = 10 
  
  def _putRoundRobin( self, ts, img ):
    qi = (self.qi+1) % self.bgN
    if self.bgQ is None:
      self.bgQ = zeros( (self.bgN,)+img.shape, int16 )
      self.tsQ = zeros( (self.bgN,), int16 )
    # arrays are ready
    self.bgQ[qi,...] = img
    self.tsQ[qi] = ts 
    self.qi = qi
    self.trr = ts

  def _outBlobs( self, t, b ):
    if b is None:
      self.out.write("%.2f, 0, 0, 0\n" % t)
    else:
      for n,(x,y) in enumerate(b.coordinates()):
        self.out.write("%.2f, %d, %d, %d\n" % (t,n+1,x,y))          
    self.out.flush()

  def _showBlobs( self, b ):
    raw = self.raw
    if b is not None:
      dl = raw.dl()
      for xy in b.coordinates():
        dl.circle(xy,10,Color.RED,filled=1)
        dl.circle(xy,5,Color.BLACK,filled=1)
    raw.save(self.win)
    
  def work( self ):
    raw = self.cam.getImage()
    self.raw = raw
    img = raw.getGrayNumpy()
    self.img = img
    t = now()
    if self.bg is not None:
      ind = img > (self.bg+3*self.bgs+self.guard)
      ind = ind & (img > 240)
      self.ind = ind
      b = Image(ind).findBlobs()
      if self.out:
        self._outBlobs(t,b)
      self._showBlobs(b)
    if t-self.trr>self.rrRate:
      self._putRoundRobin( t, img )
      self.bg = mean( self.bgQ, axis=0 )
      self.bgs = std( self.bgQ, axis=0 )

  def run( self ):
    self.cam.getImage().save(self.win)
    try:
      while not self.win.isDone():
        self.work()
        time.sleep(0.05)
    except KeyboardInterrupt:
      self.out.close()
      print "\n"+"*"*40+"\nSafely terminated\n"+"*"*40
      raise

if __name__=="__main__":
  import sys
  if len(sys.argv) != 2:
    sys.stderr.write("Usage: %s <filename>\n" % sys.argv[0])
    fn = 'foo.csv.gz'
  else:
    fn = sys.argv[1]
    #if not fn.endswith('.gz'):
    #  fn = fn + ".gz"
  sys.stderr.write("  output to %s\n" % fn)
  lt = LaserTracker(fn)
  lt.run()
