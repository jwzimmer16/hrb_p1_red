import pygame.midi as pygm
import sys
from joy.events import MIDIEVENT, JoyEvent

DEV = None

class Channels(object):
  def __init__(self):
    self._keys = set()
    
  def update( self, dic ):
    self._keys.update(dic)
    self.__dict__.update(dic)    
        
class MidiInput( object ):
  BUF_SIZE = 1024
  
  def __init__(self,name,ID):
    self.mi = pygm.Input(ID,MidiInput.BUF_SIZE)
    self.name = name
    self.ID = ID
    self.at = Channels()
    assert pygm.get_device_info(ID)[2], "Must be an input device"

  def rawEventIter(self):
    while self.mi.poll():
      for evt in self.mi.read(MidiInput.BUF_SIZE):
        yield evt
    return

class KorgNanoKontrol( MidiInput ):
  N2C = {
    (0,'rewind',0) : (176, 47),
    (0,'play',0) : (176, 45),
    (0,'forward',0) : (176, 48),
    (0,'repeat',0) : (176, 49),
    (0,'stop',0) : (176, 46),
    (0,'record',0) : (176, 44),
    (1,'btnU',1) : (176, 23),
    (1,'btnL',1) : (176, 33),
    (1,'slider',1) : (176, 2),
    (1,'knob',1) : (176, 14),
    (1,'btnU',2) : (176, 24),
    (1,'btnL',2) : (176, 34),
    (1,'slider',2) : (176, 3),
    (1,'knob',2) : (176, 15),
    (1,'btnU',3) : (176, 25),
    (1,'btnL',3) : (176, 35),
    (1,'slider',3) : (176, 4),
    (1,'knob',3) : (176, 16),
    (1,'btnU',4) : (176, 26),
    (1,'btnL',4) : (176, 36),
    (1,'slider',4) : (176, 5),
    (1,'knob',4) : (176, 17),
    (1,'btnU',5) : (176, 27),
    (1,'btnL',5) : (176, 37),
    (1,'slider',5) : (176, 6),
    (1,'knob',5) : (176, 18),
    (1,'btnU',6) : (176, 28),
    (1,'btnL',6) : (176, 38),
    (1,'slider',6) : (176, 8),
    (1,'knob',6) : (176, 19),
    (1,'btnU',7) : (176, 29),
    (1,'btnL',7) : (176, 39),
    (1,'slider',7) : (176, 9),
    (1,'knob',7) : (176, 20),
    (1,'btnU',8) : (176, 30),
    (1,'btnL',8) : (176, 40),
    (1,'slider',8) : (176, 12),
    (1,'knob',8) : (176, 21),
    (1,'btnU',9) : (176, 31),
    (1,'btnL',9) : (176, 41),
    (1,'slider',9) : (176, 13),
    (1,'knob',9) : (176, 22),
    (2,'btnU',1) : (176, 67),
    (2,'btnL',1) : (176, 76),
    (2,'slider',1) : (176, 42),
    (2,'knob',1) : (176, 57),
    (2,'btnU',2) : (176, 68),
    (2,'btnL',2) : (176, 77),
    (2,'slider',2) : (176, 43),
    (2,'knob',2) : (176, 58),
    (2,'btnU',3) : (176, 69),
    (2,'btnL',3) : (176, 78),
    (2,'slider',3) : (176, 50),
    (2,'knob',3) : (176, 59),
    (2,'btnU',4) : (176, 70),
    (2,'btnL',4) : (176, 79),
    (2,'slider',4) : (176, 51),
    (2,'knob',4) : (176, 60),
    (2,'btnU',5) : (176, 71),
    (2,'btnL',5) : (176, 80),
    (2,'slider',5) : (176, 52),
    (2,'knob',5) : (176, 61),
    (2,'btnU',6) : (176, 72),
    (2,'btnL',6) : (176, 81),
    (2,'slider',6) : (176, 53),
    (2,'knob',6) : (176, 62),
    (2,'btnU',7) : (176, 73),
    (2,'btnL',7) : (176, 82),
    (2,'slider',7) : (176, 54),
    (2,'knob',7) : (176, 63),
    (2,'btnU',8) : (176, 74),
    (2,'btnL',8) : (176, 83),
    (2,'slider',8) : (176, 55),
    (2,'knob',8) : (176, 65),
    (2,'btnU',9) : (176, 75),
    (2,'btnL',9) : (176, 84),
    (2,'slider',9) : (176, 56),
    (2,'knob',9) : (176, 66),
    (3,'btnU',1) : (176, 107),
    (3,'btnL',1) : (176, 116),
    (3,'slider',1) : (176, 85),
    (3,'knob',1) : (176, 94),
    (3,'btnU',2) : (176, 108),
    (3,'btnL',2) : (176, 117),
    (3,'slider',2) : (176, 86),
    (3,'knob',2) : (176, 95),
    (3,'btnU',3) : (176, 109),
    (3,'btnL',3) : (176, 118),
    (3,'slider',3) : (176, 87),
    (3,'knob',3) : (176, 96),
    (3,'btnU',4) : (176, 110),
    (3,'btnL',4) : (176, 119),
    (3,'slider',4) : (176, 88),
    (3,'knob',4) : (176, 97),
    (3,'btnU',5) : (176, 111),
    (3,'btnL',5) : (176, 120),
    (3,'slider',5) : (176, 89),
    (3,'knob',5) : (176, 102),
    (3,'btnU',6) : (176, 112),
    (3,'btnL',6) : (176, 121),
    (3,'slider',6) : (176, 90),
    (3,'knob',6) : (176, 103),
    (3,'btnU',7) : (176, 113),
    (3,'btnL',7) : (176, 122),
    (3,'slider',7) : (176, 91),
    (3,'knob',7) : (176, 104),
    (3,'btnU',8) : (176, 114),
    (3,'btnL',8) : (176, 123),
    (3,'slider',8) : (176, 92),
    (3,'knob',8) : (176, 105),
    (3,'btnU',9) : (176, 115),
    (3,'btnL',9) : (176, 124),
    (3,'slider',9) : (176, 93),
    (3,'knob',9) : (176, 106),
    #(4,'btnU',1) : (176, 16),
    #(4,'btnL',1) : (176, 17),
    #(4,'slider',1) : (176, 7),
    #(4,'knob',1) : (176, 10),
    (4,'btnU',2) : (177, 16),
    (4,'btnL',2) : (177, 17),
    (4,'slider',2) : (177, 7),
    (4,'knob',2) : (177, 10),
    (4,'btnU',3) : (178, 16),
    (4,'btnL',3) : (178, 17),
    (4,'slider',3) : (178, 7),
    (4,'knob',3) : (178, 10),
    (4,'btnU',4) : (179, 16),
    (4,'btnL',4) : (179, 17),
    (4,'slider',4) : (179, 7),
    (4,'knob',4) : (179, 10),
    (4,'btnU',5) : (180, 16),
    (4,'btnL',5) : (180, 17),
    (4,'slider',5) : (180, 7),
    (4,'knob',5) : (180, 10),
    (4,'btnU',6) : (181, 16),
    (4,'btnL',6) : (181, 17),
    (4,'slider',6) : (181, 7),
    (4,'knob',6) : (181, 10),
    (4,'btnU',7) : (182, 16),
    (4,'btnL',7) : (182, 17),
    (4,'slider',7) : (182, 7),
    (4,'knob',7) : (182, 10),
    (4,'btnU',8) : (183, 16),
    (4,'btnL',8) : (183, 17),
    (4,'slider',8) : (183, 7),
    (4,'knob',8) : (183, 10),
    (4,'btnU',9) : (184, 16),
    (4,'btnL',9) : (184, 17),
    (4,'slider',9) : (184, 7),
    (4,'knob',9) : (184, 10),
  } # ENDS: N2C
  C2N = dict( [(x,y) for y,x in N2C.iteritems()] )

  def __init__(self,*argv,**kw):
    MidiInput.__init__(self,*argv,**kw)
    
  def eventIter( self ):
    for (a,b,c,d),t in self.rawEventIter():
      dcr = self.C2N.get((a,b),None)
      if dcr is None:
        continue
      if dcr[0]>0:
        self.at.__dict__["sc%d_%s%d" % dcr] = c
      else:
        self.at.__dict__[dcr[1]] = c
      yield dcr+(c,)


def joyEventIter():
  """(protected) generate all pending events as JoyApp events
  This is used internally by JoyApp's event pump to collect midi events
  """
  for k,dev in DEV.iteritems():
    for sc,kind,index,value in dev.eventIter():
      yield JoyEvent( MIDIEVENT,
        dev=k,sc=sc,kind=kind,index=index,value=value
      )
      
if 1: # function to help create an N2C dictionary 
  import time, sys
  def foo(f='rewind'):
    nm = [ (0,k,0) for k in [
      'rewind',
      'play',
      'forward', 
      'repeat',
      'stop',
      'record'
    ]]
    for sc in xrange(1,5):
      for k in xrange(1,10):
        nm.append((sc,'btnU',k))
        nm.append((sc,'btnL',k))
        nm.append((sc,'slider',k))
        nm.append((sc,'knob',k))
    while nm and nm[0] != f:
      nm.pop(0)
    last = (None,None)
    for k in nm:
       print "(%d,'%s',%d)" % k,
       sys.stdout.flush()
       while True:
           time.sleep(0.1)
           l = list(DEV[3].rawEventIter())
           if not l:
             continue
           if l[-1][0][:2] != last[:2]:
             break
       last = l[-1][0]  
       print ": (%d,%d)," % (last[0],last[1])
       sys.stdout.flush()
     
def init():
   """
   Initialize the pygame midi interface and enumerate the devices in
   the .DEV module variable
   """
   global DEV
   if DEV is not None:
     return
   DEV = {}
   pygm.init()
   for k in xrange(pygm.get_count()):
     _,nm,inp,_,_ = pygm.get_device_info(k)
     if not inp:
       continue
     cls = {
       'nanoKONTROL MIDI 1' : KorgNanoKontrol,
     }.get( nm, None )
     if cls:
       DEV[k] = cls( nm, k )
       
if __name__=="__main__":
  from time import sleep, time as now
  from joy.events import describeEvt
  from sys import stdout
  print "Running test"
  init()
  t0 = now()
  while True:
    for evt in joyEventIter():
      print "\r%6.2f "%(now()-t0),
      print describeEvt(evt), "   ",
      sys.stdout.flush()
    sleep(0.05)
    
  
