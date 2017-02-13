import sys
import yaml
from joy import *
import ckbot.hitec as hitec
import ckbot.pololu as pololu
import ckbot.dynamixel as dynamixel

class ScanRobot( JoyApp ):
  def __init__(self,robot, cfg={}):
    cfg0 = dict(duration=30, rate=1)
    cfg0.update(cfg)
    JoyApp.__init__( self, robot=robot, cfg=cfg0 )
    self.robotPop = robot
    
  def onStart( self ):
    self.next = self.now + self.cfg.rate
    self.stopTime = self.now + self.cfg.duration
    
  def onEvent( self, evt ):
    if evt.type==KEYDOWN and evt.key in [32,13]:
      self.robot.populate( **self.robotPop )
      return
    if evt.type==QUIT or evt.type==KEYDOWN:
      self.stop()
    if self.now < self.next: 
      return
    lst = []
    for nid in self.robot.getLive():
      mod = self.robot.get(nid,None)
      if mod is None:
        lst.append('<Unknown 0x%02x>' % nid)
      else:
        if hasattr(mod,'go_slack'):
          mod.go_slack()
        lst.append(mod.name)
    progress( "Seeing: %s" % ", ".join(lst) )
    self.next = self.now + self.cfg.rate
    if self.now>self.stopTime:
      self.stop()

if __name__=="__main__":
  robot = {}
  cfg = {}
  # Table of available busses
  busTbl = {
    'hitec' : hitec,
    'pololu' : pololu,
    'dynamixel' : dynamixel,
    'h' : hitec,
    'p' : pololu,
    'd' : dynamixel
  }
  bus = None
  args = list(sys.argv[1:])
  while args:
    arg = args.pop(0)
    if arg=='--mod-count' or arg=='-c':
      N = int(args.pop(0))
      robot.update(count=N)
    elif arg=='--names' or arg=='-n':
      names = yaml.safe_load( '{%s}'% args.pop(0) )
      robot.update( names=names )
    elif arg=='--walk' or arg=='-w':
      robot.update( walk=True )
    elif arg=='--any' or arg=='-a':
      robot.update( count=None )
    elif arg=='--duration' or arg=='-d':
      cfg.update( duration = float(args.pop(0)) )
    elif arg=='--port' or arg=='-p':
      cfg.update( port = args.pop(0))
    elif arg=='--frequency' or arg=='-f':
      cfg.update( rate = float(args.pop(0)) )
    elif arg=='--bus' or arg=='-b':
      bus = busTbl.get(args.pop(0),None)
      if bus is None:
        sys.stderr.write("Unknown bus '%s'\n" % bus)
        sys.exit(2)   
      robot['protocol']=bus
    elif arg=='--help' or arg=='-h':
      robot=None
      break
    else:
      sys.stderr.write("Unknown option '%s'\n" % arg )
      sys.exit(1)
    # ENDS cmdline parsing loop
  # Usage message, if needed
  if not robot:
      sys.stdout.write("""
  Usage: %s [options]
  
    Scan bus for robot
    
    Options:      
      --mod-count <number> | -c <number>
        Search for specified number of modules at startup
      
      --names <names> | -n <names>
        Where <names> is a valid YAML mapping from node numbers
        to names, e.g. '0x36: head, 223: tail'
      
      --walk | -w
        Attempt to walk the Object Dictionary of the modules found
      
      --bus <bus-name> | -b <bus-name>
        Supported busses are 'hitec', 'pololu' and 'dynamixel'
        Bus names may be abbreviated to a single character
      
      --port <device-port> | -p <device-string>
        Access the bus on the specified device. Device strings are
        described in port2port.py newConnection
        
      --any | -a
        Scan any number of modules
    """ % sys.argv[0])
      sys.exit(1)
  app = ScanRobot(robot,cfg=cfg)
  app.run()

