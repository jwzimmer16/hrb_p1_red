# file robotSimulator.py simulates a robot in an arena

from sensorPlanTCP import SensorPlanTCP
from robotSim import DummyRobotSim,RobotSimInterface
from joy import JoyApp, progress
from joy.decl import *
from joy.plans import Plan
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )
from pylab import randn,dot,mean,exp,newaxis
from autoPlan import AutoPlan

class MoveForward(Plan):
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 10
    # Duration of travel [sec]
    self.dur = 3
    # Number of intermediate steps
    self.N = 10
    # Noise level for forward motion
    self.dNoise = 0.05
    
  def behavior(self):
    s = self.simIX
    # Compute step along the forward direction
    step = (dot([1,-1,-1,1],s.tagPos)*2.0/float(self.N)*self.dist)[newaxis,:]
    dt = self.dur / float(self.N)
    for k in xrange(self.N):
      # Move all tag corners forward by distance, with some noise
      s.tagPos = s.tagPos + step * (1+randn()*self.dNoise)
      yield self.forDuration(dt)

class Turn(Plan):
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Angle to turn [rad]
    self.ang = 0.1
    # Duration of travel [sec]
    self.dur = 3.0
    # Number of intermediate steps
    self.N = 10
    # Noise level for turn motion
    self.aNoise = 0.005
    
  def behavior(self):
    s = self.simIX
    # Compute rotation step
    dt = self.dur / float(self.N)
    rot = exp(1j * self.ang / float(self.N))
    for k in xrange(self.N):
      # Get current tag location
      z = dot(s.tagPos,[1,1j])
      c = mean(z)
      # Rotate with angle noise
      zr = c + (z-c) * rot * exp(1j*randn()*self.aNoise)
      # Store as new tag 
      s.tagPos[:,0] = zr.real
      s.tagPos[:,1] = zr.imag
      yield self.forDuration(dt)
          
          
class RobotSimulatorApp( JoyApp ):
  """Concrete class RobotSimulatorApp <<singleton>>
     A JoyApp which runs the DummyRobotSim robot model in simulation, and
     emits regular simulated tagStreamer message to the desired waypoint host.
     
     Used in conjection with waypointServer.py to provide a complete simulation
     environment for Project 1
  """    
  def __init__(self,wphAddr=WAYPOINT_HOST,*arg,**kw):
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml", *arg, **kw
      ) 
    self.srvAddr = (wphAddr, APRIL_DATA_PORT)
    
  def onStart( self ):
    # Set up socket for emitting fake tag messages
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
    s.bind(("",0))
    self.sock = s
    # Set up the sensor receiver plan
    self.sensor = SensorPlanTCP(self,server=self.srvAddr[0])
    self.sensor.start()
    self.robSim = DummyRobotSim(fn=None)
    self.moveP = MoveForward(self,self.robSim)
    self.turnP = Turn(self,self.robSim)
    self.autoP = AutoPlan(self, self.sensor, self.moveP, self.turnP)
    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1/20.0)
    progress("Using %s:%d as the waypoint host" % self.srvAddr)
    self.T0 = self.now

  def showSensors( self ):
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
    else:
      progress( "Waypoints: << no reading >>" )
  
  def emitTagMessage( self ):
    """Generate and emit and update simulated tagStreamer message"""
    self.robSim.refreshState()
    # Get the simulated tag message
    msg = self.robSim.getTagMsg()
    # Send message to waypointServer "as if" we were tagStreamer
    self.sock.sendto(msg, self.srvAddr)
    
  def onEvent( self, evt ):
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus(): 
      self.showSensors()
      progress( self.robSim.logLaserValue(self.now) )
      # generate simulated laser readings
    elif self.timeForLaser():
      self.robSim.logLaserValue(self.now)
    # update the robot and simulate the tagStreamer
    if self.timeForFrame(): 
      self.emitTagMessage()

    if evt.type == KEYDOWN:
      if evt.key == K_UP and not self.moveP.isRunning():
        self.moveP.dist = 0.2
        self.moveP.start()
        return progress("(say) Move forward")
      elif evt.key == K_DOWN and not self.moveP.isRunning():
        self.moveP.dist = 0.2
        self.moveP.start()
        return progress("(say) Move back")
      if evt.key == K_LEFT and not self.turnP.isRunning():
        self.turnP.ang = 0.3
        self.turnP.start()
        return progress("(say) Turn left")
      if evt.key == K_RIGHT and not self.turnP.isRunning():
        self.turnP.ang = -0.3
        self.turnP.start()
        return progress("(say) Turn right")
      elif evt.key == K_a:
        self.autoP.start()
        return progress("(say) Moving autonomously")
      elif evt.key == K_s:
        self.autoP.stop()
        return progress("(say) Stop autonomous control")
        
    # Use superclass to show any other events
      else:
        return JoyApp.onEvent(self,evt)
    return # ignoring non-KEYDOWN events

if __name__=="__main__":
  print """
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer. 
  """
  import sys
  if len(sys.argv)>1:
      app=RobotSimulatorApp(wphAddr=sys.argv[1], cfg={'windowSize' : [160,120]})
  else:
      app=RobotSimulatorApp(wphAddr=WAYPOINT_HOST, cfg={'windowSize' : [160,120]})
  app.run()


    
'''
class RobotSimulatorApp( JoyApp ):
  """Concrete class RobotSimulatorApp <<singleton>>
     A JoyApp which runs the DummyRobotSim robot model in simulation, and
     emits regular simulated tagStreamer message to the desired waypoint host.
     
     Used in conjection with waypointServer.py to provide a complete simulation
     environment for Project 1
  """    
  def __init__(self,wphAddr=WAYPOINT_HOST,*arg,**kw):
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml", *arg, **kw
      ) 
    self.srvAddr = (wphAddr, APRIL_DATA_PORT)
    
  def onStart( self ):
    # Set up socket for emitting fake tag messages
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
    s.bind(("",0))
    self.sock = s
    # Set up the sensor receiver plan
    self.sensor = SensorPlanTCP(self,server=self.srvAddr[0])
    self.sensor.start()
    self.robSim = RedRobotSim(self.sensor, fn=None)
    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1/20.0)
    progress("Using %s:%d as the waypoint host" % self.srvAddr)
    self.T0 = self.now

  def showSensors( self ):
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
    else:
      progress( "Waypoints: << no reading >>" )
  
  def emitTagMessage( self ):
    """Generate and emit and update simulated tagStreamer message"""
    self.robSim.refreshState()
    # Get the simulated tag message
    msg = self.robSim.getTagMsg()
    # Send message to waypointServer "as if" we were tagStreamer
    self.sock.sendto(msg, self.srvAddr)
    
  def onEvent( self, evt ):
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus(): 
      self.showSensors()
      progress( self.robSim.logLaserValue(self.now) )
      # generate simulated laser readings
    elif self.timeForLaser():
      self.robSim.logLaserValue(self.now)
    # update the robot and simulate the tagStreamer
    if self.timeForFrame(): 
      self.emitTagMessage()

    if evt.type == KEYDOWN:
      if evt.key == K_UP:
        self.robSim.move(0.5)
        return progress("(say) Move forward")
      elif evt.key == K_DOWN:
        self.robSim.move(-0.5)
        return progress("(say) Move back")
      elif evt.key == K_LEFT:
        self.robSim.turn(0.5)
        return progress("(say) Turn left")
      elif evt.key == K_RIGHT:
        self.robSim.turn(-0.5)
        return progress("(say) Turn right")
      elif evt.key == K_a:
        self.robSim.startAuto()
        return progress("(say) Moving autonomously")
      elif evt.key == K_s:
        self.robSim.stopAuto()
        return progress("(say) Stop autonomous control")
    # Use superclass to show any other events
      else:
        return JoyApp.onEvent(self,evt)
    return # ignoring non-KEYDOWN events

if __name__=="__main__":
  print """
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer. 
  """
  import sys
  if len(sys.argv)>1:
      app=RobotSimulatorApp(wphAddr=sys.argv[1], cfg={'windowSize' : [160,120]})
  else:
      app=RobotSimulatorApp(wphAddr=WAYPOINT_HOST, cfg={'windowSize' : [160,120]})
  app.run()
'''

