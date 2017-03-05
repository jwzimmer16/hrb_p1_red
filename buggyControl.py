# -*- coding: utf-8 -*-
"""
File: buggyControl.py

Top level Joyapp controller for our buggy. Enables manual control and initiation
of autonomous mode.

Created on Thu Feb 23 17:50:00 2017

@author: jwzimmer
"""
from joy import JoyApp, progress
from joy.decl import *
from joy.plans import Plan
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )
from syncmx import *
from sensorPlanTCP import SensorPlanTCP

class MovePlan( Plan ):
    
    def __init__(self,app):
        Plan.__init__(self,app,**kw)
        self.joyapp = app 
    def behavior(self):
        self.joyapp.lwheel.desRPM = 2
        self.joyapp.rwheel.desRPM = 2
        self.joyapp.turret.desRPM = 0
	
    
class RotatePlan( Plan ):
    
    def __init__(self,app):
        Plan.__init__(self,app)   
        
    def behavior(self):
        pass
    


class RedBuggyApp( JoyApp ):
    
    def __init__(self, wphAddr=WAYPOINT_HOST, *arg,**kw):
        JoyApp.__init__( self,
          confPath="$/cfg/JoyApp.yml", *arg, **kw
          ) 
        self.srvAddr = (wphAddr, APRIL_DATA_PORT)
        
    def onStart(self):
        # Init sensor plan
        self.sensor = SensorPlanTCP(self,server=self.srvAddr[0])
        self.sensor.start()
        # init servo objects
        self.lwheel = ServoWrapperMX(self,0x02)
        self.rwheel = ServoWrapperMX(self,0x55)
        self.turret = ServoWrapperMX(self,0x4c)
        self.lwheel.start()
        self.rwheel.start()
        self.turret.start()
        # Load plans here
        self.moveP = MovePlan(self)
        self.turnP = Rotate(self)
        self.autoPlan = AutoPlan(self, self.sensor, self.moveP, self.turnP)
    
    def onEvent(self, evt):
        if evt.type != KEYDOWN:
                return
                
        if evt.type == KEYDOWN:
            if evt.key == K_UP and not self.moveP.isRunning():
                # Forward plan
                self.moveP.start()
                return progress("(say) Move forward")
            elif evt.key == K_DOWN and not self.moveP.isRunning(): 
                # Backward plan
                return progress("(say) Move back")
            elif evt.key == K_LEFT and not self.turnP.isRunning():
                # Turn left plan
                return progress("(say) Turn left")
            elif evt.key == K_RIGHT and not self.turnP.isRunning():
                # Turn right plan
                return progress("(say) Turn right")
            elif evt.key == K_a and not(self.turnP.isRunning() or self.moveP.isRunning()):
                self.autoPlan.start()
                return progress("(say) Moving autonomously")
            elif evt.key == K_s and not(self.turnP.isRunning() or self.moveP.isRunning()):
                self.autoPlan.stop()
                return progress("(say) Stop autonomous control")
        
     
#runs on main        
if __name__=="__main__":
    print """
    Running the robot simulator

    Listens on local port 0xBAA (2986) for incoming waypointServer
    information, and also transmits simulated tagStreamer messages to
    the waypointServer. 
    """
    import sys
    if len(sys.argv)>1:
        app=RedBuggyApp(wphAddr=sys.argv[1], robot = dict(3))
    else:
        app=RedBuggyApp(wphAddr=WAYPOINT_HOST, robot = dict(3))
    app.run()
