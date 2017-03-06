
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
from autoPlan import AutoPlan
from joy import *
SERVO_NAMES = {
0.02:'MX1', 0x55: 'MX2', 0x4C: 'MX3'
}
class MovePlan( Plan ):
    
    def __init__(self,app):
	Plan.__init__(self,app) 
    	self.r = self.app.robot.at

    def behavior(self):
        self.r.lwheel.set_torque(1);
	self.r.rwheel.set_torque(1);
	
    
class RotatePlan( Plan ):
    
    def __init__(self,app):
        Plan.__init__(self,app)   
        self.r = self.app.robot.at

    def behavior(self):
        self.r.lwheel.set_torque(1);
	self.r.rwheel.set_torque(1);
    


class RedBuggyApp( JoyApp ):
    
    def __init__(self, wphAddr=WAYPOINT_HOST, *arg,**kw):
        JoyApp.__init__( self,
          confPath="$/cfg/JoyApp.yml", *arg, **kw
          ) 
        self.srvAddr = (wphAddr, APRIL_DATA_PORT)
        
    def onStart(self):
        # Init sensor plan
        #self.sensor = SensorPlanTCP(self,server=self.srvAddr[0])
        #self.sensor.start()
        # Load plans here
        self.moveP = MovePlan(self)
        self.turnP = RotatePlan(self)
        #self.autoPlan = AutoPlan(self, self.sensor, self.moveP, self.turnP)
    
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
                #self.autoPlan.start()
                return progress("(say) Moving autonomously")
            elif evt.key == K_s and not(self.turnP.isRunning() or self.moveP.isRunning()):
                #self.autoPlan.stop()
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
        app=RedBuggyApp(wphAddr=sys.argv[1], robot = dict(count = 3))
    else:
        app=RedBuggyApp(wphAddr=WAYPOINT_HOST, robot = dict(count = 3))
    app.run()
