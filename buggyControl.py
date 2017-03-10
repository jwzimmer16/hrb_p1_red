

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
from autoPlanSM import AutoPlanSM
from joy import *
import time

SERVO_NAMES = {
    0x02:'MX1', 0x55:'MX2', 0x4C:'MX3'
}
MOVE_TORQUE = 0.2
MOVE_DUR = 0.25
WAIT_DUR = 5

TURN_TORQUE = 0.2
TURN_DUR = 0.2
ROBOT_WIDTH = 0.3 #in cm
TURRET_TORQUE = 0.1


# Changed this to just a testing app. Specifically, tests how movePlan and turnPlan work
# when called consecutively (i.e. testing that the delays work, verifying that the autoPlan
# has a reasonable method of executing consecutive actions)
class dummyAutoPlan( Plan ):
    """
    Plan that moves buggy forward or backward by setting equal torque on wheels
    for a fixed duration of time.

    Torque and Duration are public variables that can be modified on the fly.
    To move buggy backwards set torque to a negative value.
    """

    def __init__(self,app, movePlan, turnPlan ):
	Plan.__init__(self,app)
        self.r = self.app.robot.at
        self.torque = MOVE_TORQUE
        self.moveP = movePlan
        self.turnP = turnPlan
	self.wait = 6

    def behavior(self):
        # Set torque to move wheels. Right wheel must be opposite of left wheel
        # due to orientation of motors
        self.turnP.torque = self.torque
        self.moveP.torque = -1*self.torque
        yield self.moveP
        yield self.turnP
        yield self.moveP
        yield self.turnP
        yield self.moveP
        yield self.turnP

        yield self.forDuration(self.wait)

	self.r.lwheel.set_torque(0)
        self.r.rwheel.set_torque(0)
        self.r.turret.set_torque(0)


class MovePlan( Plan ):
    """
    Plan that moves buggy forward or backward by setting equal torque on wheels
    for a fixed duration of time.

    Torque and Duration are public variables that can be modified on the fly.
    To move buggy backwards set torque to a negative value.
    """

    def __init__(self,app):
	Plan.__init__(self,app)
        self.r = self.app.robot.at
        self.torque = MOVE_TORQUE
        self.dur = MOVE_DUR
        self.wait = WAIT_DUR


    def behavior(self):
        # Set torque to move wheels. Right wheel must be opposite of left wheel
        # due to orientation of motors
        self.r.lwheel.set_torque(-1*self.torque)
        self.r.rwheel.set_torque(self.torque)

        yield self.forDuration(self.dur)

	self.r.lwheel.set_torque(0)
        self.r.rwheel.set_torque(0)
        self.r.turret.set_torque(0)


class RotatePlan( Plan ):
    """
    Plant that rotates buggy by setting torque on wheels for fixed duration.

    Torque and direction are public so they can be modified to change direction
    and speed of rotation. To rotate buggy in opposite direction make torque a
    negative number.
    """

    def __init__(self,app):
        Plan.__init__(self,app)
        self.r = self.app.robot.at
        self.torque = TURN_TORQUE
        self.dur = TURN_DUR
        self.wait = WAIT_DUR


    def behavior(self):

        # Set torque to move wheels. Motors are oriented to move in opposite
        # directions with same torque
        self.r.lwheel.set_torque(self.torque)
        self.r.rwheel.set_torque(self.torque)
        self.r.turret.set_torque(self.torque*0.4)        

        yield self.forDuration(self.dur)

	self.r.lwheel.set_torque(0)
        self.r.rwheel.set_torque(0)
        self.r.turret.set_torque(0)

class TurretPlan( Plan ):
    """
    Plant that rotates buggy by setting torque on wheels for fixed duration.

    Torque and direction are public so they can be modified to change direction
    and speed of rotation. To rotate buggy in opposite direction make torque a
    negative number.
    """

    def __init__(self,app):
        Plan.__init__(self,app)
        self.r = self.app.robot.at
        self.turretSpeed = TURRET_TORQUE

    def behavior(self):
        # Set timeout stamp
        timeout = time.time() + 0.2

        # Set torque to adjust turret
        self.r.turret.set_torque(self.turretSpeed)        

        # Set torque to 0 after time duration to stop moving
        while True:
            if time.time() > timeout:
                self.r.turret.set_torque(0)
                break
     	yield

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
        # Load plans here
        self.moveP = MovePlan(self)
        self.turnP = RotatePlan(self)
        self.turretP = TurretPlan(self)
        self.dummyAutoP = dummyAutoPlan(self, self.moveP, self.turnP)
        #switched this to autplan sm
        self.autoP = AutoPlanSM(self,self.sensor, self.moveP, self.turnP)

    def onEvent(self, evt):
        if evt.type != KEYDOWN:
                return

        if evt.type == KEYDOWN:
            if evt.key == K_UP and not self.moveP.isRunning():
                # Forward plan
                self.moveP.torque = MOVE_TORQUE
                self.moveP.start()
                return progress("(say) Move forward")
            elif evt.key == K_DOWN and not self.moveP.isRunning():
                # Backward plan
                self.moveP.torque = -1 * MOVE_TORQUE
                self.moveP.start()
                return progress("(say) Move back")
            elif evt.key == K_LEFT and not self.turnP.isRunning():
                # Turn left plan
                self.turnP.torque = -1 * TURN_TORQUE
                self.turnP.start()
                return progress("(say) Turn left")
            elif evt.key == K_RIGHT and not self.turnP.isRunning():
                # Turn right plan
                self.turnP.torque =  TURN_TORQUE
                self.turnP.start()
                return progress("(say) Turn right")
            elif evt.key == K_a and not(self.turnP.isRunning() or self.moveP.isRunning()):
                self.dummyAutoP.start()
                return progress("(say) Moving autonomously")
            elif evt.key == K_s and not(self.turnP.isRunning() or self.moveP.isRunning()):
                self.dummyAutoP.stop()
                return progress("(say) Stop autonomous control")
            elif evt.key == K_m and not (self.turnP.isRunning() or  self.moveP.isRunning()):
                self.turretP.turretSpeed = TURRET_TORQUE 
                self.turretP.start()
                return progress("(say) adjusted turret")
            elif evt.key == K_n and not (self.turnP.isRunning() or  self.moveP.isRunning()):
                self.turretP.turretSpeed = -1*TURRET_TORQUE 
                self.turretP.start()
                return progress("(say) adjusted turret")
            elif evt.key == K_e and not (self.turnP.isRunning() or  self.moveP.isRunning()):
                self.autoP.start()
                return progress("(say) Moving autonomously")
            elif evt.key == K_r and not (self.turnP.isRunning() or  self.moveP.isRunning()):
                self.autoP.stopping()
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
