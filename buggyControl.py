

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
import time

SERVO_NAMES = {
    0x02:'MX1', 0x55:'MX2', 0x4C:'MX3'
}
MOVE_TORQUE = 0.04
MOVE_DUR = 5

TURN_TORQUE = 0.04
TURN_DUR = 5


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


    def behavior(self):
        # Set timeout stamp
        timeout = time.time() + self.dur

        # Set torque to move wheels. Right wheel must be opposite of left wheel
        # due to orientation of motors
        self.r.lwheel.set_torque(self.torque)
        self.r.rwheel.set_torque(-1 * self.torque)

        # Set torque to 0 after time duration to stop moving
        while True:
            if time.time() > timeout:
                self.r.lwheel.set_torque(0)
                self.r.rwheel.set_torque(0)
                break

    def stopping(self):
        # Set torque on both wheels to zero, used as a backup if buggy needs
        # to be stopped
        self.r.lwheel.set_torque(0)
        self.r.rwheel.set_torque(0)

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


    def behavior(self):
        # Set timeout stamp
        timeout = time.time() + self.dur

        # Set torque to move wheels. Motors are oriented to move in opposite
        # directions with same torque
        self.r.lwheel.set_torque(self.torque)
        self.r.rwheel.set_torque(self.torque)
        self.r.turret.set_torque(-self.torque)        

        # Set torque to 0 after time duration to stop moving
        while True:
            if time.time() > timeout:
                self.r.lwheel.set_torque(0)
                self.r.rwheel.set_torque(0)
                break

    def stopping(self):
        # Set torque on both wheels to zero, used as a backup if buggy needs
        # to be stopped
        self.r.lwheel.set_torque(0)
        self.r.rwheel.set_torque(0)


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
                self.moveP.torque = MOVE_TORQUE
                self.moveP.start()
                return progress("(say) Move forward")
            elif evt.key == K_DOWN and not self.moveP.isRunning():
                # Backward plan
                self.moveP.torque = -1 * MOVE_TORQUE
                self.movep.start()
                return progress("(say) Move back")
            elif evt.key == K_w and not self.moveP.isRunning() or self.moveP.isRunning():
                self.moveP.stopping()
                self.turnP.stopping()
                return progress("(say) Stopping")
            elif evt.key == K_LEFT and not self.turnP.isRunning():
                # Turn left plan
                self.turnP.torque = TURN_TORQUE
                self.turnP.start()
                return progress("(say) Turn left")
            elif evt.key == K_RIGHT and not self.turnP.isRunning():
                # Turn right plan
                self.turnP.torque = -1 * TURN_TORQUE
                self.turnP.start()
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
