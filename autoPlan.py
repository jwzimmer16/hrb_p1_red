# -*- coding: utf-8 -*-
"""
File: autoPlan.py

Contains the high level AutoPlan class which makes autonomous movement decisions
based on state and sensor information.

Created on Wed Feb 22 13:30:48 2017

@author: jwzimmer
"""

# The main program is a JoyApp
from joy import Plan

# Include all the modeling functions provided to the teams
#  this ensures that what the server does is consistent with the model given
#  to students during the development process
#from waypointShared import *
# Class uses sensor data from SensorPlanTCP object
#from sensorPlanTCP import SensorPlanTCP




class AutoPlan( Plan ):
    """
    AutoPlan is a concrete Plan subclass that uses SensorPlanTCP to get sensor
    data in order to make decisions in combination with tracked state info
    to move a robot autonomously through a track
    """
  
    def __init__(self, app, sensorPlan, movePlan, turnPlan, *arg, **kw):
        # Init superclass
        Plan.__init__(self, app, *arg, **kw )
        # Establish member objects/variables

        
        # Sensor plan object
        self.sp = sensorPlan
        # Dictionary containing information about state
        self.stateInfo = {}
        # Initialize stateInfo with holder values
        self.initState()
        # Save movement and turning plans
        self.moveP = movePlan
        self.turnP = turnPlan   

        
    def initState( self ):
        ts,f,b = self.sp.lastSensor

        self.stateInfo["ts"] = ts
        self.stateInfo["f"] = f
        self.stateInfo["b"] = b
        self.stateInfo["estPos"] = (0,0)
        self.stateInfo["numWaypoints"] = 0
        
        # initialize constants        
        self.MOVE_DIST = 0.1
        self.TURN_DIST = 0.3
        self.SENSE_THRESH = 5
        self.DIFF_THRESH = 40
        
    def updateState( self, timestamp, forw, back, waypoints):
        self.stateInfo["ts"] = timestamp
        self.stateInfo["f"] = forw
        self.stateInfo["b"] = back

            
    def behavior( self ):
        """
        Plan main loop
        """
        while True:
            ts,f,b = self.sp.lastSensor
            ts_w,w = self.sp.lastWaypoints
            
            if ts > self.stateInfo["ts"]:
            
                if (f<self.SENSE_THRESH or b<self.SENSE_THRESH):
                    if (len(w) == 4):
                    # Sensors not orthogonal and no delta info available
                    # Should be for initial movement from 1st waypoint
                        #self.move(self.MOVE_DIST)
                        self.moveP.dist = self.MOVE_DIST
                        yield self.moveP.start()
                        
                
                    elif (f < self.SENSE_THRESH and b >= self.SENSE_THRESH):
                        #self.turn(self.TURN_DIST)
                        #self.move(self.MOVE_DIST)
                        self.turnP.ang = self.TURN_DIST
                        self.moveP.dist = self.MOVE_DIST
                        yield self.turnP.start()
                        yield self.moveP.start()
                    
                    elif (b < self.SENSE_THRESH and f >= self.SENSE_THRESH):
                        #self.turn(-self.TURN_DIST)
                        #self.move(self.MOVE_DIST)
                        self.turnP.ang = -self.TURN_DIST
                        self.moveP.dist = self.MOVE_DIST
                        yield self.turnP.start()
                        yield self.moveP.start()
                        
                    else:
                        #self.turn(self.TURN_DIST * 5)
                        #self.move(self.MOVE_DIST * 2)
                        self.turnP.ang = 2*self.TURN_DIST
                        self.moveP.dist = 2*self.MOVE_DIST
                        yield self.turnP.start()
                        yield self.moveP.start()
                    
                elif (f > self.SENSE_THRESH and b > self.SENSE_THRESH):
                    
                    dist_dif = f-b
                    
                    if (abs(dist_dif) < self.DIFF_THRESH) :
                        #self.move(self.MOVE_DIST)
                        self.moveP.dist = self.MOVE_DIST
                        yield self.moveP.start()
                    elif (dist_dif >= self.DIFF_THRESH):
                        #self.turn(-self.TURN_DIST)
                        #self.move(self.MOVE_DIST)
                        self.turnP.ang = -self.TURN_DIST
                        self.moveP.dist = self.MOVE_DIST
                        yield self.turnP.start()
                        yield self.moveP.start()
                        
                    elif (dist_dif <= self.DIFF_THRESH):
                        #self.turn(self.TURN_DIST)
                        #self.move(self.MOVE_DIST)
                        self.turnP.ang = -self.TURN_DIST
                        self.moveP.dist = self.MOVE_DIST
                        yield self.turnP.start()
                        yield self.moveP.start()
                        
            self.updateState(ts,f,b,w)
            
      