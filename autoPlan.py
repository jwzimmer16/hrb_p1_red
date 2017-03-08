# -*- coding: utf-8 -*-
"""
File: autoPlan.py

Contains the high level AutoPlan class which makes autonomous movement decisions
based on state and sensor information.

Created on Wed Feb 22 13:30:48 2017

@author: jwzimmer
"""

# The main program is a JoyApp
from joy import Plan, progress

# Include all the modeling functions provided to the teams
#  this ensures that what the server does is consistent with the model given
#  to students during the development process
#from waypointShared import *
# Class uses sensor data from SensorPlanTCP object
#from sensorPlanTCP import SensorPlanTCP


# initialize constants 
MOVE_TORQUE = 0.2
MOVE_DUR = 0.5

TURN_TORQUE = 0.2
TURN_DUR = 0.25

ROBOT_WIDTH = 0.3 #in cm
TURRET_TORQUE = 0.1

SENSE_THRESH = 5
DIFF_THRESH = 10


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
	self.stop = False  

        
    def initState( self ):
        ts,f,b = self.sp.lastSensor

        self.stateInfo["ts"] = ts
        self.stateInfo["f"] = f
        self.stateInfo["b"] = b
        self.stateInfo["estPos"] = (0,0)
        self.stateInfo["numWaypoints"] = 0
        
        
    def updateState( self, timestamp, forw, back, waypoints):
        self.stateInfo["ts"] = timestamp
        self.stateInfo["f"] = forw
        self.stateInfo["b"] = back

            
    def behavior( self ):
        """
        Plan main loop
        """
        while not self.stop:
            ts,f,b = self.sp.lastSensor
            ts_w,w = self.sp.lastWaypoints
            progress("(say)f: " +str(f))
	    progress("(say)b: "+str(b))
   
            if ts > self.stateInfo["ts"]:
            
                if (f<SENSE_THRESH or b<SENSE_THRESH):
                    if (len(w) == 4):
                    # Sensors not orthogonal and no delta info available
                    # Should be for initial movement from 1st waypoint

                        self.moveP.torque = MOVE_TORQUE
                        yield self.moveP

                    elif (f < SENSE_THRESH and b >= SENSE_THRESH):

                        self.turnP.torque = -TURN_TORQUE
                        self.moveP.torque = MOVE_TORQUE
                        yield self.turnP
                        yield self.moveP

                    elif (b < SENSE_THRESH and f >= SENSE_THRESH):

                        self.turnP.torque = TURN_TORQUE
                        self.moveP.torque = MOVE_TORQUE
                        yield self.turnP
                        yield self.moveP

                    else:
                        self.turnP.torque = 2*TURN_TORQUE
                        self.moveP.torque = 2*MOVE_TORQUE
                        yield self.turnP
                        yield self.moveP 
                    
                elif (f > SENSE_THRESH and b > SENSE_THRESH):
                    
                    dist_dif = f-b
                    
                    if (abs(dist_dif) < DIFF_THRESH):
   
                        self.moveP.torque = MOVE_TORQUE
                        yield self.moveP

                    elif (dist_dif >= DIFF_THRESH):

                        self.turnP.torque = -TURN_TORQUE
                        self.moveP.torque = MOVE_TORQUE
                        yield self.turnP
                        yield self.moveP
                        
                    elif (dist_dif <= DIFF_THRESH):

                        self.turnP.torque = TURN_TORQUE
                        self.moveP.torque = MOVE_TORQUE
                        yield self.turnP
                        yield self.moveP
                        
            self.updateState(ts,f,b,w)
            yield 
     
    def stopping(self): 
        # Set torque on both wheels to zero, used as a backup if buggy needs
        # to be stopped
        self.r.lwheel.set_torque(0)
        self.r.rwheel.set_torque(0)
        self.r.turret.set_torque(0)
	self.stop = True
        
      