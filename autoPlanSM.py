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
MOVE_DUR = 0.1

TURN_TORQUE = 0.2
TURN_DUR = 0.1

TURRET_TORQUE = 0.1

SENSE_THRESH = 2
DIFF_THRESH = 1

WAIT_DUR = 3


class AutoPlanSM( Plan ):
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
	self.wait = WAIT_DUR

        
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


    # Assuming f is right, b is left
    # Sense thresh near point where robot off the line        
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
                # Off the line
                if (f<SENSE_THRESH or b<SENSE_THRESH):
                    progress("(say) in state 1")
                    # compare f and b directy to determine action: 
                    # previous logic may have been over-constrained  
                    if ( f == 0 ):
                        progress("(say) turning left toward line")
                        self.turnP.torque = -TURN_TORQUE
                        self.moveP.torque = MOVE_TORQUE
                        yield self.turnP
                        yield self.moveP

                    elif (b == 0): #b>f
                        progress("(say) turning right toward line")
                        self.turnP.torque = TURN_TORQUE
                        self.moveP.torque = MOVE_TORQUE
                        yield self.turnP
                        yield self.moveP

                    else:
                        progress("(say) move and wait")
                        self.moveP.torque = 2*MOVE_TORQUE
                        yield self.moveP 
			yield self.forDuration(2)

                elif (f > SENSE_THRESH and b > SENSE_THRESH):
                    progress("(say) in state 2")
                    dist_dif = f-b
                    self.moveP.torque = MOVE_TORQUE
                    yield self.moveP
                    """
                    if (abs(dist_dif) < DIFF_THRESH):
                        progress("(say) in state 1")
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
                    """
            # pause after every action because there is sensor lag
            yield self.forDuration(self.wait)
                
            self.updateState(ts,f,b,w)
            yield 
     
    def stopping(self): 
        # Set torque on both wheels to zero, used as a backup if buggy needs
        # to be stopped
        self.r.lwheel.set_torque(0)
        self.r.rwheel.set_torque(0)
        self.r.turret.set_torque(0)
	self.stop = True
        
      