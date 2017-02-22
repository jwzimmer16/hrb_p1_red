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
from waypointShared import *
# Class uses sensor data from SensorPlanTCP object
from sensorPlanTCP import SensorPlanTCP



class AutoPlan( Plan ):
    """
    AutoPlan is a concrete Plan subclass that uses SensorPlanTCP to get sensor
    data in order to make decisions in combination with tracked state info
    to move a robot autonomously through a track
    """
  
    def __init__(self, app, sensorPlan, *arg, **kw):
        # Init superclass
        Plan.__init__(self, app, *arg, **kw )
        # Establish member objects/variables
        
        # Sensor plan object
        self.sp = sensorPlan
        # Dictionary containing information about state
        self.stateInfo = {}
        # Initialize stateInfo with holder values
        self.initState()
        
    def initState( self ):
        ts,f,b = self.sensorPlan.lastSensor
        ts_w,w = self.sensorPlan.lastWaypoints
        self.stateInfo["ts"] = ts
        self.stateInfo["f"] = f
        self.stateInfo["b"] = b
        self.stateInfo["estPos"] = w[0]
        self.stateInfo["numWaypoints"] = len(w)
  
    def behavior( self ):
        """
        Plan main loop
        """
        while True:
            ts,f,b = self.sensorPlan.lastSensor
            ts_w,w = self.sensorPlan.lastWaypoints
            
            
      