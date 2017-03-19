# -*- coding: utf-8 -*-
"""
File: autoPlan.py

Contains the high level AutoPlanSM class, which implements control based on sensor reading
differences (perpendicularity of robot) and sum (distance from line) to determine actions 
in state machine.

Created on Wed Feb 22 13:30:48 2017

@author: jwzimmer, jswordy, menonmeg
"""

# The main program is a JoyApp
from joy import Plan, progress
import logging
import time
import math


# initialize constants 
MOVE_TORQUE = 0.2
MOVE_DUR = 0.5

TURN_TORQUE = 0.2
TURN_DUR = 0.25

RIGHT_TORQUE = TURN_TORQUE
LEFT_TORQUE = -TURN_TORQUE

ROBOT_WIDTH = 0.3 #in cm
TURRET_TORQUE = 0.1

SENSE_THRESH = 90

# two regions of interest: 
OFF_LINE = 90
ON_LINE = 150

DIFF_THRESH = 20 # acceptable threshold to be line-following
SUM_THRESH = 370
MIN_THRESH = 5 # minimum sensor threshold to indicate whether ortho to line
ANGLE_THRESH = 15 #acceptable threshold for robot trajectory alignment, degrees

WAIT_DUR = 0.5

TURN_RES = 90/6 #number of degrees per autonomous turn command

logdatetime = time.strftime("%I:%M:%S_%m_%d")
logging.basicConfig(format='(logdatetime)s:%(levelname)s:%(message)s',
		    filename='autoLog_' + logdatetime+'.log',
		    datefmt='%m/%d/%Y %I:%M:%S %p',
		    level=logging.INFO)

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
	self.wait = WAIT_DUR

    def writeAngleInit( self ):
	ts_w,w = self.sp.lastWaypoints
	
	i = 0
	while i < (len(w) - 2):
	    v1 = math.hypot(w[i][0] - w[i+1][0],w[i][1] - w[i+1][1])
	    v2 = math.hypot(w[i+1][0] - w[i+2][0], w[i+1][1] - w[i+2][1])
	    v3 = math.hypot(w[i][0] - w[i+2][0], w[i][1] - w[i+2][1])
	    num = ((v3**2) - (v2**2) - (v1**2))/(-2*v1*v2)
	    angle = math.acos(num)
	    angle = 180 - angle*(180/math.pi)
	    self.stateInfo["trajectoryList"].append(int(-1*angle))
	    i = i + 1

	#Then, calculate vector between remaining waypoints, i.e. (0,1), (1,2), (2,3), or just (1,2) and (2,3) using loop
	#Then, calculate angle between each pair of vectors using law of cosines
	progress(str(self.stateInfo["trajectoryList"]))

    def initState( self ):
        ts,f,b = self.sp.lastSensor
	ts_w,w = self.sp.lastWaypoints
	#eventually use law of cosines on waypoint list

        self.stateInfo["ts"] = ts
        self.stateInfo["f"] = f
        self.stateInfo["b"] = b
        self.stateInfo["lastf"] = f
        self.stateInfo["lastb"] = b
	self.stateInfo["trajectoryList"] = [0]

	self.stateInfo["orientation"] = 0
	self.stateInfo["forward"] = MOVE_TORQUE
	self.stateInfo["left"] = LEFT_TORQUE
	self.stateInfo["right"] = RIGHT_TORQUE

        self.stateInfo["estPos"] = (0,0)
        self.stateInfo["numWaypoints"] = len(w)
        
    def updateTrajectory( self ):
	#eventually use law of cosines and UPDATE trajectory list to check for moved waypoints
	#someLawofCosinesFcn
	self.writeAngleInit()
	self.stateInfo["trajectory"] = self.stateInfo["trajectoryList"][0]
        
    def updateState( self, timestamp, forw, back, waypoints):
        self.stateInfo["ts"] = timestamp
        self.stateInfo["lastf"] = forw
        self.stateInfo["lastb"] = back
        self.stateInfo["trajectory"] = self.stateInfo["trajectoryList"][0]
	self.stateInfo["numWaypoints"] = len(waypoints)   

    def behavior( self ):
        """
        Plan main loop
        """
	self.writeAngleInit()
	self.stateInfo["trajectory"] = self.stateInfo["trajectoryList"][0]
 
        while not self.stop:
           
            ts,f,b = self.sp.lastSensor
	    ts_w,w = self.sp.lastWaypoints
	    
	    sensor_sum = f + b
	    sensor_diff = f - b

            if ts > self.stateInfo["ts"]:

		#Off of the line, Go hunting!
		if ( f < MIN_THRESH or b < MIN_THRESH ):
		    theta_diff = self.stateInfo["orientation"]  - self.stateInfo["trajectory"] 

		    # if within 15 degress of trajectory, move forward
		    if( abs(theta_diff) < ANGLE_THRESH ):
                        self.moveP.torque = self.stateInfo["forward"]
			yield self.moveP
                        logging.info('Off-line, moving fwd. State info (f: '+str(f)+ ' b: '+str(b)+'w: '+str(w)+ ')')

		    # if more than 15 degrees off trajectory, turn to correct
		    elif(abs(theta_diff) >= ANGLE_THRESH ):
			n = theta_diff // TURN_RES

			# turn right
			if( n > 0):
		            const = 1
			#turn left
			elif ( n <= 0):
			    const = -1

			# make n an appropriate loop index
			n = abs(n)
			#check to see if the next turn will over-wind the robot (orient>180)
			if ( abs(self.stateInfo["orientation"] - theta_diff) > 150 ):
			    n = 12 - n
			    self.stateInfo["backwards"] = True
			    self.stateInfo["forward"] = -1*MOVE_TORQUE
			    self.stateInfo["left"], self.stateInfo["right"] = RIGHT_TORQUE, LEFT_TORQUE	
                      
			for i in range(n):
		            self.turnP.torque = const*self.stateInfo["right"]
			    self.stateInfo["orientation"] += -const*TURN_RES
			    yield self.turnP
			self.moveP.torque = self.stateInfo["forward"]
		 	yield self.moveP	
                        logging.info('Off-line, turn correcting. State info (turns: '+str(const)+'*'+str(n)+ ')')	

		# according to sensor data, robot is straddling the line
                elif (f > OFF_LINE and b > OFF_LINE):
		    if ( sensor_sum < SUM_THRESH ):
		        if ( sensor_diff > DIFF_THRESH and f > b):
                            self.moveP.torque = self.stateInfo["forward"]
			    self.turnP.torque = self.stateInfo["right"]
                            yield self.turnP
			    yield self.moveP
			    self.turnP.torque = self.stateInfo["left"]
			    yield self.turnP
                            logging.info('On-line, correcting to the left. State info (f: '+str(f)+ ' b: '+str(b)+'w: '+str(w)+ ')')
		        elif ( sensor_diff > DIFF_THRESH and f < b):
                            self.moveP.torque = self.stateInfo["forward"]
			    self.turnP.torque = self.stateInfo["left"]
                            yield self.turnP
			    yield self.moveP
			    self.turnP.torque = self.stateInfo["right"]
			    yield self.turnP
                            logging.info('On-line, correcting to the right. State info (f: '+str(f)+ ' b: '+str(b)+'w: '+str(w)+ ')')

                        elif ( sensor_diff < DIFF_THRESH):
			    self.moveP.torque = self.stateInfo["forward"]
			    yield self.moveP

		    elif (sensor_sum > SUM_THRESH): 
			self.turnP.torque = self.stateInfo["left"]
			yield self.turnP
			self.stateInfo["orientation"] += TURN_RES
			t_new,f_new,b_new = self.sp.lastSensor
			logging.info('On-Line, skewed. State info (f: '+str(f)+ ' b: '+str(b)+'w: '+str(w)+ ')')
		        if( f_new + b_new > sensor_sum ):
			    self.turnP.torque = self.stateInfo["right"] 
			    yield self.turnP
			    yield self.turnP   
			    self.stateInfo["orientation"] -= 2*TURN_RES
			    logging.info('On-line, skew overcorrection State info (f: '+str(f)+ ' b: '+str(b)+'w: '+str(w)+ ')')
          	             
            # pause after every action because there is sensor lag
            yield self.forDuration(self.wait)

	    #MUST do this before updateState, before numWaypoints is reset
	    #for now, placeholder for trajectory calc function
            ts_w,w = self.sp.lastWaypoints
	    if( len(w) < self.stateInfo["numWaypoints"]):
		self.updateTrajectory()

	    self.updateState(ts,f,b,w)
	    
            yield 
     
    def stopping(self): 
        # Set torque on both wheels to zero, used as a backup if buggy needs
        # to be stopped
        self.r.lwheel.set_torque(0)
        self.r.rwheel.set_torque(0)
        self.r.turret.set_torque(0)
	self.stop = True
        
      