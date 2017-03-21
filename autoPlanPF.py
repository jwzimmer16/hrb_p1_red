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
	    angle = angle*(180/math.pi)
	    self.stateInfo["trajectoryList"].append(int(-1*(angle + self.stateInfo["trajectory"])))
	    i = i + 1

	#Then, calculate vector between remaining waypoints, i.e. (0,1), (1,2), (2,3), or just (1,2) and (2,3) using loop
	#Then, calculate angle between each pair of vectors using law of cosines
	progress(str(self.stateInfo["trajectoryList"]))
    
    def writeAngle( self ):
	ts_w,w = self.sp.lastWaypoints
	
	i = 0
	while i < (len(w) - 2):
	    v1 = math.hypot(w[i][0] - w[i+1][0],w[i][1] - w[i+1][1])
	    v2 = math.hypot(w[i+1][0] - w[i+2][0], w[i+1][1] - w[i+2][1])
	    v3 = math.hypot(w[i][0] - w[i+2][0], w[i][1] - w[i+2][1])
	    num = ((v3**2) - (v2**2) - (v1**2))/(-2*v1*v2)
	    angle = math.acos(num)
	    angle = angle*(180/math.pi)
	    self.stateInfo["trajectoryList"][i] = (int(-180+angle))
	    i = i + 1

	if (len(w) == 2):
	    self.stateInfo["trajectoryList"].pop(0)

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
	self.stateInfo["trajectory"] = 0
	self.stateInfo["trajectoryList"] = [0]

	self.stateInfo["orientation"] = 0
	self.stateInfo["forward"] = MOVE_TORQUE
	self.stateInfo["left"] = LEFT_TORQUE
	self.stateInfo["right"] = RIGHT_TORQUE

        self.stateInfo["estPos"] = (0,0)
        self.stateInfo["numWaypoints"] = len(w)
	self.stateInfo["state"] = 1 # 1: Hunting Mode, 2: Line Following, 3: Correcting
        self.stateInfo["switch"] = True
	self.stateInfo["orientationChecked"] = False #once per trajectory, want to check orientation
        
    def updateTrajectory( self ):
	#eventually use law of cosines and UPDATE trajectory list to check for moved waypoints
	#someLawofCosinesFcn
	self.writeAngle()
	self.stateInfo["trajectory"] = self.stateInfo["trajectoryList"][0]
	progress("In update trajectory, new angle is " + str(self.stateInfo["trajectory"]))
        self.stateInfo["switch"] = True
	self.stateInfo["orientationChecked"] = False 

    def updateSoftwareState( self,f,b ):
	#ts,f,b = self.sp.lastSensor
	if ( f <= MIN_THRESH or b <= MIN_THRESH or self.stateInfo["switch"] == True ): 
	    self.stateInfo["state"] = 1
	elif ( f >= OFF_LINE or b >= OFF_LINE ): 
	    self.stateInfo["state"] = 2	
	elif ( ((f > MIN_THRESH and f < OFF_LINE ) and (b > 90)) or (( b > MIN_THRESH and b < OFF_LINE ) and (f>90)) or ((b > MIN_THRESH and b < OFF_LINE)and(f > MIN_THRESH and f < OFF_LINE))):
	   self.stateInfo["state"] = 3
	progress("In software update: " + str(self.stateInfo["state"]))
	progress( "f: " + str(f))
	progress( "b: " + str(b))
        
    def updateState( self, timestamp, forw, back, waypoints):
        self.stateInfo["ts"] = timestamp
        self.stateInfo["f"] = forw
        self.stateInfo["b"] = back
        self.stateInfo["trajectory"] = self.stateInfo["trajectoryList"][0]
	self.stateInfo["numWaypoints"] = len(waypoints) 
	self.stateInfo["orientation"] = self.stateInfo["orientation"] % 360
  
    def updateLog( self, state, f, b, w):
	if (state == 1):
	    st = "Hunting Mode"
	elif ( state == 2 ):
	    st = "Line Following"
	elif ( state == 3 ):
	    st = "Correcting"
	logging.info("In State "+str(state)+":" + str(st))
        logging.info('Sensor Info: (f: '+str(self.stateInfo["f"])+ ' b: '+str(self.stateInfo["b"])+' w: '+str(w)+ ')')
        logging.info('Trajectory Info: (Current Trajectory: '+str(self.stateInfo["trajectory"])+ ' Trajectory List: '+str(self.stateInfo["trajectoryList"])+ ')')
        logging.info('Robot Info: (Orientation: '+str(self.stateInfo["orientation"])+ ')')
        logging.info('Software Info: (switch: '+ str(self.stateInfo["switch"]) + ')')    

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
		if ( self.stateInfo["state"] == 1 ):
		    #theta_diff = self.stateInfo["orientation"]  - self.stateInfo["trajectory"]
		    theta_diff = -self.stateInfo["trajectory"] 

		    # if within 15 degress of trajectory, move forward
		    if( abs(theta_diff) < ANGLE_THRESH ):
                        self.moveP.torque = self.stateInfo["forward"]
			yield self.moveP
			yield self.moveP
		        self.updateLog(1,f,b,w)

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
			#if ( abs(self.stateInfo["orientation"] + theta_diff) > 150 ):
			#    n = 12 - n
			#    self.stateInfo["backwards"] = True
			#    self.stateInfo["forward"] = -1*MOVE_TORQUE
			#    self.stateInfo["left"], self.stateInfo["right"] = RIGHT_TORQUE, LEFT_TORQUE	
                      
			for i in range(n):
		            self.turnP.torque = const*self.stateInfo["right"]
			    #self.stateInfo["orientation"] += -const*TURN_RES
			    yield self.turnP
			
			self.stateInfo["orientation"]  = self.stateInfo["trajectory"]
			self.moveP.torque = self.stateInfo["forward"]
		 	yield self.moveP
			yield self.moveP
			yield self.moveP	
			self.updateLog(1,f,b,w)

                    self.stateInfo["switch"] = False	
		    progress("switch changed" + str(self.stateInfo["switch"]))

		# according to sensor data, robot is straddling the line
		elif ( self.stateInfo["state"] == 2 ):
		    if ( sensor_sum < SUM_THRESH ):
		        if ( sensor_diff > DIFF_THRESH and f > b):
                            self.moveP.torque = self.stateInfo["forward"]
			    self.turnP.torque = self.stateInfo["right"]
                            yield self.turnP
			    yield self.moveP
			    self.turnP.torque = self.stateInfo["left"]
			    yield self.turnP
			    self.updateLog(2,f,b,w)

		        elif ( sensor_diff > DIFF_THRESH and f < b):
                            self.moveP.torque = self.stateInfo["forward"]
			    self.turnP.torque = self.stateInfo["left"]
                            yield self.turnP
			    yield self.moveP
			    self.turnP.torque = self.stateInfo["right"]
			    yield self.turnP
			    self.updateLog(2,f,b,w)

                        elif ( sensor_diff < DIFF_THRESH):
			    self.moveP.torque = self.stateInfo["forward"]
			    yield self.moveP
			

		    elif (sensor_sum > SUM_THRESH): 
			self.turnP.torque = self.stateInfo["left"]
			yield self.turnP
			self.stateInfo["orientation"] += TURN_RES
			t_new,f_new,b_new = self.sp.lastSensor
			self.updateLog(2,f,b,w)

		        if( f_new + b_new > sensor_sum ):
			    self.turnP.torque = self.stateInfo["right"] 
			    yield self.turnP
			    yield self.turnP   
			    self.stateInfo["orientation"] -= 2*TURN_RES
			    self.updateLog(2,f,b,w)

		elif ( self.stateInfo["state"] == 3 ):
		    theta_diff =  - self.stateInfo["trajectory"] 
		    n = theta_diff // TURN_RES

		    # turn right
		    if( n > 0):
			const = 1
		    #turn left
		    elif ( n <= 0):
		        const = -1

		    # make n an appropriate loop index
		    n = abs(n)
		              
		    for i in range(n):
		        self.turnP.torque = const*self.stateInfo["right"]
		        self.stateInfo["orientation"] += -const*TURN_RES
		        yield self.turnP
		    n = 6
		    for i in range(n):
			self.turnP.torque = const*self.stateInfo["right"]
			self.stateInfo["orientation"] += -const*TURN_RES
			yield self.turnP
			
		    for i in range(n):
		        self.moveP.torque = -self.stateInfo["forward"]
		        yield self.moveP

		    for i in range(n):
		        self.turnP.torque = const*self.stateInfo["right"]
		        self.stateInfo["orientation"] += const*TURN_RES
		        yield self.turnP
	

		    self.updateLog(3,f,b,w)
          	             
            # pause after every action because there is sensor lag
            yield self.forDuration(self.wait)

	    tss, fs, bs = self.sp.lastSensor
            self.updateSoftwareState(fs, bs)

            ts_w,w = self.sp.lastWaypoints
	    if( len(w) < self.stateInfo["numWaypoints"]):
		self.updateTrajectory()

	    self.updateSoftwareState(f,b)
	    self.updateState(ts,f,b,w)
	    
            yield 
     
    def stopping(self): 
        # Set torque on both wheels to zero, used as a backup if buggy needs
        # to be stopped
        self.r.lwheel.set_torque(0)
        self.r.rwheel.set_torque(0)
        self.r.turret.set_torque(0)
	self.stop = True
        
      