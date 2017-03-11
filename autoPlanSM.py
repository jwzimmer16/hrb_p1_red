# -*- coding: utf-8 -*-
"""
File: autoPlan.py

Contains the high level AutoPlanSM class, which implements control based on sensor reading
differences (perpendicularity of robot) and sum (distance from line) to determine actions 
in state machine.

Created on Wed Feb 22 13:30:48 2017

@author: jwzimmer
"""

# The main program is a JoyApp
from joy import Plan, progress
import logging
import time


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

DIFF_THRESH = 20
SUM_THRESH = 370

WAIT_DUR = 3

FILT_SAMPLES = 50 #Number of samples considered by simple moving average filter

TURN_RES = 90/12 #number of degrees per autonomous turn command

logdatetime = time.strftime("%I:%M:%S_%m_%d")
logging.basicConfig(format='(logdatetime)s:%(levelname)s:%(message)s',
		    filename='autoLog_' + logdatetime+'.log',
		    datefmt='%m/%d/%Y %I:%M:%S %p',
		    level=logging.INFO)

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
	self.stateInfo["angle"] = 0
	self.stateInfo["backwards"] = False
        self.stateInfo["estPos"] = (0,0)
        self.stateInfo["numWaypoints"] = 0
        
        
    def updateState( self, timestamp, forw, back, waypoints):
        self.stateInfo["ts"] = timestamp
        self.stateInfo["f"] = forw
        self.stateInfo["b"] = back

    # Simple moving average filter, also acts as a delay (eventually will be Kalman filter)
    # Potentially disastrous approach to measurment filtering, should be filtering
    # line position itself (but no obvious way to back f and b out from that)
    def filterState( self, ts, forward, back): 
        fSum = forward
	bSum = backward
      
	while ts > self.stateInfo["ts"] and ts < (self.stateInfo["ts"] + FILT_SAMPLES - 1):
            t,f,b = self.sp.lastSensor
	    fSum += f
      	    bSum += b
	self.stateInfo["f"] = fSum/FILT_SAMPLES 
	self.stateInfo["b"] = bSum/FILT_SAMPLES       

    def behavior( self ):
        """
        Plan main loop
        """
        while not self.stop:
           
            ts,f,b = self.sp.lastSensor
            ts_w,w = self.sp.lastWaypoints
	    sensor_sum = f + b
	    sensor_diff = f - b
	    # comment below out when filter finished
	    #self.filterState( ts, f, b)
            progress("(say)f: " +str(f))
	    progress("(say)b: "+str(b))
            progress("(say)w: "+str(w))

            if ts > self.stateInfo["ts"]:

            	#region of line-following logic
		# if the robot sensor sum is ~340, then perpendicular to line
		# if greater than 340, it is skew to the line
                if (f > OFF_LINE and b > OFF_LINE):
                    if (len(w) == 4 and f == None or b == None):
                    # Sensors not orthogonal and no delta info available
                    # Should be for initial movement from 1st waypoint

                        self.moveP.torque = MOVE_TORQUE
                        yield self.moveP
			progress("(say) Hunting")
                        logging.info('Hunting Mode. State info (f: '+str(f)+ ' b: '+str(b)+'w: '+str(w)+ ')')
                    
		    if ( sensor_sum < SUM_THRESH ):
		        if ( sensor_diff > DIFF_THRESH and f > b):
                            self.moveP.torque = MOVE_TORQUE
			    self.turnP.torque = RIGHT_TORQUE
                            yield self.turnP
			    yield self.moveP
			    self.turnP.torque = LEFT_TORQUE
			    yield self.turnP
                            logging.info('On-line, correcting to the left. State info (f: '+str(f)+ ' b: '+str(b)+'w: '+str(w)+ ')')
		        elif ( sensor_diff > DIFF_THRESH and f < b):
                            self.moveP.torque = MOVE_TORQUE
			    self.turnP.torque = LEFT_TORQUE
                            yield self.turnP
			    yield self.moveP
			    self.turnP.torque = RIGHT_TORQUE
			    yield self.turnP
                            logging.info('On-line, correcting to the right. State info (f: '+str(f)+ ' b: '+str(b)+'w: '+str(w)+ ')')

                        elif ( sensor_diff < DIFF_THRESH):
			    self.moveP.torque = MOVE_TORQUE
			    yield moveP

		    elif (sensor_sum > SUM_THRESH): 
			self.turnP.torque = LEFT_TORQUE
			yield self.turnP
			t_new,f_new,b_new = self.sp.lastSensor
			logging.info('On-Line, skewed. State info (f: '+str(f)+ ' b: '+str(b)+'w: '+str(w)+ ')')
		        if( f_new + b_new > sensor_sum ):
			    self.turnP.torque = RIGHT_TORQUE 
			    yield self.turnP
			    yield self.turnP   
			    logging.info('On-line, skew overcorrection State info (f: '+str(f)+ ' b: '+str(b)+'w: '+str(w)+ ')'
          	             
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
        
      