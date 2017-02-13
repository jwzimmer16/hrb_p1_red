# -*- coding: utf-8 -*-
"""
Created on Thu Nov  3 17:05:26 2016

@author: birds
"""
DEBUG = []
from time import time as now
from ckbot.dynamixel import MX64Module
from numpy import nan,exp,angle,abs,clip,pi,isnan
class ServoWrapperMX(object):    
    def __init__(self, servo, logger=None, **kw):
        assert isinstance(servo, MX64Module), "only works with MX"
        self.servo = servo
        self.aScl = 36000.0  # servo units per rotation
        self.rpmScl = 1 / 64.0
        self.posOfs = 0
        self.ori = 1
        self.desAng = 0
        self.desRPM = 00
        # Use this rpm in dead zone
        self.deadRPM = 10
        self.Kp = 25.0
        self.logger = logger
        self._clearV()
        self._v = nan
        self.__dict__.update(kw)
        self._ensure_motor = self._set_motor
        self._ensure_servo = self._set_servo

    def _ang2srv(self,ang):
        """
        Convert cycle angle (range 0..1) to servo position
        
        Angle 0 is mapped to .posOfs, growing in direction .ori
        """
        return int((((float(ang)-self.posOfs)*self.ori) % 1.0) * self.aScl)
    
    def _srv2ang(self,srv):
        """
        Convert cycle angle (range 0..1) to servo position
        
        Angle 0 is mapped to .posOfs, growing in direction .ori
        """
        return float(srv)/self.aScl/self.ori + self.posOfs
        
    def doCtrl(self):
        """execute an interaction of the controller update loop"""
        a = exp(1j * self.get_ang()*2*pi)
        a0 = exp(1j * self.desAng*2*pi)
        lead = angle(a / a0)
        if abs(lead)>0.7*pi:
            pFB = 0
            rpm = self.deadRPM
            if "F" in DEBUG:
                progress("FB lead %5.2g out of range" % (lead))
                    # outside of capture range; ignore
        else:
            pFB = clip(self.Kp * lead, -45, 45)
            rpm = self.desRPM - pFB
            if abs(rpm)<0.1:
                rpm = 0
            if "F" in DEBUG:
                progress("FB desRPM %g p %g" % (self.desRPM, pFB))
        if self.logger:
            self.logger.write( "ctrl", nid=self.servo.node_id,  
                              desRPM=str(self.desRPM), pFB=str(pFB) )
        # Push into the motor
        self._set_rpm(rpm)

    def _set_servo(self):
        """(private) set module in servo mode

            also configures the _ensure_* callbacks so that _ensure_* only
            sends command to motor once
        """
        self.servo.set_mode(0)
        if "h" in DEBUG:
            progress("%s.set_mode('Servo')" % (self.servo.name))
        if self.servo.get_mode() == 0:
            self._ensure_servo = lambda: None
        self._ensure_motor = self._set_motor

    def _set_motor(self):
        """(private) set module in motor mode

            also configures the _ensure_* callbacks so that _ensure_* only
            sends command to motor once
        """
        self.servo.set_mode(1)
        if "h" in DEBUG:
            progress("%s.set_mode('Motor')" % (self.servo.name))
        if self.servo.get_mode() == 1:
            self._ensure_motor = lambda: None
        self._ensure_servo = self._set_servo

    def set_ang(self, ang):
        self.desAng = float(ang) %1.0
        if "s" in DEBUG:
            progress("%s.set_angle(%g)" % (self.servo.name, ang))
        if self.logger:
            self.logger.write( "set_ang", nid=self.servo.node_id, ang=str(ang) )

    def get_ang(self):
        ang = self._srv2ang(self.servo.get_pos())
        if "g" in DEBUG:
            progress("%s.get_angle --> %g" % (self.servo.name, ang))
        if self.logger:
            self.logger.write( "get_ang", nid=self.servo.node_id, ang=str(ang) )
        self._updateV(ang)
        return ang

    def get_v(self):
        """
        Get a velocity estimate
        """
        return self._v
        
    def _clearV(self):
        """Clear state of velocity estimator"""
        self._lastA = None
        self._lastT = None

    def _updateV(self, ang):
        """
        Push angle measurement into velocity estimator
        """
        t = now()
        if self._lastA is not None:
            da = ang - self._lastA
            dt = t - self._lastT
            vNew = da / dt * 60.0  # angle is in rotations, dt seconds
            # If no previous estimate --> use current estimate
            if isnan(self._v):
                self._v = vNew
            else:  # Use first order lowpass to smooth velocity update
                a = 0.2
                self._v = self._v * (1 - a) + vNew * a
        self._lastA = ang
        self._lastT = t

    def _set_rpm(self, rpm):
        """Push an RPM setting to the motor"""
        self._ensure_motor()
        tq = clip(self.rpmScl * self.ori * rpm, -0.999, 0.999)
        if "r" in DEBUG:
            progress("%s.set_torque(%g) <-- rpm %g" % (self.servo.name, tq, rpm))
        self.servo.set_torque(tq)

if __name__=="__main__":
    import ckbot.logical as L
    from time import sleep
    from pylab import linspace
    T0 = now()
    def progress(msg,sameLine=False):
        print "%5.3g " % (now()-T0),msg,
        if not sameLine:
            print
    c = L.Cluster(count=1)
    m = c.values()[0]
    w = ServoWrapperMX(m,ori=-1,posOfs=0.5)
    a = linspace(-2,2,103)
    b = [ w._srv2ang(w._ang2srv(ai)) for ai in a ]
    assert abs((a-b+.2)%1.0-.2).max() < 0.01
    
    DEBUG[:] = ['h','F','r','s']    
    last = now()
    while 1:
        w.doCtrl()
        if now()-last>1:
            goal = 0.3 * (long(now()) % 2)
            w.set_ang( goal )
            last = now()
            progress("goal %g" % goal)
        sleep(0.03)
        