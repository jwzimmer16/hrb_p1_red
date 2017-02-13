# -*- coding: utf-8 -*-
from joy.decl import *
from joy import JoyApp, progress, DEBUG, Plan
from numpy import nan, asfarray, prod, isnan, pi, clip, sin, angle, sign, exp, round
from ckbot.dynamixel import MX64Module
from time import sleep, time as now

SERVO_NAMES = {
   0x23: 'MX1', 0x25: 'MX2'
}

class ServoWrapperMX(Plan):
    def __init__(self, app, servo, **kw):
        Plan.__init__(self,app,**kw)
        assert isinstance(servo, MX64Module), "only works with MX"
        self.servo = servo
        # servo units per rotation
        self.aScl = 36000.0  
        # Units for RPM
        self.rpmScl = 1 / 64.0
        # Position offset of zero position
        self.posOfs = 0
        # Orientation of motor; change to -1 to flip
        self.ori = 1
        # Update rate -- time to sleep between controller updates
        self.rate = 0.1 
        ### Internal variables
        self.desAng = 0
        self.desRPM = 0
        self.Kp = 25.0
        self.Kv = 0
        self._clearV()
        self._v = nan
        self.__dict__.update(kw)
        self._ensure_motor = self._set_motor
        self._ensure_servo = self._set_servo

    def behavior(self):
        """execute an interaction of the controller update loop"""
        while True:
          a = exp(1j * self.get_ang()*2*pi)
          a0 = exp(1j * self.desAng*2*pi)
          lead = angle(a / a0)
          if abs(lead)>0.7*pi:
              if "F" in DEBUG:
                  progress("FB desRPM %g out of range" % (self.desRPM))
                      # outside of capture range; ignore
              return
          pFB = clip(self.Kp * lead, -45, 45)
          if isnan(self._v):
              vFB = 0
          else:
              vFB = self.Kv * (self._v - self.desRPM)
          rpm = self.desRPM - pFB - vFB
          if abs(rpm)<0.1:
              rpm = 0
          if "F" in DEBUG:
              progress("FB desRPM %g p %g v %g" % (self.desRPM, pFB, vFB))
          # Push into the motor
          self._set_rpm(rpm)
          yield self.forDuration(self.rate)
          
    def _set_servo(self):
        """(private) set module in servo mode

            also configures the _ensure_* callbacks        
        """
        self.servo.set_mode(0)
        if "h" in DEBUG:
            progress("%s.set_mode('Servo')" % (self.servo.name))
        if self.servo.get_mode() == 0:
            self._ensure_servo = lambda: None
        self._ensure_motor = self._set_motor

    def _set_motor(self):
        """(private) set module in motor mode

            also configures the _ensure_* callbacks        
        """
        self.servo.set_mode(1)
        if "h" in DEBUG:
            progress("%s.set_mode('Motor')" % (self.servo.name))
        if self.servo.get_mode() == 1:
            self._ensure_motor = lambda: None
        self._ensure_servo = self._set_servo

    def _set_pos(self, pos):
        self.servo.set_pos(pos)

    def set_ang(self, ang):
        self.desAng = ang
        if "s" in DEBUG:
            progress("%s.set_angle(%g)" % (self.servo.name, ang))

    def get_ang(self):
        pos = self.servo.get_pos()
        ang = (pos - self.posOfs) / self.aScl / self.ori
        if "g" in DEBUG:
            progress("%s.get_angle --> %g" % (self.servo.name, ang))
        self._updateV(ang)
        return ang

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

    def adjustV(self, ratio):
        """adjust scaling of the velocity to make motors match up"""
        self.rpmScl = clip(self.rpmScl * ratio, 0.3 / 64, 3.0 / 64)

    def set_rpm(self, rpm):
        self.desRPM = rpm
        return self._set_rpm(rpm)

    def _set_rpm(self, rpm):
        """Push an RPM setting to the motor"""
        self._ensure_motor()
        tq = clip(self.rpmScl * self.ori * rpm, -0.999, 0.999)
        if "r" in DEBUG:
            progress("%s.set_torque(%g) <-- rpm %g" % (self.servo.name, tq, rpm))
        self.servo.set_torque(tq)

class SychronizedMX(JoyApp):
    def __init__(self, *arg, **kw):
        JoyApp.__init__(self, *arg, **kw)

    def onStart(self):
      self.smx = [ ServoWrapperMX(self,m)
        for m in self.robot.itermodules()
        if isinstance(m,MX64Module)
      ]
      for s in self.smx:
        s.start()
        
    def onEvent(self,evt):
      if evt.type != KEYDOWN:
        return
      else:
        f = "1234567890".find(evt.unicode)
        if f>=0:
          progress("MXs to " + str(f))
          for s in self.smx:
            s.set_ang(f*0.1)
          return
      return JoyApp.onEvent(self,evt)
      

if __name__ == '__main__':
    print """
    Example of position controlled rotation MXs
  
    The application can be terminated with 'q' or [esc]
    """
    import sys
    if len(sys.argv)!=2:
      sys.stderr.write("""
Usage:  python %s <<count>>
   count -- number of modules in cluster.
            Note: only MX64 modules will be controlled
""" % sys.argv[0])
      sys.exit(-1)
    app = SychronizedMX(
        robot=dict(count=int(sys.argv[1])
                   ,port=dict(TYPE='TTY', glob="/dev/ttyACM*", baudrate=115200)
        ))
    app.run()
