"""
Python module joy.safety

Contains classes implementing the safety protocols enforced by JoyApp

Main classes:
  SafetyError -- exception class for indicating safety violations
  BatteryVoltage -- a safety check that uses get_voltage calls and
    enforces a lower bound on voltage
"""
from loggit import progress

class SafetyError( RuntimeError ):
  """
  Error used to indicate termination because of an unsafe working
  condition being detected
  """
  pass

  
class BatteryVoltage( object ):
  """
  Concrete class managing battery safety via a collection of objects 
  that support the get_voltage method.
  """
  def __init__(self, vmin, sensors, pollRate = 1.0):
    self.sensors = sensors
    self.vmin = vmin
    self.last = 0,None
    self.rate = pollRate
    progress("BatteryVoltage will test V<%g every %g seconds" % (vmin,pollRate))
  
  def poll( self, now ):
    """Polls voltage sensors listed in self.sensors
    for voltage measurements at the rate specified by self.rate
    
    If a measurement is ready, it is stored in self.last
    as a (time,voltage) pair, and this pair is returned.
    
    Otherwise, returns (None,None)
    
    If the voltage is below vmin -- raises a SafetyError
    """
    if not self.sensors:
      return None,None
    t,v = self.last
    if now - t < self.rate:
      return None,None
    s = self.sensors.pop(0)
    v = s.get_voltage()
    self.sensors.append(s)
    self.last = now, v
    if v<self.vmin:
      msg = "DANGER: voltage %g is below %g -- shutting down" % (v,self.vmin)
      raise SafetyError(msg)
    ###progress("Voltage %g" % v )
    return self.last
    
