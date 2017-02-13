# -*- coding: utf-8 -*-
"""
FILE: sensorPlan.py

Contains the SensorPlan class, which interfaces with the waypointServer to
give sensor readings from the robot

Created on Sat Sep  6 12:02:16 2014

@author: shrevzen-home
"""

# Uses UDP sockets to communicate
from socket import (
  socket, AF_INET,SOCK_STREAM, IPPROTO_TCP, error as SocketError,
  )
# Packets are JSON enocoded
from json import loads as json_loads

# The main program is a JoyApp
from joy import Plan, progress

# Include all the modeling functions provided to the teams
#  this ensures that what the server does is consistent with the model given
#  to students during the development process
from waypointShared import *

class SensorPlanTCP( Plan ):
  """
  SensorPlan is a concrete Plan subclass that uses a TCP socket to 
  and decode WayPoint messages
  """
  def __init__( self, app, *arg, **kw ):
    if kw.has_key("server"):
      self.svrAddr = (kw['server'],WAYPOINT_MSG_PORT)
      del kw['server']
    else:
      self.svrAddr = (WAYPOINT_HOST,WAYPOINT_MSG_PORT)    
    Plan.__init__(self, app, *arg, **kw )
    self.sock = None
    self.lastSensor = (0,None,None)
    self.lastWaypoints = (0,[])
    self.buf = ''
          
  def _connect( self ):
    """Set up the socket"""
    s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)
    s.connect(self.svrAddr)
    s.setblocking(0)
    self.sock = s
    self.buf = ''
    progress("Sensor connected to %s:%d" % self.svrAddr)

  def stop( self ):
    """(called when stopping) clean up the socket, if there is one"""
    if self.sock is not None:
      self.sock.close()
    self.sock = None

  def ensureConnection( self ):
    """(sub-behavior) loops until the socket is up"""
    while True:
      # if not connected --> try to connect
      if self.sock is None:
        self._connect()
      # if not connected --> sleep for a bit
      if self.sock is None:
        yield self.forDuration(0.1)
      else: # otherwise --> done, return to parent 
        return 
  
  def sendto(self,*argv,**kw):
    """
    Expose the socket sendto to allow owner to use socket for sending
    
    Will try to connect a socket if there is none
    """
    # if not connected --> try to connect
    if self.sock is None:
      self._connect()
    return self.sock.sendto(*argv,**kw)
    
  def _nextMessage( self ):
        """
        Obtain the next message; kill socket on error. 
        
        returns '' if nothing was received
        """
        # receive an update / skip
        try:
          msg = self.sock.recv(1024)
        except SocketError, se:
          # If there was no data on the socket
          #   --> not a real error, else kill socket and start a new one
          if se.errno != 11:
            progress("Connection failed: "+str(se))
            self.sock.close()
            self.sock = None
          return ''
        # Use previously buffered data
        msg = self.buf + msg
        # End of dictionary should be end of message
        f = msg.find("}")
        if f<0:
          self.buf = msg
          return ''
        # Pull out the first dictionary
        self.buf = msg[f+1:]
        return msg[:f+1]
        
  def behavior( self ):
    """
    Plan main loop    
    """
    while True:
      # If no socket set up --> activate the ensureConnection sub-behavior to fix
      if self.sock is None:
        yield self.ensureConnection()
      msg = self._nextMessage()
      # If no message --> sleep a little and try again
      if len(msg) is 0:
          yield self.forDuration(0.3)
          continue
      # Parse the message
      dic = json_loads(msg)
      ts = self.app.now
      self.lastSensor = (ts, dic['f'], dic['b'])
      if dic.has_key("w"):
        self.lastWaypoints = (ts,dic['w'])
      # Make sure to allow others to get the CPU
      yield
