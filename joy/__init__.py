#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public
#  License as published by the Free Software Foundation; either
#  version 3.0 of the License, or (at your option) any later version.
#
#  The library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
# (c) Shai Revzen, U Penn, 2010
#
#  
#
"""
Overview
========

JoyApp provides an application framework for writing software for the CKBot
modular robots developed in [ ModLab ](http://modlabupenn.org). It can also
communicate with the [ Scratch ](http://scratch.mit.edu) visual programming
language, if it is running on the same machine.

The JoyApp framework is based on the excellent [ pygame ](http://www.pygame.org),
which it extends with several custom event types. Programmers using JoyApp will
invariably want to know about [ pygame event handling ][1].

Plans
-----

In addition to new event types, JoyApp provides support for *Plans*. Plans are
similar to threads in that they execute concurrently with each other. Unlike 
threads, they cooperatively multitask by releasing control of the processor
using the Python keyword `yield`. In addition, Plans maintain their own event
queue and event handler. Their constructor offers facilities for binding the
plan to settable `ckbot.logical.Cluster` properties or Scratch variables.

Debugging
---------

Debugging and notification are centrally accessed through the `progress`
function, which prints time-stamped messages on standard output. Many debugging
messages use `debugMsg` which identifies the object emitting the message as the
first parameter. Objects are identified using names generated by `dbgId`. The
actual debug messages generated are controlled by the singleton list `DEBUG`
which is copied into every JoyApp and Plan as `self.DEBUG`. This allows changes
to the contents of the module-wide `DEBUG` toaffect all objects, unless they
have individually overriden this behavior byusing some other list in their
`self.DEBUG`.

[1]: http://www.pygame.org/docs/ref/event.html
"""

# Standard library dependencies
import sys
import time
import types
import traceback
from itertools import izip
from warnings import warn
from math import floor,exp,pi
from glob import glob
from os import environ, getenv, sep as OS_SEP

# pygame interface or compatibility layer
import pygix

# Try to get matplotlib with Agg backend
try:
    from matplotlib import use as mpl_use
    mpl_use('Agg')
    from matplotlib.pylab import figure
except ImportError:
    print ">>> matplotlib not found; plotting disabled"
    def figure(*arg,**kw):
        return None
        
# YAML for configuration file parsing and formatting
import yaml

# Misc small functions
from misc import *

# Interface to robots
import ckbot
from ckbot.logical import Cluster,AbstractServoModule,AbstractProtocolError

# Logging interface
from loggit import progress,LogWriter,dbgId,debugMsg,PROGRESS_LOG

# Plan classes
from plans import ( Plan,
  SheetPlan, CyclePlan, FunctionCyclePlan,
  GaitCyclePlan, StickFilter, MultiClick
  )
import plans
import remote
import safety

## Set up debug topics
try:
  # If the name "DEBUG" exists, we keep the existing DEBUG topics
  #  this is useful when execfile-ing joy.py during a debugging session
  debugMsg(None,"active debugs are %s" % repr(DEBUG))
except NameError:
  DEBUG = []
  debugMsg(None,"no active debug topics")

# Connect debug hooks of sub-modules
ckbot.logical.progress = progress
plans.DEBUG = DEBUG

# Interface to Scratch
import scratch

# Joy event classes and support functions
from events import describeEvt

# MIDI interface
try:
  from midi import joyEventIter as midi_joyEventIter, init as midi_init
except ImportError:
  def midi_init():
    progress('*** MIDI support could not be started -- "import midi" failed ')
  def midi_joyEventIter():
    return []  

# Useful constants
from decl import *

try:
  if PYCKBOTPATH:
    pass
except NameError:
  # Library location specified by PYCKBOTPATH environment var
  PYCKBOTPATH = getenv('PYCKBOTPATH',__path__[0]+OS_SEP+'..'+OS_SEP+'..')
  if PYCKBOTPATH[-1:] != OS_SEP:
      PYCKBOTPATH=PYCKBOTPATH+OS_SEP



class NoJoyWarning( UserWarning ):
  """
  Warning used to indicate unknown keys in a configuration file or cfg
  parameter of the JoyApp constructor.
  """
  pass

class JoyAppConfig( object ):
  """
  Instances of the concrete class JoyAppConfig store configuration settings in
  their attributes. The list of attribute names that are actually settings is
  stored in the private member __keys.
  
  JoyAppConfig contents can be loaded and save in YAML format.
  
  Typical usage:
  >>> cfg = JoyAppConfig( spam = 'tasty', parrot = 'blue' )
  >>> cfg.update( dict( parrot='dead' ) )
  >>> cfg.save('/tmp/Monty')
  >>> cfg2 = JoyAppConfig( spam = 'yuck!' )
  >>> cfg2.load('/tmp/Monty')
  >>> print cfg2.spam
  tasty
  """
  def __init__(self,**kw):
    self.__keys=set()
    self.update(kw)
    
  def update(self,kw):
    """Update settings from a dictionary"""
    self.__dict__.update(kw)
    self.__keys.update( set(kw.iterkeys()) )
    
  def save( self, filename ):
    """
    Save configuration to a YAML file
    """
    if filename[-4:]!=".yml":
      filename = filename + ".yml"
    # Construct a dictionary from the configuration
    d = {}
    for key in self.__keys:
      d[key] = getattr(self,key)
    # Export to YAML
    f = None
    try:
      f = open(filename,"w")
      yaml.dump(d,f)
    except IOError,ioe:
      sys.stderr.write( "ERROR accessing '%s' -- %s" % (filename,str(ioe)))
    except yaml.YAMLError,yerr:
      sys.stderr.write( "ERROR creating YAML '%s' -- %s" % (filename,str(yerr)))
    finally:
      if f:
        f.close()

  def load(self,filename):
    """
    Load configuration from a YAML file
    """
    if filename[-4:]!=".yml":
      filename = filename + ".yml"
    f = None
    d = {}
    try:
      f = open(filename,"r")
      d = yaml.safe_load(f)
    except IOError,ioe:
      sys.stderr.write( "ERROR accessing '%s' -- %s" % (filename,str(ioe)))
    except yaml.YAMLError,yerr:
      sys.stderr.write( "ERROR parsing YAML '%s' -- %s" % (filename,str(yerr)))
    finally:
      if f:
        f.close()
    for key,val in d.iteritems():
      if not key in self.__keys:
        warn("Unknown configuration key '%s' ignored" % key,NoJoyWarning)
        continue
      setattr(self,key,val)
      
  
  
    
class JoyApp( object ):
  """
  Framework for robot, Scratch and pygame combined applications, with support
  for cooperative multithreading (using Plan subclasses)
      
  A JoyApp collects together several useful objects in one place. It also
  provides lifecycle event callbacks that should be overriden by subclasses.
  
  Lifecycle -- JoyApp Creation
  ============================
  __init__ constructor, specifying parameters for self.robot - a
  ckbot.logical.Cluster, self.scr - a scratch.Board interface and self.cfg - a
  JoyAppConfig.
  
  NOTE: Plan.start should never be called from a JoyApp.__init__ constructor.
  
  Lifecycle -- JoyApp Execution
  =============================
  run, which executes the remainder of the life-cycle including the main loop.
  This consists of
  
   1. onStart is called after successful initialization of all sub-systems. 
      Plan.start can be called from here; this
      is a recommended location for starting Plan instances.
  
   2. multiple calls to onEvent, including periodic calls driven by
      TIMEREVENT. These are interspersed with execution of Plan.behavior of all
      active plans. The main loop terminates due to uncaught exceptions in
      onEvent or a call to stop.
      
   3. onStop, called prior to full shutdown.
     
  """
  def __init__(self, confPath=None, robot=None, scr=None, cfg={}):
    """
    Initialize a JoyApp application. This superclass constructor MUST be called
    by all subclasses.
    
    INPUTS: (all are optional)
      confPath -- str -- path to a YAML configuration file, start with $/ to 
            search relative to the PYCKBOTPATH directory
      robot -- dict -- parameters for populating the robot interface;
            passed to ckbot.logical.Cluster when starting up;
            the resulting Cluster will be in self.robot
            Some useful parameters include arch= to set bus hardware
            and count= to set number of modules to populate with.
      scr -- dict -- parameters for the Scratch interface; passed to
            scratch.Board when starting up; the resulting scratch.Board will be
            found in self.scr
      cfg -- dict -- new/overridden configuration parameters. These override the
            configuration found in JoyApp.yml and confPath (if present). The 
            configuration will be in a JoyAppConfig instance in self.cfg

    Configuration parameters are searched for in the following order
      1. JoyApp constructor cfg keyword argument
      2. YAML file specified by JoyApp constructor confPath argument
      3. YAML file named JoyApp.yml in the current directory
            
    ATTRIBUTES:
      DEBUG -- list of str -- list of debug topics to monitor, by default shares
            the joy module DEBUG topics..
      cfg -- JoyAppConfig -- configuration settings.
      scr -- scratch.Board -- interface to Scratch
      robot -- ckbot.logical.Cluster -- interface to robot
      remote -- interface to event factory
      now -- float - current time; updated every timer event
      plans -- list of Plan-s -- (private) list of active Plan co-routines
      safety -- list of safety condition testers
    """
    self.DEBUG = DEBUG
    self.cfg = None #: EpyDoc (is a) dummy
    self.__initConf(confPath,cfg)
    if not self.cfg.logFile:
      self.logger = None
      progress('Logging disabled')
    else:
      self.logger = LogWriter(self.cfg.logFile)
      progress('Logging to "%s"' % self.cfg.logFile)
      if self.cfg.logProgress:
        progress('Progress messages will be logged')
        PROGRESS_LOG.add(self.logger)       
    #
    self.now = time.time()
    # initialize safety provider list
    self.safety = []
    # init CKBot cluster interface
    if robot is not None:
      self.__initRobot(robot)
    else:
      progress("No robot")
      self.robot = None #: Interface to CKBot robot modules
      'Interface to CKBot robot modules'
    # init Scratch interface
    if scr is None:
      progress("No connection to Scratch")
      self.scr = None 
      'Interface to the Scratch programming environment'      
    else:
      self.scr = scratch.Board(**scr)
      progress("Connected to Scratch")
      self.__scr_cast = set()
      self.__scr_upd = {}
    # initialize midi module (this is safe to call again multiple times)
    if self.cfg.midi:
      midi_init()
    # init Plan scheduling structures
    self.plans = [] #: (private) List of currently active Plan instances ("threads")
    self.__new = []    

  def __initConf(self, confPath, cfg):
    """
    (private) initialize the .cfg instance variable
    """
    # Default configuration
    self.cfg = JoyAppConfig(      
      robotPollRate = 0.1,
      positionTolerance = 50,
      keyboardRepeatDelay = 500,
      keyboardRepeatInterval = 50,
      clockInterval = 20,
      voltagePollRate = 1.0,
      minimalVoltage = 13,
      windowSize = [320,240],
      nodeNames = {},
      logFile = None,
      logProgress = False,
      remote = None,
      midi = True
    )
    pth = PYCKBOTPATH + 'cfg%sJoyApp.yml' % OS_SEP
    if glob(pth):
      self.cfg.load(pth)
      progress("Loaded %s" % pth)
    else:
      progress("Path not found '%s' (this error can be safely ignored)" % pth )
    if confPath is not None:
      if confPath.startswith("$/"):
        confPath =  PYCKBOTPATH + confPath[2:]
      if glob(confPath):
        self.cfg.load(confPath)    
        progress("Loaded %s" % confPath)
      else:
        progress("Nothing to load at confPath='%s'" % confPath)
    self.cfg.update(cfg)
  
  def __initRobot( self, robot ):
    """
    (private) Initialize the ckbot.Cluster robot interface
    
    INPUT: 
      robot -- dict -- Dictionary of settings for robot.populate
    """
    progress("Populating:")
    # Collect names from the cfg
    nn = self.cfg.nodeNames.copy()
    nn.update(robot.get('names',{}))
    robot['names']=nn
    # Show the names in the progress log
    for k,v in robot.iteritems():
      progress("\t%s=%s" % (k,repr(v)))
    # Check for both protocol= and arch= parameters
    p = robot.get('protocol',None)
    a = robot.get('arch',None)
    if a and p:
      raise ValueError,"Do not combine legacy protocol= with arch="
    # NOTE: A protocol instance can be passed in arch= parameter of Cluster
    if p:
      del robot['protocol']
      a = p
    # Make sure that robot dictionary does not duplicate the arch= parameter
    if a:
      del robot['arch']
    self.robot = Cluster(arch=a, **robot)
    if self.cfg.minimalVoltage:
      return self.__initVoltageSafety()
    
  def __initVoltageSafety( self ):
    """(private) 
    collect all robot modules that have a get_voltage() method
    ans set them up for polling
    """
    vs = []
    for m in self.robot.itermodules():
      # Collect modules that have a voltage sensing capability
      if hasattr(m,'get_voltage'):
        vs.append(m)
      if not isinstance(m,AbstractServoModule): 
        continue
      p = m.get_pos_async()
      if p is None:
        continue
      m.__promise = p
      m.__promise_time = self.now
      m.__last_pos = 0
    # If any voltage sensing modules found, and sensing was requested
    #   then add the safety provider
    if not vs:
      return
    self.safety.append(safety.BatteryVoltage(
      vmin = self.cfg.minimalVoltage,
      sensors = vs,
      pollRate = self.cfg.voltagePollRate 
    ))

  def setterOf( self, func ):
    """
    Convert a setter specification (which may be a string) to a setter function.
    Specifications include Cluster properties as supported by the
    ckbot.logical.Cluster class, single parameter callables, Scratch
    variables (indicated by pre-pending a ">"), debug messages (indicated
    by pre-pending a "#"), or any valid specification with logging (indicated
    by an additional "^" prefix)
    
    If specification is a tuple, its second member must be a function that
    pre-processes the values, e.g.: 
        ('>x',lambda x : x/2.0)
    specified output to Scratch variable 'x', after scaling down by two.
        '^Nx22/@set_pos' 
    would set the position on NID 0x22, and also log the set operation using
    the JoyApp's logger.
    If the tuple is longer than 2, entries 3 and up are used as the initial
    parameters, with the value coming in last.
    """
    if type(func)==tuple:
      pre = func[1]
      if not callable(pre):
        raise ValueError("Second member of tuple must be callable")
      fun = self.setterOf(func[0])
      func = func[2:]
      if func:
        def preProcFun( value ):
          return fun( func + (pre( value ),) )
      else:
        def preProcFun( value ):
          return fun( pre( value ) )
      func = preProcFun
    elif type(func)==str:
      # If is a Scratch sensor output
      if func[:1]==">": 
        # --> obtain curried setter function
        func = curry(self.setScratch,func[1:])
      elif func[:1]=="#":
        # --> obtain customized message emitter
        pfx = func
        def emitMsg( val ):
          progress( "%s %s" % (pfx, repr(val) ))
        func = emitMsg
      elif func[:1]=="^":
        if func[1]=="^":
          raise ValueError("Only one '^' allowed in spec '%s'" % func)
        sf = self.robot.setterOf(func[1:])
        if self.logger is not None:
          func = self.logger.setterWrapperFor( sf, attr=dict(binding=func[1:]) )
        else: # no logger --> emit as progress message and set via cluster
          fmt = func[1:] + " = %s" 
          def emitMsgAndSet( val ):
            progress( fmt % repr(val) )
            return sf(val)
          func = emitMsgAndSet
      else: # else --> obtain setter from cluster 
        func = self.robot.setterOf(func)
    return func
  

  def _posEventPump( self ):
    """(private)
    <<NOTE: feature is currently disabled in _timeslice due to timing problems
      caused by robot.getLive(). This may be solvable by replacing the call to
      getLive() by something less disruptive>>

    Check asynchronous position polling of robot modules for new updates and
    push the necessary custom events into pygame
    """
    self.robot.getLive()
    t = self.now
    for m in self.robot.itermodules():
      if (not hasattr(m,'_JoyApp__promise') or (t - m.__promise_time < self.cfg.robotPollRate )):
        continue
      if (m.__promise):
        pos = m.pna.async_parse('h',m.__promise)
        if abs(pos-m.__last_pos)>self.cfg.positionTolerance:
          evt = pygix.Event(CKBOTPOSITION, module = m.node_id, pos = pos )
          pygix.postEvent(evt)
        m.__last_pos = pos
      m.__promise = m.get_pos_async()
      m.__promise_time = t
  
  def _scrEventPump( self ):
    """(private)
    
    Check the scratch.Board inerface for new updates and
    push the necessary custom events into pygame
    """
    var,bcast = self.scr.poll()
    if var is None: #!!!need better error handling here
      self.scr = None
      progress('Scratch connection aborted')
      return
    for nm,val in var.iteritems():
      evt = pygix.Event(SCRATCHUPDATE, scr=0, var=nm, value=val )
      pygix.postEvent(evt)
    for nm in bcast:
      evt = pygix.Event(SCRATCHUPDATE, scr=0, var=nm, value=None )
      pygix.postEvent(evt)
      
  # Table of functions for converting various event kinds into Scratch sensors
  SCRATCHY = {
    JOYAXISMOTION : lambda x : ('joy%d axis%d' % (x.joy,x.axis), x.value),
    JOYBALLMOTION : lambda x : ('joy%d ball%d' % (x.joy,x.ball), x.rel),
    JOYHATMOTION : lambda x : ('joy%d hat%d'% (x.joy,x.hat), x.value) ,
    JOYBUTTONUP  : lambda x : ('joy%d btn%d' % (x.joy,x.button), None) ,
    JOYBUTTONDOWN : lambda x : ('joy%d bup%d' % (x.joy,x.button), None),
    QUIT : lambda x : ('JoyAppQuit',None),
    CKBOTPOSITION : lambda x : ('bot Nx%x pos' % x.module, x.pos),
  }    
  def scratchifyEvent( self, evt ):
    """Convert event to a corresponding scratch message and send it
    """
    if self.scr is None:
      raise RuntimeError('Scratch interface was not started')    
    fun = self.SCRATCHY.get(evt.type,None)
    if fun is None:
      return False
    msg,val = fun(evt)
    if val is None:      
      if 'S' in self.DEBUG: debugMsg('Defer scratch broadcast %s' % msg)
      # Quit events must be sent immediately
      if evt.type==QUIT:
        self.scr.broadcase(msg)
      else:
        self.__scr_cast.add(msg)
    else:
      if 'S' in self.DEBUG: debugMsg(self,'Defer scratch update %s=%s' % (msg,str(val))) 
      self.__scr_upd[msg]= val
    return True
    
  def setScratch(self, sensor, value):
    """
    Send a sensor update to Scratch
    
    This method is never used internally by JoyApp; it is primarily
    used by Plan.__init__ when binding Scratch sensors ($ prefix)
    """
    if self.scr is None:
      raise RuntimeError('Scratch interface was not started')    
    if 'S' in self.DEBUG: 
      debugMsg(self,'Scratch update %s=%s' % (sensor,str(value))) 
    self.__scr_upd[sensor]= value
    
  def _scrEventEmit(self):
    """(private)
    
    Emit deferred Scratch updates 
    """
    if self.__scr_cast:
      self.scr.broadcast( *tuple( self.__scr_cast) )
      if 'S' in self.DEBUG: 
        debugMsg(self,'Scratch broadcast %s' % " ".join(self.__scr_cast) )
      self.__scr_cast = set()
    if self.__scr_upd:
      self.scr.sensorUpdate( **self.__scr_upd )
      if 'S' in self.DEBUG: 
        debugMsg(self,'Scratch update %s' % repr(self.__scr_udp) )
      self.__scr_upd = {}

  def _pollSafety( self ):
    """(private)
    Poll all safety conditions and shut down immediately if violated
    """
    try:
      for s in self.safety:
        s.poll(self.now)
    except AbstractProtocolError, ape:
      progress("WARNING: safety test encountered a communication error '%s'" % str(ape))
    except safety.SafetyError,se:
      if self.robot:
         self.robot.off()
      self.stop()
      progress("(say) DANGER: safety conditions violated -- shutting down")
      raise
  
  def _timeslice( self, timerevt ):
    """(private)
    
    Timer events get propagated to all Plans and trigger execution 
    of a timeslice.
    """
    self.now = time.time()
    self.plans.extend(self.__new)
    self.__new = []
    if 'P' in self.DEBUG: debugMsg(self, "plans=%s" % dbgId(self.plans) )
    # Push timer events to all active plans
    for p in self.plans:
      p.push(timerevt)
    # Execute plans
    nxt = []
    for p in self.plans:
      # If a plan is running --> let it run a step
      if p.isRunning():
        p.step()
      # If it is still running --> remember to run it next time
      if p.isRunning():
        nxt.append(p)
      elif 'P' in self.DEBUG: # else --> harvested (died) 
        debugMsg(self, "harvested %s" % dbgId(p) )
    if 'P' in self.DEBUG: 
      debugMsg(self, "add plans=%s" % repr(self.__new) )
    self.plans = nxt + self.__new
    self.__new = []
  
  def onceEvery( self, tau ):
    """
    Return a function that returns True only once every
    tau time units. Typically, this is used for limiting
    debug output to appearing periodically, for example:
    
    oncePer = self.onceEvery(1)
    while True:
      if oncePer():
        print "Hello world"    
    """
    last = [self.now]
    def onceEvery():
      if self.now-last[0]>tau:
        last[0] = self.now
        return True
      return False
    return onceEvery
    
  def _startRemote( self ):
    """(private) start remote interface if configured"""  
    if self.cfg.remote is None:
      self.remote = None
      return
    self.remote = remote.Sink(self, convert=lambda x: x, **self.cfg.remote )
    progress("Starting up a remote.Sink interface")
    self.remote.start()
  
  def run( self ):
    """
    Run the JoyApp.
    
    After initializing the application, calls onStart()
    and executes event loop (onEvent()) until .stop() is called
    or an caught exception causes abnormal termination.    
    
    onStop() is called before the application is shut down.
    """
    self.screen = pygix.startup(self.cfg)
    # Obtain figure for plotting; None if running headless
    w,h = self.cfg.windowSize
    self.fig = figure(figsize=(w/80,h/80),dpi=80)
    self._frame = None
    self.isRunning = lambda : True
    self._startRemote()    
    try:
      # Call onStart subclass hook
      self.onStart()
      while self.isRunning():
        self._pollSafety()
        # DISABLED DUE TO TIMING PROBLEMS, SEE COMMENT IN _posEventPump
        #if self.robot:
        #  self._posEventPump()
        if self.scr:
          self._scrEventPump()
          self._scrEventEmit()
        if self.cfg.midi:
          for evt in midi_joyEventIter():
            pygix.postEvent(evt)
        if self._frame:
            buf,sz = self._frame
            pygix.showMplFig(buf,sz,self.screen)
            self._frame = None            
        for evt in pygix.iterEvents():
          if evt.type == TIMEREVENT:
            self._timeslice(evt)
          self.onEvent(evt)   
    except Exception,exc:
      printExc()
    # App cleanup bugs
    try:
      self.onStop()
      if self.scr:
        self.scr.close()
      if self.robot:
        self.robot.off()
      if self.logger:
        self.logger.close()  
        if self.cfg.logProgress: 
          PROGRESS_LOG.remove(self.logger)
    finally:
      pygix.shutdown()
  
  def animate(self):
      """
      Get image in self.fig for presenting next GUI update
      
      If no self.fig (headless system), call is silently ignored
      """
      if self.fig:
          self._frame = self.fig.canvas.print_to_buffer()
      
  def stop(self):
    """
    Stop the JoyApp
    """
    self.isRunning = lambda : False
   
  def _startPlan( self, plan ):
    """(private)
    Called by Plan.start() to notify JoyApp that plan should be started
    """
    if not plan in self.plans:
      self.__new.append(plan)
      
  def onStart(self):
    """(default)
    
    Override this to run initialization code after JoyApp object is initialized
    but before any plans run
    
    By default, prints a message
    """
    progress("%s --- starting up main loop" % str(self) )
  
  def onStop( self ):
    """(default)
    
    Override this to run cleanup code before shut down
    
    By default, prints a message
    """
    progress("%s --- shutting down" % str(self) )
  
  def onEvent( self, evt ):    
    """(default)
    
    Top-level event handler
    
    Override this method to install your event handling code.
    The default JoyApp displays all events in human readable form on screen.
    Hitting <escape> or closing the windowill cause it to quit.    
    """
    if evt.type != TIMEREVENT:
      progress( describeEvt(evt) )
      if self.logger:
        self.logger.write('event',**describeEvt(evt,parseOnly=1))
      if self.fig and evt.type==KEYDOWN:
          self.fig.clf()
          ax = self.fig.gca()
          ax.plot([-1,1,1,1,-1],[-1,-1,1,1,-1],'w-')
          ax.text(0,0,evt.unicode)
          self.animate()
    if evt.type==QUIT or (evt.type==KEYDOWN and evt.key in (K_q,K_ESCAPE)):
       self.stop()

def test():
  import sys
  
  cmds=dict(cfg={})
  args = list(sys.argv[1:]) # copy
  while args:
    arg = args.pop(0)
    if arg=='--with-robot' or arg=='-r':
      cmds.update(robot={})
    elif arg=='--with-scratch' or arg=='-s':
      cmds.update(scr={})
    elif arg=='--help' or arg=='-h':
      cmds = None
    elif arg=='--mod-count' or arg=='-c':
      N = int(args.pop(0))
      if not cmds.has_key('robot'): cmds.update(robot={})
      cmds['robot'].update(count=N)
    elif arg=='--logfile' or arg=='-L':
      cmds['cfg'].update(logFile=args.pop(0))
    elif arg=='--logall' or arg=='-A':
      cmds['cfg'].update(logProgress=True)
  if cmds is None:
    sys.stdout.write("""
Usage: %s [options]

  Default JoyApp -- show all events
  
  Options:
    --with-robot | -r 
      Start robot interface
      
    --with-scratch | -s
      Start with Scratch connection
      
    --mod-count <number> | -c <number>
      Search for specified number of modules at startup
      
    --logfile <filename> | -L <filename>
      Log events to a logfile
    
    --logall | -A
      Log *EVERYTHING*
  """ % sys.argv[0])
    sys.exit(1)
  app = JoyApp(**cmds)
  app.run()


