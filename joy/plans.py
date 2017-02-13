import types, sys

from pygix import EventType
from decl import *

from math import pi,exp,floor
from warnings import warn

# Logging interface
from loggit import progress, debugMsg, dbgId

from misc import curry,printExc

from events import describeEvt
DEBUG = []

class Plan( object ):
  """
  Abstract superclass of scheduled behaviors
  
  Each Plan implements a "co-routine" consisting of a behavior
  that executes in parallel with other plans. It has its own event
  queue and event handler, and its own bindings to outputs.
    
  Class Plan is use by subclassing it and overriding the behavior()
  method with a generator -- i.e. a method that has a bunch of yield
  statements in it. The yield statements MUST all be "yield None",
  which can be abbreviated to "yield" with no parameters.
  The yields indicate the locations at which the Plan returns control 
  to the owner JoyApp.
  
  Plans continue executing from where they stopped whenever the event
  handler returns True. Otherwise it should return False. Other return
  values will generate an exception.
  
  Plans may be nested; simply yield the plan that is to take over control.
  NOTE: Make sure that the nested Plan was start()-ed  
  
  Some methods are designed to be used with yield, specifically:
    untilTime, forDuration
  
  WARNING: JoyApp is single-threaded. If a behavior spends too long 
  between yield statements the entire JoyApp waits for it to complete. 
  
  Typical usage:
  >>> class MyPlan( Plan ):
  ...   def __init__(self,app):   
  ...     Plan.__init__(self,app)
  ...
  ...   def onEvent( self, evt ):
  ...     if evt.type == MOUSEMOTION:
  ...       self.x = evt.pos[0]
  ...       return True
  ...     return False
  ...
  ...   def behavior(self):
  ...     print "Move mouse to the left"
  ...     while self.x > self.app.cfg.windowSize[0]*0.25:
  ...       yield
  ...     print "Move mouse to the right"
  ...     while self.x < self.app.cfg.windowSize[0]*0.75:
  ...       yield
  ...     print "Great job!! here are 3 tail wags for you"
  ...     for wag in xrange(3):
  ...       self.tailWagTo(100)
  ...       yield self.forDuration( 500 )
  ...       self.tailWagTo(-100)
  ...       yield self.forDuration( 500 )

  BINDINGS: To allow plans to be easily adapted and re-used
  without sub-classing, the Plan constructor accepts a list
  of keyword argument "bindings" of the form <name>=<setter>,
  where <setter> is either a callable or a string naming a
  robot property to be resolved using app.robot.setterOf().
  This allows self.<name> to be used in the Plan code for
  setting values that will only be chosen at the time the
  plan subclass instance is constructed. In the example above,
  we could have used MyPlan( tailWagTo='Nx55/@set_pos' ) to create
  an instance whose self.tailWagTo sets the position in robot
  node 0x55. Similarly, tailWagTo='>tail wag' would bind the
  output to the Scratch sensor named 'tail wag'
  
  """
  def __init__( self, app, **binding ):
    """
    Initialize a Plan within some JoyApp application.    
    """
    self.DEBUG = app.DEBUG
    self.app = app
    self.__evts = []
    self.__stack = []
    if binding and not hasattr(self,'_binding'):
      self._initBindings(app,binding)

  def _initBindings( self, app, binding, allowOverride = False ):
    """(protected) initialize bindings, resolving any string valued bindings"""
    for key,func in binding.iteritems():
      if hasattr(self,key) and not allowOverride:
        raise KeyError("Plan already has an attribute '%s' -- bind another name" % key )
      if type(func) is str:
        func = app.setterOf( func )
      if not callable(func):
        warn( "Binding non-callable %s to '%s'; are you sure?" % (repr(func),key))  
      setattr(self,key,func)
    self._binding = binding
  
  def push( self, *evts ):
    """
    Push events into this Plan's event queue
    
    Events are only queued while the Plan isRunning(); otherwise
    they are silently dropped.
    """
    if not self.isRunning():
      return
    for evt in evts:
      assert type(evt) is EventType
      self.__evts.append(evt)
  
  def onEvent( self, evt ):
    """(default)
    Event handler. Override this method to handle all events that 
    were push()-ed to this Plan.
    
    The sequential behavior() in this plan only runs if onEvent
    returned True.
    
    All Plan-s recieve a copy of all TIMEREVENT events. If your
    sequential code does not need any other events, and just wants
    to execute in parallel with other Plan-s, it is safe to leave
    the default onEvent method, which always returns True 
    
    NOTE: 
      onEvent MUST return True or False. Other values raise an 
      exception at runtime. 
    """
    return True
  
  def behavior( self ):
    """
    (default)
    
    Override this method to implement the sequential behavior of a Plan
    
    This method MUST be a python iterator, i.e. must contain a "yield" statement
    """
    progress("Goodbye, cruel world!")
    yield
    
  def untilTime(self,time):
    """
    A sub-plan that sleeps until a specified time.
    
    Use this from .behavior() with a statement of the form:
       yield self.untilTime( wakeup )
    to sleep until the specified wakeup time.
    """
    while self.app.now<time:
      yield
  
  def forDuration(self,duration):
    """
    A sub-plan that sleeps for a specified duration.
    
    Use this from .behavior() with a statement of the form:
       yield self.forDuration( naplength )
    to sleep for the specified duration
    """
    end = self.app.now + duration
    while self.app.now<end:
      yield
      
  def isRunning( self ):
    """
    Returns True if and only if this Plan is currently "running",
    i.e. its .onEvent handler receives events and its .behavior
    has not terminated.
    """
    return bool(self.__stack)
    
  def start( self ):
    """
    Start execution of a Plan's behavior.
    
    If Plan is already running -- issues a warning
    """
    if self.isRunning():
      warn("Plan is already running")
      return
    # Flush event queue
    self.__evts = []
    self.__stack = [self.behavior()]
    self.__sub = None
    self.app._startPlan(self)
    self.onStart()

  def onStart( self ):
    """(default)
    
    Override this method to perform operations when Plan starts but before the
    first events are processed.
    """
    pass
  
  def stop( self, force=False ):
    """Stop (abnormally) the execution of a Plan's behavior
    
    If not running, this call is ignored.
    
    INPUT:
      force -- boolean -- stop without raising termination exceptions in co-routines
    """
    if not self.isRunning():
      return
    # Stop sub-Plans first
    if self.__sub is not None:
      self.__sub.stop(force)
    # If forced --> drop reference to the stack
    if force:
      self.__stack=[]
    else:
      # Regular termination --> close() all co-routines
      while self.__stack:
        ctx = self.__stack.pop()
        try:
          ctx.close()
        except StandardError:
          printExc()
    self.onStop()

  def onStop( self ):
    """(default)
    
    Override this method to perform operations when Plan terminates.
    """
    pass
      
  def _unwind( self, exinfo ):
    """(private)
    
    Unwind Plan co-routines by throwing exceptions up the stack 
    """
    S = self.__stack
    top = None
    sub = None
    # While stack is not empty
    while S:
      try:
        # Pop the top-most co-routine
        top = S.pop()
        # throw the exception in it
        #   if caught --> return a yielded value in sub
        #   else --> go to except clause
        sub = top.throw( *exinfo )
        # If reached exception was handled --> keep remaining stack
        break
      except StandardError:
        # exception was not caught, killing top
        # tracebacks are now concatenated, so get new head of traceback
        exinfo = sys.exc_info()
    # (only reached if exception handled)
    printExc(exinfo)
    # Put top back on stack, it isn't dead
    if top is not None:
      S.append(top)
    return sub
  
  def _stepDoEvents( self ): 
    """(private)
   
    Run onEvent() method on all incoming events
    
    OUTPUT:
      True if plan can go idle (no relevant events) or False otherwise
    """
    idle = True
    while self.__evts:
      evt = self.__evts.pop(0)
      try:        
        rc = self.onEvent(evt)
      except StandardError, se:
        printExc()
        rc = False        
      if rc is True: 
        idle = False
      elif rc is not False:
        raise TypeError("A Plan's onEvent() method MUST return True or False")
    return idle

  def _oneStep( self ):
    """(private)
    
    Execute one step from the iterator at the top of the stack
    
    OUTPUT:
      co-routine to execute as sub-Plan, or None
    """
    S = self.__stack
    top = S.pop()
    try:
      if 'p' in self.DEBUG: debugMsg(self,"stepping from "+repr(top))      
      sub = top.next()
      if 'p' in self.DEBUG: debugMsg(self,"   -->"+dbgId(sub))      
      # if reached iteration did not terminate normally or by exception
      #   --> put top back on stack
      S.append(top)
    except StopIteration:
      # Top of stack terminated -- step is done  
      sub = None
      if 'p' in self.DEBUG: debugMsg(self,"   <<terminated>> "+dbgId(sub)+repr(S))
    except StandardError:
      # An exception occurred -- unwind the stack to expose its origin
      sub = self._unwind( sys.exc_info() )
    return sub
    
  def step( self ):
    """
    Execute one time-step of Plan behavior.

    This method is called every time the .onEvent handler returns True for
    one or more events in a time-slice.
    """
    if not self.isRunning():
      raise RuntimeError("Cannot step() %s -- not isRunning()" % dbgId(self))
    if 'p' in self.DEBUG: debugMsg(self,"begin step "+repr(self.__stack))
    #
    # If handling events leaves us idle --> done
    #
    if self._stepDoEvents(): return    
    if 'p' in self.DEBUG: debugMsg(self,"event detected")
    #
    # If an active sub-Plan is running --> done
    #
    if self.__sub is not None:
      if self.__sub.isRunning(): return
      # Sub-plan finished execution; remove it
      self.__sub = None
    #
    # If execution step yielded something --> process new sub-plan
    #
    sub = self._oneStep()
    if sub is not None:
      self._stepNewSub( sub )
    #
    # If terminated --> call onStop
    if not self.isRunning():
      self.onStop()
  
  def _stepNewSub( self, sub ):
    """(private)
    
    Plan yielded a sub-plan to execute -- process it
    """
    # If another Plan
    if isinstance(sub,Plan):
      if sub.isRunning():
        raise RuntimeError("Sub-plan is already running")
      self.__sub = sub
      sub.start()
      if 's' in self.DEBUG: 
        debugMsg(self,"new sub-Plan "+dbgId(sub)+" isR "+repr(sub.isRunning()))
        debugMsg(self,"  app plans %s" % dbgId(self.app.plans) )        
      # Push Plan behavior() generator on stack
    # If a generator --> top is None,execution
    elif type(sub) is types.GeneratorType:
      self.__stack.append(sub)
      if 's' in self.DEBUG: debugMsg(self,"new sub-generator "+dbgId(sub))
    else: # --> yield returned object of the wrong type
      raise TypeError("Plan-s may only yield None, Plan-s or generators")

class SheetPlan( Plan ):
  """
  concrete class SheetPlan implements a Plan that is read from
  a "sheet" -- a rectangular list-of-lists containing a spreadsheet
  specifying the planned settings as a function of time.
  
  Sheets can be read from a file using loadCSV(), or from an
  inline multiline string using inlineCSV()
  
  The first row of the sheet consists of string headings, starting
  with the heading "t" (for time). Remaining headings specify
  bindings to properties the Plan should set. 

  By default, the column headings will get automatically converted into
  a binding using the format <<heading>>/@set_pos which means that
  columns are assumed to be names of modules whose position is to be 
  set. WARNING: this default behavior is disabled if ANY bindings are
  passed to the constructor.
  
  The first column of the sheet (2nd row an on) MUST consist of
  numbers in increasing order specifying the times at which the
  settings in that row should be applied.
  
  Empty elements in the sheet (None-s) are skipped.
  
  If no column bindings are provided, column headings are assumed 
  to be module names of servo modules and each of these is bound to
  <<module>>/@set_pos of the corresponding module. For example, the sheet:
  >>> sheet = [ 
  ...   ["t",   "Nx15"],
  ...   [0,     1000],
  ...   [0.5,  -1000] ]
  will generate a +/- 10 degree step into the position of node 0x15   
  """
  def __init__(self,app,sheet,*arg,**kw):
    if not kw:
      kw = self._autoBinding(sheet)
    Plan.__init__(self,app,*arg,**kw)
    self.setters,self.headings,self.sheet = self._parseSheet(sheet)
    self.rate = 1.0
  
  def update( self, sheet, **kw ):
    """
    Update the sheet specifying the Plan.
    
    NOTE: if the new sheet has different bindings form the old one, Weird Things Will Happen!
    """
    if self.isRunning():
      raise TypeError,"Cannot update sheet while plan is running"
    if not kw:
      kw = self._autoBinding(sheet)
    self._initBindings( self.app, kw, allowOverride = True )
    self.setters,self.headings,self.sheet = self._parseSheet(sheet)

  def _autoBinding( self, sheet, fmt="%s/@set_pos" ):
    """(private)
    
    Given a sheet, automatically treat the column heads as
    servo module names for which positions must be set
    """
    heads = sheet[0][1:]
    return dict([ (nm,fmt % nm ) for nm in heads])
    
  def setRate( self, rate ):
    """
    Set rate at which sheet should be executed, e.g.
    rate=2 means execute twice as fast
    
    Only positive rates are allowed. Rate cannot be changed
    while the Plan isRunning()
    """
    rate = float(rate)
    if rate<=0:
      raise ValueError("Rate %g should have been >0" % rate)
    self.rate = rate
  
  def getRate(self): return self.rate
  
  def _doRowSets( self, row ):
    """(private)
    
    Perform the set operations associated with a row
    """
    try:
      for func,val in zip(self.setters,row):
        if val is not None:
          func(val)
        if 'G' in self.DEBUG: 
          debugMsg(self,"%s(%s)" % (dbgId(func),repr(val))) 
    except StandardError:
      printExc()

  def _parseSheet( self, sheet ):
    """(private)
    
    Make sure that the sheet is syntactically correct
    
    OUTPUT: setters, headings, sheet
    """
    # First column heading is 't' for time    
    if sheet[0][0] != "t":
      raise KeyError('First column of sheet must have heading "t" not %s' % repr(sheet[0][0]))
    # Split off headings and contents
    headings = sheet[0][1:]
    sheet = sheet[1:]
    # Ensure rows are equal length
    w = len(headings)+1
    lb = sheet[0][0]-1
    for li in xrange(len(sheet)):
      l = sheet[li]
      if len(l)!=w:
        raise ValueError(
          'Sheet row %d has %d entries instead of %d'%(li+1,len(sheet[li]),w) )
        if type(l[0]) not in [int,float]:
          raise TypeError(
            'Sheet row %d has "%s" instead of a time' % (li+1,l[0]))
      if l[0]<=lb:
        raise IndexError(
          'Sheet row %d has non-increasing time %d' % (li+1,l[0]))
      lb = l[0]
    # Ensure that headings make sense as properties          
    try:
      res = []
      for name in headings:
        res.append(getattr(self,name))
    except AttributeError:
      raise AttributeError('Sheet addresses property "%s" which has no binding. Hint: use <property>=<setter-function> in constructor' % name)      
    return res,headings,sheet
          
  def behavior(self):
    """(final)
    
    The behavior of a SheetPlan is defined relative to the Plan's start time 
    and rate. The plan goes down the sheet row by row in the specified rate,
    sleeping until it is ready to apply the settings in that row.
    
    Once the settings of the final row are applied, the Plan terminates.

    The 't' debug topic can be used to display SheetPlan timesteps
    """
    t0 = self.app.now
    rate = self.rate
    if 't' in self.DEBUG: debugMsg(self,"started at t0=%g" % t0)
    for row in self.sheet:
      yield self.untilTime( row[0] / rate + t0 )
      if 't' in self.DEBUG: debugMsg(self,"step at t=%g" % (self.app.now-t0))
      self._doRowSets( row[1:] )      
    if 't' in self.DEBUG: debugMsg(self,"ended at t=%g" % (self.app.now-t0))

class CyclePlan( Plan ):
  """
  A CyclePlan implements a circular list of action callbacks, indexed by 
  'phase' -- a real number between 0 and 1. At any given time the CyclePlan
  has a position (stored in the private variable ._pos) on this cycle.
  
  The position can change by using .moveToPhase or by the passage of time,
  through setting a period or frequency for cycling. This frequency or period
  may also be negative, causing the CyclePlan to cycle in reverse order.
  
  In abstract, it is ambiguous in which direction around the cycle a given 
  .moveToPhase should proceed. This ambiguity is resolved by allowing the goal
  phase to be outside the range 0 to 1. Goals larger than current phase cycle
  forward and goals smaller than current phase cycle back.
  
  Whenever phase changes, all actions in the circular arc of phases between the
  previous and new phase are executed in order. If the arc consists of more
  than one cycle, phase changes to 0, the .onCycles method is called with the
  integer number of positive or negative cycles, and the remaining partial 
  cycle to the goal is called.
  """
  def __init__(self,app,actions,maxFreq=10.0,*arg,**kw):
    """
    Instantiate a CyclePlan
    
    INPUT:
      app -- JoyApp -- application containing this Plan instance
      actions -- dict -- mapping floating point phases in the range 0 to 1 to 
         python callable()-s that take no parameters.
      maxFreq -- float -- maximal cycling frequency allowed for this Plan.
    """
    Plan.__init__(self,app,*arg,**kw)
    # Maximal frequency at which to cycle completely thorugh gait
    self.maxFreq = maxFreq
    # Validate actions dictionary and construct lookup table
    steps = self.__initSteps(actions)
    self._lookup = ( [-1]+steps+[2] )
    self.actions = actions
    self.phase = 0
    self._pos = 1
    self.period = 1.0

  def __initSteps( self, actions ):
    """(private)
    
    Scan actions table, construct lookup index and resolve bindings
    """
    steps = []
    for phi,act in actions.iteritems():
      if phi<0 or phi>1:
        raise KeyError("Phase %g is outside valid range [0,1]"%phi)
      if type(act)==str:
        try:
          act = getattr(self,act)
        except AttributeError:
          raise KeyError("String action at phi=%g was not in bindings" % phi)
      if not callable(act):
        raise TypeError("Action at phi=%g is not callable in binding" % phi)
      steps.append(phi)
    steps.sort()
    return steps
  
  @staticmethod
  def bsearch( k, lst ):
    """
    Binary search for k through the sorted sequence lst
    
    Returns the position of the first element larger or equal to k
    """
    l = 0
    u = len(lst)
    while u>l:
      m = (u+l)/2
      if k>lst[m]:
        l = m+1
      else:
        u =m
    return l
    
  def setPeriod( self, period ):
    """
    Set period for cycles, 0 to stop in place
    """
    if abs(period)>1.0/self.maxFreq:
      self.period = float(period)
    elif period==0:
      self.period = 0
    elif period>0:
      self.period = 1.0/self.maxFreq
    elif period<0:
      self.period = -1.0/self.maxFreq
  
  def getPeriod( self ):
    """Return the cycling period"""
    return self.period

  def setFrequency(self, freq):
    """
    Set frequency for cycles, 0 to stop in place
    """
    if freq==0:
      self.period = 0
    elif freq>self.maxFreq:
      self.period = (1.0/self.maxFreq)
    elif freq<-self.maxFreq:
      self.period = (-1.0/self.maxFreq)
    else:
      self.period = (1.0/freq)

  def getFrequency(self):
    """
    Return the cycling frequency
    """
    if self.period: return 1.0/self.period
    return 0
    
  def onCycles(self,cyc):
    """(default)
    
    Move forward or back several complete cycles, starting and
    ending at phase 0
    
    INPUT:
      cyc -- number of cycles; nonzero
    
    The default implementation is to ignore complete cycles.
    Override in subclasses if you need an accurate count of 
    winding numbers.
    """
    if 'c' in self.DEBUG: 
      debugMsg(self,'  onCycles %d' % cyc)
  
  def moveToPhase( self, phi ):
    """
    Move from the current phase (self.phase) to a new phase phi. 
       
    phi values larger than 1.0 will always cause forward cycling through 0.0 
    phi values smaller than 0.0 will cause reverse cycling though 1.0
    Values in the range [0,1] will move via the shortest sequence
    of steps from current phase to new phase.
    """
    cyc = floor(phi)
    if phi > self.phase:
      if phi<1:
        npos = self.bsearch(phi, self._lookup)
        if self._pos < len(self._lookup)-1:
          self._doActions( slice(self._pos, npos,1) )
      else:
        npos = self.bsearch(phi-cyc, self._lookup)
        if 'c' in self.DEBUG: 
          debugMsg(self,'movePhase up %d --> -2, 1 --> %d' % (self._pos,npos)) 
        if self._pos < len(self._lookup)-1:
          self._doActions( slice(self._pos, -2,1) )
        if phi>=2:
          self.onCycles(cyc-1)
        self._doActions( slice(1, npos,1) )
    else:
      assert phi <= self.phase
      if phi > 0:
        npos = self.bsearch(phi, self._lookup)
        if self._pos>1:
          self._doActions( slice(self._pos-1, npos-1, -1) )
      else:
        npos = self.bsearch(phi-cyc, self._lookup)
        if 'c' in self.DEBUG: 
          debugMsg(self,'movePhase down %d --> 1, -2 --> %d' % (self._pos-1,npos-1)) 
        if self._pos>1:
          self._doActions( slice(self._pos-1, 0, -1 ) )
        if phi<-1:
          self.onCycles(cyc+1)
        self._doActions( slice(-2, npos-1, -1) )
    # final state is wrapped phase with new position
    self.phase = phi-cyc
    self._pos = npos
  
  def _doActions( self, slc ):
    """(private) 
    
    Do the actions associated with action table entries in the specified slice
    """
    if 'C' in self.DEBUG:
      debugMsg(self,'doActions %s ~%5.2g  ~%5.2g ' % (repr(slc),
        self._lookup[slc.start],self._lookup[slc.stop - slc.step]))
    self.seq = slc.indices(len(self._lookup))
    for self._pos in xrange(*self.seq):
      self.phase = self._lookup[self._pos]
      if 'C' in self.DEBUG:
        debugMsg(self,'   -- do %d ~%5.2g' % (self._pos,self.phase))
      try:
        self.actions[self.phase](self)  
      except StandardError:
        printExc()
  
  def behavior(self):
    """(final)
    
    CyclePlan behavior is to cycle through the actions at the
    period controlled by self.period
    
    If period is set to 0, actions will only be called as consequence
    of calling .moveToPhase
    """
    last = self.app.now    
    while True:
      if self.period:
        phi = self.phase + (last-self.app.now) / self.period
        self.moveToPhase(phi) 
        last = self.app.now
      yield      

class FunctionCyclePlan( CyclePlan ):
  """Concrete class FunctionCyclePlan implements a plan that cycles
  'phase' at an adjustable frequency through the interval [0..1],
  calling a user specified function every time the phase crosses 
  a knot point.
  
  Additional features:
  (1) An upper limit on allowed frequency
  (2) Knots points may be specified at construction or equally spaced
  (3) Function calls may be decimated so that a minimal time interval 
      elapses between calls.
      
  Typical usage is to write a function mapping phase to some kinematic
  state of a robot gait. In that case, one typically sets knots to 
  be either uniform, or preferentially represent the high-curvature
  parts of the gait data. The interval limit is set to a realistic 
  closed loop delay value, ensuring that commands to the motors are
  not sent faster than they can be processed.
  """
  def __init__(self, app, atPhiFun, N=None, knots=None, maxFreq=10.0, interval=0, *arg, **kw):
    """
    INPUTS:
      app -- JoyApp -- owner
      atPhiFun -- callable -- function that will be called with a 
            phase in the range [0..1].
      N -- integer (optional) -- number of knot points, for regularly
            spaced knot points.
      knots -- sequence -- sorted sequence of values in the range 
            [0..1], designating phases at which atPhiFun is called
      maxFreq -- float -- maximal frequency for cycling (Hz)
      interval -- float -- minimal time elapsed between calls    
    """
    if knots is None:
      if N is None or N != int(N) or N<1:
        raise ValueError('Must specify either positive integer N or knots')      
      knots = [ float(x)/N for x in xrange(N) ]
    if not callable(atPhiFun):
      raise TypeError('atPhiFun must be callable')
    self._func = atPhiFun
    self._time = app.now
    self._interval = interval
    act = {}
    for k in knots:
      act[k] = self._action
    
    CyclePlan.__init__(self,app,act,maxFreq)  
  
  def _action( self, arg ):
    # If call rate is limited
    if self._interval:
      # If it isn't time to call yet --> return
      if self.app.now<self._time:
        return
      # Set time limit for next call
      self._time = self.app.now + self._interval
    assert arg is self
    self._func( self.phase )
    
class GaitCyclePlan( CyclePlan, SheetPlan ):
  """
  GaitCyclePlan-s combine the benefits of a SheetPlan with those of a 
  CyclePlan. They represent "cyclical spreadsheets" consisting of settings
  that should be applied via the Plan bindings at various phases in the 
  cycle.
  
  GaitCyclePlan usage example:
  >>> gcp = GaitCyclePlan( app, 
  ...    sheet = [['t','x'],[0,1],[0.25,0],[0.5,-1],[0.75,0]],
  ...    x = '>pos' )
  >>> gcp.start()
  
  would make a 4-step triangle wave be emitted to Scratch variable 'pos'
  """
  def __init__(self, app, sheet, maxFreq=10.0, *arg, **binding ):
    """
    Initialize a GaitCyclePlan instance
    
    INPUTS:
      app -- JoyApp -- application containing this Plan
      sheet -- list of lists -- gait table (see SheetPlan for details)
      maxFreq -- float -- maximal gait frequency; default is 10 Hz
      
      Additional keyword arguments provide output bindings for the
      gait table columns. sheet[0] is a list of column headings, with
      sheet[0][0]=='t' the time (phase). Unlike other SheetPlans, the
      time column in a GaitCyclePlan MUST range 0.0 to 1.0

      If no column bindings are provided, column headings are assumed 
      to be module names of servo modules and each of these is bound to
      <<module>>/@set_pos of the corresponding module. For example, the sheet:
      >>> sheet = [ 
      ...   ["t",   "Nx15"],
      ...   [0,     1000],
      ...   [0.5,  -1000] ]
      will generate a +/- 10 degree square wave into the position of node 0x15 
    """    
    self.DEBUG = app.DEBUG
    if not hasattr(self,'_binding'):
      if not binding:
        binding = self._autoBinding(sheet)
      self._initBindings( app, binding )
    self.setters,self.headings,self.sheet = self._parseSheet(sheet)
    act = {}
    for row in self.sheet:
      act[row[0]] = self._action  
    CyclePlan.__init__(self, app, act, maxFreq=maxFreq, *arg)
    
  def _action( self, arg ):
    assert arg is self
    if 'g' in self.DEBUG: 
      debugMsg(self,"phase %4.2f pos %d" % (self.phase, self._pos) )
    self._doRowSets( self.sheet[self._pos-1][1:] )  

class StickFilter( Plan ):
  """
  StickFilter Plans are event processors designed to simplify the problem of
  processing joystick and midi readings into continuous-time values.
  
  The problem arises because pygame joystick events appear only when joystick
  values change -- even if they are far from 'zero'. This means that triggering
  actions directly from joystick events can lead to rather erratic responses.
  
  StickFilter solves this problem by simulating the behavior of a regularly
  sampled time-series for each incoming event type. All these time series
  have the same sample interval (self.dt), but a different linear transfer 
  function can be associated with the values in each channel.
  
  In particular, StickFilter provides methods to lowpass filter joystick 
  readings or to integrate them. The former is used to limit the "jerkiness"
  of controller requests, whereas the latter is used to control velocity
  with the joystick instead of position.
  
  Channel Names
  -------------
  
  Currently, StickFilter supports the channel name schemata listed below.
  Any given channel will only be instantiated if a filter is set for it
  and the related events are .push()-ed into the StickFilter plan.
  
  The schemata:
  joy<joystick-number>ball<ball-number>
  joy<joystick-number>hat<hat-number>
  joy<joystick-number>axis<axis-number>, e.g. joy0axis1, for joystick events
  Nx<node-id-2-HEX-digits>, e.g. Nx3C, for CKBOTPOSITION events
  midi<dev-number>sc<scene><kind><index-number> MIDI input device
  """
  def __init__(self,app,t0=None,dt=0.1):
    """
    Initialize a StickFilter 
    """
    Plan.__init__(self,app)
    # Mapping name --> filter_A, filter_B, x, y, t
    self.flt = {}
    self.dt = dt
    self.t = None
  
  def setLowpass( self, evt, tau, t0=None, func=float ):
    """
    Process evt events with a first-order lowpass with time constant tau.    
    
    Uses an approximation to the Butterworth construction    
    """
    if tau<2:
      raise ValueError('Only tau>=2 supported, got tau=%g' % tau)
    a = exp(-pi / tau)
    b = (1-a)/2.0
    return self.setFilter( evt, t0, [1,-a], [b,b], func=func )

  def setIntegrator( self, evt, gain=1, t0=None, lower=-1e9, upper=1e-9,  func=float):
    """
    Process evt events with a (leaky) integrator
    first-order lowpass with time constant tau.

    By default, integrator is not leaky.
    """
    return self.setFilter( evt, t0, [1,-1], [gain],lower=lower,upper=upper, func=func )

  def setFilter( self, evt, t0=None, A=[1.0], B=[1.0], x0=None, y0=None,lower=-1e9, upper=1e9, func=float ):
    """
    Specify a filter for an event channel.
    
    The filter equation is given by:
       A[0]*y[n] = B[0]*x[n] + B[1]*B[n-1] + ... + B[nb]*x[n-nb]
                             - A[1]*y[n-1] - ... - A[na]*y[n-na]

    or, in terms of transfer functions:
                                          nb
            B[0] + B[1] z + ... + B[nb] z 
       Y = ------------------------------------ X
                                          na
            A[0] + A[1] z + ... + A[na] z 
    
    INPUTS:
      evt -- event object from channel, or its name as string
      t0 -- float -- initial time to start channel, or None to use self.app.now
      A,B -- sequences of floats -- transfer function
      x0 -- sequence of len(B) floats -- initial state of x values (default 0-s)
      y0 -- sequence of len(A) floats -- initial state of y values (default 0-s)
      lower -- number -- lower limit on filter values (saturation value)
      upper -- number -- upper limit on filter values (saturation value)
      func -- callable -- input mapping applied to channel values before filtering
    """
    # Identify the event class
    if isinstance(evt,EventType):
      evt,_ = self.nameValFor(evt)
    if type(evt) != str:
      raise TypeError(
        "Event '%s' must be string / pygix.Event" % repr(evt))
    # Make sure parameters are lists of floats with matching lengths
    if x0 is None: x0 = [0.0]*len(B)
    else: x0 = [float(x0i) for x0i in x0] 
    if y0 is None: y0 = [0.0]*len(A)
    else: y0 = [float(y0i) for y0i in y0] 
    A = [ float(a) for a in A ]
    B = [ float(b) for b in B ]
    if len(x0) != len(B): 
      raise ValueError('len(x0)=%d; must equal len(A)=%d' % (len(x0),len(A)))
    if len(y0) != len(A): 
      raise ValueError('len(y0)=%d; must equal len(B)=%d' % (len(y0),len(B)))
    # Test func
    if not callable(func):
      raise TypeError("'func' must be a callable")
    # Store
    if t0 is None: t0 = self.app.now
    self.flt[evt] = (tuple(A),tuple(B),x0,y0,[],[t0],(lower,upper),func)    

  def feed(self, evt ):
    """
    Feed an event into the StickFilter.
    Retrieves value and puts it on input queue for filter 
    
    INPUT:
      evt -- EventType object      
    """
    # Search for key name of this event
    key,val = self.nameValFor(evt)
    if (key is None) or (not self.flt.has_key(key)): 
      return 
    # Retrieve filter state
    flt = self.flt[key]
    # Overwrite queue to contain this value
    flt[4][:]=[flt[-1](val)]

  def setToZero(self, evt):
    """
    Set a specific filter to zero
    Sets all previous state variables (X, Y) to zero
    
    INPUT:
      evt -- EventType object or string name of event channel
    """
    # Search for key name of this event
    key,val = self.nameValFor(evt)
    if (key is None) or (not self.flt.has_key(key)): 
      raise KeyError("No filter for %s" % repr(evt))       
    # Retrieve filter state
    flt = self.flt[key]
    # Overwrite queue to contain this value
    A,B,X,Y,Q,(last,),(lb,ub),func = flt
    X[:]=[0]*len(X)
    Y[:]=[0]*len(Y)
    self.flt[key] = ( A,B,X,Y,Q,(last,),(lb,ub),func )

  def _runFilterTo(self, flt, t):
    """(private)
    
    run filter off input in its queue until time t
    INPUT:
        flt -- a filter state
        t -- float -- time
    """
    A,B,X,Y,Q,(last,),(lb,ub),func = flt
    # Interpolating up to time t, feed samples into filter
    # System equation is:
    #  a[0]*y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[nb]*x[n-nb]
    #                        - a[1]*y[n-1] - ... - a[na]*y[n-na]    
    ts = last
    t -= self.dt
    while ts<t:
      ts += self.dt
      # Repeat previous sample / take from queue Q
      if Q: X.insert(0,Q.pop(0))
      else: X.insert(0,X[0])
      X.pop()
      ## Initialize filter
      z0 = sum( ( b*x for b,x in zip(B,X) ) )
      z1 = sum( ( a*y for a,y in zip(A[1:],Y[:-1]) ) )
      ## Store new result
      y = (z0-z1)/A[0]
      y = max(min(y,ub),lb)
      Y.insert(0,y)
      Y.pop()
      #DEBUG# progress("A %s B %s X %s Y %s" % tuple(map(repr,(A,B,X,Y))))
      #DEBUG# progress("-FLT- %9g %g %g %g %g" % (ts,X[0],Y[0],z0,z1))
    # Update filter timestamp
    flt[-3][0] = ts
    return ts
    
  def getValue( self, evt ):
    """
    Obtain the filtered value for an event channel by using an event 
    object or the string name of the event.
    """
    # Identify the event class
    if isinstance(evt,EventType):
      evt,_ = self.nameValFor(evt)
    if type(evt) != str:
      raise TypeError(
        "Event '%s' must be string / pygix.Event" % repr(evt))
    # Retrieve last value from filter
    flt = self.flt[evt]
    return flt[3][0]

  def getterOf( self, evt ):
    """
    Obtain a getter function for an event channel
    """    
    return curry( self.getValue, evt )
    
  @classmethod
  def nameValFor( cls, evt ):
    '''
    Generate channel name (a string) from an event object
    '''
    if evt.type==JOYAXISMOTION:
      return 'joy%daxis%d' % (evt.joy,evt.axis), evt.value
    elif evt.type==JOYBALLMOTION:
      return 'joy%dball%d' % (evt.joy,evt.ball), evt.rel
    elif evt.type==JOYHATMOTION:
      return 'joy%dhat%d' % (evt.joy,evt.hat), evt.value
    elif evt.type==CKBOTPOSITION:
      return 'Nx%02X' % evt.module, evt.pos
    elif evt.type==MIDIEVENT:
      return 'midi%dsc%d%s%d' % (evt.dev,evt.sc,evt.kind,evt.index), evt.value
    return None, None
    
  def onEvent( self, evt ):
    """(final)
    
    Handle incoming events. Timer events allow the filters to update state;
    other events are pushed into the filters with .feed() and do not require
    the behavior() to run (hence return False).
    """
    if evt.type==TIMEREVENT:
      return True
    self.feed(evt)
    return False

  def behavior( self ):
    """(final)
    
    Runs periodically every dt time units.
    
    Evolves states of all filters up to the current time.
    """
    while True:
      yield self.forDuration(self.dt)
      for flt in self.flt.itervalues():
        if self.t is None: t = self.app.now
        else: t = self.t
        self._runFilterTo( flt, t )

class MultiClick( Plan ):
  """
  The MultiClick Plan class is an event processing aid for using events that
  come in <something>UP and <something>DOWN pairs. <something> can currently
  be KEY, MOUSEBUTTON or JOYBUTTON.
  
  Because UP and DOWN events happen for individual keys, it is cumbersome to
  write interfaces that use key combinations, or use multi-button game 
  controller combinations as commands. MultiClick exists to address this
  difficultly.
  
  MultiClick takes in UP and DOWN events and processes them into two kinds
  of events: Click and MultiClick. Click events occur when an UP event is
  received that quickly followed the corresponding DOWN event, indicating a
  short "click". If the DOWN event is quickly followed by more DOWN events,
  these are combined until no additional events are received for a while.
  At that point, a "MultiClick" event is generated. Similarly, UP events are
  combined if they are received close to each other. If a "Click" occurs while
  another "MultiClick" is happening, the MultiClick is re-generated after the
  "Click".
  
  Example
  ------- 
 
  To illustrate these ideas, assume three buttons 1 2 and 3 and take a '-' 
  to indicate a long delay. The following timeline demonstrates these ideas:
  DOWN 1 
  UP 1  --> click 1
  DOWN 1
  DOWN 2
  -     --> MultiClick [1,2]
  UP 1
  -     --> MultiClick [2]
  DOWN 3
  UP 3  --> Click 3, MultiClick [2]
  -
  UP 2
  -     --> (optional) MultiClick []
  
  Usage
  -----
  For convenience, the default behavior for MultiClick is to call event 
  handler methods of the JoyApp that owns it. This means that for typical use
  cases the MultiClick Plan can be used as is, without subclassing.
  
  MultiClick will call self.app.onClick( self,evt ) for click events, where evt
  is the UP event that generated the click.
  
  MultiClick will call self.app.onMultiClick( self, evts ) for multi-click
  events, where evts is a dictionary whose values are the DOWN events that
  combined into this multi-click combination.
  
  The MultiClick object is passed to these event handlers to allow events
  from multiple MultiClick Plans to be distinguished by the event handler.
  
  Alternatively, MultiClick can be subclassed. Subclasses may override the
  .onClick(evt) and .onMultiClick(evts) methods to process the events.
  
  NOTE: the MultiClick .onEvent returns False unless the onClick or
    onMultiClick handlers return boolean True. Thus the .behavior never gets to
    run -- but a subclass may override the .behavior and use a True return
    value from the JoyApp to control its execution.
  """
  def __init__(self,app,allowEmpty=False,delay=0.2):
    """
    Initialize a MultiClick Plan
    INPUTS:
      app -- JoyApp -- application running this Plan
      allowEmpty -- boolean -- should MultiClick events indicating no keys are
         currently pressed (empty set payload) be emitted. Default is False
      delay -- float -- delay used for merging multiple events
    
    NOTE:
      MultiClick can only combine the events you .push() to it. For example,
      a MultiClick that only gets JOYBUTTONDOWN and JOYBUTTONUP from one
      joystick will only generate Click and MultiClick events from that
      joystick's buttons -- no keyboard, no mouse, no other joysticks.
    """
    Plan.__init__(self,app)
    self._when = None
    self.delay = delay
    self._acts = {}
    self.allowEmptyEvent = allowEmpty
  
  @classmethod
  def nameFor( cls, evt ):
    '''
    Generate a hashable name for each event we handle
    '''
    if evt.type in [KEYDOWN,KEYUP]:
      return 'key%08x' % (evt.key | (evt.mod<<10))
    elif evt.type in [MOUSEBUTTONUP,MOUSEBUTTONDOWN]:
      return 'mouse%d' % evt.button
    elif evt.type in [JOYBUTTONDOWN,JOYBUTTONUP]:
      return 'joy%dbtn%d' % (evt.joy,evt.button)
    return None    
      
  def onEvent( self, evt ):
    """(final)
    
    Process incoming UP or DOWN events. Use TIMEREVENT to test whether a
    key combination has persisted long enough for emitting a MultiClick.
    """
    # If a TIMEREVENT and key combination has been stable for a while
    if (evt.type==TIMEREVENT
        and self._when is not None
        and self.app.now > self._when):
      self._when = None
      if self._acts or self.allowEmptyEvent: 
        return (self.onMultiClick( self._acts.copy() ) is True)
      return False
    # Find the lookup key for this event
    nm = self.nameFor(evt)
    if nm is None:
      return False
    #
    A = self._acts
    res = False
    if evt.type in [KEYDOWN,MOUSEBUTTONDOWN,JOYBUTTONDOWN]:
      A[nm] = (evt,self.app.now)
      self._when = self.app.now + self.delay
    elif evt.type in [KEYUP,MOUSEBUTTONUP,JOYBUTTONUP]:
      if A.has_key(nm):
        if self.app.now<self._when:
          res = self.onClick(evt)
        elif len(A)==1:
          self._when =self.app.now + self.delay
        del A[nm]
    return (res is True)
  
  def behavior( self ):
    """(default)
    
    Override in subclass if you plan to have onEvent return True
    """
    while True:
      yield
      debugMsg(self,"onEvent returned True") 
      
  def onClick(self, evt):
    """(default)
    Punts to self.app.OnClick(self,evt)
    
    Override in subclass to process "Click" events -- i.e. events
    that represent short (less than self.delay) keypresses / clicks
    
    INPUT:
        evt -- pygame event
    """
    return self.app.onClick(self,evt)
  
  def onMultiClick(self, evts):
    """(default)
    Punts to self.app.onMultiClick(self,evts)

    Override in subclass to process "MultiClick" events -- i.e. 
    events that represent one or more keys held together for long
    (i.e. more than self.delay). 
    
    A key can be clicked while other keys are MultiClicked. This
    will generate a new multi-click after the short click is over.
    
    When all keys are up, a multi-click can be generated with an
    empty evts dictionary. This feature is controlled by
    self.allowEmptyEvents, which is set by the allowEmpty 
    parameter to the contstructor.
    
    INPUT:
        evts -- dictionary -- event category --> pygame event       
    """
    return self.app.onMultiClick(self,evts)


class AnimatorPlan(Plan):
    """
    Concrete class AnimatorPlan 
    
    Wrapper for making easy animations using matplotlib and JoyApp.
    
    USAGE:
      The AnimatorPlan constructor is given a function that takes a matplotlib
      figure object and updates the figure each iteration.
    """
    def __init__(self,app,fun=None,fps=20):
      Plan.__init__(self,app)
      self.set_fps(fps)
      self.set_generator(fun)
    
    def set_fps(self,fps):
      """Set FPS for animation"""      
      self.delay = 1.0 / fps

    def set_generator(self,fun):
      """
      Set the frame generator function
      INPUT:
         fun -- function -- takes matplotlib.figure and returns a generator
            that updates the figure every .next() call
      """
      assert callable(fun)
      self.fun = fun
      
    def behavior(self):
      for fr in self.fun(self.app.fig):
        self.app.animate()
        yield self.forDuration(self.delay)
