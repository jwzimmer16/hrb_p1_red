"""
FILE: pygix.py

pygame isolation layer. This Isolation layer interfaces with pygame when available.
When pygame is not available, it provides similar functionality to allow core
JoyApp functions to work, and imports the relevant constants from the binary 
pygame modules (which have very few dependencies, and so can be compiled anywhere 
you have SDL)
"""

# Note: substantial ugliness is encoded in constants to allow it to use 
#  c implementations of various pygame architectures transparently
#  regardless of which is used, it becomes the 'constants' sub-module
#  and its IMPL member indicates which implementation was used
from constants import *

if IMPL=="pygame":
  # Pygame dependencies
  import pygame
  import pygame.joystick
  import pygame.event
  _OLD_REPEAT = None
  
  def startup( cfg ):
    """
    Initialization performed on start
    
    return pygame screen object
    """
    pygame.init()
    # set up key repeating so we can hold down the key to scroll.
    global _OLD_REPEAT
    _OLD_REPEAT = pygame.key.get_repeat ()
    pygame.key.set_repeat (cfg.keyboardRepeatDelay, cfg.keyboardRepeatInterval)
    pygame.time.set_timer(TIMEREVENT, cfg.clockInterval)
    J = [ pygame.joystick.Joystick(k)
            for k in xrange(pygame.joystick.get_count()) ]
    for joy in J:
      joy.init()  
    screen = pygame.display.set_mode(cfg.windowSize)
    pygame.display.flip()
    return screen

  def shutdown():
    pygame.key.set_repeat (*_OLD_REPEAT)
    pygame.quit()

  def postEvent(evt ):
    return pygame.event.post( evt )
     
  def Event(*argv, **kw ):
    return pygame.event.Event(*argv,**kw)

  def event_name(evt ):
    return pygame.event.event_name(evt)
    
  def iterEvents():
    evts = pygame.event.get()    
    for evt in evts:
      if evt.type == TIMEREVENT:
        pygame.display.update()
      yield evt

  def showMplFig(buf,sz,scr,bg=(255,255,255)):
    img = pygame.image.frombuffer(buf,sz,'RGBA')
    scr.fill(bg)
    scr.blit(img,(0,0))

  def get_impl():
    return True

  EventType = pygame.event.EventType
        
else: # pygame import failed
  assert IMPL is not None
  from time import sleep, time as now
  
  _EVENT_Q = []  
  _TIMESLICE = 0.05
  _TNEXT = now()
  USEREVENT = 100
  
  def startup( cfg ):
    """
    Initialization performed on start
    
    return pygame screen object
    """
    global _TIMESLICE, _TNEXT
    _TIMESLICE = cfg.clockInterval / 1000.0
    _TNEXT = now()+_TIMESLICE
    return None

  def shutdown():
    pass

  def postEvent(evt ):
    global _EVENT_Q
    _EVENT_Q.append(evt)
  
  class Event( object ):
    def __init__(self, typecode, attr={}):
      self.type=typecode
      if attr:
        self.__dict__.update(**attr)

  ### NOTE: this table needs to be verified!
  EVENT_NAMES = {
     0 : 'QUIT',    
     1 : 'ACTIVEEVENT',
     2 : 'KEYDOWN',    
     3 : 'KEYUP',	     
     4 : 'MOUSEMOTION',
     5 : 'MOUSEBUTTONUP',
     6 : 'MOUSEBUTTONDOWN',
     7 : 'JOYAXISMOTION',  
     8 : 'JOYBALLMOTION',  
     9 : 'JOYHATMOTION',   
    10 : 'JOYBUTTONUP',    
    11 : 'JOYBUTTONDOWN',  
    12 : 'VIDEORESIZE',
    13 : 'VIDEOEXPOSE',    
    14 : 'CKBOTPOSITION',  
    15 : 'SCRATCHUPDATE',  
    16 : 'TIMEREVENT',     
    17 : 'MIDIEVENT'
  }
  def event_name(evType):
    return EVENT_NAMES.get(evType, None)
    
  def iterEvents():
    global _EVENT_Q,_TNEXT
    while _EVENT_Q:
      yield _EVENT_Q.pop(0)
    t = now()
    if t<_TNEXT:
      sleep(_TNEXT-t)
    yield Event( TIMEREVENT )
    _TNEXT = t+_TIMESLICE

  def showMplFig(buf,sz,bg=(255,255,255)):
    "Running headless"
    pass

  def get_impl():
    return True

  EventType = type(Event(0))

############################################################################
# These constants are defined for both implementations
############################################################################

## Declare new event type numbers
# TIMEREVENT-s are generated regularly and used to drive Plan execution
TIMEREVENT = USEREVENT
# CKBOTPOSITION events indicate a change in the position of a robot module 
#   a "change" needs to be at least `positionTolerance` units, and position
#   changes are polled `robotPollRate` seconds apart. Both `positionTolerance`
#   and `robotPollRate` are configuration parameters and can be set via the
#   JoyApp.yml configuration file.
CKBOTPOSITION = USEREVENT+1
# SCRATCHUPDATE events indicate a change in a Scratch channel
SCRATCHUPDATE = USEREVENT+2
# MIDIEVENT-s indicate input from a MIDI device
MIDIEVENT = USEREVENT+3
  
EVENT_STRUCTURE = {
    QUIT             : (),
    ACTIVEEVENT      : ('gain', 'state'),
    KEYDOWN          : ('unicode','key','mod'),
    KEYUP	         : ('key','mod'),
    MOUSEMOTION	     : ('pos','rel','buttons'),
    MOUSEBUTTONUP    : ('pos','button'),
    MOUSEBUTTONDOWN  : ('pos','button'),
    JOYAXISMOTION    : ('joy','axis','value'),
    JOYBALLMOTION    : ('joy','ball','rel'),
    JOYHATMOTION     : ('joy','hat','value'),
    JOYBUTTONUP      : ('joy','button'),
    JOYBUTTONDOWN    : ('joy','button'),
    VIDEORESIZE      : ('size','w','h'),
    VIDEOEXPOSE      : (),
    CKBOTPOSITION    : ('module','pos'),
    SCRATCHUPDATE    : ('scr','var','value'),
    TIMEREVENT       : (),
    MIDIEVENT        : ('dev','sc','kind','index','value'),
  }
  
JOY_EVENT_NAMES = {
    SCRATCHUPDATE : "ScratchUpdate",
    CKBOTPOSITION : "CKBotPosition",
    TIMEREVENT : "TimerEvent",
    MIDIEVENT : "MIDIEvent"
  }

