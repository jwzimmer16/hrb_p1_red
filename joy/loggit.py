"""
loggit.py implements a general purpose logging interface and some convenience
functions for emitting both debug and progress messages.

The interface consists of the LogWriter class and several utility functions,
most notable of which is progress().

Functions:
  dbgId, debugMsg -- used for generating useful debug messages
  
  progress -- emit a text message "immediately" as a progress indication for 
    the user. Progress messages can also be emitted using speech synthesis 
    and may be automatically logged to one or more LogWriter-s.
  
  iterlog -- iterator for log entries
  
Data Format:
  loggit logs are GNU-zipped (using the gzip module) streams of 'safe' YAML
  documents. YAML is a human and machine readable text format, much more
  friendly than XML; see http://www.yaml.org . Each document contains a single
  YAML mapping object, with at least a TIME integer and a TOPIC string. By
  virtue of the YAML standard, mapping keys are sorted in ASCII lexicographic
  order, so that if all other log entry mapping keys start with lowercase
  letters the first two keys will be TIME and TOPIC, e.g.
  
  --- {TIME: 286447, TOPIC: getter, func_name: get, value: 7}
"""
import gzip, yaml
import types
from time import time
from sys import stdout
from speak import say

# Time at which this module was imported. Used as base time for all `progress`
#   messages printed.
T0 = time()

PROGRESS_LOG = set()

def dbgId(obj):
  """Generate a unique, human readable name for an object"""
  if type(obj) in [tuple,list,set]:
    return '[%s]' % ", ".join( (dbgId(x) for x in obj) )
  return '%s@%04x' % (obj.__class__.__name__,id(obj) % (0x10000-1))

def debugMsg(obj,msg):
  """
  Print a "debug" message. Debug messages are indicated by a leading 'DBG'
  and the dbgId() of the object sending the message. This is used to make
  it easier to identify the actual object that generated a given message.
  """
  nm = dbgId(obj)
  progress(("DBG %s " % nm)+msg.replace("\n","\n   : "))    

__NEED_NL = False
def progress(msg,sameLine=False):
  """
  Print a progress message to standard output. The message will have a 
  pre-pended timestamp showing seconds elapsed from the moment the joy
  module was imported.
  
  Messages that start with the string "(say) " will be read out loud on
  systems that support it. See the speak module for details.
  
  Progress messages flush standard output, so they display immediately.
  
  A copy of progress messages is write()n to every logger in PROGRESS_LOG
  
  if sameLine is True, message is prefixed by a "\r" instead of ending with
  a "\n" -- showing it on the same line in the terminal
  """
  global __NEED_NL
  t = time()-T0
  if sameLine:
      stdout.write("\r%6.2f: %s" % (t,msg))
      __NEED_NL = True
  else:
      if __NEED_NL:
          stdout.write("\n")
      stdout.write("%6.2f: %s\n" % (t,msg))
      __NEED_NL = False
  stdout.flush()
  if msg[:6]=="(say) ":
    say(msg[6:])
  for logger in PROGRESS_LOG:
    logger.write('logmsg',progress=msg)

class LogWriter( object ):
  """
  Concrete class LogWriter provides an interface for logging data.
  
  Logs may be written to a stream or a file. The default file format is
  a gzipped YAML file. If output is sent to a stream, the output is an
  uncompressed YAML.
  
  Each .write() operation maps into a single YAML document, allowing the
  yaml module's safe_load_all method to parse them one at a time.
  
  Typical usage:
  >>> L = LogWriter('mylog.yml.gz')
  >>> L.write( foo = 'fu', bar = 'bar' )
  >>> L.close()
  >>> for data in iterlog('mylog.yml.gz'):
  >>>   print data
  
  """
  def __init__(self,stream=None,sync=False):
    """
    INPUTS:
      stream -- stream -- output stream to use, must support write, flush, close
             -- str -- filename to use (.gz added automatically if not present)
             -- None -- logger started in uninitialized state; use open()
      sync -- boolean -- set to force a flush after every log entry.

    ATTRIBUTES:
      .s -- the open stream
      .sync -- auto-flush flag
      .timeUnit -- time unit for log timestamps; default is millisecond
    """
    if stream is not None:
      self.open(stream)
    else:
      self.s = None
    self.sync = sync
    self.timeUnit = 1e-3
    
  def open( self, stream ):
    """
    Open a logging stream. 
    INPUT:
      stream -- str -- a file name for the log. A .gz will be appended if not
                present. Log will be stored as a gzip YAML file.
             -- file-like -- an object with .write() .flush() and .close() 
                methods. Log will be sent as YAML text, with a single .write()
                for each entry of the log.
    """
    if type(stream)==str:
      if stream[-3:]!='.gz':
        stream = stream + ".gz"
      stream = gzip.open(stream,"w")
    self.s = stream    
    
  def write( self, topic, **kw ):
    """
    Write a log entry. Each entry consists of a topic string and an 
    automatically generated timestamp, and may include a dictionary of 
    additional attributes supplied as keyword arguments.
    
    The log entry will be emitted as a YAML document (entry starting with ---)
    that contains a mapping with keys TIME for the timestamp (in units of
    .timeUnit) and TOPIC for the topic.
    
    If .sync is set, the stream will be flushed after each entry. 
    """
    if self.s is None:
      raise IOError("LogWriter stream is not open")
    t = long((time()-T0)/self.timeUnit)
    kw.update(TIME=t,TOPIC=topic)
    entry = "--- " + yaml.safe_dump(kw)
    # print "~~> ",entry
    self.s.write(entry)
    if self.sync:
      self.flush()
    
  def close( self ):
    """
    Close a logger output stream
    """
    self.s.close()
    self.s = None
  
  def flush( self ):
    """
    Flush the logger output stream
    """
    self.s.flush()

  def getterWrapperFor( self, fun, fmt=lambda x : x, attr={} ):
    """
    Wrap the specified callable and log results of calls.
    INPUTS:
      fun -- callable -- "getter" function with no params
      fmt -- callable -- formatting function for the parameter. 
                Must return something yaml can safe_dump()
      attr -- dict -- dictionary of extra attributes for log entry 
    OUTPUTS:
      callable function that calls fun() and logs the results   
      
    Typical usage is to take an existing getter function and replace it in 
    in place with the wrapped getter:
    >>> def dummyGetter():
    >>>   return int(raw_input())
    >>> L = joy.loggit.LogWriter("foo")
    >>> g = L.getterWrapperFor(dummyGetter
        ,fmt=lambda x : "0x%X" % x
        , attr=dict(sparrow = 'african'))
    >>> g()
    258
    258
    >>> L.close()
    >>> !gunzip -c foo.gz
    --- {TIME: ????, TOPIC: getter, func_name: dummyGetter, sparrow: african, value: '0x102'}
    """
    if type(fun) is types.MethodType:
      fn = fun.__func__.__name__
    elif type(fun) is types.FunctionType:
      fn = fun.func_name
    else:
      fn = "<%s instance>" % fun.__class__.__name__
    def _getWrapper():
      val = fun()
      self.write("getter",func_name=fn, value=fmt(val), **attr)
      return val
    return _getWrapper
    
  def setterWrapperFor( self, fun, ifmt = lambda x : x, ofmt = None, attr={} ):
    """
    Wrap the specified callable and log parameters of calls.
    INPUTS:
      fun -- callable function with one parameter
      name -- function name to use in 
      ifmt -- formatting function for arguments
      ofmt -- formatting function for results / None to omit result
    OUTPUTS:
      callable function that logs the parameter and then
      calls fun() with it and logs the results    
    
    Usage -- see example for getterWrapperFor
    """
    fn = fun.__func__.__name__
    def _setWrapper(arg):
      res = fun(arg)
      if ofmt is None:
        self.write("setter",func_name=fn, argument=ifmt(arg),**attr )
      else:
        self.write("setter",func_name=fn, argument=ifmt(arg), result=ofmt(res), **attr)
      return res
    return _setWrapper
    
def iterlog( filename ):
  """
  Iterate over a logfile returning entry values
  
  Entries have keys TIME and TOPIC for timestamp and topic
  
  OUTPUT: t,topic,val
    t -- timestamp
    val -- dictionary with log entry values
  """
  f = gzip.open( filename,"r")
  try:
    for entry in yaml.safe_load_all(f):
      yield entry
  finally:
    f.close()

