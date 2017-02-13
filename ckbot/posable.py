"""File posable.py

Provides basic functionality for "posable programming", i.e. programming a robot by recording its poses and then playing them back.

Main Classes and Functions
==========================

class PoseRecoder -- provides most of the functionality, allowin poses to be recorded and played back.

function recordFromCluster -- an interactive interface on top of a PoseRecorder, allowing users to easily record poses.
"""

from time import time as now, sleep
from sys import stdout as STDOUT
from numpy import asarray
from cmd import Cmd
from re import split as re_split
from scipy.interpolate import interp1d

class PoseRecorder( object ):
    """Concrete class PoseRecorder

    Provides a convenient interface for 'poseable programming' by allowing
    positions of servos to be recorded and played back.

    Convenience methods for saving and loading .csv files are also provided

    For typical usage, see the source code for recordFromCluster()
    """
    def __init__(self,mods):
        """Initialize the PoseRecorder to record a list of modules
        
        INPUT:
          mods -- sequence of modules with .get_pos(), .set_pos() and .go_slack() methods
        """
#        for m in mods:
#            assert isinstance(m,Module),"%s must be a module" % str(m)
        self.servos = mods
        self.record_time = None
        self.reset()
        self.off()

    def getPose(self):
        """collect positions of all servos"""
        return [ m.get_pos() for m in self.servos ]

    def _set_pose(self,pose):
        """(private) set positions of all servos"""
        for m,v in zip(self.servos, pose):
           m.set_pos(v)

    def off(self):
        """Emegency Stop for all servos -- .go_slack()-s all modules, ignoring any exceptions"""
        for m in self.servos:
          try:
            m.go_slack()
          except Exception, ex:
            print "Exception in %s.go_slack():" % str(m), str(ex)

    def reset( self ):
        """Reset the recording, allowing a new recording to be started"""
        self.plan = []
        self.record_time = None

    def snap( self, t = None ):
        """Add a snapshot of the current pose to the recording"""
        if not self.plan:
          t = 0
        if t is None:
          t = self.plan[-1][0]+1
        self.plan.append( [t]+self.getPose())

    def snapClosed( self, dt = 1 ):
        """Add the first pose at the end of the recording, with specified delay.
        This allows loops to be cleanly repeated.

        INPUT:
          dt -- float -- delay in seconds between last and first pose
        """
        if not self.plan:
            raise IndexError("No recoding found; call .snap() at least once")
        self.plan.append( [self.plan[-1][0]+dt]+self.plan[0][1:] )

    def show( self, stream=STDOUT, delim=" ", fmt="%5d", hfmt="%5s", rdel="\n", slc=slice(None)):
        """Write the current recording out on a stream in a text-based format
 
        INPUT:
          stream -- output stream object. Must support .write()
          delim -- str -- delimiter user between columns
          fmt -- str -- format string for numbers (except time)
          hfmt -- str -- format string for column headings
          rdel -- str -- row delimiter
          slc -- slice -- range of pose sequence to show
        """
        stream.write( delim.join(
          ["%5s" % "t"]
          +[hfmt % m.name for m in self.servos[slc]]
          ) + rdel )
        for pose in self.plan[slc]:
          stream.write( delim.join(
            ["%5d" % pose[0]]
            +[fmt % v for v in pose[1:]]
          ) + rdel )

    def unsnap(self):
        """Drop the last pose from the recording"""
        if self.plan:
            self.plan.pop(-1)

    def appendFrom( self, stream, cdel=",", dt=1  ):
        """append a recording from a textual representation
        
        INPUT:
          stream -- list -- a recording with one line per list entry
                 -- file -- a file containing a recording
          cdel -- str -- column delimiter
          dt -- float -- gap in time between last entry and newly loaded data
        
        Caveats: 
          (1) columns must match the list of servos for which the 
              PoseRecorder was configured.
          (2) the first (header) row of the file is ignored
    
        Example: load froma literal multiline string
        >>> pr.appendFrom('''
        0, 1000, 0, 1000
        1,    0, 1000, 0
        '''.split("\n"))
        """
        if type(stream) is not list:
            stream = stream.readlines()
        t0 = 0 if not self.plan else (dt+self.plan[-1][0])
        for nl,line in enumerate(stream[1:]):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            val = [float(v.strip()) for v in line.split(cdel)]
            if len(val) != len(self.servos)+1:
                raise ValueError("%d: Found %d values instead of %d" 
                   % (nl+1,len(val)-1,len(self.servos)))
            val[0] += t0
            self.plan.append(val)
 
    def loadCSV( self, stream ):
        """Syntactic sugar for loading a .csv file using .appendFrom()"""
        self.reset()
        self.appendFrom(stream)

    def saveCSV( self, stream=STDOUT ):
        """Syntactic sugar for using .show() to write a .csv formatted recording

        INPUT:
          stream -- a file object or a file path. The '.csv' will be added if missing
        
        Typical usage:
          pr.saveCSV('motion.csv')
        """
        if type(stream) is str:
            if not stream.endswith(".csv"):
                stream = stream + ".csv"
            stream = open(stream, "w")
        self.show(stream, delim=", ", fmt="%d", hfmt='"%s"')
        if stream is not STDOUT:
          stream.close()

    def playback( self, period=None, count=1, rate = 0.03 ):
        """Play back the current recording one or more times.

           INPUT:
             period -- float / None -- duration for entire recording
               if period is None, the recording timestamps are used
             count -- integer -- number of times to play
             rate -- float -- delay between commands sent (sec)
            """
        # playback current pose for a given amount of time and with a given period
        if not self.plan:
          raise ValueError("No recording -- .snap() poses first!")
          return
        gait = asarray(self.plan,int)
        gaitfun = interp1d( gait[:,0], gait[:,1:].T ) # Gait interpolater function
        dur = gait[-1,0]-gait[0,0]
        if period is None:
          period = self.plan[-1][0] - self.plan[0][0]
        t0 = now()
        t1 = t0
        try:
          while t1-t0 < period*count:
            t1 = now()
            phi = (t1-t0)/period
            phi %= 1.0
            goal = gaitfun(phi*dur).round()
            print "\rphi: %.2f: " % phi, " ".join([
                "%6d" % g for g in goal
            ])
            self._set_pose( goal )	
            sleep(rate)
        except KeyboardInterrupt:
            self.off()
            raise    

class PoseRecorderCLI( Cmd ):
    """Concrete class PoseRecorderCLI provides a cmd.Cmd based commandline
    interface to a PoseRecorder.

    Start one and give the 'help' command to get more information
    """
    def __init__(self,pr,clust=None):
        """
        INPUTS:
          pr -- PoseRecorder -- PoseRecorder to control
          clust -- Cluster / None -- Cluster; to allow the general
            command interface
        """
        Cmd.__init__(self)
        self.prompt = "PoseRecorder >> "
        assert isinstance(pr,PoseRecorder)
        self.pr = pr
        self.pr_duration = None
        self.pr_count = 1
        self.clust = clust
        if clust is None:
          self.tgt = []
        else:
          self.tgt = clust.values()

    def run(self):
        self.cmdloop()

    def emptyline(self):
        Cmd.emptyline(self)
        self.do_show()

    def do_target(self,line):
        """Specify target for commands
        
        Target may be either * or a space-separate list of module names
        """
        if self.clust is None:
          print "No Cluster specified; 'target' command is not available"
          return
        line = line.strip()
        if line == "*":
          self.tgt = self.clust.values()
        else:
          try:
            self.tgt = [getattr(self.clust.at,nm) for nm in line.split(" ")]
          except AttributeError,ae:
            print "Unknown module:",ae
            return
          
    def do_cmd(self,line):
        """Specify command to broadcast to targets"""
        if self.clust is None:
          print "No Cluster specified; 'cmd' command is not available"
          return
        sp = re_split("\s+",line,1)
        if len(sp)>1:
          cmd,val = sp[0],int(sp[1])
        else:
          cmd,val = sp[0],None
        #
        res = []
        for m in self.tgt:
          try:
            if val is None:
              res.append(getattr(m,cmd)())
            else:
              res.append(getattr(m,cmd)(val))
          except AttributeError,ae:
            print "Module",m.name,"does not support '%s'" % cmd
            res.append(None)
        #
        print " ".join([
            "%6s" % m.name for m in self.clust.itervalues()])
        print " ".join([
            "%6s" % str(v) for v in res])
        
    def do_show(self,line=None):
        """Show the current recording in text form"""
        print "# duration ",self.pr_duration," count ",self.pr_count
        self.pr.show()

    def do_pose(self,line=None):
        """Append the current pose to the recording"""
        self.pr.snap()

    def do_reset(self,line=None):
        """Reset the recording"""
        self.pr.reset()

    def do_off(self,line=None):
        """Emergency stop the recording -- go_slack() all modules"""
        self.pr.off()

    def do_closeLoop(self,line):
        """closeLoop <dt> -- Close a loop in the recording, with delay <dt>
        """
        try:
            f = float(line)
        except ValueError:
            f = 1
        self.pr.snapClosed(f)

    def do_drop(self,line):
        """Drop the last pose from the recording"""
        self.pr.unsnap()

    def do_save(self,line):
        """save<file> Save recording to a .csv file"""
        try:
            self.pr.saveCSV(line)
        except IOError,ioe:
            print ioe
    
    def do_load(self,line):
        """Append recording from a .csv file
           load <file>
        """
        try:
            stream = open(line,"r").readlines()
        except IOError,ioe:
            print ioe
            return
        self.pr.appendFrom(stream)

    def do_count(self,line):
         """Set number of times to loop"""
         try:
            self.pr_count = int(line)
         except ValueError,ve:
            print ve

    def do_duration(self,line):
         """Set duration of one playback period (empty for automatic)"""
         if not line:
             self.pr_duration = None
         else:
             try:
                 self.pr_duration = float(line)
             except ValueError, ve:
                 print ve

    def do_run(self,line=None):
         """Run the recording. Set duration and cycle count with 'duration' and 'count' commands"""
         try:
             self.pr.playback(self.pr_duration,self.pr_count)
         except KeyboardInterrupt:
             self.pr.off()

    def do_inline(self,line=None):
         "Show inline code for recording"
         STDOUT.write(".appendFrom('''")
         self.pr.show(delim=",",rdel="; ")
         STDOUT.write("'''.split(';'))\n")

    def do_EOF(self,line=None):
        return self.do_quit()

    def do_quit(self,line=None):
        "quit"
        return True
     
    def do_exit(self,line=None):
        "quit"
        return True
        
def recordFromCluster( c ):
    """Interactively prompt user and record poses from all modules of a cluster
    INPUT:
      c -- Cluster to record from
    OUTPUT: PoseRecorder with the recording
    """
    p = PoseRecorder(c.values())
    while True:
        p.reset()
        while True:
            if raw_input("<Enter> to store <q><Enter> when done: ") is 'q':
                break
            p.snap()
            p.show()
        if raw_input("Have successfuly recorded? (y/n): ") is 'y':
            break    	  
    return p
