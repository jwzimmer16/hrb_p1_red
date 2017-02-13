#!/usr/bin/env python
"""
The Dynamixel Configuration tool is used to configure dynamixel servos to
preset values. It also serves as a tool too locate dynamixel servos and 
change their IDs

-- Typical usage is as follows:
   # Change servo at ID 0x01 to ID 0x05 and use paramters from rx64-servo.yml
   >>> ./dynamixel-configure.py -c 0x01 -n 0x05 rx64-servo.yml 

   ... more examples ...
"""
from sys import argv, exit as sys_exit
from getopt import getopt, GetoptError
from yaml import load, dump
from base64 import decodestring
from struct import unpack
from time import time as now, sleep
from ckbot.ckmodule import progress
import ckbot.dynamixel as dynamixel

DEBUG = []

class dynamixelConfigurator:
    """
    SOME USEFUL DESCRIPTION
    """
    
    def __init__( self, argv, cur_nid=0x01, cfg=None ):
        """
        Initialize the dynamixelConfigurator, the configurator reads in 
        arguments and sets servo paramters accordingly 
        """
        self.DEBUG = []
        self.argv = argv
        self.cur_nid = cur_nid
        self.new_nid = None
        self.cfg = cfg        
        self.p = None
        self.withReset = False
        self.baud = None
        
    def usage( self ):
        print """
              Usage:
                -h, --help : Returns usage 
                -s, --scan : Scan to find all servos and baudrates on the bus 
                -c, --cur  : Desired servo ID to configure
                -n, --new  : New servo ID
                -f, --yaml : .yml file to read parameters from
                -Q, --opaque : configure from an opaque configuration string
                -R, --reset : Start by sending a reset command
                -d, --debug : Set debug flag
                -p, --port : set an alternative port for the bus 
                              (see port2port.newConnection)
              """
        return 

    def parseArgs( self, argv ):
        """
        Parse input arguments, handle if sensible 
        """
        if len(argv) < 1:
            self.usage()
            progress('No arguments passed, see Usage \n')
            return False

        try:
          opts, args = getopt( argv, "Rhb:n:c:f:d:sp:Q:", ["help", "new:", "cur:", "yam:l", "debug", "scan", "baud:", "port:","reset","opaque:"])
            
        except GetoptError:
            progress( "Invalid Arguments" )
            self.usage()
            return False

        scan = False
        self.opaque = False
        for opt, arg in opts:
            if opt in ("-h", "--help"):
                self.usage()
                sys_exit()
            elif opt in ( "-n", "--new"):
                self.new_nid = int( arg, 0 )
            elif opt in ("-c", "--cur"):
                self.cur_nid = int( arg, 0 )
            elif opt in ("-b", "--baud"):
                self.baud = int(arg)
                progress("No scan, fixed baudrate: %d " % self.baud)
            elif opt in ("-f", "--yaml"):    
                progress("Loading yaml configuration: %s" % arg )
                self.cfg = load( open( arg ))
            elif opt in ("-Q", "--opaque"):
                progress("Configuring from opaque string: %s" % repr(arg))
                self.cfg = load( decodestring( arg.replace("+","\n") ))
                self.opaque = True
            elif opt in ("-s", "--scan"):
                scan = True
            elif opt in ("-R", "--reset"):
                self.withReset = True  
            elif opt in ("-p", "--port"):
                self.p = dynamixel.Protocol(dynamixel.Bus(port=arg))
        return scan

    def _defaultPort( self ):
        self.p = dynamixel.Protocol()
          
    def scanAll( self ):
        """ 
        A utility function that will scan all baudrates and return all
        nodes found. 
        Exits upon completion
        """
        if self.p is None:
          self._defaultPort()
        nid_str = []
        for baud in self._baudPlan():
            progress("Testing baudrate %d" % baud)
            self.p.bus.reconnect(baudrate = baud)
            self.p.reset()
            for nid in self.p.pnas.keys():
                tpe = self.read_type(nid)
                nid_str.append(' ID 0x%02x: %s Module found at baudrate %d ' % (nid, tpe, baud))
        if not nid_str:
            progress('--- REPORT -- No modules found' )
            return
        progress( '--- REPORT %s' % ('-'*50))
        for s in nid_str:
            progress(s)

    def _safe_nid( self, nid):
        """
        Check to ensure that we don't try to overwrite another nid
        """
        if self.p.pnas.has_key(nid):
            progress("ID 0x%02x already exists cannot set new node id" % nid)
            return False
        return True

    def write_nid(self, nid_old, nid_new):
        """
        Write old node id to new node id
        """
        if not self._safe_nid( nid_new ):
            return            
        # Update nid if allowed
        if nid_new == 0x01:
            progress("Warning: NID 0x01 is reserved for configuration")
        node = self.p.pnas[nid_old]
        progress("Setting old NID 0x%02x to new NID 0x%02x" % (nid_old, nid_new))
        node.mem_write_sync( getattr( node.mm, 'ID'), nid_new)
        return  

    def write_config( self, nid, opaque = False ):
        """
        Write node parameters as defined by config (cfg)
        
        INPUT:
          nid -- int -- node ID
          opaque -- bool -- if true, configuration is not shown on stdout
        """
        if self.cfg is None:
            progress("No configuration defined")
            return 
        if self.p is None:
          self._defaultPort()
        baud = None
        # For memory address names set those attributes 
        node = self.p.pnas[nid]
        if self.opaque:
          progress('Configuring ID 0x%02x with parameters %s' % (nid, self.cfg.keys()))
        else:
          progress('Configuring ID 0x%02x with parameters %s' % (nid, self.cfg))
        node = self.p.pnas[nid]
        for name, val in self.cfg.iteritems():
            # Check for node id setting 
            if not opaque:
              print name, val
            if name == 'ID':
                print " Setting ID through .yml file not allowed, see --help"
                sys_exit(-7)
            # Check for baudrate change, if so, write baud last 
            if name == 'baud':
                baud = val
                continue
            # This is pulled out of DynamixelModule         
            if hasattr( node.mm, name ):
                if opaque:
                  print "Writing %s" % repr(name)
                else:
                  print "Writing %s to %s" % (repr(name), repr(val))
                node.mem_write_sync( getattr( node.mm, name ), val )
            else:
                raise KeyError("Unknown address '%s'" % name)
        # Write baudrate last 
        if baud is not None:
          print "Writing baud rate ",baud
          node.mem_write_sync( getattr( node.mm, 'baud' ), baud )

    def read_type(self, nid):
        """
        Read the servo type, returned upon scanAll, ... maybe scan?
        """
        if self.p is None:
          self._defaultPort()
        node = self.p.pnas[nid]
        cw_limit = node.mem_read_sync( node.mm.cw_angle_limit) 
        ccw_limit = node.mem_read_sync( node.mm.ccw_angle_limit)
        if cw_limit==0 and ccw_limit==0:
            return 'CR'
        return 'Servo'       

    def _baudPlan( self ):
        """(private) plan order in which baudrates are scanned
        """
        if self.baud:
          return [self.baud]
        bauds = [ b 
            for b in self.p.bus.getSupportedBaudrates()
            if (b>=1200 and b<=1e6 and not b in [57600,115200]) ]
        bauds.sort(reverse=True)
        bauds.insert(0, 57600)
        bauds.insert(0, 115200)
        return bauds
        
    def scan(self, nid, timeout=0.5, retries=1 ):
        """
        Check if a given node id exists on any baudrates,
        returns nid if successful, None if not 
        
        INPUTS:
        p -- object -- protocol object 
        nid -- int -- node id to scan for
        
        """
        if self.p is None:
          self._defaultPort()
        progress("Scanning for 0x%02X" % nid)
        for baud in self._baudPlan():
            self.p.bus.reconnect(baudrate = baud)
            while True:
              try:
                self.p.reset(nodes=[nid], ping_rate=0.1)
                break
              except dynamixel.ProtocolError,pe:
                print str(pe), "retrying..."
            if self.p.pnas.has_key(nid):
                tpe = self.read_type(nid)
                progress(' ID 0x%02x: %s Module found at baudrate %d ' % (nid, tpe, baud))
                return
        progress( 'ID 0x%02x not found' % nid )
        return 

    def reset_nid( self ):
        """
        Reset servo at cur_nid
        """
        if not self._safe_nid(1):
          progress("A node ID 0x01 already exists on the bus. Reset aborted")
          return
        progress("Resetting 0x%02X... will be to 0x01, at 57600 baud")
        self.p.bus.reset_nid( self.cur_nid )
        self.p.bus.reconnect( 57600 )
        self.cur_nid = 1          
        
    def run( self ):
        """
        Read in and parse arguments, perform necessary configuration 
        """
        scan = self.parseArgs( self.argv )
        if scan:
          return self.scanAll()       
        # Find desired nid
        self.scan( self.cur_nid )
        if self.withReset:
          self.reset_nid()
        # Check that new nid is safe 
        if not self._safe_nid( self.new_nid ):
            return 
        # Configure, if requested
        if self.cfg:
          self.write_config( self.cur_nid, self.opaque )
          # After changing configuration we may be at a new baudrate so rescan
          if self.cfg.has_key('baud') and (self.new_nid != self.cur_nid):
            self.scan( self.cur_nid )
        # If nid change requested --> do it
        if self.new_nid:
          self.write_nid( self.cur_nid, self.new_nid )
        return 

if __name__ == "__main__":
    """
    Run the configurator
    """
    dc = dynamixelConfigurator( argv[1:] )
    dc.run()

