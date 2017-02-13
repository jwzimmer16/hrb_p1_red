import curses
import time
import traceback
import sys
from ckbot.can import *

"""
Make sure the linux kernel is matched with the peakcan drivers.

ls -l usr/src to see what linux-> points to. 
"""

class pcv():
    """
    Main PyCANView Class. 
    """

## Constructor
    def __init__(self):
        """
        ATTRIBUTES:
        msgdict -- dict -- dictionary of msg_id: msg attributes
        updated -- list[int] -- list of message IDs recently updated (past second)
        seen -- int -- number of heartbeat messages seen
        """
        self.mode = 0
        self.line = 0
        self.msgdict = {} # id: (id, data, count, period)

        # Modlab-specific, need to reorganize this
        self.updated = 0 # list of msgs updated within the last second
        self.seen = 0

        self.cleanup = 0 # A little housekeeping

        self.scrn = curses.initscr()
        self.scrn.nodelay(1)

        curses.start_color()
        curses.noecho()
        curses.cbreak()

        self.connect()

    def connect(self):
        try:
            self.b = Bus()
            self.scrn.addstr(0,0,"CAN Connected!")
        except BusError:
            self.scrn.addstr(0,0,"No CAN Connection found. Press 'r' to retry, 'q' to quit.")
            self.b = None

    def cleanscrn(self):
        self.cleanup = self.cleanup + 1
        if self.cleanup == 200:
            self.cleanup = 0
            self.scrn.clear()
            self.scrn.refresh()

## Quit Events
    def end(self):
        """
        Exits program safely.
        """
        if self.b:
            self.b.close()
        curses.nocbreak()
        curses.echo()
        curses.endwin()

## Display Modes
    def dispHeader(self, mode_name):
        header_str = "ID\tMSG\t\t\t\t\tCOUNT\tPERIOD\t%s"%mode_name
        seen_str = "Number of Hearbeats Seen: %d"%self.seen

        living = 0
        for Msg in self.msgdict.itervalues():
            if Msg.isLive() and Msg.isHeartbeat():
                living = living + 1;

        living_str = "Number of Heartbeats Living: %d"%living

        self.scrn.addstr(3,0,header_str) # Header
        self.scrn.addstr(4,0,"-"*62) # Make it look prettier with lines...
        self.scrn.addstr(0,0,seen_str) # Seen heartbeats
        self.scrn.addstr(1,0,living_str) # Living heartbeats

        currDispY = 5

        return currDispY # vertical displacement in display (how much space header takes up)

    def dispMsg(self,msg_id):
        """
        Formats string in MSGID DATA COUNT PERIOD order and aligns with defaults
        """
        
        Msg = self.msgdict[msg_id]

        rawdata = Msg.data
        count = Msg.count
        period = Msg.period
        data = ""
        for entry in rawdata:
            data = data + "%02X  "%entry
        
        if Msg.isLive():
            msgstr = "%X*\t%s\t%d\t%.3f"%(int(msg_id),str(data),count,period)
        else:
            msgstr = "%X\t%s\t%d\t%.3f"%(int(msg_id),str(data),count,period)

        return msgstr

    def dispMode0(self):
        """
        Default Display Mode:
        Sorts by Messages Priority-NID order
        """
        i = self.dispHeader("Msg_Priority");

        for msg_id in self.msgdict.iterkeys():
            ## msgid  msg  count  period
            msgstr = self.dispMsg(msg_id)
            self.scrn.addstr(i,0,msgstr)
            i = i + 1

        # Used to clean up the screen occasionally if any display mishaps occur
        #self.cleanscrn()

    def dispMode1(self):
        """
        Sorts by NID order
        """
        i = self.dispHeader("NID")

        msg_ids = self.msgdict.keys()
        msg_ids.sort(self.__sortByNID)
        
        for msg_id in msg_ids:
            msgstr = self.dispMsg(msg_id)
            self.scrn.addstr(i,0,msgstr)
            i = i + 1

        #self.cleanscrn()

    def __sortByNID(self,x,y):
        """Inputs are msg_ids"""
        return int((x&0xFF) - (y&0xFF)) 

## Key Press Events (Modify this)
    def exec_q(self):
        """Exit."""
        self.end()

    def exec_r(self):
        """Reconnect bus"""
        self.msgdict = {}
        self.seen = 0;
        self.scrn.clear()
        self.scrn.refresh()
        self.connect()

    def exec_m(self):
        """Change Display Mode"""
        self.scrn.clear()
        self.scrn.refresh()
        self.mode = (self.mode + 1)%2
        #self.mode = 0

## Default Behaviour (Modify this)
    def default(self):
        if self.b is None:
            return

        msg = self.b.read()

        # Refresh msgdict if new msg found
        if msg is not None:
            msg_id = msg.ID
            msg_data = list(msg.DATA)
            
            # If msg ID has been seen before, increment counter
            if self.msgdict.has_key(msg_id):
                Msg = self.msgdict[msg_id]
                Msg.count = Msg.count + 1
                currTime = time.time()
                Msg.period = currTime - Msg.timestamp
                Msg.timestamp = currTime
            # Otherwise, add it to the list of seen messages
            else:
                count = 0
                period = 0
                timestamp = time.time()
                self.msgdict[msg_id] = CANMsg(msg_id,msg_data,count,period,timestamp)

                if self.msgdict[msg_id].isHeartbeat():
                    self.seen = self.seen + 1; # How many heartbeats have been seen?

        # To reorganize, simply change the mode
        eval('self.dispMode%d()'%self.mode)

## Main loop (Do NOT modify this)
    def mainloop(self):
        """
        This method should NOT be modified,
        instead the default, behaviours, and exec_<string> methods
        should be modified to change behaviour

        NOTE: 'q' is already being used
        """
        while True:
            c = self.scrn.getch()
            
            if c == curses.ERR:
                self.default()
            elif c == 113: # if it's q
                break;
            elif c <= 255 and c >= 0:
                ex = "exec_%s"%chr(c)
                if ex in dir(self):
                    eval("self.%s()"%ex)
            else:
                #self.behaviours(c)
                self.default()

        self.end()

class CANMsg( object ):
    def __init__(self, msg_id, msg_data, count, period, timestamp):
        self.id = msg_id
        self.data = msg_data
        self.count = count
        self.period = period
        self.timestamp = timestamp
    
    def isHeartbeat(self):
        return (self.id >> 8) == 7

    def isLive(self, timeout=1):
        """
        INPUT:
        timeout -- int -- seconds to wait before a msg is considered NOT 'alive'
        """
        currTime = time.time()
        lastUpdated = self.timestamp
        timeDiff = currTime - lastUpdated
        if (timeDiff <= timeout):
            return True
        return False
            

if __name__== '__main__':
    helpmsg = """
usage: python pycan.py

Controls:
    "q"   :   quits the program
    "r"   :   refreshes the program and reconnects CAN
    "m"   :   toggle display order (Default is Msg Priority)

Key:
    "*"   :   If * is displayed by a msg ID, then that message has been updated recently

This module is a Linux terminal implementation of PCANView for Windows.
Required packages include ckbot.can from MODLab's svn repository
"""

    if "-h" in sys.argv or "--help" in sys.argv:
        print helpmsg
    else:
        pycan = None

        try:
            print "Connecting..."
            pycan = pcv()
            pycan.mainloop()
        except StandardError:
            if pycan is not None:
                pycan.end()
            traceback.print_exc()
