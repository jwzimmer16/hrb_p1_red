"""
File: wixelTDMA

Implement TDMA interface via wixel boards
"""

from numpy import zeros
from copy import deepcopy
from multiprotocol import MultiProtocol
from port2port import Connection, newConnection
from struct import pack, unpack

DEFAULT_WIXEL_PORT = dict(
  TYPE='tty',
  glob='/dev/ttyACM*',
  baudrate=115200
  )

def generate_timeplan(tasks):
    #Eventually, I'll build up the list "tasks" to be of the format [[node, period, 
    #duration, start]]
    tasks = deepcopy(list(tasks))
    
    tasks[0].append(0) #Assigning first task to first time slot of 1/32 duration
    slotNum = 0 # Start counting from slot number 0

    # Nested for loop to assign starting slot numbers to all the nodes
    for i in xrange(1, len(tasks)): # cycle through entire list, to assign starting 
        #slot values for each node

        taskSlotAssigned = False # Boolean value to denote if node has been assigned 
        #a starting slot

        while not taskSlotAssigned: # while starting slot number for current node 
            #isn't determined yet, do the following:

            slotBusy = False # Boolean value denoting if slot 'slotNum' has already been
            #assigned to an earlier node

            for j in xrange(0,i): # Cycle through previous nodes

                if ((slotNum - tasks[j][3]) % tasks[j][1]) < tasks[j][2]: # Check if slot
                    #'slotNum' is occupied by this earlier node, i.e. slotNum is within 
                    #'slotWidth' distance of any of the earlier nodes

                    slotBusy = True # if yes, then update 'slotBusy'
                    newSlotNum = slotNum + tasks[j][2] - (slotNum - tasks[j][3])%tasks[j][1]
                    # update slotNum to (slotNum + {slotWidth of node} - (slotNum - {starting slot of node})%slotPeriod); 
                    #this moves the slotNum to the next slot number immediately following this node's transmission slots
                    #print tasks[j]
                    # print '\tslotNum {0} is occupied by {1}. Setting slotNum to {2}'.format(slotNum, tasks[j][0], newSlotNum)
                    slotNum = newSlotNum


            if not slotBusy: # if, after going through all previous nodes, we find that 
                #the slot hasn't been assigned to any node yet, assign it to be the starting slot of this node.

                taskSlotAssigned = True
                tasks[i].append(slotNum)
                # print 'ASSIGNED: slot {0} to {1}'.format(slotNum, tasks[i][0])
                break


    # Populate 1 'period' of the timeplan; i.e., the minimum duration which can be 
    #repeated to obtain the entire timeplan for this frame

    # That is, if we have the following slot assignments: [a,b,b,c,a,b,b,c,a,b,b,c...], then one period = [a,b,b,c]
    periodDuration = 0
    for i in xrange(0,len(tasks)):
        if tasks[i][1] > periodDuration: # The periodicity of a sum of signals, which 
            #are a multiple of some common factor, is the periodicity of the lowest frequency signal
            periodDuration = tasks[i][1]

    period = zeros([1,periodDuration]).tolist()[0]


    # Populate the slots in this 'period' with the node names.
    for i in xrange(0,len(tasks)):
        for j in xrange(tasks[i][3], periodDuration, tasks[i][1]):
            for k in xrange(0, tasks[i][2]):
                if (period[j+k]):
                    print '\nCollision!'
                    period[j+k] = 'xxx'
                period[j+k] = tasks[i][0]

    pkt = [255] # 0xFF
    # To construct the packet to send across to the hub
    for task in tasks:
        start = task[3]
        duration = task[2]
        timePeriod = task[1]
        pkt.extend([start,duration,timePeriod])

    return pkt
    
class NodeConnection( Connection ):
  """
  Concrete Connction subclass specialized to send and recieve dat via
  a specific TMDA node
  
  ALGORITHM:
    write -- prepend prefix and send via hub
    read -- pull from message queue
  """
  def __init__(self, hubwrite, hubisopen ):
    self.isOpen = hubisopen
    self.write = hubwrite
    self.q = []
    
  def open( self ):
    """Make the connection active"""
    pass

  def enqueue( self, msg ):
    self.q.append(msg)
    
  def read( self, length ):
    if not self.q:
      return None
    return self.q.pop(0)[:length]
  
  def close( self ):
    """Disconnect; further traffic may raise an exception
    """
    pass

  def reconnect( self, **changes ):
    """Try to reconnect with some configuration changes"""
    raise IOError('Cannot reconnect TDMA Nodes')
  
class Hub( MultiProtocol ):
  """
  Concrete MultiProtocol subclass for Wixel TDMA connections
  
  TYPICAL USAGE:
    H = Hub()
    P1 = Dynamixel.Protocol( Dynamixel.Bus( 
        port = H.getNewNodeConnection( tid1 )
        ))
    H.addSubProtocol( P1 )
    P2 = Dynamixel.Protocol( Dynamixel.Bus( 
        port = H.getNewNodeConnection( tid2 )
        ))
    H.addSubProtocol( P2 )
    # start loop with H.update() called
  """
  def __init__(self, wixelPort = None ):
    """
    INPUT:
    """
    MultiProtocol.__init__(self,*argv,**kw)
    # Make sure the protocols are initialized right w.r.t to nodes
    
    # Initialize connection to the hub wixel
    if wixelPort is None:
      wixelPort = newConnection(DEFAULT_WIXEL_PORT)
    elif type(wixelPort) in [dict,str]:
      wixelPort = newConnection(wixelPort)
    elif assert isinstance(wixelPort,Connection)
    self.wix = wixelPort
    # Collection of TDMA nodes
    self.tdn = {}
    # Timeplan 
    self.tp = []

  def getNewNodeConnection( self, tid ):
    """
    INPUT:
       tid -- TDMA node ID
    OUTPUT:
       a NodeConnection instance that sends data via that node
       This is registered in self.tdn
    """
    assert not self.tdn.has_key(tid), "TMDA node ID can only be initialized once"
    pfx = pack('B',tid) 
    nc = NodeConnection(
      hubwrite = lambda msg : self.wix.write( pfx + msg ),
      hubisopen = self.wix.isOpen
    )
    self.tdn[tid] = nc
    return nc
    
  def _demuxIncoming( self ):
    """
    (private) scan incoming messages and demultiplex them into
    the respective NodeConnection queues
    """
    """
    Using a buffer, reassemble messages from the wixel
    once a complete message is received, find which tdn it belongs 
    to and queue it there. If an unknown tdn, record as error
    also detect any bad framing of data from wixel
    (note: wixel receives framed messages from HW)
    """
    pass
  
  def update( self ):
      """Update all sub-protocols and collect heartbeats"""
      hb = {}
      # Loop over all protocols contained in this one
      for p in self.nim.iterowners():
          p.update()
          # Collect heartbeats from the protocols
          for inid,val in p.heartbeats.iteritems():
              xnid = self.nim.mapI2X(p,inid)
              hb[xnid] = val
      self.heartbeats = hb
      """!!!
      if enough time has passed, self._emitTimeplan()
      """
      return self._demuxIncoming()

  def _emitTimeplan( self ):
    """(private) emit timeplan, generating a default one of needed"""
    """
    if self.tp is empty, give a generic slot to each node
    otherwise use the self.tp to generate timeplan packet(s)
    and send them
    """
    pass
  
  
  
