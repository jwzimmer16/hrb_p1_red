"""
FILE: multiprotocol.py defines the MultiProtocol class, a class that maps 
multiple Protocol instances under a single umbrella, allowing multiple robot
communication busses and protocols to be used from a single user application
"""

from ckmodule import AbstractProtocol, AbstractBus

class NidMapper( object ):
    """class NidMapper implements a mapping between "external" and "internal"
    node ID values. The internal node ID values are associated with owners,
    which are typically Protocol instances.
    
    IMPLEMENTATION:
      
      The current implementation is silly -- it gives each owner an address
      space of a given number of bits (default 8). External node ID values
      consist of owner ID bits followed by node ID bits
    """
    
    def __init__( self, bits=8 ):
        self.tbl = {}
        self.itbl = {}
        self.bitShift = bits
        self.mask = (1L<<bits)-1
    
    def iterowners( self ):
        """Iterate over all the node owners"""
        return self.tbl.itervalues()
        
    def addOwner( self, owner, oid=None ):
        """
        Add a new node owner
        
        INPUT:
            owner -- hashable -- "owner" of a node range; must be hashable
            oid -- int -- owner ID to use, or None to have one generated
              If an owner ID is re-used, an exception will be raised
        """
        if oid is None:
            oid = len(self.tbl)+1
        else:
            oid = int(oid)
        if self.tbl.has_key(oid):
            raise KeyError("Owner ID %d re-uses existing ID" % oid )
        self.tbl[oid] = owner
        self.itbl[hash(owner)] = oid
    
    def mapX2I( self, nid ):
        """Map external node ID into an owner and internal node ID"""
        nid = int(nid)
        oid = nid >> self.bitShift
        # This looks for the owner; will fail with KeyError if owner ID unknown
        owner = self.tbl[oid]
        inid = (nid & self.mask)
        return owner, inid
    
    def mapI2X( self, owner, nid ):
        """Map owner and node ID into external node ID"""
        nid = int(nid)
        oh = hash(owner)
        assert nid == (nid & self.mask), "Node ID is within address range"
        return (self.itbl[oh]<<self.bitShift) | nid
        
class MultiProtocol( AbstractProtocol ):
    def __init__(self,*subs):
        """
        Create a protocol which multiplexes multiple sub-protocols
        
        For convenience, these sub-protocols can be listed sequentially
        in the constructor
        """
        AbstractProtocol.__init__(self)
        self.nim = NidMapper()
        if subs:
            if isinstance(subs[0], AbstractBus):
                raise ValueError("MultiProtocol() cannot accept a Bus parameter")
            self.addSubProtocols(*subs)
    
    def addSubProtocols(self, *subs):
        """
        Add one or more sub-protocols
        """
        for sub in subs:
            self.nim.addOwner( sub )

    def off( self ):
        """Broadcast the p.off() call to any supporting subs"""
        for o in self.nim.iterowners():
            if hasattr(o,'off') and callable(o.off):
                o.off()
                
    def reset( self, *argv, **kwarg ):
        """Broadcast the p.reset() call to any supporting subs"""
        for o in self.nim.iterowners():
            if hasattr(o,'reset') and callable(o.reset):
                o.reset(*argv, **kwarg)
    
    def scan( self, *argv, **kwarg ):
        """
        Broadcast the p.scan() call to any supporting subs, then
        collect results and map to external NIDs
        """
        res = []
        for p in self.nim.iterowners():
            if not hasattr(p,'scan') or not callable(p.scan):
                continue
            s = p.scan(*argv, **kwarg)
            res.extend([ self.nim.mapI2X(p,inid) for inid in s])
        return res    
       
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
    
    def hintNodes( self, nodes ):
        """Hint the existence of specified NIDs"""
        tbl = {}
        # Collect nodes by owner
        for xnid in nodes:
            o,inid = self.nim.mapX2I(xnid)
            if tbl.has_key(o):
                tbl[o].append(inid)
            else:
                tbl[o] = [inid]
        # Hint the relevant nodes to each sub-protocol
        for p,nids in tbl.iteritems():
            p.hintNodes(nids)
            
    def generatePNA( self, nid ):
        """
        Generate a ProtocolNodeAdaptor for the specified nid, using the
        protocol which owns it
        """
        p,inid = self.nim.mapX2I(nid)
        return p.generatePNA(inid)
  
if __name__=="__main__":
    # Unit test
    import nobus as NB
    from logical import Cluster
    p1 = NB.Protocol()
    p2 = NB.Protocol()
    NIDS = [0x101, 0x102, 0x107, 0x202, 0x203]
    mp = MultiProtocol(p1,p2)
    c = Cluster( 
              arch=NB,count=len(NIDS),fillMissing=True, required=NIDS
    )
    
