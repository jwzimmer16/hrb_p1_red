#!/usr/bin/env python

# Standard libraries
import sys
import os
import time
from time import time as now
import glob
from traceback import print_stack
# Extra libraries
import wx
import yaml

# Dependencies on ModLab code
from logical import Cluster, DelayedPermissionError
import logical

class Misc( object ):
  def __init__(self,**kw):
    self.__dict__.update( kw )

class ODTable(object):
    """
    The ODTable (ObjectDictionaryTable) is a concrete class that provides the functionality 
    needed to store ODObjects (ObjectDictionaryObjects) in a dictionary format
    """    
    def __init__(self):
      """
      ATTRIBUTES:
        tbl -- Dictionary of clp-->ODO
        idx -- List of clp in display order
        protocol -- CAN protocol object (cached)
      """
      self.c = Cluster()
      self.clear()

    def clear( self ):
      self.tbl = {}
      self.idx = []
      
    def refresh(self): #Need to handle situation where module comes back to life
      change = False
      live = self.c.getLive()
      for odo in self.tbl.itervalues():
        if odo.alive != (odo.mod.node_id in live):
          odo.alive = not odo.alive
          change = True
      return change
            
    def find(self, clp):
      """
      Find odo if it exists, or dynamically create a new one if not
      """
      return self.tbl.get(clp,None)
      
    def add(self, clp):
      res = ODObject()
      res.fromClp( self.c, clp )
      self.tbl[clp]=res
      self.idx.append(clp)
      return res

    def scan(self,nid=None):
      if nid:
        self.c.populate(required=set([nid]),walk=1)
      else:
        self.c.populate(walk=1)
      changed = False
      for clp in self.c.iterprop():
        if self.tbl.has_key(clp):
          continue
        self.add(clp)
        changed = True
      return changed
      
    def save(self, filename):
        """
        Saves a list representation of the ODTable
        """
        # open the save file
        try:
          f = open( filename,"w")
          yaml.dump( self.idx, stream=f )
          f.close()
        except IOError, ioe:
          #!!!
          raise
        #print "ODTable saved at %s" % filename

    def load(self, filename):
        """
        Loads an ODTable from a file or stream

        INPUT:
          stream -- str -- filepath that contains a YAML representation of ODTable

        OUTPUT:
          True on success, False otherwise
        """
        try:
          f = open(filename, "r")
          lst = yaml.safe_load(f)
          f.close()
        except IOError,ioe:
          #!!!
          raise ioe
        except yaml.YAMLError, yerr:
          #!!!
          raise yerr
        
        if type(lst) != list:
          raise TypeError,"ODOTable document must contain a list of nodes"
        #print "LOAD", lst
        od_tbl = {}
        bad = []
        for clp in lst:
          try:
            odo = ODObject()
            odo.fromClp( self.c, clp )
            od_tbl[clp] = odo
          except AttributeError:
            bad.append(clp)
            progress("Property '%s' not found. Did you scan bus?" % clp)
        # Remove any "bad" properties found
        for clp in bad:
          lst.remove(clp)
        self.tbl = od_tbl
        self.idx = lst
        return True
        
    def genDisplayList( self ):
        """
        Generates a list representation of the ODObjects in display order
        
        OUTPUT: a generator of odo objects
        """
        for nm in self.idx:
          yield self.tbl[nm]
          
    def genView(self):
        """
        Generates a list representation of the ODObjects to be used by GUI's
        
        OUTPUT: a generator of tuples (group,capability)
          group is string rep of node ID
          capability is odo.getName
        """
        for nm in self.idx:
          yield ("node %s" % Cluster.parseClp(nm)[1],nm)

    @staticmethod
    def isNodeText( txt ):
      "Check if text is a node caption"
      return len(txt)>4 and txt[:4]=="node"
                
class ODObject(object):
    """
    Concrete class used to represent settable/gettable properties of a Cluster    
    """
    def __init__(self):
        """
        ATTRIBUTES:
           clp -- str -- cluster property name
           set_func -- python function -- set function
           get_func -- python function -- get function
        """
        self.clp = ''
        self.mod = None
        self.perm = ''
        self.set_func = None
        self.get_func = None
        self.alive = False

    def fromClp( self, clust, clp ):
        self.clp = clp
        perm = ''
        self.mod = clust.modOfClp(clp)
        g = clust.getterOf(clp)
        if not isinstance(g,DelayedPermissionError):
          perm = perm + "R"
        else:
          g = None
        s = clust.setterOf(clp)
        if not isinstance(s,DelayedPermissionError):
          perm = perm + "W"
        else:
          s = None
        self.get_func = g
        self.set_func = s
        self.perm = perm
        self.alive = True
        
    def set(self, value):
        if self.alive and self.set_func:
          self.set_func(value)
          return True
        return False

    def get(self):
        if self.alive and self.get_func:
          return self.get_func()
        return None
        
    def setDead(self):
        print_stack()
        self.alive = False

    def getName( self ):
      return self.clp
      
    @staticmethod
    def byName( odt, nm ):
      return odt.tbl[nm]
      
class GaitCol(object):
  def __init__(self, odo):
    assert isinstance( odo, ODObject ),repr(odo)
    self.odo = odo
    self.value = None

  def changeODO(self, odo):
    assert isinstance( odo, ODObject ),repr(odo)
    self.odo = odo

  def getHeading(self):
    return self.odo.getName()

  def getValue(self):
    val = self.odo.get()
    if val is None:
      return False      
    self.value = val
    return val   
    
  def setValue(self, value):
    return self.odo.set(value)

  def saver( self ):
    return self.odo.getName()

  @staticmethod
  def loader( odt, key ):
    odo = odt.find(key)
    if odo is None:
      odo = odt.add(key)
    return GaitCol( odo )

class GaitTable(object):
  def __init__(self):
    self.cols = []
    
  def iterCols( self ):
    return iter(self.cols)

  def addGC( self, gc ):
    assert isinstance(gc,GaitCol)
    self.cols.append(gc)
    
  def delGC( self, gc ):
    assert isinstance(gc,GaitCol)
    self.cols.remove(gc)
    
  def save(self, fn ):
    #print "!!!GT.save",repr(self.cols)
    sobj = [ col.saver() for col in self.iterCols() ] 
    fi = None
    try:
      fi = open(fn, "w")
      yaml.dump( sobj, fi )
    except IOError,ioe:
      print "ERROR writing '%s':" % fn, ioe
    if fi:
      fi.close()
          
  def load(self, filename, odt ):
    try:
      fi = open(filename, "r")
      sobj = yaml.safe_load( fi )
    except yaml.YAMLError, yerr:
      #!!! report error in GUI
      print "ERROR in YAML file '%s' :" % filename, yerr
      return yerr
    except IOError,ioe:
      #!!! report error in GUI
      print "ERROR reading file '%s' :" % filename, ioe
      return ioe
    cols = [ GaitCol.loader( odt, svs ) for svs in sobj ] 
    self.cols = cols
    return None
    
# ---------------------------- VIEW CLASSES ----------------------

class GaitTableFrame(wx.Frame):
    def __init__(self, cmds ):
      size = (610,290)
      wx.Frame.__init__(self, None, -1, title="CKBot GUI", size=size)

      ##Attributes
      self.gt_panel = GaitTablePanel(self, cmds)
      self._buildMenu(cmds)
      
      self.timer = wx.Timer(self)
      self.timer.Start(50, oneShot=False)
      self.Bind(wx.EVT_TIMER, self.OnTimer)
      self.lastTic = now()
      self.tic = 0.2 #seconds
      self.cmd_timerTic = cmds.timerTic

      self.CreateStatusBar()
      self.SetMinSize(size)

    AUTO_ID = 100
    def _newMenu(self,items):
      menu = wx.Menu()
      for item in items:
        ID = None
        if type(item)==str and item[0]=='-':
          menu.AppendSeparator()
          continue          
        if len(item)==2:
          cmd, txt= item
        elif len(item)==3:
          cmd, txt, ID = item
        else:
          raise TypeError,"Expected 3-tuple or 2-tuple"
        assert callable(cmd)
        cap,help = [ x.strip() for x in txt.split('/?/') ]
        if ID is None:
          ID = GaitTableFrame.AUTO_ID
          GaitTableFrame.AUTO_ID += 1       
        menu.AppendItem( wx.MenuItem( menu, ID, cap, help ))
        self.Bind(wx.EVT_MENU, cmd, id=ID)
      return menu
        
    def _buildMenu( self, cmds ):
        menubar = wx.MenuBar()
        menubar.Append(self._newMenu([
          (cmds.prefs,      '&Preferences\tCtrl+P           /?/ View Preferences Dialog'),
          (cmds.close,      'E&xit                          /?/ Exist program',       wx.ID_EXIT)
        ]),'&File')
        menubar.Append(self._newMenu([
          (cmds.saveGT,     '&Save Gait\tCtrl+S             /?/ Save Gait Table',     wx.ID_SAVE),
          (cmds.saveAsGT,   'Save Gait As...\tCtrl+Shift+S  /?/ Save Gait Table As...'),
          (cmds.loadGT,     '&Open Gait\tCtrl+O             /?/ Open Gait Table',     wx.ID_OPEN)
        ]),'&Gait')
        menubar.Append(self._newMenu([
          (cmds.saveODT,    '&Save ODT\tCtrl+D              /?/ Save Object Dictionary Table'),
          (cmds.saveAsODT,  'Save ODT &As...\tCtrl+Shift+D  /?/ Save Object Dictionary Table As...'),
          (cmds.loadODT,    '&Load ODT\tCtrl+L              /?/ Load ODTable'),
          '-----',
          (cmds.addGC,      'Add &Column\tCtrl+A            /?/ Add Gait Column'),
          (cmds.scan,       'Scan &Bus\tF1                  /?/ Scan CAN Bus for modules'),
          (cmds.clear,      '&Clear Dictionary\tCtrl+R      /?/ Remove all dictionary entries'),
          (cmds.viewTree,   'View &Tree\tCtrl+T             /?/ View Object Dictionary List')
        ]),'&Dictionary')
        menubar.Append(self._newMenu(['----']), '&Help')
        self.SetMenuBar(menubar)

    def browse(self, type, title, filename, filetype = "*.*"):
        dlg = wx.FileDialog(self, title, os.getcwd(), filename, filetype, type)
        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()
            return os.path.basename(path)
        return None

    def browseLoadODT( self, fn ):
      return self.browse(wx.OPEN, "Select Object Dictionary to load",fn,"*.odo")

    def browseLoadGT( self, fn ):
      return self.browse(wx.OPEN, "Select Gait Table to load",fn,"*.sgt")
      
    def browseSaveODT( self, fn ):
      return self.browse(wx.SAVE, "Select Object Dictionary to save",fn,"*.odo")

    def browseSaveGT( self, fn ):
      return self.browse(wx.SAVE, "Select Gait Table to save",fn,"*.sgt")

    def updateGCs( self, gcs ):
      self.gt_panel.setPanelsFor( gcs )
      
    def OnTimer(self, evt):
      dt = now() - self.lastTic
      if dt > self.tic:
        self.cmd_timerTic()
        self.lastTic = now()


class GaitTablePanel(wx.Panel):

    def __init__(self, parent, cmds ):
      wx.Panel.__init__(self, parent)
      
      #Attributes
      self.parent = parent
      self.gcps = []
      # Commands to be inherited by all GaitColPanels we create
      self.cmds = Misc(
        setGC = cmds.setGC,
        delGC = cmds.delGC,
      )
      
      #Static Boxes
      self.cmdSBoxS = wx.StaticBox(self, -1, 'Commands')
      self.cmdSBox = wx.StaticBoxSizer(self.cmdSBoxS)
      self.gtSBoxS = wx.StaticBox(self, -1, 'Gait Table', size=(600,160))
      self.gtSBox = wx.StaticBoxSizer(self.gtSBoxS)

      #Buttons
      self.addColBtn = wx.Button(self, label="+")
      self.addColBtn.Bind(wx.EVT_BUTTON, cmds.addGC)
      self.runGaitBtn = wx.Button(self, label="Run Gait")
      self.runGaitBtn.Bind(wx.EVT_BUTTON, cmds.runGait )
      self.runGaitBtn.SetBackgroundColour("Red")

      #BoxSizers
      self.vBox = wx.BoxSizer(wx.VERTICAL)
      self.cmdBox = wx.BoxSizer(wx.HORIZONTAL)
      self.gcBox = wx.BoxSizer(wx.HORIZONTAL)

      self.cmdBox.Add(self.addColBtn)
      self.cmdBox.Add(self.runGaitBtn)
      self.cmdSBox.Add(self.cmdBox)
      self.gtSBox.Add(self.gcBox)
      self.vBox.AddMany([self.cmdSBox,self.gtSBox])
      
      self.SetSizer(self.vBox)
      
      #Miscellanious 
      self.SetMinSize(self.vBox.GetMinSize())
        
    def setPanelsFor( self, gcs ):
      gcs = list(gcs)
      # While we have more panels than needed
      while len(gcs)<len(self.gcps):
        gcp = self.gcps.pop()
        gcp.Destroy()
      # If we need more panels --> add them
      if len(gcs)>len(self.gcps):
        extra = gcs[len(self.gcps):]
        gcs = gcs[:len(self.gcps)]
        while extra:
          self.newCol(extra.pop(0))
      # Overwrite contents of matching panels
      assert len(gcs) <= len(self.gcps)
      for pnl,gc in zip( self.gcps, gcs ):
        pnl.setGC(gc)
      # do panel updates        
      self.vBox.RecalcSizes()
      self.gtSBox.RecalcSizes()
      self.Update()
      
    def OnAddCol(self, evt):
      #!!!
      raise RuntimeError()
      gc_pnl = self.newCol()
      gc_pnl.OnSelectODO(evt)        

    def OnRunGait(self, evt):
        pass

    def newCol( self, gc ):
      gc_pnl = GaitColPanel(self, self.cmds)
      self.gcps.append(gc_pnl)
      self.gcBox.Add(gc_pnl)
      self.gtSBox.RecalcSizes()
      gc_pnl.setGC( gc )
      return gc_pnl

    def OnPreferenceDialog(self, evt):
        self.preferenceDlg.Show()

    def UpdateGCStatus(self):
        for gcp in self.gcps:
            gcp.updateODO()

    def UpdateGCIndex(self):
        for ind in xrange(len(self.gcps)):
            gcp = self.gcps[ind]
            if gcp.gc:
                gcp.gc.ind = ind

    def Clear(self):
        for gcp in self.gcps:
            ind = self.gcps.index(gcp)
            gcp = self.gcps.pop(ind)
            if gcp:
                gcp.Destroy()
            self.gcps.insert(ind,None)

        self.gcps = []

        self.vBox.RecalcSizes()

    def RemoveGCP(self, gcp):
        if gcp.gc:
            gc = gcp.gc
        else:
            gc = None
        if gc:
            self.gait_tbl.rmCol(gc)
        ind = self.gcps.index(gcp)
        self.gcps.pop(ind)
        self.gcps.insert(ind,None)
        gcp.Destroy()

        self.vBox.RecalcSizes()

    def Message(self, txt, type):
        #print "%s: %s"%(type,txt)

        if type == 'Error':
            dlg = wx.MessageDialog(None, txt, type, wx.OK|wx.ICON_ERROR)
        elif type == 'Warning':
            dlg = wx.MessageDialog(None, txt, type, wx.OK|wx.ICON_EXCLAMATION)
        else:
            dlg = wx.MessageDialog(None, txt, 'Question', wx.OK|wx.ICON_QUESTION)
        dlg.ShowModal()

class GaitColPanel(wx.Panel):
    def __init__(self, parent, cmds ):
        wx.Panel.__init__(self, parent, size=(150,50))
        #Attributes
        self.parent = parent
        self.gc = None 
        self.focus = False
        self.living = False #!!! lint?
        SIZE = (100,32)
        self.cmd_setGC = cmds.setGC
        self.cmd_delGC = cmds.delGC
        self.selODOBtn = wx.Button(self, label="None", size=SIZE)
        self.selODOBtn.Bind(wx.EVT_BUTTON, self.OnSelectODO)

        self.valueTxt = wx.TextCtrl(self, style=wx.TE_PROCESS_ENTER, size=SIZE)
        self.valueTxt.Bind(wx.EVT_KILL_FOCUS, self.OnKillFocus)
        self.valueTxt.Bind(wx.EVT_SET_FOCUS, self.OnFocus)
        self.valueTxt.Bind(wx.EVT_TEXT_ENTER, self.OnSet)
        self.valueTxt.Disable()

        self.setBtn = wx.Button(self, label="Set", size=SIZE)
        self.setBtn.Bind(wx.EVT_BUTTON, self.OnSet)
        self.setBtn.Disable()

        self.removeBtn = wx.Button(self, label="-", size=SIZE)
        self.removeBtn.Bind(wx.EVT_BUTTON, self.OnRemove)

        #Sizer
        self.vBox = wx.BoxSizer(wx.VERTICAL)
        self.vBox.Add(self.selODOBtn)
        self.vBox.Add(self.valueTxt)
        self.vBox.Add(self.setBtn)
        self.vBox.Add(self.removeBtn)

        self.SetSizer(self.vBox)

    def setGC( self, gc ):
      assert isinstance( gc, GaitCol ),repr(gc)
      self.gc = gc
      self.updateODO()
      
    def OnSelectODO(self, evt):
      self.cmd_setGC( self )
    
    def OnSet(self, evt):
      try:
        val = int(self.valueTxt.GetValue())
      except ValueError:
        return False
      except TypeError:
        return False
      return self.gc.setValue(val)

    def OnRemove(self, evt):
      self.cmd_delGC( self.gc )

    def OnFocus(self, evt):
        self.focus = True

    def OnKillFocus(self, evt):
        self.focus = False

    def Remove(self):
        self.parent.gait_tbl.rmCol(self.gc)
        
    def updateODO(self):
      odo = self.gc.odo
      self.selODOBtn.SetLabel(odo.getName())
      print "Setter ",odo.set_func,"Getter ",odo.get_func, odo
      if odo.set_func:
        self.setBtn.Enable()
      else:
        self.setBtn.Disable()
      if odo.get_func:
        self.valueTxt.Enable()
        self.Get()
      else:
        self.valueTxt.Disable()
      self.living = odo.get_func or odo.set_func

    def Get(self):
      if self.focus:
        return        
      value = self.gc.getValue()
      self.valueTxt.SetValue(repr(value))

class GTPPreferences( object ):
  def __init__(self, pref=None):
    if pref is None:
      pref = {
        "refresh-freq": 100,
        "update-ODT-freq": 0,
        "save-ODT-freq": 0,
        "load-GT-freq": 0,
        "save-GT-freq": 0,
        "read-log": False,
        "read-log-filename": "example.log"
      }
    self.pref = pref

  def update(self, pref):
    self.pref.update( pref )
    
  def __getitem__(self, item):
    return self.pref[item]
    
  def __setitem__(self, item, value):
    self.pref[item] = value

  def save(self):
    try:
      f = open("ckbot_gui.pref","w")
      yaml.dump(self.pref)
      f.close()
    except IOError, ioe:
      #!!! replace with real error handling
      raise

  def load( self ):
    """
    Load preference changes from file
    """
    fn = "ckbot_gui.pref"
    if not glob.glob(fn):
      prefs = None
    else:
      try:
        f = open(fn,"r")
        prefs = yaml.safe_load(f)
        f.close()
      except IOError, ioe:
        #!!! replace with real error handling
        raise
      except yaml.YAMLError, yerr:
        raise
    if prefs is not None:
      if type(prefs) != dict:
        #!!!   replace with real error handling
        raise TypeError,"YAML prefs must be a dictionary"     
      self.update(prefs)      
    
class GTPPreferencesDlg(wx.Dialog):
    #Feature Coming Soon to a GUI Near You!

    def __init__(self, parent ):
        wx.Dialog.__init__(self, parent, title="Preferences", size=(250, 210))

        self.parent = parent
        self.prefs = None
        self.vbox = wx.BoxSizer(wx.VERTICAL)

        self.GTSBox = wx.StaticBox(self, -1, 'Gait Table', (5, 5), (240, 150))
        return #!!!
        self.GTSaveCB = wx.CheckBox(self.GTSBox, -1, 'Auto Refresh', (15, 30), style=wx.RB_GROUP)
        wx.CheckBox(panel, -1, '16 Colors', (15, 55))
        wx.CheckBox(panel, -1, '2 Colors', (15, 80))
        wx.CheckBox(panel, -1, 'Custom', (15, 105))
        wx.TextCtrl(panel, -1, '', (95, 105))

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        okButton = wx.Button(self, -1, 'Ok', size=(70, 30))
        closeButton = wx.Button(self, -1, 'Close', size=(70, 30))
        hbox.Add(okButton, 1)
        hbox.Add(closeButton, 1, wx.LEFT, 5)

        vbox.Add(panel)
        vbox.Add(hbox, 1, wx.ALIGN_CENTER | wx.TOP | wx.BOTTOM, 10)

        self.SetSizer(vbox)

    def openWith( self, prefs ):
      """
      Associate dialog with a GTPPreferences object and
      pop the dialog open.
      """
      #!!! implement
      self.prefs = prefs
      self.ShowModal()
      self.Hide()
    
class ODOSelect(wx.Dialog):
    """
    A GUI class that generate a dialog from an ODTable view and 
    allows the user to select an ODO.
    
    When the selection is made, the TreeView is hidden and the
    completion callback is made with the view string selection.
    If no selection is made, completion is called with None.
    
    Usage pseudocode:
      odos = ODOSelect( <<some-parent>> )
      loop
        if odtbl changed:
          odos.populate( odtbl.genView() )
        odos.useAndCall( <<selection-callback>> )      
    """
    def __init__(self, parent):
        wx.Dialog.__init__(self, parent, title="ODTable View")
        #Attributes
        self.tree = wx.TreeCtrl(self, style=wx.TR_DEFAULT_STYLE)
        self.completion = None
        self.Bind(wx.EVT_TREE_ITEM_ACTIVATED, self.OnActivated, self.tree)
        self.Bind(wx.EVT_CLOSE, self.OnClose)
        
    def populate( self, viewIter ):
      """
      Repopulate the tree with items generated by the iterable viewIter
      """
      # Remove previous items, if any
      self.tree.DeleteAllItems()
      self.root = self.tree.AddRoot("Modules on CAN:")        
      currRoot = []
      for grp,cap in viewIter:
        #print "Populate ",grp,cap                  
        # If view is a capability under a new group --> create new root
        if grp != currRoot:
            currRoot = grp
            modRoot = self.tree.AppendItem(self.root, currRoot)            
        self.tree.AppendItem(modRoot, cap)
      
    def useAndCall( self, cb, *args ):
      """
      Pop up the ODO tree, allowing the user to select a tree leaf.
      The item in the tree leaf is passed in a call to cb(item).
      If the dialog is closed with no selection, cb(None) is called.
      """
      assert self.completion is None
      assert (cb is None) or callable(cb)
      self.completion = (cb,)+args
      self.Show()
      
    def OnActivated(self, evt):
        item = evt.GetItem()
        obj_str = self.tree.GetItemText(item)
        #print "Tree::Activated: '%s'" % obj_str
        if ODTable.isNodeText( obj_str ):
          return False
        # Complete selection
        if self.completion[0]:
          self.completion[0]( obj_str, *self.completion[1:])
        self.completion = None
        self.Hide()
        return True

    def OnClose(self, evt):
        if self.completion[0]:
          self.completion[0]( None, *self.completion[1:])
        self.completion = None
        self.Hide()
        return True
                    
class CKBotGUI( wx.PySimpleApp ):
  def __init__(self):
    wx.PySimpleApp.__init__(self,None)
    ## Controller interface -- things the view can ask for
    # take all methods that start with cmd_ and strip the cmd_
    self.cmds = Misc()
    for nm,fun in self.__class__.__dict__.iteritems():
      if nm[:4]!='cmd_':
        continue
      setattr( self.cmds, nm[4:], getattr( self, nm ))
    # ------------- MODEL ----------------
    self.model = Misc(
      prefs = GTPPreferences(),
      odt = ODTable(),
      gt = GaitTable()
    )
    self.model.prefs.load()
    # ------------- VIEW ----------------
    gtFr = GaitTableFrame(self.cmds)
    self.view = Misc(
      gtFr = gtFr,
      prefDlg = GTPPreferencesDlg(gtFr),
      loadODT = gtFr.browseLoadODT,
      saveODT = gtFr.browseSaveODT,
      loadGT = gtFr.browseLoadGT,
      saveGT = gtFr.browseSaveGT,      
      odoDlg = ODOSelect(gtFr)
    )    
    self.GT_filename = "example.sgt"
    self.ODT_filename = "example.odo"

  def run(self):
    self.view.gtFr.Show()
    self.MainLoop()

  def odoByName( self, oNm ):
    return self.model.odt.find(oNm)

  def _changedOD( self ):
    self.view.odoDlg.populate( self.model.odt.genView() )

  def _changedGT( self ):
    self.view.gtFr.updateGCs( self.model.gt.iterCols() )

  def cmd_close( self, evt ):
    self.model.prefs.save()
    self.view.gtFr.Close()
  
  def cmd_loadODT( self, evt ):
    fn = self.view.loadODT(self.ODT_filename)
    if fn:
      self.ODT_filename = fn
    else:
      return False
      
    if not self.model.odt.load(fn):
      return False
      
    self._changedOD()
    self._changedGT()
    return True      
    
  def cmd_saveAsODT( self, evt ):
    fn = self.view.saveODT(self.ODT_filename)
    if fn:
      self.ODT_filename = fn          
    return self.cmd_saveODT(evt)
    
  def cmd_saveODT( self, evt ):
    return self.model.odt.save(self.ODT_filename)

  def cmd_viewTree( self, evt ):
    self.view.odoDlg.useAndCall( lambda x : None )
  
  def cmd_addGC( self, evt ):
    #print "addGC"
    self.view.odoDlg.useAndCall( self.complete_addGC )
 
  def complete_addGC( self, oNm ):
    if oNm is None:
      #print "addGC cancelled"
      return False
    #print "addGC", oNm
    gt = self.model.gt
    gt.addGC( GaitCol(self.odoByName(oNm)) )
    self._changedGT()

  def cmd_setGC( self, gcp ):
    #print "setGC"
    self.view.odoDlg.useAndCall( self.complete_setGC, gcp )
    
  def complete_setGC( self, oNm, gcp ):
    if oNm is None:
      #print "setGC(complete) cancelled"
      return False
    #print "setGC(complete)", oNm
    gcp.gc.changeODO( self.odoByName(oNm) )
    self._changedGT()

  def cmd_delGC( self, gc ):
    self.model.gt.delGC( gc )
    self._changedGT()
  
  def cmd_runGait( self ):
    pass
  
  def cmd_loadGT( self, filename ):
    fn = self.view.loadGT(self.GT_filename)
    if fn:
      self.GT_filename = fn
    else:
      return False
      
    if self.model.gt.load(fn, self.model.odt):
      return False
    #print "LOADED ",repr(list(self.model.gt.iterCols()))
    self._changedOD()
    self._changedGT()
    return True
 
  def cmd_saveAsGT( self, evt ):
    fn = self.view.saveGT(self.GT_filename)
    if fn:
      self.GT_filename = fn          
    return self.cmd_saveGT(evt)

  def cmd_saveGT( self, evt ):
    return self.model.gt.save(self.GT_filename)

  def cmd_clear( self, evt ):
    self.model.odt.clear()
    self._changedOD()    

  def cmd_scan( self, evt ):
    if self.model.odt.scan():
      self._changedOD()
  
  def cmd_prefs( self, evt ):
    self.view.prefDlg.openWith( self.model.prefs )

  def cmd_timerTic( self ):
    #!!! limit by autoscan rate
    # If we found changes --> update GUI
    if self.model.odt.refresh():
      self._changedOD()
    for gcp in self.view.gtFr.gt_panel.gcps:
      gcp.Get()
    #!!! autosave of ODT, GT
    
def progress(msg):
  sys.stdout.write(msg+"\n")
  sys.stdout.flush()
logical.progress = progress

if __name__=="__main__":
    CKBotGUI().run()
    # jEdit :wrap=None:
