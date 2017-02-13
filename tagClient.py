from joy import JoyApp
from joy.plans import AnimatorPlan
from joy.decl import KEYDOWN  

from socket import socket, AF_INET, SOCK_DGRAM, error as SocketError
from numpy import array, mean
from json import loads as json_loads

if __name__ != "__main__":
  raise RuntimeError("Run this as a script")

try:
  s.close()
except Exception:
  pass

def _animation(fig):
  s = socket(AF_INET, SOCK_DGRAM )
  s.bind(("",0xB00))
  s.setblocking(0)
  fig.clf()
  ax = fig.add_subplot(111)
  msg = None
  while True:
    try:
      # read data as fast as possible
      m = s.recv(1<<16)
      if m and len(m)>2:
        msg = m
      continue
    except SocketError, se:
      # until we've run out; last message remains in m
      pass
    # make sure we got something
    if not msg:
      yield
      continue
    # display it
    ax.cla()
    for d in json_loads(msg):
      a = array(d['p'])
      ax.plot( a[[0,1,2,3,0],0], a[[0,1,2,3,0],1], '.-b' )
      ax.plot( a[[0],0], a[[0],1], 'og' )
      ax.text( mean(a[:,0]), mean(a[:,1]), d['i'], ha='center',va='center' )
      ax.axis([0,1024,0,768])
    yield
    
      
class App(JoyApp):
  def onStart(self):
      AnimatorPlan(self,_animation).start()

  def onEvent(self,evt):
      if evt.type == KEYDOWN:
        return JoyApp.onEvent(self,evt)
        
app = App(cfg={'windowSize':(1080,740)})
app.run()
  