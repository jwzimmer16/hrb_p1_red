from ckbot.logical import Cluster
from sys import argv, exit, stderr
from time import sleep

if __name__!="__main__":
  exit()

nids = None
port = None
count = 1
out = stderr
arg = argv[1:]
while arg:
  if arg[0] == "-n":
    nids = map(int,arg[1].split(","))
    arg[:2]=[]
    continue
  if arg[0] == "-c":
    count = int(arg[1])
    arg[:2]=[]
    continue
  if arg[0] == "-p":
    port = eval(arg[1])
    arg[:2]=[]
    continue
  if arg[0] == "-o":
    out = open(arg[1],"w")
    arg[:2]=[]
    continue
  print """
    Options:
       -n <node-id>
         Which node to use
       -c <count>
         How many nodes to look for
       -p <port>
         Port descriptor (see port2port.newConnection)
       -o <output-file>
         Specify output file. Otherwise goes to STDERR
  """
  exit(5)

c = Cluster( port=port, count=count )
for nid in (nids if nids is not None else c.iterkeys()):
  m = c[nid]
  out.write("# Node ID : %d\n" % nid)
  if m.mem is None:
    out.write("# No .mem member -- properties are not readable\n")
    continue
  for adr,(nm,fmt) in m.mcu._ADDR_DCR.iteritems():
    out.write("%s : %d\n" % (nm,m.mem[adr]))
  sleep(0.02)

