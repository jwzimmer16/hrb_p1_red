import re
import sys

if len(sys.argv)<2:
  sys.stderr.write("""
  Usage: pictopy.py <pic-inc-filename>
  
  Reads the file <pic-inc-filename> assembly .INC file and writes a matching python implementation to stdout
  """)
  sys.exit(2)

#
#PORTA      EQU  H'0ABC'
#
RE_REG = re.compile('\s*(\w+)\s+EQU\s+H\'([0-9a-fA-F]+)\'\s*(?:;(.*))?')

#
#   ; ---- BLAH Bits -----
#
RE_BITS = re.compile('\s*;\s*-*\s+(\w+) Bits .*')
#
#   ; comment
#
RE_COMMENT = re.compile('\s*;(.*)')

#
#  #define parrot dead as a doornail
#
RE_DEF = re.compile('\s*#define\s+(\w+)\s+(.*)\s*(?:;.*)')

out = []
defs = {}
regs = set()
field = None
for line in open(sys.argv[1],'r'):
    m = RE_BITS.match(line)
    if m:
      out.append( "#" + line[:-1] )
      field = m.group(1)
      continue
    m = RE_COMMENT.match(line)
    if m:
      out.append( "#" +m.group(1)[:-1] )
      field = None
      continue
    m = RE_REG.match(line)
    if m:
      reg,val,doc = m.groups()
      if field:
        reg = "%s_BIT" % reg
      if doc:
        out.append( "%s = 0x%04X # %s" % (reg,long(val,16),doc) )
      else:
        out.append( "%s = 0x%04X" % (reg,long(val,16)) )
      regs.add(reg)
      continue
    m = RE_DEF.match(line)
    if m:
      defs[m.group(1)] = m.group(2)[:-1]
      continue
    # Echo empty lines to output
    if not line.strip():
      out.append("")

if 0: # if we wanted to substitute all the macros into output
 for line in out:
    if line and line[0] != '#':
      more = True
      while more:
        more = False
        for mac,val in defs.iteritems():
           if line.find(mac)>=0:
              line = line.replace(mac,val)
              more = True
              print "# ",mac," --> ",val," in '%s'" % line
    print line

# but instead, I noticed all macros are just assignments, so:
print """
# 
# WARNING: this file was automatically generated using %s
#
# Python interface generate from %s
#

""" % (sys.argv[0], sys.argv[1])

sys.stdout.writelines(( l+"\n" for l in out ))
if defs:
  print """
#
#  #define-s converted to assigments:
#
"""
sys.stdout.writelines(( "%s = %s # from #define %s %s\n" % (k,v,k,v) 
  for k,v in defs.iteritems() 
  if v in regs )) 

