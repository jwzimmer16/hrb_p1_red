#  This library is free software; you can redistribute it and/or
#  modify it and use it for any purpose you wish.
#
#  The library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
#
# (c) Shai Revzen, U Penn, 2010
#  
"""
misc.py defines several useful functions that don't rightly belong in any obvious location

Functions
---------

curry -- Curry a function, returning a callable with the value of the initial parameters pre-selected. One of the mainstays of functional programming.

printExc -- Print a formatted stack-trace to standard error

loadCSV -- Load a CSV file into a list of lists

inlineCSV -- Similar to loadCSV, but takes the CSV text from a multiline python string
"""

import traceback
import sys

def curry(fn, *cargs, **ckwargs):
  """
  Curry a function, returning a callable with the value of the
  initial parameters pre-selected. For any function f
    curry(f,*arg0)(*arg1,**kw) f(x,*(arg0+arg1),**kw)
  so, for example:
    f(x,y) = curry(f,x)(y) = curry(f,x,y)()
  """
  def call_fn(*fargs, **fkwargs):
      d = ckwargs.copy()
      d.update(fkwargs)
      return fn(*(cargs + fargs), **d)
  return call_fn

def printExc( exinfo=None ):
  """
  Print a formatted stack-trace to standard error 
  """
  # Print the stack trace so far to stderr
  if exinfo is None:
    exinfo = sys.exc_info()
  msg = ["<< "*10+"\n"]
  msg.extend(traceback.format_exception(*exinfo))
  msg.append(">> "*10+"\n")
  sys.stderr.writelines(msg)

def loadCSV( fn ):
  """
  Load a CSV file into a list of lists, converting empty cells into
  None, numbers as floats and quoted strings as strings.
 
  In addition to standard CSV format, empty (white only) lines and
  lines starting with "#" are ignored
  """
  if type(fn)==str:
    f = open(fn,'r')
  else:
    assert iter(fn),"Must be iterable returning lines"
    f = fn
  def convert(x):
    x = x.strip()
    if not x or x=='""': return None
    if x[0]=='"': return x[1:-1]
    return float(x)
  res = [ 
    [ convert(x) for x in l.split(",") ] 
    for l in f if l.strip() and l[:1]!="#"
  ]
  if type(fn)==str:
    f.close()
  return res

def inlineCSV( txt ):
  """
  Similar to loadCSV, but takes the CSV text from a multiline 
  python string, e.g.
  
  >>> sheet = inlineCSV('''
  ... # Non-CSV Unix shell style comments and empty lines are ignored
  ... ,         "sparrow",  "banana"
  ... "weight", 5,          1
  ... ''')
  """
  return loadCSV(( l for l in txt.split("\n") ))

