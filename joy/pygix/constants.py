"""
FILE: constants.py

  This file contains some seriously ugly code whose job is to import the
  constants needed by joy from whatever implementation of pygame is 
  available.

  First, the full blows pygame import is attempted. If that doesn't work out,
  a hard coded list of implementations is tried out. Those are expected to
  be constants_ARCH.DYN where ARCH is some architecture, and DYN is the
  dynamic loader format for the python environment.
  
  An extra wrinkle is that these files are extracted from compiled pygame
  build trees for the architecture, so they all declare themselves into 
  a module named 'constants' rather than one whole name is that of the file.

  In summary -- you don't want to know any of this if it works.

  HOW TO ADD AN IMPLEMENTATION
  ============================

  Lets assume you want to add an implementation for a hypothetical
  architecture called HRM96 in an environment whose equivalent of .dll and .so
  is called .dyl

  (1) Compile pygame best you can. If needed you can tweak the build tree -- 
    but make it build constants.dyl 
  
  (2) Copy and rename constants.dyl --> pygix/constants_HRM96.dyl
      (NOTE: remember to 'git add' it too!)

  (3) Add 'HRM96' to the ARCH list in this file (search for 'ARCH=')

  If all is well:

  $ cd pyckbot/py 
  $ python
  >>> from joy.decl import *
  >>> IMPL
   'HRM96'
"""
from sys import stderr
  
try:
  from pygame.locals import *
  IMPL = "pygame"
except ImportError: # pygame import failed
  stderr.write("*** pygame missing; using compatibility wrapper instead\n")
  IMPL = None
  
if IMPL is None:
  ARCH = ["x86_64","x86_32"]
  import imp
  fpd = None
  # Look for various extension module types
  for IMPL in ARCH:
    try:
      # See if we can find one
      stderr.write('*** Trying architecture %s\n' % IMPL)
      fpd = imp.find_module('joy/pygix/constants_'+IMPL)
      impl = imp.load_module('constants', *fpd)
      stderr.write('***   ==>> SUCCESS!\n')
      break
    except ImportError:
      continue
  if fpd is None:
    raise ImportError("No implementation of 'constants' was found")
  # Load what we found
  try:
    loc = locals()
    for nm,val in impl.__dict__.iteritems():
      if nm.startswith("__"):
        continue
      loc[nm]=val
  finally:
    # It's our job to close the file
    if fpd[0]:
      fpd[0].close()
  # Clean up the namespace
  del fpd,imp,loc,nm,val,ARCH,impl

del stderr
