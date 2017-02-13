"""
FILE: defaults.py

Contains configuration defaults for logical.py

This file exists so you won't need to edit core files to change
default behavior of applications using logical.py and related classes
"""
import dynamixel

# Default cluster hardware architecture
DEFAULT_ARCH = dynamixel
# Default interface to use for 
DEFAULT_PORT = dict(
  TYPE='tty',
  baudrate=115200
)

