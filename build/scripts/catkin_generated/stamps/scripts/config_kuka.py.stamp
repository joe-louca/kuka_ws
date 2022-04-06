#!/usr/bin/env python

# Variables as ros parameters
from math import pi
from rospy import get_param
from rospy import set_param
from rospy import get_time
from rospy import init_node


init_node('config', anonymous=True)

try:
    latency = get_param('latency')
except:
    latency = 0.0
latency /= 2000.0               # One way(/2) latency from ms to s (/1000)

start_time = get_time()        # Start time in float (s)
rate_hz = 200                  # Loop rate (Hz)
timestep = 1/rate_hz           # Time per loop (s)
ft_factor = 50.0               # scale factor for haption forces (%)
workspace_factor = 50.0        # scale factor for haption workspace (%)

set_param('start_time', start_time)
set_param('latency', latency)
set_param('rate_hz', rate_hz)
set_param('timestep',  timestep)
