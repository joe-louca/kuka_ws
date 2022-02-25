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
latency /= 1000.0               # One way latency (ms to s)

start_time = get_time()         # time in float seconds
rate_hz = 100                   # Loop rate (Hz) 10 (was 500)
pos_step_size = 1.0             # Position step size (mm)
rot_step_size = 1.0 * pi/180    # Rotation step size (deg to rads)
f_threshold = 100.0             # Force sensor threshold for rigid collisions (N)
t_threshold = 5.0               # Torque sensor threshold (Nm)
friction_f_threshold = 30.0     # Force sensor threshold for frictional forces (N)
friction_t_threshold = 1.0      # Torque sensor threshold for frictional torque (Nm)
velocity = [0.5]                # Kuka velocity - Range 0-1
RT_timestep = 1.0/20            # Timestep between kuka commands (s)
ft_factor = 50.0               # scale factor for haption forces (%)
workspace_factor = 50.0        # scale factor for haption workspace (%)


j1 =  -25*pi/180
j2 =  -15*pi/180
j3 =   15*pi/180
j4 =   95*pi/180
j5 =  -25*pi/180
j6 =  -30*pi/180
j7 =    0*pi/180
initial_jpos = [j1,j2,j3,j4,j5,j6,j7]
initial_pos = [-589, 100.0, 649.0, -170.0*pi/180, 80.0*pi/180, -170.0*pi/180] #degs: 0, -90, 0

set_param('start_time', start_time)
set_param('latency', latency)
set_param('rate_hz', rate_hz)
set_param('pos_step_size', pos_step_size)
set_param('rot_step_size', rot_step_size)
set_param('f_threshold',  f_threshold)
set_param('t_threshold',  t_threshold)
set_param('friction_f_threshold',  friction_f_threshold)
set_param('friction_t_threshold',  friction_t_threshold)
set_param('velocity',  velocity)
set_param('RT_timestep',  RT_timestep)

set_param('ax', {'x':0.0, 'y':0.0, 'z':0.0, 'roll':0.0, 'pitch':0.0, 'yaw':0.0})
set_param('but', {'x_press':0})
set_param('kuka_jpos', initial_jpos)
set_param('kuka_pos', initial_pos)
set_param('initial_kuka_pos', initial_pos)
set_param('initial_jpos', initial_jpos)
set_param('delayed_pos_cmd', initial_pos)
