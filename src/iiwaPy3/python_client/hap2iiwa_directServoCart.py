# -*- coding: utf-8 -*-

from iiwaPy3 import iiwaPy3
from math import pi
import time
from datetime import datetime
import rospy
import numpy as np

class CopControl:
    def __init__(self):   
        self.start_time = datetime.now()                            # Start time for getSecs()

        # Setup connection to iiwa
        self.IP_of_robot = '172.31.1.148'        
        self.connection_state = False
        self.connect_to_iiwa()

        # Get some parameters
        self.velocity = rospy.get_param('velocity')
        self.time_step = rospy.get_param('RT_timestep')
        self.commandsList=[]
        
        # Move to an initial position
        initPos = rospy.get_param('initial_jpos')
        self.iiwa.movePTPJointSpace(initPos, self.velocity)

        # Start direct servo control
        print('Press Ctrl-C to exit...')
        if self.connection_state:                                   # If connected to iiwa
            try:
                self.iiwa.realTime_startDirectServoCartesian()      # Start servo
                self.t_0 = self.getSecs()                           # Refreshable start time

                while True:                                         # Until Ctrl-C
                    pos_cmd = rospy.get_param('delayed_pos_cmd')
                    self.commandsList.append(pos_cmd)
                    if (self.getSecs()-self.t_0)>self.time_step:    # If elapsed time for this step > desired time_step
                        self.move_cmd()                             # Send command to iiwa

            except KeyboardInterrupt:                           # Ctrl-C to exit
                pass
            
        self.iiwa.realTime_stopDirectServoCartesian()              # Stop direct servo
        self.disconnect_from_iiwa()                             # Disconnect

    def connect_to_iiwa(self):
        # Check if already connected to the robot
        if self.connection_state:
            print("Already connected to the robot on IP: " + self.IP_of_robot)
            return

        # If the program made it to here, then there is no connection yet so try to connect
        print("Connecting to robot at ip: " + self.IP_of_robot)
        try:
            self.iiwa = iiwaPy3(self.IP_of_robot)
            self.connection_state = True
            print("Connection established successfully")
            return
            
        except:
            print("Error, could not connect at the specified IP")
            return            

    def disconnect_from_iiwa(self):
        # Check if there is an active connection
        print("Disconnecting from robot")
        if self.connection_state == False:
            print("Already offline")
            return

        # If made it to here, then there is an active connection so try to disconnect
        try:
            self.iiwa.close()
            self.connection_state = False
            print("Disconnected successfully")
            
        except:
            print("Error could not disconnect")
            return


    def move_cmd(self):
        if len(self.commandsList) > 0:                  # If there are commands to act on
            pos_cmd = self.commandsList[0]              # Get most recent joint command
            self.kuka_pos = self.iiwa.sendEEfPositionGetActualEEFpos(pos_cmd)         # Send command  & get pos
            self.kuka_jpos = self.iiwa.getJointsPos()   # Get joint positions
            rospy.set_param('kuka_jpos', self.kuka_jpos)# Save kuka_jpos as param
            rospy.set_param('kuka_pos', self.kuka_pos)  # save kuka_pos as param

            self.t_0 = self.getSecs()                   # Refresh start time after move
            self.commandsList = []                      # Clear commands list

    def getSecs(self):
        dt = datetime.now() - self.start_time
        secs = (dt.days * 24 * 60 * 60 + dt.seconds)  + dt.microseconds / 1000000.0
        return secs

        
CopControl()
