# -*- coding: utf-8 -*-
"""
About the script:
An exmple on controlling KUKA iiwa robot from
Python3 using the iiwaPy3 class

Modified on 3rd-Jan-2021

@author: Mohammad SAFEEA

"""
from iiwaPy3 import iiwaPy3
from math import pi
import time
from datetime import datetime
import rospy

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
        j1 = -40*pi/180
        j2 =  75*pi/180
        j3 =   0*pi/180
        j4 =  50*pi/180
        j5 = -20*pi/180
        j6 =  35*pi/180
        j7 =  0*pi/180
        initPos = [j1,j2,j3,j4,j5,j6,j7]
        self.iiwa.movePTPJointSpace(initPos, self.velocity)

        # Start direct servo control
        print('Press Ctrl-C to exit...')
        if self.connection_state:                                   # If connected to iiwa
            try:
                self.iiwa.realTime_startDirectServoCartesian()      # Start servo
                self.t_0 = self.getSecs()                           # Refreshable start time

                while True:                                         # Until Ctrl-C
                    self.get_cmd()                                  # Get joints from copellia
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

    def get_cmd(self):
        pos_cmd = rospy.get_param('pos_cmd')
        # TO DO - match to BRL ref frames...
        self.commandsList.append(pos_cmd)

    def move_cmd(self):
        if len(self.commandsList) > 0:                  # If there are commands to act on
            pos_cmd = self.commandsList[0]              # Get most recent joint command
            self.kuka_jpos = self.iiwa.sendEEfPositionGetActualJpos(pos_cmd)         # Send command
            rospy.set_param('kuka_jpos', self.kuka_jpos)  # Save kuka_pos as param
            self.t_0 = self.getSecs()                   # Refresh start time after move
            self.commandsList = []                      # Clear commands list

    def getSecs(self):
        dt = datetime.now() - self.start_time
        secs = (dt.days * 24 * 60 * 60 + dt.seconds)  + dt.microseconds / 1000000.0
        return secs      

CopControl()
