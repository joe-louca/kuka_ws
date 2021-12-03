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
        self.velocity = [0.1]
        
        # Move to an initial position
        print('Moving to home position')
        initPos = [0, 0, 0, 0, 0, 0, 0]
        self.iiwa.movePTPJointSpace(initPos, self.velocity)
        time.sleep(3)

        iiwa_pos = self.iiwa.getEEFCartizianPosition()
        print('Home pos =')
        print(iiwa_pos)

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




        
CopControl()
