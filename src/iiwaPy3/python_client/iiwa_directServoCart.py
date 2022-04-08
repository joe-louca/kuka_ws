# -*- coding: utf-8 -*-

from iiwaPy3 import iiwaPy3
from math import pi
import time
from datetime import datetime
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray

class KukaControl:
    def __init__(self):          
        self.start_time = datetime.now()                            # Start time for getSecs()
        self.frame_id = 0
        self.timestamp = rospy.Time.now()
        
        
        # Setup connection to iiwa
        self.IP_of_robot = '172.31.1.148'        
        self.connection_state = False
        self.connect_to_iiwa()

        # Set some parameters
        timestep = 0.001#0.002           # Secs (Good at 0.01) - Docs says >20ms for servo control
        rate_hz = 200#100               # Hz
        kuka_joints_msg = Float32MultiArray() # Initialise msg to publish
        cmd = [None, None, None, None, None, None]
        kuka_position = [None, None, None, None, None, None]

        try:        
            # Move to home pos
            print('Kuka Arm: Moving to home position')
            start_pos = [0, -25*pi/180, 0, 75*pi/180, 0, -75*pi/180, 0]
            #start_pos = [0, 0*pi/180, 0, 90*pi/180, 0, -20*pi/180, 0]
            self.iiwa.movePTPJointSpace(start_pos,[0.1])

            kuka_pos = self.iiwa.getEEFPos() # (in mm and rads)
            #-582.4117252266703   0.029790546497991836   545.6431178144777   3.141476197750022   -0.08725588947971301   3.141521033243545

            # Start servo for soft realtime control
            self.iiwa.realTime_startDirectServoCartesian()      
            time.sleep(1)

            pub = rospy.Publisher('/kuka_joints', Float32MultiArray, queue_size=1)
            sub = rospy.Subscriber('/v_kuka_in', TwistStamped, self.callback, queue_size=1)

            t_0 = self.getSecs()
            cycle_counter = 0
            self.send_msg_ = False
            r = rospy.Rate(rate_hz)
            print('Haption: Hold stick vertical and hold grey footswitch to move')
            print('Haption: Press left button to toggle the gripper')

            while not rospy.is_shutdown():
                if self.send_msg_:
                    for i in range(3):
                        cmd[i] = self.vel[i]*10000*timestep + kuka_pos[i]   # in mm
                        cmd[i+3] = self.vel[i+3]*timestep*20 + kuka_pos[i+3]  # in rads
                        
                    if (self.getSecs()-t_0)>timestep:
                        intrinsic_latency = rospy.Time.now() - self.timestamp           # To check system latency
                        #print(intrinsic_latency.to_sec())
                        kuka_pos = self.iiwa.sendEEfPositionGetActualEEFpos(cmd)        # Send position command
                        
                        kuka_joints = self.iiwa.getJointsPos()
                        kuka_joints_msg.data = kuka_joints
                        pub.publish(kuka_joints_msg)                           # Publish current joint positions
                        
                        self.send_msg_ = False
                        t_0=self.getSecs()
                        cycle_counter=cycle_counter+1
                r.sleep()
                
            totalT= self.getSecs()-t0;
            self.iiwa.realTime_stopDirectServoCartesian()               # Stop direct servo
            
        except:        
            print('KUKA: An error occured')

        self.disconnect_from_iiwa()                                 # Disconnect
        
    def connect_to_iiwa(self):
        # Check if already connected to the robot
        if self.connection_state:
            print("Kuka Arm: Already connected to the robot on IP: " + self.IP_of_robot)
            return

        # If the program made it to here, then there is no connection yet so try to connect
        print("Kuka Arm: Connecting to robot at ip: " + self.IP_of_robot)
        try:
            self.iiwa = iiwaPy3(self.IP_of_robot)
            self.connection_state = True
            print("Kuka Arm: Connection established successfully")
            return
            
        except:
            print("Kuka Arm: Error, could not connect at the specified IP")
            return            

    def disconnect_from_iiwa(self):
        # Check if there is an active connection
        print("Kuka Arm: Disconnecting from robot")
        if self.connection_state == False:
            print("Kuka Arm: Already offline")
            return

        # If made it to here, then there is an active connection so try to disconnect
        try:
            self.iiwa.close()
            self.connection_state = False
            print("Kuka Arm: Disconnected successfully")
            
        except:
            print("Kuka Arm: Error could not disconnect")
            return

    def callback(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z
        wx = msg.twist.angular.x
        wy = msg.twist.angular.y
        wz = msg.twist.angular.z
        self.timestamp = msg.header.stamp
        
        self.vel = [vx, vy, vz, wz, -wy, -wx]      # (in mm and rads) (KUKA ABC angles are in Z, Y, X order)
        self.send_msg_ = True

    # returns the elapsed seconds since the start of the program
    def getSecs(self):
       dt = datetime.now() - self.start_time
       secs = (dt.days * 24 * 60 * 60 + dt.seconds)  + dt.microseconds / 1000000.0
       return secs

if __name__ == '__main__':
    rospy.init_node('KUKA', anonymous=True)
    try:
        KukaControl()
    except rospy.ROSInterruptException:
        pass
