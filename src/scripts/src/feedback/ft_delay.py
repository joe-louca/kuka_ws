#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray
import time
import numpy as np



 
class FT:
    def add_delay(self, added_row, delayed_tbl):
        latency = rospy.get_param('latency')
        delayed_tbl.append(added_row)                                   # Add new row to end of list (newest at bottom)
        row_len = len(added_row)-1                                      # Length of added row
        tbl_len = len(delayed_tbl)                                      # Number of rows in of table                        

        retrieved_row = []
        retrieved = False
        elapsed_time = rospy.get_time()
        
        for i in range(tbl_len):                                        # Starting at the oldest, For each row   
            if elapsed_time > delayed_tbl[i][row_len] + latency:        # If row is old enough
                retrieved_row = delayed_tbl[i][:row_len]                # Get this row and remove timestamp
                delayed_tbl = delayed_tbl[i+1:]                         # Keep only new rows (all rows at this point and below)            
                retrieved = True                                        # Update marker
                break
        
        return retrieved_row, retrieved, delayed_tbl


    
    def call_Ax_ft(self, ft_msg):
        ## Get sensor readings from Axia
        ax_fx = ft_msg.wrench.force.x
        ax_fy = ft_msg.wrench.force.y
        ax_fz = ft_msg.wrench.force.z
        ax_tx = ft_msg.wrench.torque.x
        ax_ty = ft_msg.wrench.torque.y
        ax_tz = ft_msg.wrench.torque.z

        Ax_F_sensor = np.array([[ax_fx],
                                [ax_fy],
                                [ax_fz]])
        Ax_T_sensor = np.array([[ax_tx],
                                [ax_ty],
                                [ax_tz]])
        frame_id = ft_msg.header.seq
        frame_t = float(str(ft_msg.header.stamp))/1000000000 - self.start_time

        
        ## Remove bias (baseline axia reading without any tool attached)
        #Ax_F_bias = np.array([[-11.6],
        #                      [- 7.4],
        #                      [-31.7]])
        #Ax_T_bias = np.array([[-0.52],
        #                      [ 1.07],
        #                      [ 0.13]])
        ## Remove bias (baseline axia reading with tool attached) - THIS IS GOOD
        Ax_F_bias = np.array([[-14.2], # -25.5 X Up, -14.04 X Left, -2.5 X down, -14.3 X right
                              [-15.3], # -15.2 Y left, -4 Y down, -15.4 Y right, -26.7 Y up
                              [-45.5]])# -45.5 Z back, -33.7 Z down, -57 Z Up, -45.5 Z front
        Ax_T_bias = np.array([[-1.14], # -1.13 X Up, -1.15 X Down
                              [ 1.27], # 1.28 Y Up, 1.26 Y Down
                              [ 0.09]])        
        
        Ax_F_sensor = Ax_F_sensor - Ax_F_bias
        Ax_T_sensor = Ax_T_sensor - Ax_T_bias
        
        
        ## Convert to world frame
        if self.Rot_ready_:
            W_F_sensor = np.dot(self.W_R_Ax, Ax_F_sensor)
            W_T_sensor = np.dot(self.W_R_Ax, Ax_T_sensor)
            self.Rot_ready_ = False

            hap_F = W_F_sensor
            hap_T = W_T_sensor
            
            ## Remove Tool force
            if self.Tool_ready_:
                #hap_F = W_F_sensor - self.W_F_tool
                #hap_T = W_T_sensor - self.W_T_tool
                self.Tool_ready_ = False
            
                hap_F = W_F_sensor
                hap_T = W_T_sensor
                # Timestamp 
                t = rospy.get_time()
                timestamped_ft = [hap_F[0][0], hap_F[1][0], hap_F[2][0], hap_T[0][0], hap_T[1][0], hap_T[2][0], frame_id, frame_t, t]

                # Store and retrieve delayed ft readings
                ft, retrieved, self.delayed_ft_tbl = self.add_delay(timestamped_ft, self.delayed_ft_tbl)
     
                if retrieved:

                    # Convert to python floats
                    ft_x = ft[0].item()
                    ft_y = ft[1].item()
                    ft_z = ft[2].item()
                    ft_tx = ft[3].item()
                    ft_ty = ft[4].item()
                    ft_tz = ft[5].item()

                    # Build publish message
                    self.pub_msg.wrench.force.x = ft_x*0.1
                    self.pub_msg.wrench.force.y = ft_y*0.1
                    self.pub_msg.wrench.force.z = ft_z*0.1
                    self.pub_msg.wrench.torque.x = ft_tx*0.1
                    self.pub_msg.wrench.torque.y = ft_ty*0.1
                    self.pub_msg.wrench.torque.z = ft_tz*0.1
                    self.pub_msg.header.seq = ft[6]
                    self.pub_msg.header.stamp = rospy.get_rostime()#ft[7]#to_sec()

                    # Send to haption.cpp
                    rospy.set_param('ft_delay/fx', ft_x*0.1)
                    rospy.set_param('ft_delay/fy', ft_y*0.1)
                    rospy.set_param('ft_delay/fz', ft_z*0.1)
                    rospy.set_param('ft_delay/tx', ft_tx*0.1)
                    rospy.set_param('ft_delay/ty', ft_ty*0.1)
                    rospy.set_param('ft_delay/tz', ft_tz*0.1) 
                    
                    ## Mark message as ready to send
                    self.send_msg_ = True
        return


    def call_Tool_ft(self, tool_msg):
        tool_fx = tool_msg.wrench.force.x
        tool_fy = tool_msg.wrench.force.y
        tool_fz = tool_msg.wrench.force.z
        tool_tx = tool_msg.wrench.torque.x
        tool_ty = tool_msg.wrench.torque.y
        tool_tz = tool_msg.wrench.torque.z

        self.W_F_tool = np.array([[tool_fx],
                                  [tool_fy],
                                  [tool_fz]])
        self.W_T_tool = np.array([[tool_fx],
                                  [tool_fy],
                                  [tool_fz]])


        self.Tool_ready_ = True
        return

    def call_Rot(self, rot_msg):
        r11 = rot_msg.data[0]
        r12 = rot_msg.data[1]
        r13 = rot_msg.data[2]
        r21 = rot_msg.data[3]
        r22 = rot_msg.data[4]
        r23 = rot_msg.data[5]
        r31 = rot_msg.data[6]
        r32 = rot_msg.data[7]
        r33 = rot_msg.data[8]

        self.W_R_Ax = np.array([[r11, r12, r13],
                                [r21, r22, r23],
                                [r31, r32, r33]])

        self.Rot_ready_ = True
        return    



    def __init__(self):
        self.start_time = rospy.get_param('start_time')
        self.delayed_ft_tbl = []
        self.pub_msg = WrenchStamped()
        
        self.send_msg_ = False
        self.Rot_ready_ = False
        self.Tool_ready_ = False
        
        rospy.Subscriber("/netft_data", WrenchStamped, self.call_Ax_ft, queue_size=1)
        rospy.Subscriber("/ft_tool_delay", WrenchStamped, self.call_Tool_ft, queue_size=1)
        rospy.Subscriber("/W_R_Ax_delay", Float32MultiArray, self.call_Rot, queue_size=1)
        
        pub = rospy.Publisher('/delayed_ft', WrenchStamped, queue_size=1)

        rate_hz = rospy.get_param('rate_hz')
        r = rospy.Rate(rate_hz)
        
        while not rospy.is_shutdown():
            if self.send_msg_:
                pub.publish(self.pub_msg)
                self.send_msg_ = False
            r.sleep()

 
if __name__ == '__main__':
    rospy.init_node('ft_delay')
    try:
        foo = FT()
    except rospy.ROSInterruptException:  pass
