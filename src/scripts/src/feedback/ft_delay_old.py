#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np

def add_delay(added_row, delayed_tbl):
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


def ft_callback(msg):
    # Tool compensation equations taken from "Learning optimal variable admittance control for rotational motion in human-robot co-manipulation (2015)"
    
    global delayed_ft_tbl
    global delayed_ft_msg
    global biased
    global F_bias
    global T_bias

    factor = 0.5
    if rospy.has_param('rot_W2ee/11'):

        # Get W_Rot_ee of ax w.r.t world --- NEED TO ADD A DELAY TO THIS
        W_R_Ax_11 = rospy.get_param('rot_W2ee/11') # from copsim
        W_R_Ax_12 = rospy.get_param('rot_W2ee/12') # from copsim
        W_R_Ax_13 = rospy.get_param('rot_W2ee/13') # from copsim
        W_R_Ax_21 = rospy.get_param('rot_W2ee/21') # from copsim
        W_R_Ax_22 = rospy.get_param('rot_W2ee/22') # from copsim
        W_R_Ax_23 = rospy.get_param('rot_W2ee/23') # from copsim
        W_R_Ax_31 = rospy.get_param('rot_W2ee/31') # from copsim
        W_R_Ax_32 = rospy.get_param('rot_W2ee/32') # from copsim
        W_R_Ax_33 = rospy.get_param('rot_W2ee/33') # from copsim
        W_R_Ax = np.array([[W_R_Ax_11, W_R_Ax_12, W_R_Ax_13],
                            [W_R_Ax_21, W_R_Ax_22, W_R_Ax_23],
                            [W_R_Ax_31, W_R_Ax_32, W_R_Ax_33]])

        
        # Get the inverse rotation matrix
        Ax_R_W = np.linalg.inv(W_R_Ax)


        # Check if axia bias reading has been retrieved
        if not biased:
            Ax_F_sensor = np.array([[msg.wrench.force.x*factor],
                                    [msg.wrench.force.y*factor],
                                    [msg.wrench.force.z*factor]])
            Ax_T_sensor = np.array([[msg.wrench.torque.x],
                                    [msg.wrench.torque.y],
                                    [msg.wrench.torque.z]])

            F_bias = np.dot(W_R_Ax, Ax_F_sensor)
            T_bias = np.dot(W_R_Ax, Ax_T_sensor)

            biased = True

        if biased:
            # Get force-torque reading from axia
            Ax_F_sensor = np.array([[msg.wrench.force.x*factor],
                                     [msg.wrench.force.y*factor],
                                     [msg.wrench.force.z*factor]])
            Ax_T_sensor = np.array([[msg.wrench.torque.x],
                                    [msg.wrench.torque.y],
                                    [msg.wrench.torque.z]])

            frame_id = msg.header.seq
            frame_t = float(str(msg.header.stamp))/1000000000 - start_time


            # Convert Sensor readings to world frame
            W_F_sensor = np.dot(W_R_Ax, Ax_F_sensor)
            W_T_sensor = np.dot(W_R_Ax, Ax_T_sensor)
            
            """
            # Remove force and torquebias from axia readings
            W_F_sensor = W_F_sensor - F_bias
            W_T_sensor = W_T_sensor - T_bias
            W_T_sensor = np.array([[0],[0],[0]])
            """

            """
            # Calculate force due to tool.
            W_F_tool = np.array([[0],
                                 [0],
                                 [mass_tool*-9.8]]) # Tool always provides a downwards force of mass * gravity
            
            # Calculate torque due to tool (= mass * gravity * distance from axis in XY plane)
            W_P_tool = np.dot(W_R_Ax, Ax_P_tool)
            px = W_P_tool[0]
            py = W_P_tool[1]
            pz = W_P_tool[2]
            W_T_Tool = np.array([[py*mass_tool*-9.8],
                                 [px*mass_tool*-9.8],
                                 [0.0                  ]]) # Never any torque from the tool around the z axis as the gravity force is parallel to z

            # Remove force and torque due to tool     
            W_F_sensor -= W_F_tool
            W_T_sensor -= W_T_tool
            """

            # Timestamp
            t = rospy.get_time()
            timestamped_ft = [W_F_sensor[0][0], W_F_sensor[1][0], W_F_sensor[2][0], W_T_sensor[0][0], W_T_sensor[1][0], W_T_sensor[2][0], frame_id, frame_t, t]

            # Store and retrieve delayed ft readings
            ft, retrieved, delayed_ft_tbl = add_delay(timestamped_ft, delayed_ft_tbl)

            if retrieved:
                # Get delayed force to apply to haption
                delayed_ft_msg.wrench.force.x = ft[0]
                delayed_ft_msg.wrench.force.y = ft[1]
                delayed_ft_msg.wrench.force.z = ft[2]
                delayed_ft_msg.wrench.torque.x = ft[3]
                delayed_ft_msg.wrench.torque.y = ft[4]
                delayed_ft_msg.wrench.torque.z = ft[5]
                delayed_ft_msg.header.seq = ft[6]
                delayed_ft_msg.header.stamp = ft[7]#.to_sec()

                # Convert to python floats
                ft_x = ft[0].item()
                ft_y = ft[1].item()
                ft_z = ft[2].item()
                ft_tx = ft[3].item()
                ft_ty = ft[4].item()
                ft_tz = ft[5].item() 

                # Send to haption.cpp
                rospy.set_param('ft_delay/fx', ft_x)
                rospy.set_param('ft_delay/fy', ft_y)
                rospy.set_param('ft_delay/fz', ft_z)
                rospy.set_param('ft_delay/tx', ft_tx)
                rospy.set_param('ft_delay/ty', ft_ty)
                rospy.set_param('ft_delay/tz', ft_tz) 
        

       
def AXIA():
    global delayed_ft_tbl
    global latency
    global delayed_ft_msg
    global start_time
    global mass_tool
    global p_com
    global biased
    global F_bias
    global T_bias

    biased = False
    F_bias = np.array([[0.0],[0.0],[0.0]])
    T_bias = np.array([[0.0],[0.0],[0.0]])

    mass_tool = 1.0 #kg             mass of tool
    Ax_P_tool = np.array([[0.00],
                          [0.00],
                          [0.08]]) #m   

    rospy.init_node('ft_delay_node', anonymous=True)

    #start_time = rospy.get_rostime()#.to_sec()
    start_time = rospy.get_param('start_time')  # float secs
    
    delayed_ft_tbl = []
    latency = rospy.get_param('latency')
    delayed_ft_msg = WrenchStamped()

    rate_hz = rospy.get_param('rate_hz')
    
    ft_pub = rospy.Publisher('/delayed_ft', WrenchStamped, queue_size=1)    
    rospy.Subscriber('netft_data', WrenchStamped, ft_callback, queue_size=1)
    

    r = rospy.Rate(rate_hz)
    
    while not rospy.is_shutdown():
        if delayed_ft_msg.wrench.force.x:
            ft_pub.publish(delayed_ft_msg)
        r.sleep()

  
if __name__ == '__main__':
    AXIA()
