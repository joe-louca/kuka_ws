#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np

def add_delay(added_row, delayed_tbl):
    delayed_tbl.insert(0, added_row)                                # Add new row to table
    row_len = len(added_row)-1
    tbl_len = len(delayed_tbl)                              
    retrieved_row = []
    retrieved = False
    elapsed_time = rospy.get_time()
    
    for i in range(tbl_len):                                        # For each row
        if elapsed_time > delayed_tbl[i][row_len] + latency:        # If row is old enough
            retrieved_row = delayed_tbl[i]                          # Get this row
            retrieved_row = retrieved_row[:row_len]             # Remove timestamp
            delayed_tbl = delayed_tbl[:(-tbl_len+i-1)]              # Remove old rows
            retrieved = True                                        # Update marker
            break
    
    return retrieved_row, retrieved, delayed_tbl

def Eu2Rot(eulers) :
    R_x = np.array([[1, 0,                  0                ],
                    [0, np.cos(eulers[2]), -np.sin(eulers[2])],
                    [0, np.sin(eulers[2]),  np.cos(eulers[2])]])
 
    R_y = np,array([[ np.cos(eulers[1]), 0, np.sin(eulers[1])],
                    [ 0,                 1, 0                ],
                    [-np.sin(eulers[1]), 0, np.cos(eulers[1])]])

    R_z = np.array([[np.cos(eulers[0]), -np.sin(eulers[0]), 0],
                    [np.sin(eulers[0]),  np.cos(eulers[0]), 0],
                    [0,                  0,                 1]])
    
    R = np.dot(R_z, np.dot(R_y, R_x) )

    return R

def ft_callback(msg):
    global delayed_ft_tbl

    # Get force-torque reading
    F_sensor = np.array([[msg.wrench.force.x],
                         [msg.wrench.force.y],
                         [msg.wrench.force.z]])
    T_sensor = np.array([msg.wrench.torque.x],
                        [msg.wrench.torque.y],
                        [msg.wrench.torque.z]])
    
    # Get reference frame and conver to rot matrix
    tf_W_ee = rospy.get_param('kuka_pos')   # x, y, z, Rz, Ry, Rx
    eu_W_ee = tf_W_ee[3:]                   # Rz, Ry, Rx (yaw, pitch, roll)
    rot_W_ee = Eu2Rot(eu_W_ee)              # Rot Matrix - world to ee
    
    # Convert ref frame Eulers to Rot Matrix
    F_world = np.dot(rot_W_ee, F_sensor)
    T_world = np.dot(rot_W_ee, T_sensor)

    # Timestamp
    t = rospy.get_time()
    ft = [F_world[0][0], F_world[1][0], F_world[2][0], T_world[0][0], T_world[1][0], T_world[2][0], t]
    
    # Store and retrieve delayed ft readings
    ft, retrieved, delayed_ft_tbl = add_delay(timestamped_ft, delayed_ft_tbl)

    if retrieved:
        rospy.set_param('ft_delay/fx', ft[0])
        rospy.set_param('ft_delay/fy', ft[1])
        rospy.set_param('ft_delay/fz', ft[2])
        rospy.set_param('ft_delay/tx', ft[3])
        rospy.set_param('ft_delay/ty', ft[4])
        rospy.set_param('ft_delay/tz', ft[5])    

                
       
def AXIA():   
    rospy.init_node('sim_ft_sub_node', anonymous=True)
    rospy.Subscriber('sim_netft_data', WrenchStamped, ft_callback, queue_size=1)
    rospy.spin()

  
if __name__ == '__main__':
    global delayed_ft_tbl
    global latency
    delayed_ft_tbl = []
    latency = rospy.get_param('latency')
    AXIA()
