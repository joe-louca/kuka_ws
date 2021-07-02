#!/usr/bin/env python3

import rospy
import numpy as np
from math import sqrt
from std_msgs.msg import Float32MultiArray

# Calculates rotation matrix to euler angles
def Rot2Eu(R) :
    sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = np.arctan2( R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2( R[1,0], R[0,0])
    else :
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def Eu2Rot(eulers) :
    R_x = np.array([[1, 0,               0                ],
                    [0, cos(eulers[0]), -np.sin(eulers[0])],
                    [0, sin(eulers[0]),  np.cos(eulers[0])]])
 
    R_y = np,array([[ np.cos(eulers[1]), 0, np.sin(eulers[1])],
                    [ 0,                 1, 0            ],
                    [-np.sin(eulers[1]), 0, np.cos(eulers[1])]])

    R_z = np.array([[cos(eulers[2]), -sin(eulers[2]), 0],
                    [sin(eulers[2]),  cos(eulers[2]), 0],
                    [0,              0,             1]])
    
    R = np.dot(R_z, np.dot(R_y, R_x) )

    return R


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


def tf_callback(msg):
    global delayed_ft_tbl

    # Build Matrix from array
    R_W2Ax = np.array([[msg.data[0], msg.data[1], msg.data[2], msg.data[3] ],
                       [msg.data[4], msg.data[5], msg.data[6], msg.data[7] ],
                       [msg.data[7], msg.data[8], msg.data[9], msg.data[10]],
                       [0,           0,           0,           1           ]])


    # Get netft_data
    fx = rospy.get_param('netft_data/fx')
    fy = rospy.get_param('netft_data/fy')
    fz = rospy.get_param('netft_data/fz')
    tx = rospy.get_param('netft_data/tx')
    ty = rospy.get_param('netft_data/ty')
    tz = rospy.get_param('netft_data/tz')
    
    # Convert torque values to Rot
    Force_SensorFrame = np.array([[fx],
                                  [fy],
                                  [fz]])
    Torque_SensorFrame = Eu2Rot([tx, ty, tz])
    
    # Transform to world frame
    Force_WorldFrame = np.dot(R_W2Sensor, FORCE_SensorFrame) 
    TorqueR_WorldFrame = np.dot(R_W2Ax, Torque_SensorFrame)
    
    # Convert Torque back to angles to angles
    Torque_WorldFrame = Rot2Eu(TorqueR_WorldFrame)

    # Build list and timestampfor storage
    fx = Force_WorldFrame[0][0]
    fy = Force_WorldFrame[1][0]
    fz = Force_WorldFrame[2][0]
    tx = Torque_WorldFrame[0]
    ty = Torque_WorldFrame[1]
    tz = Torque_WorldFrame[2]
    t = rospy.get_time()
    timestamped_ft = [fx, fy, fz, tx, ty, tz, t]
     
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
    rospy.init_node('tf_sub_node', anonymous=True)
    rospy.Subscriber('delayed_axia_tf', Float32MultiArray, tf_callback, queue_size=1)
    rospy.spin()

  
if __name__ == '__main__':
    global delayed_ft_tbl
    global latency
    delayed_ft_tbl = []
    latency = rospy.get_param('latency')
    AXIA()
