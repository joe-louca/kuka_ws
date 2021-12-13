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
 
    R_y = np.array([[ np.cos(eulers[1]), 0, np.sin(eulers[1])],
                    [ 0,                 1, 0                ],
                    [-np.sin(eulers[1]), 0, np.cos(eulers[1])]])

    R_z = np.array([[np.cos(eulers[0]), -np.sin(eulers[0]), 0],
                    [np.sin(eulers[0]),  np.cos(eulers[0]), 0],
                    [0,                  0,                 1]])
    
    R = np.dot(R_z, np.dot(R_y, R_x) )

    return R

def ft_callback(msg):
    global delayed_ft_tbl
    global delayed_ft_msg

    # Get force-torque reading
    F_sensor = np.array([[msg.wrench.force.x],
                         [msg.wrench.force.y],
                         [msg.wrench.force.z]])
    T_sensor = np.array([[msg.wrench.torque.x],
                        [msg.wrench.torque.y],
                        [msg.wrench.torque.z]])
    frame_id = msg.header.seq
    frame_t = msg.header.stamp.to_sec() - start_time

    # Get force due to tool mass
    tool_ft_fx = rospy.get_param('sim_tool_ft/fx')
    tool_ft_fy = rospy.get_param('sim_tool_ft/fy')
    tool_ft_fz = rospy.get_param('sim_tool_ft/fz')
    tool_ft_tx = rospy.get_param('sim_tool_ft/tx')
    tool_ft_ty = rospy.get_param('sim_tool_ft/ty')
    tool_ft_tz = rospy.get_param('sim_tool_ft/tz')
    
    # Get reference frame and conver to rot matrix
    tf_W_ee = rospy.get_param('kuka_pos')   # x, y, z, Rz, Ry, Rx
    eu_W_ee = tf_W_ee[3:]                   # Rz, Ry, Rx (yaw, pitch, roll)
    rot_W_ee = Eu2Rot(eu_W_ee)              # Rot Matrix - world to ee
    
    # Convert ref frame Eulers to Rot Matrix
    F_world = np.dot(rot_W_ee, F_sensor)
    T_world = np.dot(rot_W_ee, T_sensor)

    # Remove tool force
    F_world[0][0] -= tool_ft_fx
    F_world[1][0] -= tool_ft_fy
    F_world[2][0] -= tool_ft_fz
    T_world[0][0] -= tool_ft_tx
    T_world[1][0] -= tool_ft_ty
    T_world[2][0] -= tool_ft_tz    
    
    # Timestamp
    t = rospy.get_time()
    timestamped_ft = [F_world[0][0], F_world[1][0], F_world[2][0], T_world[0][0], T_world[1][0], T_world[2][0], frame_id, frame_t, t]
    
    # Store and retrieve delayed ft readings
    ft, retrieved, delayed_ft_tbl = add_delay(timestamped_ft, delayed_ft_tbl)

    if retrieved:
        delayed_ft_msg.wrench.force.x = ft[0]
        delayed_ft_msg.wrench.force.y = ft[1]
        delayed_ft_msg.wrench.force.z = ft[2]
        delayed_ft_msg.wrench.torque.x = ft[3]
        delayed_ft_msg.wrench.torque.y = ft[4]
        delayed_ft_msg.wrench.torque.z = ft[5]
        delayed_ft_msg.header.seq = ft[6]
        delayed_ft_msg.header.stamp = ft[7].to_sec()
        #delayed_ft_msg.header.stamp = '' #string

        ft_x = ft[0].item()
        ft_y = ft[1].item()
        ft_z = ft[2].item()
        ft_tx = ft[3].item()
        ft_ty = ft[4].item()
        ft_tz = ft[5].item() 
        
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
    global start_time = rospy.get_rostime()#.to_sec()
    
    delayed_ft_tbl = []
    latency = rospy.get_param('latency')
    delayed_ft_msg = WrenchStamped()

    rate_hz = rospy.get_param('rate_hz')
    
    rospy.init_node('ft_delay_node', anonymous=True)
    ft_pub = rospy.Publisher('/delayed_ft', WrenchStamped, queue_size=1)    
    rospy.Subscriber('netft_data', WrenchStamped, ft_callback, queue_size=1)
    

    r = rospy.Rate(rate_hz)
    
    while not rospy.is_shutdown():
        if delayed_ft_msg.wrench.force.x:
            ft_pub.publish(delayed_ft_msg)
        r.sleep()

  
if __name__ == '__main__':
    AXIA()
