#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np

def add_delay(added_row, delayed_tbl):
    latency = rospy.get_param('latency')
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

    if rospy.has_param('sim_tool_ft'):
        # Get force-torque reading
        F_sensor = np.array([[msg.wrench.force.x],
                             [msg.wrench.force.y],
                             [msg.wrench.force.z]])
        #T_sensor = np.array([[msg.wrench.torque.x],
        #                    [msg.wrench.torque.y],
        #                    [msg.wrench.torque.z]])
        T_sensor = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

        # Get force due to tool mass from copsim
        tool_ft_fx = -5##rospy.get_param('sim_tool_ft/fx') #~20N downwards
        tool_ft_fy = -5#rospy.get_param('sim_tool_ft/fy')
        tool_ft_fz = -30#rospy.get_param('sim_tool_ft/fz')
        tool_ft_tx = 0#rospy.get_param('sim_tool_ft/tx')
        tool_ft_ty = 0#rospy.get_param('sim_tool_ft/ty')
        tool_ft_tz = 0#rospy.get_param('sim_tool_ft/tz')
        
        # Remove tool force
        F_sensor[0][0] -= tool_ft_fx
        F_sensor[1][0] -= tool_ft_fy
        F_sensor[2][0] -= tool_ft_fz
        T_sensor[0] -= tool_ft_tx
        T_sensor[1] -= tool_ft_ty
        T_sensor[2] -= tool_ft_tz
        
        frame_id = msg.header.seq
        frame_t = msg.header.stamp - start_time



        # Convert forces to world frame
        rot_ee2W11 = rospy.get_param('rot_W2ee/11') # from copsim
        rot_ee2W12 = rospy.get_param('rot_W2ee/12') # from copsim
        rot_ee2W13 = rospy.get_param('rot_W2ee/13') # from copsim
        rot_ee2W21 = rospy.get_param('rot_W2ee/21') # from copsim
        rot_ee2W22 = rospy.get_param('rot_W2ee/22') # from copsim
        rot_ee2W23 = rospy.get_param('rot_W2ee/23') # from copsim
        rot_ee2W31 = rospy.get_param('rot_W2ee/31') # from copsim
        rot_ee2W32 = rospy.get_param('rot_W2ee/32') # from copsim
        rot_ee2W33 = rospy.get_param('rot_W2ee/33') # from copsim
        rot_ee2W = np.array([[rot_ee2W11, rot_ee2W12, rot_ee2W13],
                             [rot_ee2W21, rot_ee2W22, rot_ee2W23],
                             [rot_ee2W31, rot_ee2W32, rot_ee2W33]])
        
        F_world = np.dot(rot_ee2W, F_sensor)
        
        #tf_W_ee = rospy.get_param('kuka_pos')   # x, y, z, Rz, Ry, Rx
        #eu_W_ee = tf_W_ee[3:]                   # Rz, Ry, Rx (yaw, pitch, roll)
        #rot_W_ee = Eu2Rot(eu_W_ee)              # Rot Matrix - world to ee
        

        


     
        
        # Timestamp
        t = rospy.get_time()
        timestamped_ft = [F_world[0][0], F_world[1][0], F_world[2][0], T_sensor[0], T_sensor[1], T_sensor[2], frame_id, frame_t, t]
        
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
            ft_tx = ft[3]#.item()
            ft_ty = ft[4]#.item()
            ft_tz = ft[5]#.item() 
            
            rospy.set_param('ft_delay/fx', ft_x)
            rospy.set_param('ft_delay/fy', ft_y) #### MOVE THIS TO HAPTION
            rospy.set_param('ft_delay/fz', ft_z)
            rospy.set_param('ft_delay/tx', ft_tx)
            rospy.set_param('ft_delay/ty', ft_ty)
            rospy.set_param('ft_delay/tz', ft_tz)    

                
       
def AXIA():
    global delayed_ft_tbl
    global latency
    global delayed_ft_msg
    global start_time

    rospy.init_node('ft_delay_node', anonymous=True)

    start_time = rospy.get_rostime()#.to_sec()
    
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
