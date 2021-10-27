#!/usr/bin/env python

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

def ft_callback(msg):
    global delayed_ft_tbl
    global delayed_ft_msg

    # Get force-torque reading (alread in correct frame from copsim)
    FT = [msg.wrench.force.x, msg.wrench.force.y ,msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

    # Timestamp
    t = rospy.get_time()
    timestamped_ft = [FT[0], FT[1], FT[2], FT[3], FT[4], FT[5], t]
    
    # Store and retrieve delayed ft readings
    ft, retrieved, delayed_ft_tbl = add_delay(timestamped_ft, delayed_ft_tbl)

    if retrieved:
        fx = ft[0]
        fy = ft[1]
        fz = ft[2]
        tx = ft[3]
        ty = ft[4]
        tz = ft[5]
        
        rospy.set_param('ft_delay/fx', fx)
        rospy.set_param('ft_delay/fy', fy)
        rospy.set_param('ft_delay/fz', fz)
        rospy.set_param('ft_delay/tx', tx)
        rospy.set_param('ft_delay/ty', ty)
        rospy.set_param('ft_delay/tz', tz)    
                
       
def AXIA():   
    rospy.init_node('sim_ft_sub_node', anonymous=True)
    sub = rospy.Subscriber('sim_netft_data', WrenchStamped, ft_callback, queue_size=1)
    rospy.spin()        

  
if __name__ == '__main__':
    global delayed_ft_tbl
    global latency
    delayed_ft_tbl = []
    latency = rospy.get_param('latency')
    AXIA()
