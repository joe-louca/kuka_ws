#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
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

def rot_callback(msg):
    global delayed_tbl
    global delayed_msg

    rot = msg.data
    
    # Timestamp
    t = rospy.get_time()
    timestamped_rot = [rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8], t]
    
    # Store and retrieve delayed msg
    rot, retrieved, delayed_tbl = add_delay(timestamped_rot, delayed_tbl)

    if retrieved:
        delayed_msg.data = rot
                
       
def ROT(): # Adds delay to align rot matrix with current kuka frame
    global delayed_tbl
    global latency
    global delayed_msg
    
    delayed_tbl = []
    
    rospy.init_node('Rot_delay', anonymous=True)
    latency = rospy.get_param('latency')
    rate_hz = rospy.get_param('rate_hz')

    delayed_msg = Float32MultiArray()

    delay_pub = rospy.Publisher('/W_R_Ax_delay', Float32MultiArray, queue_size=1)    
    rospy.Subscriber("/W_R_Ax", Float32MultiArray, rot_callback, queue_size=1)

    r = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        if delayed_msg.data:
            delay_pub.publish(delayed_msg)
        r.sleep()
  
if __name__ == '__main__':
    ROT()
