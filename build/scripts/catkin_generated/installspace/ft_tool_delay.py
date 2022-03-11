#!/usr/bin/env python3

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
    global delayed_ft_tbl
    global delayed_ft_msg

    # Get force-torque reading (alread in correct frame from copsim)
    tool_ft = [msg.wrench.force.x, msg.wrench.force.y ,msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
    frame_id = msg.header.seq
    frame_t = float(str(msg.header.stamp))/1000000000 - start_time
    
    # Timestamp
    t = rospy.get_time()
    timestamped_ft = [tool_ft[0], tool_ft[1], tool_ft[2], tool_ft[3], tool_ft[4], tool_ft[5], frame_id, frame_t, t]
    
    # Store and retrieve delayed ft readings
    ft, retrieved, delayed_ft_tbl = add_delay(timestamped_ft, delayed_ft_tbl)

    if retrieved:
        fx = ft[0]
        fy = ft[1]
        fz = ft[2]
        tx = ft[3]
        ty = ft[4]
        tz = ft[5]

        delayed_ft_msg.wrench.force.x = fx
        delayed_ft_msg.wrench.force.y = fy
        delayed_ft_msg.wrench.force.z = fz
        delayed_ft_msg.wrench.torque.x = tx
        delayed_ft_msg.wrench.torque.y = ty
        delayed_ft_msg.wrench.torque.z = tz
        delayed_ft_msg.header.seq = '/foo'
        delayed_ft_msg.header.stamp = rospy.get_rostime()#.to_sec()
        
        rospy.set_param('ft_delay/fx', fx)
        rospy.set_param('ft_delay/fy', fy)
        rospy.set_param('ft_delay/fz', fz)
        rospy.set_param('ft_delay/tx', tx)
        rospy.set_param('ft_delay/ty', ty)
        rospy.set_param('ft_delay/tz', tz)    
                
       
def TOOL(): # Adds delay to align tool forces with current kuka frame
    global delayed_ft_tbl
    global latency
    global delayed_ft_msg
    global start_time
    
    delayed_ft_tbl = []
    
    rospy.init_node('tool_ft_delay', anonymous=True)
    latency = rospy.get_param('latency')
    rate_hz = rospy.get_param('rate_hz')
    start_time = rospy.get_param('start_time')  # float secs

    delayed_ft_msg = WrenchStamped()

    delay_pub = rospy.Publisher('/ft_tool_delay', WrenchStamped, queue_size=1)    
    rospy.Subscriber('/ft_tool', WrenchStamped, ft_callback, queue_size=1)

    r = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        if delayed_ft_msg.wrench.force.x:
            delay_pub.publish(delayed_ft_msg)
        r.sleep()
  
if __name__ == '__main__':
    TOOL()
