#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

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


def callback(msg):
    global delayed_tbl
    global delayed_msg
    global checker
                
    # Timestamp
    t = rospy.get_time()
    timestamped_msg = [msg, t]

    # Store and retrieve delayed ft readings
    retrieved_line, retrieved, delayed_tbl = add_delay(timestamped_msg, delayed_tbl)

    if retrieved:
        delayed_msg = retrieved_line[0]
        checker = False


def main():
    global delayed_tbl
    global latency
    global delayed_msg
    global checker
    checker = False
    delayed_tbl = []
    delayed_msg = Float64()
    
    latency = rospy.get_param('latency')
    rate_hz = rospy.get_param('rate_hz')
    
    # Prepare the publisher and subscriber
    rospy.init_node('timestep_delay', anonymous=True)
    pub = rospy.Publisher('/delayed_timestep', Float64, queue_size=1)
    sub = rospy.Subscriber('/timestep', Float64, callback, queue_size=1)    

    r = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        if delayed_msg.data:
            pub.publish(delayed_msg)
            checker = False
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

