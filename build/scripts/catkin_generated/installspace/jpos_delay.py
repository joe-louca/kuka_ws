#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

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
            retrieved_row = retrieved_row[:row_len]                 # Remove timestamp
            delayed_tbl = delayed_tbl[:(-tbl_len+i-1)]              # Remove old rows
            retrieved = True                                        # Update marker
            break
    
    return retrieved_row, retrieved, delayed_tbl


def jpos_callback(msg):
    global delayed_cmd_tbl
    global delayed_jpos_msg

    j1 = msg.data[0]
    j2 = msg.data[1]
    j3 = msg.data[2]
    j4 = msg.data[3]
    j5 = msg.data[4]
    j6 = msg.data[5]
    j7 = msg.data[6]
    frame = msg.data[7]
    frame_t = msg.data[8]

            
    # Timestamp
    t = rospy.get_time()
    timestamped_cmd = [j1, j2, j3, j4, j5, j6, j7, frame, frame_t, t]
    
    # Store and retrieve delayed ft readings
    jpos_cmd, retrieved, delayed_cmd_tbl = add_delay(timestamped_cmd, delayed_cmd_tbl)

    if retrieved:
        delayed_jpos_msg.data = [jpos_cmd[0], jpos_cmd[1], jpos_cmd[2], jpos_cmd[3], jpos_cmd[4], jpos_cmd[5], jpos_cmd[6], jpos_cmd[7], jpos_cmd[8]]

        # Set as ROS param
        jpos_cmd_param = [jpos_cmd[0], jpos_cmd[1], jpos_cmd[2], jpos_cmd[3], jpos_cmd[4], jpos_cmd[5], jpos_cmd[6], jpos_cmd[7], jpos_cmd[8]]
        rospy.set_param('delayed_jpos_cmd', jpos_cmd_param)


def main():
    global delayed_cmd_tbl
    global latency
    global delayed_jpos_msg
    delayed_cmd_tbl = []
    latency = rospy.get_param('latency')
    delayed_jpos_msg = Float32MultiArray()

    rate_hz = rospy.get_param('rate_hz')
    
    # Prepare the publisher and subscriber
    rospy.init_node('jpos_delay_node', anonymous=True)
    pos_pub = rospy.Publisher('/delayed_jpos_cmd', Float32MultiArray, queue_size=1)
    sub = rospy.Subscriber("jpos_cmd", Float32MultiArray, jpos_callback, queue_size=1)    

    r = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        if delayed_jpos_msg.data:
            pos_pub.publish(delayed_jpos_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

