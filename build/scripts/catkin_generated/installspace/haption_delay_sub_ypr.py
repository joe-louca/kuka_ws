#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray



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
            retrieved_row = retrieved_row[:row_len]                 # Remove timestamp
            delayed_tbl = delayed_tbl[:(-tbl_len+i-1)]              # Remove old rows
            retrieved = True                                        # Update marker
            break
    
    return retrieved_row, retrieved, delayed_tbl


def haption_callback(msg):
    global delayed_hap_cmd_tbl
    global delayed_pos_msg
    
    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]
    a = msg.data[3]
    b = msg.data[4]
    c = msg.data[5]
            
    # Timestamp
    t = rospy.get_time()
    timestamped_cmd = [x, y, z, a, b, c, t]

    # Store and retrieve delayed ft readings
    hap_cmd, retrieved, delayed_hap_cmd_tbl = add_delay(timestamped_cmd, delayed_hap_cmd_tbl)

    if retrieved:
        delayed_pos_msg.data = [hap_cmd[0], hap_cmd[1], hap_cmd[2], hap_cmd[3], hap_cmd[4], hap_cmd[5]]
        hap_param = [hap_cmd[0], hap_cmd[1], hap_cmd[2], hap_cmd[3], hap_cmd[4], hap_cmd[5]]

        rospy.set_param('delayed_pos_cmd_ypr', hap_param)


def main():
    global delayed_hap_cmd_tbl
    global latency
    global delayed_pos_msg
    delayed_hap_cmd_tbl = []
    latency = rospy.get_param('latency')
    delayed_pos_msg = Float32MultiArray()

    rate_hz = rospy.get_param('rate_hz')
    
    rospy.init_node('hap_delay_sub_node', anonymous=True)
    kuka_pos_pub = rospy.Publisher('/delayed_kuka_pos_cmd', Float32MultiArray, queue_size=1)
    
    sub = rospy.Subscriber("hap_move_pub", Float32MultiArray, haption_callback, queue_size=1)    

    r = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        if delayed_pos_msg.data:
            kuka_pos_pub.publish(delayed_pos_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

