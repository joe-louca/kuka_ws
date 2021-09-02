#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray

pub_msg = Float32MultiArray()

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

def haption_callback(msg):
    global delayed_hap_cmd_tbl
    x = msg.twist.linear.x
    y = msg.twist.linear.y
    z = msg.twist.linear.z
    roll = msg.twist.angular.x
    pitch = msg.twist.angular.y
    yaw = msg.twist.angular.z
    gripper_control = msg.header.frame_id

    if gripper_control == 'y':
        gripper_control = 1
    else:
        gripper_control = 0
            
    # Timestamp
    t = rospy.get_time()
    timestamped_cmd = [x, y, z, roll, pitch, yaw, gripper_control, t]

    # Store and retrieve delayed ft readings
    hap_cmd, retrieved, delayed_hap_cmd_tbl = add_delay(timestamped_cmd, delayed_hap_cmd_tbl)

    if retrieved:
        Float32MultiArray.data = [hap_cmd[0], hap_cmd[1], hap_cmd[2], hap_cmd[3], hap_cmd[4], hap_cmd[5]]
        hap_param = [hap_cmd[0], hap_cmd[1], hap_cmd[2], hap_cmd[3], hap_cmd[4], hap_cmd[5]]
        rospy.set_param('delayed_pos_cmd', hap_param)
        #rospy.set_param('delayed_sim_pos_cmd', hap_param)
        rospy.set_param('delayed_gripper_cmd', hap_cmd[6])

def main():
    global delayed_hap_cmd_tbl
    global latency
    delayed_hap_cmd_tbl = []
    latency = rospy.get_param('latency')
    rate_hz = rospy.get_param('rate_hz')
    
    rospy.init_node('hap_delay_sub_node', anonymous=True)
    pub = rospy.Publisher('/delayed_sim_pos_cmd', Float32MultiArray, queue_size=1)
    sub = rospy.Subscriber("hap_move_pub", TwistStamped, haption_callback, queue_size=1)    
    r = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        pub.publish(pub_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    
'''


def haption_callback(msg):
    global delayed_hap_cmd_tbl
    x = msg.twist.linear.x
    y = msg.twist.linear.y
    z = msg.twist.linear.z
    roll = msg.twist.angular.x
    pitch = msg.twist.angular.y
    yaw = msg.twist.angular.z
    gripper_control = msg.header.frame_id

    if gripper_control == 'y':
        gripper_control = 1
    else:
        gripper_control = 0
            
    # Timestamp
    t = rospy.get_time()
    timestamped_cmd = [x, y, z, roll, pitch, yaw, gripper_control, t]

    # Store and retrieve delayed ft readings
    hap_cmd, retrieved, delayed_hap_cmd_tbl = add_delay(timestamped_cmd, delayed_hap_cmd_tbl)

    if retrieved:
        hap_param = [hap_cmd[0], hap_cmd[1], hap_cmd[2], hap_cmd[3], hap_cmd[4], hap_cmd[5]]
        rospy.set_param('delayed_pos_cmd', hap_param)
        rospy.set_param('delayed_sim_pos_cmd', hap_param)
        rospy.set_param('delayed_gripper_cmd', hap_cmd[6])   
                        
            
def main():
    rospy.init_node('hap_delay_sub_node', anonymous=True)
    rospy.Subscriber("hap_move_pub", TwistStamped, haption_callback, queue_size=1)
    rospy.spin()
    

if __name__ == '__main__':
    global delayed_hap_cmd_tbl
    global latency
    delayed_hap_cmd_tbl = []
    latency = rospy.get_param('latency')
    main()
'''

