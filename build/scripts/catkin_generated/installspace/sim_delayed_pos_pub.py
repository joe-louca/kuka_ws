#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

def main():
    rate_hz = rospy.get_param('rate_hz')
    pub = rospy.Publisher('delayed_pos_cmd', Float32MultiArray, queue_size=1)
    rospy.init_node('delayed_pos_cmd_node', anonymous=True)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        try:
            delayed_pos_cmd = rospy.get_param('delayed_pos_cmd')
            msg = Float32MultiArray()
            msg.data = delayed_pos_cmd
            pub.publish(msg)
        except:
            pass
        
        rate.sleep()
        
if __name__ == '__main__':
    main()