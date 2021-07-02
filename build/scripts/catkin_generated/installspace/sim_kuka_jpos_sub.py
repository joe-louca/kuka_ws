#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

def kuka_jpos_callback(msg):
    kuka_jpos = [msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6]]
    rospy.set_param('kuka_jpos', kuka_jpos)
    
def main():
    rospy.init_node('sim_kuka_jpos_node', anonymous=True)
    rospy.Subscriber("sim_kuka_jpos", Float32MultiArray, kuka_jpos_callback, queue_size=1)
    rospy.spin()
    

if __name__ == '__main__':
    main()


