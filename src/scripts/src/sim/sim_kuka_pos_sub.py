#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def kuka_pos_callback(msg):
    kuka_pos = [msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]]
    rospy.set_param('kuka_pos', kuka_pos)
    
def main():
    rospy.init_node('sim_kuka_pos_node', anonymous=True)
    rospy.Subscriber("sim_kuka_pos", Float32MultiArray, kuka_pos_callback, queue_size=1)
    rospy.spin()
    

if __name__ == '__main__':
    main()


