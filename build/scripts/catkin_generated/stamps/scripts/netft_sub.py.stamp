#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped

def ft_callback(msg):
    ft = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
    rospy.set_param('netft_data/fx', ft[0])
    rospy.set_param('netft_data/fy', ft[1])
    rospy.set_param('netft_data/fz', ft[2])
    rospy.set_param('netft_data/tx', ft[3])
    rospy.set_param('netft_data/ty', ft[4])
    rospy.set_param('netft_data/tz', ft[5])
                
       
def AXIA():   
    rospy.init_node('ft_sub_node', anonymous=True)
    rospy.Subscriber('netft_data', WrenchStamped, ft_callback, queue_size=1)
    rospy.spin()

  
if __name__ == '__main__':
    AXIA()
