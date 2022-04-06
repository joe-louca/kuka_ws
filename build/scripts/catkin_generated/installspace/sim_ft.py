#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped

class FT:   
    def callback(self, msg):
        self.pub_msg = msg
        self.send_msg_ = True
        return

    def __init__(self):
        self.pub_msg = TwistStamped()
        self.send_msg_ = False
        
        rospy.Subscriber("/sim_netft_data", TwistStamped, self.callback, queue_size=1)
        pub = rospy.Publisher('/F_kuka_out', TwistStamped, queue_size=1)

        rate_hz = rospy.get_param('rate_hz')
        r = rospy.Rate(rate_hz)
        
        while not rospy.is_shutdown():
            if self.send_msg_:
                pub.publish(self.pub_msg)
                self.send_msg_ = False
            r.sleep()

 
if __name__ == '__main__':
    rospy.init_node('sim_ft')
    try:
        foo = FT()
    except rospy.ROSInterruptException:  pass
