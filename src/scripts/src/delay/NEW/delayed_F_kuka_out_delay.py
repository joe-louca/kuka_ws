#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped

class DELAY():
    def callback(self, msg):
        self.checker = True
        self.callback_msg = msg

    def __init__(self):       
        # Prepare the publisher and subscriber
        rospy.init_node('delayed_F_kuka_delay', anonymous=True)
        pub = rospy.Publisher('/delayed_delayed_F_kuka_out', TwistStamped, queue_size=1)
        sub = rospy.Subscriber('/delayed_F_kuka_out', TwistStamped, self.callback, queue_size=1)    

        # Prepare messages
        pub_msg = TwistStamped()
        self.callback_msg = TwistStamped()
        self.checker = False

        # Get latency and refresh rate
        latency = rospy.get_param('latency')
        rate_hz = rospy.get_param('rate_hz')

        # Prepare stored messages table based on rate & latency
        msgs_to_store = int(round(latency/(1.0/float(rate_hz)),0))
        rate_hz = frames_to_store/latency
        delayed_msg_tbl = [None]*msgs_to_store
        store = 0
        full_store = False

        # Set refresh rate
        r = rospy.Rate(rate_hz)

        # Stop looping if ROS shuts down
        while not rospy.is_shutdown():
            # If message has been heard
            if self.checker:
                delayed_msg_tbl[store] = self.callback_msg
                store += 1

                # Check if store is full
                if store == msgs_to_store:
                    store = 0
                    full_store = True

                # If msg deck is full, start retrieving messages
                if full_store:
                    pub_msg = delayed_msg_tbl[store]
                    pub.publish(pub_msg)
                    
            r.sleep()

if __name__ == '__main__':
    DELAY()

