#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time

def main():
    msg = Float64()
    
    # Prepare the publisher and subscriber
    rospy.init_node('timestep', anonymous=True)
    pub = rospy.Publisher('/timestep', Float64, queue_size=1)
    rate_hz = rospy.get_param('rate_hz')
    r = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        msg.data = time.time() # seconds to microsecond precision
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    main()
