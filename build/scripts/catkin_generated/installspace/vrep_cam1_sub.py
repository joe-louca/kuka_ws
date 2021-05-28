#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def cam_callback(msg):
    t = rospy.get_time()    
    br = CvBridge()
    frame = br.imgmsg_to_cv2(msg)
    frame = cv2.flip(frame, 0)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    if frame_retrieved:
        cv2.imshow('cam1 stream', frame)
        cv2.waitKey(1)
    
def main():   
    # Subscribes to /tf and saves position and rotation arrays to global config variables 
    rospy.init_node('cam1_sub_node', anonymous=True)
    rospy.Subscriber('cam1', Image, cam_callback, queue_size=1)
    rospy.spin()
    
if __name__ == '__main__':
    global delayed_frames
    global delayed_ts
    global latency
    delayed_frames = []
    delayed_ts = []
    latency = rospy.get_param('latency')
    main()
