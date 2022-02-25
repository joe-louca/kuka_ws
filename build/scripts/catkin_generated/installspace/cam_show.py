#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
  
def cam1_callback(msg):
    global frame1
    bridge = CvBridge()
    frame1 = bridge.imgmsg_to_cv2(msg)
    cv2.imshow('cam1 stream', frame1)

def cam2_callback(msg):
    #global frame2
    bridge = CvBridge()
    self.frame2 = bridge.imgmsg_to_cv2(msg)


def main():   
    global frame1
    #global frame2
    
    rospy.init_node('cam_sub_node', anonymous=True)
    rospy.Subscriber('/cam1', Image, cam1_callback, queue_size=1)
    #rospy.Subscriber('/cam2', Image, self.cam2_callback, queue_size=1)

    #cv2.imshow('cam2 stream', self.frame2)
    cv2.waitKey(1)

    rospy.spin()
    
if __name__ == '__main__':
    global frame1
    #global frame2
    main()
