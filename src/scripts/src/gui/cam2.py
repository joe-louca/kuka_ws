#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CAMERA:   
    def add_delay(added_row, delayed_tbl):
        latency = rospy.get_param('latency')
        delayed_tbl.append(added_row)                                   # Add new row to end of list (newest at bottom)
        row_len = len(added_row)-1                                      # Length of added row
        tbl_len = len(delayed_tbl)                                      # Number of rows in of table                        

        retrieved_row = []
        retrieved = False
        elapsed_time = rospy.get_time()
        
        for i in range(tbl_len):                                        # Starting at the oldest, For each row   
            if elapsed_time > delayed_tbl[i][row_len] + latency:        # If row is old enough
                retrieved_row = delayed_tbl[i][:row_len]                # Get this row and remove timestamp
                delayed_tbl = delayed_tbl[i+1:]                         # Keep only new rows (all rows at this point and below)            
                retrieved = True                                        # Update marker
                break
        
        return retrieved_row, retrieved, delayed_tbl


    def __init__(self):
        # Get rate and latency parameters
        #rate_hz = rospy.get_param('rate_hz')
        rate_hz = 35 # fps
        self.latency = rospy.get_param('latency')

        # Set up frame storage lists
        delayed_frame_tbl = []

        # Connect to camera
        cam_address = 'http://192.168.132.60:4747/video'
        cap = cv2.VideoCapture(cam_address)

        # Initialise CV to ros image converter
        bridge = CvBridge()

        # Initialise publisher
        rospy.init_node('cam2_node', anonymous=True)
        pub = rospy.Publisher("cameras/cam2", Image,queue_size=1)
        
        rate = rospy.Rate(rate_hz)

        while not rospy.is_shutdown():
            # Get image
            ret, frame = cap.read() # 640x480 size
            height, width, layers = frame.shape
            new_h = int(height / 2)
            new_w = int(width / 2)
            resize = cv2.resize(frame, (new_w, new_h))
            # Store with timestamp
            t = rospy.get_time()
            timestamped_frame = [resize, t]
            
            frame, retrieved, delayed_frame_tbl = self.add_delay(timestamped_frame, delayed_frame_tbl)
            
            if retrieved:
                # Convert to image and publish delayed frame
                f = frame[0]
                pub.publish(bridge.cv2_to_imgmsg(f, "bgr8"))
                
            rate.sleep()

        cap.release() 


        
if __name__ == '__main__':
    CAMERA()
 
