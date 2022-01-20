import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CAMERA:   
    def add_delay(self, added_row, delayed_tbl):
        delayed_tbl.insert(0, added_row)                                # Add new row to table
        row_len = len(added_row)-1
        tbl_len = len(delayed_tbl)                              
        retrieved_row = []
        retrieved = False
        elapsed_time = rospy.get_time()
        
        for i in range(tbl_len):                                        # For each row
            if elapsed_time > delayed_tbl[i][row_len] + self.latency:   # If row is old enough
                retrieved_row = delayed_tbl[i]                          # Get this row
                retrieved_row = retrieved_row[:(row_len)]               # Remove timestamp
                delayed_tbl = delayed_tbl[:(-tbl_len+i-1)]              # Remove old rows
                retrieved = True                                        # Update marker
                break
        
        return retrieved_row, retrieved, delayed_tbl


    def __init__(self):
        # Get rate and latency parameters
        rate_hz = rospy.get_param('rate_hz')
        self.latency = rospy.get_param('latency')

        # Set up frame storage lists
        delayed_frame_tbl = []

        # Connect to camera
        cam_address = 'http://192.168.0.150:4747/video'
        cap = cv2.VideoCapture(cam_address)

        # Initialise CV to ros image converter
        bridge = CvBridge()

        # Initialise publisher
        rospy.init_node('cam1_node', anonymous=True)
        pub = rospy.Publisher("cam1", Image,queue_size=1)
        
        rate = rospy.Rate(rate_hz)

        while not rospy.is_shutdown():
            # Get image
            ret, frame = cap.read()

            # Store with timestamp
            t = rospy.get_time()
            timestamped_frame = [frame, t]
            
            frame, retrieved, delayed_frame_tbl = self.add_delay(timestamped_frame, delayed_frame_tbl)
            
            if retrieved:
                # Convert to image and publish delayed frame
                f = frame[0]
                pub.publish(bridge.cv2_to_imgmsg(f, "bgr8"))
                
            rate.sleep()

        
if __name__ == '__main__':
    CAMERA()
 
