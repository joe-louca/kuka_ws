#!/usr/bin/env python

import rospy
import numpy as np
import os
import cv2
import time
import signal

class CAMERAS:
    def QuitAll(self):
        os.system('stop_kuka')
        time.sleep(1)
        rospy.signal_shutdown('Quitting Programme - Shutting down ROS')
            
    def draw_text(self, img, text,
              font=cv2.FONT_HERSHEY_SIMPLEX,
              pos=(5, 5),
              font_scale=1,
              font_thickness=2,
              text_color=(0, 0, 0),
              text_color_bg=(255, 255, 255)
              ):
        x, y = pos
        text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
        text_w, text_h = text_size
        cv2.rectangle(img, pos, (x + text_w, y + text_h), text_color_bg, -1)
        cv2.putText(img, text, (x, y + text_h + font_scale - 1), font, font_scale, text_color, font_thickness)

        return text_size

    def __init__(self):
        fps = 20.0
        latency = rospy.get_param('latency')            # One way latency in seconds
        frames_to_store = int(round(latency/(1/fps),0)) # Number of frames to store based on latency and target fps
        fps = frames_to_store/latency                   # Calculate adjusted fps to have quick store
        delayed_frame_tbl = [None]*frames_to_store
        store = 0
        full_store = False

        
        
        rospy.init_node('user_cam', anonymous=True)
        rate = rospy.Rate(fps)    
        #path = '~/gui_test.mp4'
        
        trial_id = rospy.get_param('trial_id')
        pp_id = rospy.get_param('participant_id')
        if int(trial_id) == 0:
            path = '~/UserTrialData/P'+str(pp_id)+'/Baseline_taskcam.mp4'
        else:
            path =  '~/UserTrialData/P'+str(pp_id)+'/T'+str(trial_id)+'_taskcam.mp4'
        
        vid1_capture = cv2.VideoCapture('/dev/video6') #4 or 6
        vid2_capture = cv2.VideoCapture('/dev/video4')
        
        #vid1_capture = cv2.VideoCapture('/home/joe/kuka_ws/src/scripts/src/user/blink_detection_demo.mp4')
        #vid2_capture = cv2.VideoCapture('/home/joe/kuka_ws/src/scripts/src/user/blink_detection_demo.mp4')
        
        width = 640
        height = 480
        splitter_width = 50
        
        vid1_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        vid1_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        vid1_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        vid1_capture.set(cv2.CAP_PROP_FPS,fps)
        
        vid2_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        vid2_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        vid2_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        vid2_capture.set(cv2.CAP_PROP_FPS,fps)

        self.button = [0, 50, 0, width*2+splitter_width]
        button_im = np.zeros((50,width*2+splitter_width), np.uint8)
        button_im[button[0]:button[1],button[2]:button[3]] = 180 
        
        vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
        output = cv2.VideoWriter(os.path.expanduser(path), vid_cod, fps, (width*2+50,height))

        start_time = rospy.get_param('start_time')
        
        while start_time == -1:
            start_time = rospy.get_param('start_time')
            time.sleep(0.001) #wait 1ms
            
        while not rospy.is_shutdown():
            # Read the frame from the camera
            success1, frame1 = vid1_capture.read()
            success2, frame2 = vid2_capture.read()
            #frame2 = frame1
            #success2 = True

            if success1 & success2:
                timestamp = str(round(rospy.get_time() - start_time,2))
                self.draw_text(frame1, 'Camera 1')
                self.draw_text(frame2, 'Camera 2')
                self.draw_text(frame1, timestamp, pos=(5, height-40), font_thickness=1)
                
                spacer = np.zeros((height,splitter_width,3), np.uint8)
                both = np.concatenate((frame1,spacer), axis=1)   #1 : horz, 0 : Vert. 
                both = np.concatenate((both,frame2), axis=1)   #1 : horz, 0 : Vert. 

                both = np.concatenate((both,button_im), axis=0)
                
                # Store frame
                delayed_frame_tbl[store] = both
                store += 1

                if store == frames_to_store:
                    store = 0
                    full_store = True
                    
                if full_store:
                    # Retrieve frame
                    both = delayed_frame_tbl[store]
                    
                    # Write the frame to file
                    output.write(both)

                    # Display the frame
                    cv2.imshow("Camera Views", both)
                    if cv2.waitKey(50) & 0xFF == ord('q'):
                        QuitAll(self)
                        break
            else:
                print('error reading stream')

            # Sleep
            rate.sleep()

        vid1_capture.release()
        vid2_capture.release()
        output.release()
        cv2.destroyAllWindows()



if __name__ == '__main__':
    CAMERAS()

