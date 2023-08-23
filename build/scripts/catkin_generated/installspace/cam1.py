#!/usr/bin/env python3

import rospy
import numpy as np
import os
import cv2
import time
import signal
import gc


class CAMERAS:
    def QuitAll(self):
        os.system('rosnode list | grep KUKA | xargs rosnode kill') #stop_kuka
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
        if latency != 0:
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
            path = '~/UserTrialData/P'+str(pp_id)+'/Baseline_taskcam1.mp4'
        else:
            path =  '~/UserTrialData/P'+str(pp_id)+'/T'+str(trial_id)+'_taskcam1.mp4'
        
        vid1_capture = cv2.VideoCapture('/dev/video2')
        assert vid1_capture.isOpened()
        
        width = 640
        height = 480
        splitter_width = 50
        
        vid1_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        vid1_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        vid1_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        vid1_capture.set(cv2.CAP_PROP_FPS,fps)
        
        vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
        output = cv2.VideoWriter(os.path.expanduser(path), vid_cod, fps, (width,height))

        start_time = rospy.get_param('start_time')
        
        #while start_time == -1:
        #    start_time = rospy.get_param('start_time')
        #    time.sleep(0.001) #wait 1ms

        #print('next')
        time = 0
        while not rospy.is_shutdown():
            start_time = rospy.get_param('start_time')
            # Read the frame from the camera
            success1, frame1 = vid1_capture.read()

            if success1:
                self.draw_text(frame1, 'Camera 1')
                    
                if not start_time == -1:
                    time = round(rospy.get_time() - start_time,2)
                    timestamp = str(time)
                    self.draw_text(frame1, timestamp, pos=(5, height-40), font_thickness=1)
                
                # Store frame
                if latency != 0:
                    delayed_frame_tbl[store] = frame1
                    store += 1
                else:
                    full_store = True

                if store == frames_to_store:
                    store = 0
                    full_store = True
                    
                if full_store:
                    # Retrieve frame
                    if latency != 0:
                        frame1 = delayed_frame_tbl[store]
                    
                    # Write the frame to file
                    if not start_time == -1:
                        output.write(frame1)

                    # Display the frame
                    cv2.imshow("Camera Views", frame1)
                    if time > 120:
                        self.QuitAll()
                        break
                    if cv2.waitKey(50) & 0xFF == ord('q'):
                        self.QuitAll()
                        break
            else:
                print('error reading stream')

            # Sleep
            rate.sleep()


        vid1_capture.release()
        output.release()
        cv2.destroyAllWindows()
        del delayed_frame_tbl
        gc.collect()


if __name__ == '__main__':
    CAMERAS()

