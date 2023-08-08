#!/usr/bin/env python
import rospy
import numpy as np
import os
import cv2
import time


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
        fps = 24.0
        latency = rospy.get_param('latency')              # One way latency in seconds
        frames_to_store = int(round(latency/(1/fps),0))     # Number of frames to store based on latency and target fps
        if latency != 0:
            fps = frames_to_store/latency                   # Calculate adjusted fps to have quick store
        delayed_frame_tbl = [None]*frames_to_store
        store = 0
        full_store = False

        # Set up video captures
        vid1_capture = cv2.VideoCapture('/dev/video2')
        #vid2_capture = cv2.VideoCapture('/dev/video0')
        
        width = 640
        height = 480
        spacer_width = 50
        
        vid1_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        vid1_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        vid1_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        vid1_capture.set(cv2.CAP_PROP_FPS,fps)

        #vid2_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        #vid2_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        #vid2_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        #vid2_capture.set(cv2.CAP_PROP_FPS,fps)

        # Set up video recording
        trial_id = rospy.get_param('trial_id')
        pp_id = rospy.get_param('participant_id')
        if int(trial_id) == 0:
            path = '~/UserTrialData/P'+str(pp_id)+'/Baseline_taskcam.mp4'
        else:
            path =  '~/UserTrialData/P'+str(pp_id)+'/T'+str(trial_id)+'_taskcam1.mp4'
        vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
        output = cv2.VideoWriter(os.path.expanduser(path), vid_cod, fps, (width,height))
        
        # Read frames on a loop
        t0 = rospy.get_param('start_time')
        while True:
            t_loop = time.time()
            # Read the frame from the camera
            success1, frame1 = vid1_capture.read()
            #success2, frame2 = vid2_capture.read()
            
            if success1:# & success2:
                # Draw camera labels
                self.draw_text(frame1, 'Remote - Cam 1')
                #self.draw_text(frame2, 'Camera 2')
                
                if t0 == -1:
                    # Get start time from ros
                    t0 = rospy.get_param('start_time')
                    t_elapsed = round(time.time() - t0, 2)
                else:
                    # Draw timestamp
                    t_elapsed = round(time.time() - t0, 2)
                    timestamp = str(t_elapsed)
                    #self.draw_text(frame1, timestamp, pos=(5, height-40), font_thickness=1)

                # Stitch images
                #spacer = np.zeros((height,spacer_width,3), np.uint8)
                #both = np.concatenate((frame1,spacer), axis=1)   #1 : horz, 0 : Vert. 
                #both = np.concatenate((both,frame2), axis=1)   #1 : horz, 0 : Vert.                 
                both = frame1
                # Add delay                    
                if latency != 0:
                    delayed_frame_tbl[store] = both
                    store += 1
                else:
                    full_store = True
                
                if store == frames_to_store:
                    store = 0
                    full_store = True

                if full_store:
                    if latency != 0:
                        both = delayed_frame_tbl[store]

                    if not t0 == -1:
                        # Write the frame to file
                        output.write(both)
                    
                    # Display the frame
                    cv2.imshow("Camera 1", both)
                    #if t0 != -1:
                    #    if t_elapsed > 130+latency:
                    #        self.QuitAll()
                    #        break
                    if (cv2.waitKey(50) & 0xFF == ord('q')):
                        self.QuitAll()
                        break


            # Sleep at fps rate
            fps_wait = True
            while fps_wait:
                if (time.time() - t_loop >= 1/fps):
                    fps_wait = False

        vid1_capture.release()
        #vid2_capture.release()
        output.release()
        cv2.destroyAllWindows()


#if __name__ == '__main__':
CAMERAS()

