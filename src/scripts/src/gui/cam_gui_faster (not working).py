#!/usr/bin/env python

# import the necessary packages
from threading import Thread
import cv2
import imutils
import numpy as np
import os
import time
import rospy
import signal
class WebcamVideoStream2:
    def __init__(self, src=0, fps=20.0, width=640, height=480):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_FPS,fps)
        
        (self.grabbed, self.frame) = self.stream.read()
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self
    
    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()
            
    def read(self):
        # return the frame most recently read
        return self.frame
    
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
        self.stream.release()
        
class WebcamVideoStream:
    def __init__(self, src=0, fps=20.0, width=640, height=480):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_FPS,fps)
        
        (self.grabbed, self.frame) = self.stream.read()
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self
    
    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()
            
    def read(self):
        # return the frame most recently read
        return self.frame
    
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
        self.stream.release()

class CAMERAS():
    def __init__(self):
        fps = 20.0
        width = 640
        height = 480
        splitter_width = 50

        # Start video stream threads
        cam1 = '/dev/video2'
        cam2 = '/dev/video0'
        vs1 = WebcamVideoStream(src=cam1, fps=fps, width=width, height=height).start()
        vs2 = WebcamVideoStream2(src=cam2, fps=fps, width=width, height=height).start()

        # Set up video recorder
        #trial_id = rospy.get_param('trial_id')
        #pp_id = rospy.get_param('participant_id')
        trial_id = 0
        pp_id = 0
        if int(trial_id) == 0:
            path = '~/UserTrialData/P'+str(pp_id)+'/Baseline_taskcam.mp4'
        else:
            path =  '~/UserTrialData/P'+str(pp_id)+'/T'+str(trial_id)+'_taskcam.mp4'
        vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
        output = cv2.VideoWriter(os.path.expanduser(path), vid_cod, fps, (width*2+50,height))

        # Set up latency store
        latency = rospy.get_param('latency')            # One way latency in seconds
        frames_to_store = int(round(latency/(1/fps),0)) # Number of frames to store based on latency and target fps
        if latency != 0:
            fps = frames_to_store/latency                   # Calculate adjusted fps to have quick store
        delayed_frame_tbl = [None]*frames_to_store
        store = 0
        full_store = False
        
        # Initialise ros node
        rospy.init_node('cam_gui', anonymous=True)
        rate = rospy.Rate(fps)
        time = 0

        while not rospy.is_shutdown():
            #start_time = rospy.get_param('start_time')
            start_time = 1
            
            # Get frames
            frame1 = vs1.read()
            frame2 = vs2.read()
            #frame2 = frame1

            # Resize and join into single image
            frame1 = imutils.resize(frame1, height=height, width=width)
            frame2 = imutils.resize(frame2, height=height, width=width)
            self.draw_text(frame1, 'Camera 1')
            self.draw_text(frame2, 'Camera 2')
            if not start_time == -1:
                time = round(rospy.get_time() - start_time,2)
                timestamp = str(time)
                self.draw_text(frame1, timestamp, pos=(5, height-40), font_thickness=1)
            spacer = np.zeros((height,splitter_width,3), np.uint8)
            both = np.concatenate((frame1,spacer), axis=1)   #1 : horz, 0 : Vert. 
            both = np.concatenate((both,frame2), axis=1)   #1 : horz, 0 : Vert.
            
            # Store and add delay
            if latency != 0:
                delayed_frame_tbl[store] = both
                store += 1
            else:
                full_store = True

            if store == frames_to_store:
                store = 0
                full_store = True
                
            if full_store:            
                # Retrieve frame
                if latency!= 0:
                    both = delayed_frame_tbl[store]

                # Display
                cv2.imshow("Camera Views", both)
                key = cv2.waitKey(1) &0xFF

                # Save as video
                if not start_time == -1:
                    output.write(both)
                    
                # Break if
                if time > 120:
                    self.QuitAll()
                    break
                
                if cv2.waitKey(50) & 0xFF == ord('q'):
                    self.QuitAll()
                    break
                
            rate.sleep()            

        cv2.destroyAllWindows()
        vs1.stop()

    def QuitAll(self):
        os.system('rosnode list | grep KUKA | xargs rosnode kill') #stop_kuka
        os.system('rosnode list | grep record | xargs rosnode kill') # stop recording
        time.sleep(1)

        #os.system('rosnode list | grep -v rosout | xargs rosnode kill') # kill all nodes except rosout
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
    
CAMERAS()

