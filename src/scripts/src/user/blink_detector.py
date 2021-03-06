#!/usr/bin/env python

import rospy
import os

# import the necessary packages
from scipy.spatial import distance as dist
from imutils.video import FileVideoStream
from imutils.video import VideoStream
from imutils import face_utils
import numpy as np
import argparse
import imutils
import time
import dlib
import cv2
import csv

def eye_aspect_ratio(eye):
    # compute the euclidean distances between the two sets of
    # vertical eye landmarks (x, y)-coordinates
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])

    # compute the euclidean distance between the horizontal
    # eye landmark (x, y)-coordinates
    C = dist.euclidean(eye[0], eye[3])

    # compute the eye aspect ratio
    ear = (A + B) / (2.0 * C)

    # return the eye aspect ratio
    return ear
 
def BLINK():
    # Create txt data file
    f = open(os.path.expanduser('~/kuka_ws/data/Blinks_record.txt'), 'w')

    EYE_AR_THRESH = 0.28            # Eye aspect ratio - blink threshold. lower = more closed
    EYE_AR_CONSEC_FRAMES = 3        # Consectutive frames for a blink
    COUNTER = 0

    detector = dlib.get_frontal_face_detector()     # initialise dlib's face detector (HOG-based)
    predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat") # create facial landmark predictor

    # Grab the indexes of the facial landmarks for the left and right eye from face_utils
    (lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
    (rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

    # start the video stream thread
    #vs = FileVideoStream("blink_detection_demo.mp4").start()        # for test mp4 vid
    vs = cv2.VideoCapture("blink_detection_demo.mp4")
    fps = vs.get(cv2.CAP_PROP_FPS)
    success, image = vs.read()
    scale_percent = 40
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)

    count = 0
    success = True
    
    #vs = VideoStream(src=0).start()                                # for laptop webcam
    time.sleep(1.0)

    # loop over frames from the video stream
    start_time = time.time()*1000.0
    #while success:
    while True:
        count += 1
        try:
            #frame = vs.read()                               # grab frame
            #frame = imutils.resize(frame, width=450)        # resize

            success, frame = vs.read()
            frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)        # resize
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # convert to grayscale
            rects = detector(gray, 0)                       # detect faces in the grayscale frame

            # loop over the face detections
            for rect in rects:
                # determine the facial landmarks for the face region, then
                # convert the facial landmark (x, y)-coordinates to a NumPy
                # array
                shape = predictor(gray, rect)
                shape = face_utils.shape_to_np(shape)

                # extract the left and right eye coordinates, 
                leftEye = shape[lStart:lEnd]
                rightEye = shape[rStart:rEnd]

                # use coordinates to compute the eye aspect ratio for both eyes
                leftEAR = eye_aspect_ratio(leftEye)
                rightEAR = eye_aspect_ratio(rightEye)

                # average the eye aspect ratio together for both eyes
                ear = (leftEAR + rightEAR) / 2.0
                

                # compute the convex hull for the left and right eye, then
                # visualize each of the eyes
                leftEyeHull = cv2.convexHull(leftEye)
                rightEyeHull = cv2.convexHull(rightEye)
                cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
                cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)

                # check to see if the eye aspect ratio is below the blink
                # threshold, and if so, increment the blink frame counter
                if ear < EYE_AR_THRESH:
                        COUNTER += 1

                # else eye aspect ratio is not below the blink threshold
                else:
                    # if the eyes were closed for a sufficient number of
                    # then increment the total number of blinks
                    if COUNTER >= EYE_AR_CONSEC_FRAMES:
                        #t = time.time()*1000.0 - start_time
                        t = count / fps # time in seconds
                        f.write(str(t))
                        f.write(';\n')

                    # reset the eye frame counter
                    COUNTER = 0

            
        
        except:
            break


    # do a bit of cleanup
    f.close()
    vs.release()
    cv2.destroyAllWindows()
    #vs.stop()

if __name__ == '__main__':
    BLINK()
 
