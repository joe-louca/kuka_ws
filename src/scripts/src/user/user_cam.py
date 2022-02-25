#!/usr/bin/env python

import rospy
import os
import cv2


def RECORD_USER():
    trial_id = rospy.get_param('trial_id')
    latency = rospy.get_param('latency')
    pp_id = rospy.get_param('participant_id')
    latency = int(latency*1000)
    path =  '/media/joe/My Passport/data/P'+str(pp_id)+'/P'+str(pp_id)+'_H'+str(trial_id)+'_'+str(latency)+'_usercam.mp4'
    
    vid_capture = cv2.VideoCapture(0)
    vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
    output = cv2.VideoWriter(os.path.expanduser(path), vid_cod, 20.0, (640,480))
    
    while not rospy.is_shutdown():
        success, frame = vid_capture.read()
        #cv2.imshow("UserCam", frame)
        output.write(frame)

    vid_capture.release()
    output.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    RECORD_USER()
