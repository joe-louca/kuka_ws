#!/usr/bin/env python3

import rospy
import os
import cv2


def RECORD_USER():
    rospy.init_node('user_cam', anonymous=True)
    rate = rospy.Rate(20)

    trial_id = rospy.get_param('trial_id')
    pp_id = rospy.get_param('participant_id')
    if int(trial_id) == 0:
        path = '~/RemoteHD/UserTrialData/P'+str(pp_id)+'/_Baseline_usercam.mp4'
    else:
        path =  '~/RemoteHD/UserTrialData/P'+str(pp_id)+'/_T'+str(trial_id)+'_usercam.mp4'
    
    vid_capture = cv2.VideoCapture(0)
    vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
    output = cv2.VideoWriter(os.path.expanduser(path), vid_cod, 20.0, (640,480))
    
    while not rospy.is_shutdown():
        success, frame = vid_capture.read()
        cv2.imshow("UserCam", frame)
        output.write(frame)
        rate.sleep()

    vid_capture.release()
    output.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    RECORD_USER()
