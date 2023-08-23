#!/usr/bin/env python

import rospy
import os
import cv2
import time


def RECORD_USER():
    rospy.init_node('user_cam', anonymous=True)
    rate = rospy.Rate(20)

    trial_id = rospy.get_param('trial_id')
    pp_id = rospy.get_param('participant_id')
    if int(trial_id) == 0:
        path = '~/UserTrialData/P'+str(pp_id)+'/Baseline_usercam.mp4'
    else:
        path =  '~/UserTrialData/P'+str(pp_id)+'/T'+str(trial_id)+'_usercam.mp4'
    
    vid_capture = cv2.VideoCapture('/dev/video2')
    vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
    output = cv2.VideoWriter(os.path.expanduser(path), vid_cod, 20.0, (640,480))

    start_time = rospy.get_param('start_time')
    
    while not rospy.is_shutdown():
        success, frame = vid_capture.read()
        if success:
            timestamp = str(round(rospy.get_time() - start_time,2))
            draw_text(frame, timestamp, pos=(5, 480-40), font_thickness=1)

            #cv2.imshow("UserCam", frame)
            output.write(frame)
        rate.sleep()

    vid_capture.release()
    output.release()
    cv2.destroyAllWindows()

def draw_text(img, text,
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

if __name__ == '__main__':
    RECORD_USER()
