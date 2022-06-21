#!/usr/bin/env python

import rospy
import numpy as np
import os
import cv2


def RECORD_USER():
    rospy.init_node('user_cam', anonymous=True)
    rate = rospy.Rate(20)
    #start_time = rospy.get_param('start_time') # float (s)
    start_time = rospy.get_time()
    path = '~/gui_test.mp4'
    
    #trial_id = rospy.get_param('trial_id')
    #pp_id = rospy.get_param('participant_id')
    #if int(trial_id) == 0:
    #    path = '~/RemoteHD/UserTrialData/P'+str(pp_id)+'/_Baseline_usercam.mp4'
    #else:
    #    path =  '~/RemoteHD/UserTrialData/P'+str(pp_id)+'/_T'+str(trial_id)+'_usercam.mp4'

    cam1_id = '/dev/video6'
    cam2_id = '/dev/video4'

    #vid1_capture = cv2.VideoCapture(cam1_id)

    vid1_capture = cv2.VideoCapture(4) #4 or 6
    
    vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
    output = cv2.VideoWriter(os.path.expanduser(path), vid_cod, 20.0, (640*2+50,480))

    while not rospy.is_shutdown():
        # Read the frame from the camera
        success1, frame1 = vid1_capture.read()

        if success1:
            timestamp = str(round(rospy.get_time() - start_time,2))
            draw_text(frame1, 'Camera 1')
            draw_text(frame1, timestamp, pos=(5, 450), font_thickness=1)

            # Write the frame to file
            output.write(frame1)

            # Display the frame
            cv2.imshow("Camera Views", frame1)
            if cv2.waitKey(50) & 0xFF == ord('q'):
                break

        # Sleep
        rate.sleep()

    vid1_capture.release()
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
