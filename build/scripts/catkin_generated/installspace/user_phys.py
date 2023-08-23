#!/usr/bin/env python3

import serial
import time
import rospy
from std_msgs.msg import Float32MultiArray
import time

def main():
    SER_PORT = "/dev/ttyACM0"
    BAUD_RATE = 9800
    ser = serial.Serial(SER_PORT,BAUD_RATE)
    time.sleep(3)
    
    start_time = rospy.get_param('start_time')
    rate_hz = 5000
    rospy.init_node('user_physiology', anonymous=True)
    pub = rospy.Publisher('/user_phys', Float32MultiArray, queue_size=1)
    user_phys_msg = Float32MultiArray()
    
    r = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        data = ser.readline()
        data = data.decode()
        data_list = data.split(", ")
        try:
            EMG1 = float(data_list[0])
            EMG2 = float(data_list[1])
            HR = float(data_list[2])
            t = rospy.get_time() - start_time

            user_phys_msg.data = [EMG1, EMG2, HR, t]
            pub.publish(user_phys_msg)
        except:
            pass
        r.sleep()     
     

if __name__ == '__main__':
    main()

