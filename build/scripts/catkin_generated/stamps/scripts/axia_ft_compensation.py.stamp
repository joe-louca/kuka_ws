#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
import time
import numpy as np


class FT:   
    def call_Ax_ft(self, ft_msg):
        ## Get sensor readings from Axia
        ax_fx = ft_msg.wrench.force.x
        ax_fy = ft_msg.wrench.force.y
        ax_fz = ft_msg.wrench.force.z
        ax_tx = ft_msg.wrench.torque.x
        ax_ty = ft_msg.wrench.torque.y
        ax_tz = ft_msg.wrench.torque.z
        frame_id = ft_msg.header.seq
        
        if not self.biased:
            bias_ready = rospy.get_param('bias_ready')
            if bias_ready:
                self.fx_bias = ax_fx
                self.fy_bias = ax_fy
                self.fz_bias = ax_fz
                self.biased = True

        # Apply bias to sensor readings
        Ax_Fx_sensor = ax_fx - self.fx_bias
        Ax_Fy_sensor = ax_fy - self.fy_bias
        Ax_Fz_sensor = ax_fz - self.fz_bias
              
        ## Convert to world frame (rotate 180 around y)
        self.pub_msg.twist.linear.x = -Ax_Fx_sensor/3
        self.pub_msg.twist.linear.y =  Ax_Fy_sensor/3
        self.pub_msg.twist.linear.z = -Ax_Fz_sensor/3
        self.pub_msg.twist.angular.x = 0.0
        self.pub_msg.twist.angular.y = 0.0
        self.pub_msg.twist.angular.z = 0.0
        self.pub_msg.header.seq = frame_id
        self.pub_msg.header.stamp = rospy.get_rostime()        
        self.send_msg_ = True

        return


    def __init__(self):
        self.pub_msg = TwistStamped()
        
        self.send_msg_ = False
        self.Rot_ready_ = False
        self.Tool_ready_ = False
        self.biased = False
        
        self.fx_bias = 0.0
        self.fy_bias = 0.0
        self.fz_bias = 0.0

        rospy.init_node('Force_Compensation')
        rospy.Subscriber("/netft_data", WrenchStamped, self.call_Ax_ft, queue_size=1)
        pub = rospy.Publisher('/F_kuka_out', TwistStamped, queue_size=1)

        rate_hz = rospy.get_param('rate_hz')
        r = rospy.Rate(rate_hz)
        
        while not rospy.is_shutdown():
            if self.send_msg_:
                pub.publish(self.pub_msg)
                self.send_msg_ = False
            r.sleep()

 
if __name__ == '__main__':
    foo = FT()
