#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
import time
import numpy as np


class FT:   
    def call_Rob_ft(self, ft_msg):
        ## Get sensor readings from Axia
        rob_fx = ft_msg.wrench.force.x
        rob_fy = ft_msg.wrench.force.y
        rob_fz = ft_msg.wrench.force.z
        rob_tx = ft_msg.wrench.torque.x
        rob_ty = ft_msg.wrench.torque.y
        rob_tz = ft_msg.wrench.torque.z

        Rob_F_sensor = np.array([[rob_fx],
                                 [rob_fy],
                                 [rob_fz]])
        Rob_T_sensor = np.array([[rob_tx],
                                 [rob_ty],
                                 [rob_tz]])
        frame_id = ft_msg.header.seq
        
        ## Remove bias (baseline roboiq reading with tool attached)
        Rob_F_bias = np.array([[-16], #
                               [-81], #
                               [ -0]])#
        Rob_T_bias = np.array([[ 0.58], #
                               [ 0.11], #
                               [-0.07]])#     
        
        Rob_F_sensor = Rob_F_sensor - Rob_F_bias
        Rob_T_sensor = Rob_T_sensor - Rob_T_bias
        
        
        ## Convert to world frame
        if self.Rot_ready_:
            W_F_sensor = np.dot(self.W_R_Rob, Rob_F_sensor)
            W_T_sensor = np.dot(self.W_R_Rob, Rob_T_sensor)
            self.Rot_ready_ = False

            hap_F = W_F_sensor
            hap_T = W_T_sensor
            
            ## Remove Tool force
            if self.Tool_ready_:
                self.Tool_ready_ = False
                
                # Convert to python floats
                ft_x = W_F_sensor[0][0].item()
                ft_y = W_F_sensor[1][0].item()
                ft_z = W_F_sensor[2][0].item()
                ft_tx = W_T_sensor[0][0].item()
                ft_ty = W_T_sensor[1][0].item()
                ft_tz = W_T_sensor[2][0].item()

                # Build publish message
                self.pub_msg.twist.linear.x = ft_x *0.5   # Scale torques to a good haption range
                self.pub_msg.twist.linear.y = ft_y *0.5
                self.pub_msg.twist.linear.z = ft_z *0.5
                self.pub_msg.twist.angular.x = ft_tx *0.0 # Ignore torques
                self.pub_msg.twist.angular.y = ft_ty *0.0
                self.pub_msg.twist.angular.z = ft_tz *0.0
                self.pub_msg.header.seq = frame_id
                self.pub_msg.header.stamp = rospy.get_rostime()
                
                ## Mark message as ready to send
                self.send_msg_ = True
        return


    def call_Tool_ft(self, tool_msg):
        tool_fx = tool_msg.wrench.force.x
        tool_fy = tool_msg.wrench.force.y
        tool_fz = tool_msg.wrench.force.z
        tool_tx = tool_msg.wrench.torque.x
        tool_ty = tool_msg.wrench.torque.y
        tool_tz = tool_msg.wrench.torque.z

        self.W_F_tool = np.array([[tool_fx],
                                  [tool_fy],
                                  [tool_fz]])
        self.W_T_tool = np.array([[tool_fx],
                                  [tool_fy],
                                  [tool_fz]])


        self.Tool_ready_ = True
        return

    def call_Rot(self, rot_msg):
        r11 = rot_msg.data[0]
        r12 = rot_msg.data[1]
        r13 = rot_msg.data[2]
        r21 = rot_msg.data[3]
        r22 = rot_msg.data[4]
        r23 = rot_msg.data[5]
        r31 = rot_msg.data[6]
        r32 = rot_msg.data[7]
        r33 = rot_msg.data[8]

        self.W_R_Rob = np.array([[r11, r12, r13],
                                [r21, r22, r23],
                                [r31, r32, r33]])

        self.Rot_ready_ = True
        return    

    def __init__(self):
        self.pub_msg = TwistStamped()
        
        self.send_msg_ = False
        self.Rot_ready_ = False
        self.Tool_ready_ = False
        
        rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, self.call_Rob_ft, queue_size=1)
        rospy.Subscriber("/ft_tool", WrenchStamped, self.call_Tool_ft, queue_size=1)
        rospy.Subscriber("/W_R_Ax", Float32MultiArray, self.call_Rot, queue_size=1)
        
        pub = rospy.Publisher('/F_kuka_out', TwistStamped, queue_size=1)

        rate_hz = rospy.get_param('rate_hz')
        r = rospy.Rate(rate_hz)
        
        while not rospy.is_shutdown():
            if self.send_msg_:
                pub.publish(self.pub_msg)
                self.send_msg_ = False
            r.sleep()

 
if __name__ == '__main__':
    rospy.init_node('Force_Compensation')
    try:
        foo = FT()
    except rospy.ROSInterruptException:  pass
