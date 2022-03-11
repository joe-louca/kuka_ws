#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 2F gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' topic using the 'Robotiq2FGripper_robot_output' msg type for sending commands to a 2F gripper.
"""

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from std_msgs.msg import Int16
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep

def gripper_cmd_callback(msg):
    global gripper_cmd
    gripper_cmd = msg.data

def publisher():
    global gripper_cmd

    """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
    rospy.init_node('Robotiq2FGripperSimpleController')    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
    sub = rospy.Subscriber('/delayed_gripper_cmd', Int16, gripper_cmd_callback)
    rate_hz = rospy.get_param('rate_hz')
    
    command = outputMsg.Robotiq2FGripper_robot_output();
    r = rospy.Rate(rate_hz)
    while True:
        if rospy.has_param('delayed_gripper_cmd'):
            gripper_cmd = rospy.get_param('delayed_gripper_cmd')
            prev_gripper_cmd = gripper_cmd
            break
        else:
            pass

    # Send reset command
    command.rACT = 0  # activate
    command.rGTO = 0  # go to action
    command.rATR = 0  # Reset??
    command.rPR = 0 # closed
    command.rSP = 0 # speed
    command.rFR = 0 #force
    pub.publish(command)
    sleep(0.1)
    
    # Send activate command - activate
    command.rACT = 1  # activate
    command.rGTO = 1  # go to action
    command.rATR = 0  # Reset??
    command.rPR = 130 # open
    command.rSP = 100 # speed
    command.rFR = 100 # force
    pub.publish(command)
    sleep(0.1)
    
    while not rospy.is_shutdown():
        try:
            gripper_cmd = rospy.get_param('delayed_gripper_cmd')
            if (gripper_cmd != prev_gripper_cmd):
                
                # build command msg
                if gripper_cmd == 1:
                    command.rACT = 1  # activate
                    command.rGTO = 1  # go to action
                    command.rATR = 0  # Reset??
                    command.rPR = 240 # closed
                    command.rSP = 50 # speed
                    command.rFR = 150 #force
                else:
                    command.rACT = 1  # activate
                    command.rGTO = 1  # go to action
                    command.rATR = 0  # Reset??
                    command.rPR = 130   # position open
                    command.rSP = 255 # speed
                    command.rFR = 150 #force

                # publish to gripper        
                pub.publish(command)
            prev_gripper_cmd = gripper_cmd

        except:
            pass
        
        r.sleep()
                        

if __name__ == '__main__':
    publisher()
