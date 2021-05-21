#!/usr/bin/env python

import rospy
import math

class INPUT_CONTROL:

    def add_delay(self, added_row, delayed_tbl):
        delayed_tbl.insert(0, added_row)                                # Add new row to table
        row_len = len(added_row)
        tbl_len = len(delayed_tbl)                              
        retrieved_row = []
        retrieved = False
        elapsed_time = rospy.get_time()

        for i in range(tbl_len):                                        # For each row
            if elapsed_time > delayed_tbl[i][row_len] + self.latency:   # If row is old enough
                retrieved_row = delayed_tbl[i]                          # Get this row
                retrieved_row.pop()                                     # Remove timestamp
                delayed_tbl = delayed_tbl[:(-tbl_len+i-1)]              # Remove old rows
                retrieved = True                                        # Update marker
                break
        
        return retrieved_row, retrieved
                
    def gripper_toggle(self, gripper_cmd):
        if gripper_cmd == 0:
            gripper_cmd = 1
        else:
            gripper_cmd = 0
        return gripper_cmd

      
    def __init__(self):
        rospy.init_node('joy_control_node', anonymous=True)
        rate_hz = rospy.get_param('rate_hz')
        rate = rospy.Rate(rate_hz)
        
        # Parameters
        pos_step_size = rospy.get_param('pos_step_size')   
        rot_step_size = rospy.get_param('rot_step_size')
        self.latency = rospy.get_param('latency')   

        # Some variables
        button_press_current = 0
        button_press_previous = 0
        gripper_cmd = 0     # 0 = open, 1 = closed
        self.delayed_kuka_pos = []
        self.delayed_input_cmd = []
        
        # User instructions
        print('######### starting joystick control')
        print('######### X-axis -/+: L/R L Stick, Y-axis -/+: U/D L stick, Z-axis -/+: U/D D-Pad')
        print('######### Roll -/+: L/R R Stick, Pitch -/+: U/D R stick, Yaw -/+: L1/R1')
        print('######### Open/Close gripper: X')

        while not rospy.is_shutdown():        

            # Read params from joy subscriber        
            ax_x = rospy.get_param('ax/x')
            ax_y = rospy.get_param('ax/y')
            ax_z = rospy.get_param('ax/z')
            ax_roll = rospy.get_param('ax/roll')
            ax_pitch = rospy.get_param('ax/pitch')
            ax_yaw = rospy.get_param('ax/yaw')
            x_press = rospy.get_param('but/x_press')

            # Get current end effector pose from KUKA
            kuka_jpos = rospy.get_param('kuka_jpos')
########################## TO DO ########################
            kuka_pos = FK(kuka_jpos) # returning x,y,z,yaw,pitch,roll                                         
########################## TO DO ########################

            # Store current pose & read delayed current pose (return delay)
            t = rospy.get_time()
            timestamped_kuka_pos = kuka_pos.append(t)
            kuka_pos, retrieved = add_delay(timestamped_kuka_pos, self.delayed_kuka_pos)

            if retrieved:
                # Store current joy input & read delayed command (outward delay)
                timestamped_input_cmd = [ax_x, ax_y, ax_z, ax_yaw, ax_pitch, ax_roll, x_press, t]
                input_cmd, retrieved = add_delay(timestamped_input_cmd, self.delayed_input_cmd)
                
                if retrieved:
                    ax = [joy_input[0], joy_input[1], joy_input[2], joy_input[3], joy_input[4], joy_input[5]]
                    but = [joy_input[6]]
                
                    ## Calculate new end effector goal pose, based on the current pose & command input:
                    pos_cmd_x = kuka_pos[0] + pos_step_size * ax[0]
                    pos_cmd_y = kuka_pos[1] + pos_step_size * ax[1]
                    pos_cmd_z = kuka_pos[2] + pos_step_size * ax[2]                
                    pos_cmd_yaw = kuka_pos[3] + rot_step_size * ax[5]
                    pos_cmd_pitch = kuka_pos[4] + rot_step_size * ax[4]
                    pos_cmd_roll = kuka_pos[5] + rot_step_size * ax[3]
    
                    # If gripper button is pressed, toggle gripper pose
                    button_press_previous = button_press_current
                    button_press_current = but[0]
                    if button_press_previous < button_press_current:
                        gripper_cmd = gripper_toggle(gripper_cmd)

                    # Send delayed delayed commands to param server    
                    pos_cmd = [pos_cmd_x, pos_cmd_y, pos_cmd_z, pos_cmd_yaw, pos_cmd_pitch, pos_cmd_roll]
                    rospy.set_param('pos_cmd', pos_cmd)
                    rospy.set_param('gripper_cmd', gripper_cmd) 

            rate.sleep()

if __name__ == '__main__':
    INPUT_CONTROL()

