#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

# Adds return delay for kuka 
class KUKA_JOINTS:   
    def add_delay(self, added_row, delayed_tbl):
        delayed_tbl.insert(0, added_row)                                # Add new row to table
        row_len = len(added_row)
        tbl_len = len(delayed_tbl)                              
        retrieved_row = []
        retrieved = False
        elapsed_time = rospy.get_time()

        for i in range(tbl_len):                                        # For each row
            if elapsed_time > delayed_tbl[i][row_len-1] + self.latency:   # If row is old enough
                retrieved_row = delayed_tbl[i]                          # Get this row
                retrieved_row.pop()                                     # Remove timestamp
                delayed_tbl = delayed_tbl[:(-tbl_len+i-1)]              # Remove old rows
                retrieved = True                                        # Update marker
                break
        
        return retrieved_row, retrieved

    def __init__(self):
        rate_hz = rospy.get_param('rate_hz')
        self.latency = rospy.get_param('latency')
        self.delayed_jpos = []
        self.delayed_pos = []
        
        pub = rospy.Publisher('delayed_kuka_jpos', Float32MultiArray, queue_size=1)
        rospy.init_node('kuka_jpos_node', anonymous=True)
        rate = rospy.Rate(rate_hz)

        while not rospy.is_shutdown():
            # Get current kuka joint angles / ee position & store with timestamp
            kuka_jpos = rospy.get_param('kuka_jpos')
            kuka_pos = rospy.get_param('kuka_pos')

            # Store timestamp & retrieve delayed joint angles
            t = rospy.get_time()
            kuka_jpos.append(t)
            kuka_pos.append(t)

            jpos, j_retrieved = self.add_delay(kuka_jpos, self.delayed_jpos)
            pos, p_retrieved = self.add_delay(kuka_pos, self.delayed_pos)
            
            if j_retrieved:
                # Publish delayed jpos
                msg = Float32MultiArray()
                msg.data = jpos
                pub.publish(msg)
                rospy.set_param('delayed_kuka_jpos', jpos)

                if p_retrieved:
                    rospy.set_param('delayed_kuka_pos', pos)
                
            rate.sleep()
        
if __name__ == '__main__':
    KUKA_JOINTS()
