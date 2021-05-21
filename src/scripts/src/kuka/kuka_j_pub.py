#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

class KUKA_JOINTS:
    
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

    def __init__(self):
        rate_hz = rospy.get_param('rate_hz')
        self.latency = rospy.get_param('latency')
        self.delayed_jpos = []
        
        pub = rospy.Publisher('kuka_jpos', Float64MultiArray, queue_size=1)
        rospy.init_node('kuka_jpos_node', anonymous=True)
        rate = rospy.Rate(rate_hz)

        while not rospy.is_shutdown():
            # Get current kuka joint angles & store with timestamp
            kuka_jpos = rospy.get_param('kuka_jpos')
            elapsed_time = rospy.get_time()
            timestamped_jpos = kuka_jpos.append(elapsed_time)

            # Retrieve delayed joint angles
            jpos, retrieved = add_delay(timestamped_jpos, self.delayed_jpos)

            if retrieved:
                # Publish delayed jpos
                msg = Float64MultiArray()
                msg.data = jpos
                pub.publish(msg)
                
            rate.sleep()
        
if __name__ == '__main__':
    KUKA_JOINTS()
