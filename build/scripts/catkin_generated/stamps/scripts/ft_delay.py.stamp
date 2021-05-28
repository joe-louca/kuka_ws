#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import WrenchStamped

class AXIA:
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


    def ft_callback(msg):
        t = rospy.get_time()
        timestamped_ft = [msg.Wrench.force[0], msg.Wrench.force[1], msg.Wrench.force[2], msg.Wrench.torque[3], msg.Wrench.torque[4], msg.Wrench.torque[5], t]
        ft, retrieved = self.add_delay(timestamped_ft, self.delayed_ft_table)

        if retrieved:      
            rospy.set_param('ft_delay/fx', ft[0])
            rospy.set_param('ft_delay/fy', ft[1])
            rospy.set_param('ft_delay/fz', ft[2])
            rospy.set_param('ft_delay/tx', ft[3])
            rospy.set_param('ft_delay/ty', ft[4])
            rospy.set_param('ft_delay/tz', ft[5])
        

        
    def __init__(self):   
        self.delayed_ft_table = []
        self.latency = rospy.get_param('latency')
        rospy.init_node('ft_sub_node', anonymous=True)
        rospy.Subscriber('ft_sensor', WrenchStamped, self.ft_callback, queue_size=1)
        rospy.spin()

  
if __name__ == '__main__':
    AXIA()
