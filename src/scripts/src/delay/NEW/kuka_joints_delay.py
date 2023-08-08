#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

class DELAY():
    def callback(self, msg):
        self.checker = True
        self.callback_msg = msg

    def __init__(self):       
        # Prepare the publisher and subscriber
        rospy.init_node('kuka_joints_delay', anonymous=True)
        pub = rospy.Publisher('/delayed_kuka_joints', Float32MultiArray, queue_size=1)
        sub = rospy.Subscriber('/kuka_joints', Float32MultiArray, self.callback, queue_size=1)    

        # Prepare messages
        pub_msg = Float32MultiArray()
        self.callback_msg = Float32MultiArray()
        self.checker = False

        # Get latency and refresh rate
        latency = rospy.get_param('latency')
        rate_hz = rospy.get_param('rate_hz')

        # Prepare stored messages table based on rate & latency
        msgs_to_store = int(round(latency/(1.0/float(rate_hz)),0))
        rate_hz = frames_to_store/latency
        delayed_msg_tbl = [None]*msgs_to_store
        store = 0
        full_store = False

        # Set refresh rate
        r = rospy.Rate(rate_hz)

        # Stop looping if ROS shuts down
        while not rospy.is_shutdown():
            # If message has been heard
            if self.checker:
                delayed_msg_tbl[store] = self.callback_msg
                store += 1

                # Check if store is full
                if store == msgs_to_store:
                    store = 0
                    full_store = True

                # If msg deck is full, start retrieving messages
                if full_store:
                    pub_msg = delayed_msg_tbl[store]
                    pub.publish(pub_msg)
                    
            r.sleep()

if __name__ == '__main__':
    DELAY()



#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def add_delay(added_row, delayed_tbl):
    latency = rospy.get_param('latency')
    delayed_tbl.append(added_row)                                   # Add new row to end of list (newest at bottom)
    row_len = len(added_row)-1                                      # Length of added row
    tbl_len = len(delayed_tbl)                                      # Number of rows in of table                        

    retrieved_row = []
    retrieved = False
    elapsed_time = rospy.get_time()
    
    for i in range(tbl_len):                                        # Starting at the oldest, For each row   
        if elapsed_time > delayed_tbl[i][row_len] + latency:        # If row is old enough
            retrieved_row = delayed_tbl[i][:row_len]                # Get this row and remove timestamp
            delayed_tbl = delayed_tbl[i+1:]                         # Keep only new rows (all rows at this point and below)            
            retrieved = True                                        # Update marker
            break
    
    return retrieved_row, retrieved, delayed_tbl


def callback(msg):
    global delayed_tbl
    global delayed_msg
    global checker
                
    # Timestamp
    t = rospy.get_time()
    timestamped_msg = [msg, t]

    # Store and retrieve delayed ft readings
    retrieved_line, retrieved, delayed_tbl = add_delay(timestamped_msg, delayed_tbl)

    if retrieved:
        delayed_msg = retrieved_line[0]
        checker = True


def main():
    global delayed_tbl
    global latency
    global delayed_msg
    global checker
    checker = False
    delayed_tbl = []
    delayed_msg = Float32MultiArray()
    
    latency = rospy.get_param('latency')
    rate_hz = rospy.get_param('rate_hz')
    
    # Prepare the publisher and subscriber
    rospy.init_node('kuka_joints_delay', anonymous=True)
    pub = rospy.Publisher('/delayed_kuka_joints', Float32MultiArray, queue_size=1)
    sub = rospy.Subscriber('/kuka_joints', Float32MultiArray, callback, queue_size=1)    

    r = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        if checker:
            pub.publish(delayed_msg)
            checker = False
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

