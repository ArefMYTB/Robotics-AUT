#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int16
from hw1.msg import proximity

direction_index = 0 # Up

def get_min(data_list):
    min_value = min(data_list)

    min_index = [i for i,val in enumerate(data_list) if val==min_value]
    return min_index[0]

def direction(data):
    global direction_index
    pub1 = rospy.Publisher('direction1', Int16, queue_size=10)
    pub2 = rospy.Publisher('direction2', Int16, queue_size=10)

    data_list = [data.up, data.right, data.down, data.left]
    min_distance = get_min(data_list) # new direction
    tempt = min_distance - direction_index
    rospy.loginfo("last: %s, now: %s, The mean = %s", direction_index, min_distance, tempt*90)
    direction_index = min_distance

    if not rospy.is_shutdown():
        pub1.publish(direction_index*90)
        pub2.publish(direction_index*90)
    

def listener():
    rospy.init_node('control', anonymous=True)
    rospy.Subscriber("distance", proximity, direction)

    rospy.spin()



if __name__ == '__main__':
    listener()
