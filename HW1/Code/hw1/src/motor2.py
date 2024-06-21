#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int16

def print_direction(data):
    rospy.loginfo(rospy.get_caller_id() + ": The direction is %s", data)

def listener():
    rospy.init_node('motor2', anonymous=True)
    rospy.Subscriber("direction2", Int16, print_direction)

    rospy.spin()



if __name__ == '__main__':
    listener()
