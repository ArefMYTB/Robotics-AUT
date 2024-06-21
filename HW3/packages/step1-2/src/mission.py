#!/usr/bin/python3

import sys
import rospy
from random import randint
import math
from hw2.srv import GetNextDestination, GetNextDestinationResponse
from std_msgs.msg import String, Int16

def goal_generator(req):

    dist = 0
    while(dist < 5):  
        next_x = randint(-10,10)
        next_y = randint(-10,10)
        dist = math.dist([req.current_x,req.current_y], [next_x,next_y])

    return GetNextDestinationResponse(next_x, next_y)

if __name__ == '__main__':
    
    rospy.init_node('mission', anonymous=True)
    res = rospy.Service('GetNextDestination', GetNextDestination, goal_generator)
    rospy.spin()