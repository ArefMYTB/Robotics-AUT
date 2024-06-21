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

# def talker(data):
     
#      pub = rospy.Publisher('goal_coordinates', coordinates, queue_size=10)

#      next_coordinates = coordinates()
#      # randomly generates next coordinates and check if it is in more than 5m distance of current possition of robot
#      next_coordinates.next_x, next_coordinates.next_y = goal_generator(data.current_x, data.current_y)

#      if not rospy.is_shutdown():
#           rospy.loginfo(next_coordinates)
#           pub.publish(next_coordinates)
		

# def listener():
#     rospy.init_node('mission', anonymous=True)
#     # rospy.Subscriber("current_possition", coordinates, talker)

#     rospy.Service('GetNextDestination', GetNextDestination, goal_generator)
#     rospy.spin()



if __name__ == '__main__':
    
    rospy.init_node('mission', anonymous=True)
    # test(sys.argv[1])

    res = rospy.Service('GetNextDestination', GetNextDestination, goal_generator)
    print(res)
    rospy.spin()