#!/usr/bin/python3

import rospy

from hw3.msg import ClosestObstacle
from geometry_msgs.msg import Twist

import math


class Control():
    def __init__(self):
        rospy.init_node('control', anonymous=True)

        self.linear_speed = float(rospy.get_param('~linear_speed'))
        self.angular_speed = float(rospy.get_param('~angular_speed'))
        self.distance_threshold = 2

        rospy.Subscriber('/ClosestObstacle', ClosestObstacle, self.avoid_obstacle)
        self.speed_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pass

    def avoid_obstacle(self, msg):
        distance = msg.distance
        direction = msg.direction

        if (distance >= self.distance_threshold) or \
                (math.pi/2 < abs(direction) < 3 * math.pi / 2):
            self.run()
        else: 
            self.rotate(direction)

        rospy.sleep(0.1)

    def run(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0
        self.speed_publisher.publish(twist)

    def rotate(self, direction):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed if direction > 0 else -self.angular_speed
        self.speed_publisher.publish(twist)


if __name__ == '__main__':
    control = Control()
    rospy.spin()