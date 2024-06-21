#!/usr/bin/env python3

from nav_msgs.msg import Odometry
import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from math import sqrt


class Control:

    def __init__(self):
        rospy.init_node('control', anonymous=False)

        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.callback)

        rospy.on_shutdown(self.plotting)

        self.controller_type = 'PID'

        self.goal_x = 10
        self.goal_y = 0
        self.current_x = 0
        self.current_y = 0

        self.k_p = 0.1
        self.k_i = 0.005
        self.k_d = 5
       
        self.dt = 0.005
        self.threshold = 0.1

        self.sum_i = 0.0
        self.prev_err = 0.0

        self.r = rospy.Rate(1/self.dt)

        self.errors = []

    def go_to_goal(self):

        err = self.get_distance() - self.threshold
        self.errors.append(err)

        # P
        P = self.k_p * err

        # I
        self.sum_i += err * self.dt
        I = self.k_i * self.sum_i

        # D
        D = self.k_d * (err - self.prev_err) / self.dt

        self.prev_err = err

        sum_PID = P
        if self.controller_type == 'PD':
            sum_PID + D
        elif self.controller_type == 'PID':
            sum_PID + I + D

        twist = Twist()
        twist.linear.x = sum_PID
        twist.angular.z = 0

        rospy.loginfo(f"error : {err} speed : {twist.linear.x}")
        
        return twist



    def plotting(self):
        rospy.loginfo('Stopping the robot...')
        self.cmd_publisher.publish(Twist())

        plt.plot(list(range(len(self.errors))), self.errors, label='errors')
        plt.axhline(y=0,color='R')
        plt.draw()
        plt.legend(loc="upper left", frameon=False)
        plt.show()
        
        rospy.sleep(1)

    def callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def get_distance(self):

        distance = abs(sqrt(((self.goal_x-self.current_x) ** 2) + ((self.goal_y-self.current_y) ** 2)))
        return distance
    
    def run(self):
        rospy.sleep(5)
        while not rospy.is_shutdown() and self.threshold < self.get_distance():
            twist = self.go_to_goal()
            self.cmd_publisher.publish(twist)
            self.r.sleep()

if __name__ == "__main__":
    control = Control()
    control.run()