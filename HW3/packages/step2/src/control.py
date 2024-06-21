#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, pi, sqrt, tan, exp, sin, cos, degrees
from random import randint
import tf
import matplotlib.pyplot as plt 
import numpy as np

class Control:
    
    def __init__(self):

        rospy.init_node("control", anonymous=False)

        #subscriber & publisher
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.callback)

        shape = "rectangle"

        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0

        self.goal_x = 0
        self.goal_y = 0

        # angular velocity
        self.kp_a = 0.1
        self.ki_a = 0
        self.kd_a = 10
        self.Iz = 0
        self.previous_yaw_error = 0.0
        
        # linear velocity
        self.kp_l = 0.05
        self.ki_l = 0.005
        self.kd_l = 10
        self.Ix = 0
        self.previous_x_error = 0.0

        self.dt = 0.005        
        self.threshold = 1

        self.rate = rospy.Rate(1.0/self.dt)
        self.errors = []

        self.width = 6
        self.height = 4

        switcher = {
            "rectangle": self.rectangle,
            "star": self.star,
            "spiral": self.spiral
        }

        self.X, self.Y = switcher.get(shape, self.default)()

        self.current_index = 1
        
        self.angular_velocity = 0.1


    def callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))

    def publish_twist_msg(self, linear_velocity, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_z
        self.cmd_publisher.publish(twist_msg)
        rospy.sleep(0.1)

    def get_linear_error(self):
        return sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
    
    def get_angular_error(self):
        return atan2(self.goal_y - self.current_y, self.goal_x - self.current_x) - self.current_yaw

    def rotate(self):

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        next_yaw = atan2(dy, dx)

        self.diff_angle = next_yaw - self.current_yaw
        if self.diff_angle > pi:
            self.diff_angle -= 2*pi
        elif self.diff_angle < -pi:
            self.diff_angle += 2*pi

        print("Different angle: ", degrees(self.diff_angle))

        # Do the rotate
        while abs(self.diff_angle) > 0.1:
            self.publish_twist_msg(linear_velocity=0, angular_z=self.angular_velocity if self.diff_angle > 0 else -self.angular_velocity)
            self.diff_angle = next_yaw - self.current_yaw
            if self.diff_angle > pi:
                self.diff_angle -= 2*pi
            elif self.diff_angle < -pi:
                self.diff_angle += 2*pi
                
        self.publish_twist_msg(linear_velocity=0, angular_z=0)
        rospy.loginfo("rotating done")
        self.GO = True

    def move(self):
        while not rospy.is_shutdown() and self.threshold < self.get_linear_error():
            linear_error = self.get_linear_error()
            self.Ix += linear_error * self.dt
            linear_velocity = (
                self.kp_l * linear_error
                + self.ki_l * self.Ix
                + self.kd_l * (linear_error - self.previous_x_error) / self.dt
            )
            self.previous_x_error = linear_error

            angular_error = self.get_angular_error()
            if abs(angular_error) < 0.05:
                angular_error = 0
            self.Iz += angular_error * self.dt
            angular_z = (
                self.kp_a * angular_error
                + self.ki_a * self.Iz
                + self.kd_a * (angular_error - self.previous_yaw_error) / self.dt
            )
            self.previous_yaw_error = angular_error
            
            self.errors.append(linear_error)

            self.publish_twist_msg(linear_velocity, abs(angular_z) if angular_error >=0 else -abs(angular_z))

    def default(self):
        pass

    def rectangle(self):

        X1 = 0
        Y1 = 0
        Y2 = 0
        X2 = X1 + self.width
        X3 = X1 + self.width
        Y3 = Y1 + self.height
        Y4 = Y1 + self.height
        X4 = 0
        return ([X1, X2, X3, X4]), ([Y1, Y2, Y3, Y4])
        
    def star(self):
        X = [0, 1.5, 0.25, 0.75, -0.5, -1.75, -1.25, -2.5, -1, -0.5]
        Y = [0, 0, -1, -2.5, -1.5, -2.5, -1, 0, 0, 1.5]
        return X, Y

    def spiral(self):
        a = 0.17
        k = tan(a)
        X , Y = [] , []

        for i in range(150):
            t = i / 20 * pi
            dx = a * exp(k * t) * cos(t)
            dy = a * exp(k * t) * sin(t)
            X.append(dx)
            Y.append(dy)
        return np.array(X), np.array(Y)

    def get_new_goal(self):
        if self.current_index + 1 < len(self.X):
            distance = ((self.X[self.current_index] -self.current_x)**2+(self.Y[self.current_index]-self.current_y)**2)
            print("distance to next point: ", distance)
            self.current_index += 1
        else: 
            self.current_index = 0
        return self.X[self.current_index], self.Y[self.current_index]

    # go 
    def go_to_goal(self):
        
        for i in range(len(self.X)):
            self.goal_x, self.goal_y = self.get_new_goal()
            self.rotate()
            self.move()

            self.Iz = 0
            self.previous_yaw_error = 0.0
            # self.Ix = 0
            # self.previous_x_error = 0.0
        
            rospy.sleep(1)
            

if __name__ == "__main__":
    controller = Control()
    
    controller.go_to_goal()







