#!/usr/bin/python3

import rospy
import tf
from random import randint
import math


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from hw5.srv import GetNextDestination

from math import radians, atan2, sqrt, pi

class Control:

    def __init__(self) -> None:
        
        rospy.init_node("control", anonymous=False)

        #subscriber & publisher
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.callback)

        self.serv = rospy.ServiceProxy('GetNextDestination', GetNextDestination)
        self.diff_angle = radians(0) # rad

        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0

        self.goal_x = 0
        self.goal_y = 0

        self.GO = True
        self.use_service = False


        # angular velocity
        self.kp_a = 0.1
        self.ki_a = 0
        self.kd_a = 5
        self.Iz = 0
        self.previous_a_error = 0.0
        
        # linear velocity
        self.kp_l = 0.1
        self.ki_l = 0.005
        self.kd_l = 1
        self.Ix = 0
        self.previous_l_error = 0.0

        self.dt = 0.005        
        self.threshold = 1

        self.rate = rospy.Rate(1.0/self.dt)
        self.errors = []

        self.angular_velocity = 0.1

    def callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))

    # get new goal
    def get_new_goal(self):
        print("Getting new goal")

        print("X, Y = ", self.current_x, " , ", self.current_y)
        
        if self.use_service:
            try:  
                res = self.serv(self.current_x, self.current_y)
                next_x, next_y = res.next_x, res.next_y
            except rospy.ServiceException as e:
                print("Service call failed: ", e)
                return None
        else:
            dist = 0
            while(dist < 2):  
                next_x = randint(-5,5)
                next_y = randint(-5,5)
                dist = math.dist([self.current_x,self.current_y], [next_x,next_y])

        self.goal_x = next_x
        self.goal_y = next_y

        print("New X, Y = ", self.goal_x, " , ", self.goal_y)
        print("Dist: ", dist)

    def rotate(self):

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        next_yaw = atan2(dy, dx)

        self.diff_angle = next_yaw - self.current_yaw
        if self.diff_angle > pi:
            self.diff_angle -= 2*pi
        elif self.diff_angle < -pi:
            self.diff_angle += 2*pi

        print("Different angle: ", math.degrees(self.diff_angle))

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
                + self.kd_l * (linear_error - self.previous_l_error) / self.dt
            )
            self.previous_l_error = linear_error

            angular_error = self.get_angular_error()
            if abs(angular_error) < 0.05:
                angular_error = 0
            self.Iz += angular_error * self.dt
            angular_z = (
                self.kp_a * angular_error
                + self.ki_a * self.Iz
                + self.kd_a * (angular_error - self.previous_a_error) / self.dt
            )
            self.previous_a_error = angular_error
            
            self.errors.append(linear_error)

            self.publish_twist_msg(linear_velocity, abs(angular_z) if angular_error >=0 else -abs(angular_z))

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

    # go 
    def go_to_goal(self):
        
        for i in range(4):
            self.get_new_goal()
            self.rotate()
            self.move()

            self.Iz = 0
            self.previous_a_error = 0.0
            self.Ix = 0
            self.previous_l_error = 0.0
        
            rospy.loginfo(f"The robot moved to destination {i+1}")
            rospy.sleep(1)
            

if __name__ == "__main__":
    controller = Control()
    
    controller.go_to_goal()