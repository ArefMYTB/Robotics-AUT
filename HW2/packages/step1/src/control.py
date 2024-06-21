#!/usr/bin/python3

import rospy
import tf
from random import randint
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rosservice
from std_msgs.msg import String, Int16

from hw2.srv import GetNextDestination
from math import radians, atan2, sqrt, pi

class Control:

    def __init__(self) -> None:
        
        rospy.init_node("control", anonymous=False)

        #subscriber & publisher
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # linear & angular speed
        self.linear_speed = float(rospy.get_param('~linear_speed')) # m/s
        self.angular_speed = float(rospy.get_param('~angular_speed')) # rad/s
        self.serv = rospy.ServiceProxy('GetNextDestination', GetNextDestination)
        self.diff_angle = radians(0) # rad

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.goal_x = 0
        self.goal_y = 0

        self.epsilon = float(rospy.get_param('~epsilon'))

        self.GO = True
        self.use_service = False

    # get new goal
    def get_new_goal(self):
        print("Getting new goal")

        print("X, Y = ", self.x, " , ", self.y)
        
        if self.use_service:
            try:  
                res = self.serv(self.x, self.y)
                next_x, next_y = res.next_x, res.next_y
            except rospy.ServiceException as e:
                print("Service call failed: ", e)
                return None
        else:
            dist = 0
            while(dist < 2):  
                next_x = randint(-5,5)
                next_y = randint(-5,5)
                dist = math.dist([self.x,self.y], [next_x,next_y])

        self.goal_x = next_x
        self.goal_y = next_y

        print("New X, Y = ", self.goal_x, " , ", self.goal_y)
        print("Dist: ", dist)

    def rotate(self):

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        next_yaw = atan2(dy, dx)

        self.yaw = self.get_heading()
        self.diff_angle = next_yaw - self.yaw
        if self.diff_angle > pi:
            self.diff_angle -= 2*pi
        elif self.diff_angle < -pi:
            self.diff_angle += 2*pi

        print("Different angle: ", math.degrees(self.diff_angle))

        # Do the rotate
        while abs(self.diff_angle) > self.epsilon:
            self.publish_twist_msg(angular_z=self.angular_speed if self.diff_angle > 0 else -self.angular_speed)
            self.yaw = self.get_heading()
            self.diff_angle = next_yaw - self.yaw
            if self.diff_angle > pi:
                self.diff_angle -= 2*pi
            elif self.diff_angle < -pi:
                self.diff_angle += 2*pi
                
        self.publish_twist_msg(angular_z=0)
        rospy.loginfo("rotating done")
        self.GO = True


    def publish_twist_msg(self, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = angular_z
        self.cmd_publisher.publish(twist_msg)
        rospy.sleep(0.1)

    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw

    def calculate_error(self, error):
        print("error: ", error)

    # go 
    def go_to_goal(self):
        
        iter_num = 0
        dist_error = -1

        while not rospy.is_shutdown():

            if iter_num > 5:
                self.linear_speed = 0

            # get current position
            msg = rospy.wait_for_message("/odom" , Odometry)
            pose = msg.pose.pose.position

            self.x = pose.x
            self.y = pose.y
            self.yaw = self.get_heading()

            distance = abs(sqrt(((self.goal_x-self.x) ** 2) + ((self.goal_y-self.y) ** 2)))

            # run while you're getting closer 
            if distance > dist_error and self.GO == True:
                dist_error = 100
                self.calculate_error(distance)
                self.get_new_goal()
                self.rotate()
                iter_num += 1
                # break
            else:
                dist_error = distance

            # print("new distance: ", distance)                

            # do the running
            if self.GO == True:
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_publisher.publish(twist)
                
                continue
            
            self.cmd_publisher.publish(Twist())
            
            rospy.sleep(1)
            

if __name__ == "__main__":
    controller = Control()
    
    controller.go_to_goal()