#!/usr/bin/python3

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

class Control():


    def __init__(self):
        
        rospy.init_node('control', anonymous=True)
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        self.angular_k_i = 0.1
        self.angular_k_p = 0.6
        self.angular_k_d = 7
        
        self.dt = 0.005
        self.v = 0.5
        self.D = 1.25 # treshold
        rate = 1/self.dt
        
        self.r = rospy.Rate(rate)
        self.errs = []


    def distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng = laser_data.ranges[:180]
        d = min(rng)
        return d

    
    def run(self):
        
        d = self.distance_from_wall()    
        sum_i_theta = 0
        prev_theta_error = 0
        
        twist = Twist()
        twist.angular.z = 0
        twist.linear.x = self.v

        while not rospy.is_shutdown():
            self.cmd_publisher.publish(twist)

            err = d - self.D
            self.errs.append(err)
            sum_i_theta += err * self.dt
            
            P_a = self.angular_k_p * err
            I_a = self.angular_k_i * sum_i_theta
            D_a = self.angular_k_d * (err - prev_theta_error)

            rospy.loginfo(f"P : {P_a} I : {I_a} D : {D_a}")
            twist.angular.z = P_a + I_a + D_a
            if abs(twist.angular.z) > math.radians(30):
                twist.linear.x = self.v / 4
            else:
                twist.linear.x = self.v
            prev_theta_error = err         
            
            rospy.loginfo(f"error : {err} speed : {twist.linear.x} theta : {twist.angular.z}")
            
            d = self.distance_from_wall()

            self.r.sleep()
            

if __name__ == "__main__":
    controller = Control()
    
    controller.run()

