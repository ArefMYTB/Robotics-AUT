#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from hw3.msg import ClosestObstacle

class Sensor(): 
    def __init__(self):
        rospy.init_node('sensor', anonymous=True)

        rospy.Subscriber('/scan', LaserScan, self.laser_scan)
        self.pub = rospy.Publisher('/ClosestObstacle', ClosestObstacle, queue_size=10)
        pass

    def laser_scan(self, msg):
        ranges = msg.ranges

        # Clip the range values between msg.range_min and msg.range_max
        ranges = [max(min(r, msg.range_max), msg.range_min) for r in ranges]

        distance = min(ranges)
        try:
            angle_index = ranges.index(distance)
            direction = msg.angle_min + angle_index * msg.angle_increment
        except ValueError: # all the obstacles are far away
            print(ValueError)
        
        msg = ClosestObstacle()
        msg.distance = distance
        msg.direction = direction
        self.pub.publish(msg)

        rospy.loginfo(f"direction={direction}, distance={distance}")
        rospy.sleep(0.1)

if __name__ == '__main__':
    sensor = Sensor()
    rospy.spin()