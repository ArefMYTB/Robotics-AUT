#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist
from turtlebot3_object_tracker.srv import detection, detectionRequest
from math import atan2

class Controller:
    def __init__(self) -> None:
        # Use these Twists to control your robot
        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()

        # The "p" parameter for your p-controller, TODO: you need to tune this
        self.angular_vel_coef = 1

        # TODO: Create a service proxy for your human detection service
        rospy.wait_for_service('srv_detection')
        self.detection_service_proxy = rospy.ServiceProxy('srv_detection', detection)
        
        # TODO: Create a publisher for your robot "cmd_vel"
        self.cmd_vel_publisher = rospy.Publisher('/follower/cmd_vel', Twist, queue_size=1)

    
    def run(self) -> None:
        try:
            while not rospy.is_shutdown():
                object_info, image_size = self.srv_detection()

                if object_info:
                    self.go_to_goal(object_info, image_size)
                else:
                    self.cmd_vel_publisher.publish(self.freeze)

                rospy.sleep(0.1)

        except rospy.exceptions.ROSInterruptException:
            pass
    
    def srv_detection(self):
        try:
            request = detectionRequest()
            request.object_label = "person"

            response = self.detection_service_proxy(request)

            return response.object_info, response.image_size

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % str(e))

    def go_to_goal(self, object_info, image_size):
        error = self.calculate_error(object_info, image_size)
        self.move.angular.z = self.angular_vel_coef * error

        self.cmd_vel_publisher.publish(self.move)

    def calculate_error(self, object_info, image_size):
        
        image_width = image_size[1]

        desired_position = image_width / 2
        current_center = (object_info[0] + object_info[1]) / 2

        error = atan2(desired_position - current_center, image_width)

        return error

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()
    

