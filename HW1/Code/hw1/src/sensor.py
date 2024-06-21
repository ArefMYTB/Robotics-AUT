#!/usr/bin/python3

import rospy
from random import randint
from std_msgs.msg import String
from hw1.msg import proximity
	
def talker():
	
	pub = rospy.Publisher('distance', proximity, queue_size=10)
	rospy.init_node('sensor', anonymous=True)
	rate = rospy.Rate(1) #Hz
	
	while not rospy.is_shutdown():
		msg = proximity()
		msg.up = randint(10,200)
		msg.right = randint(10,200)
		msg.down = randint(10,200)
		msg.left = randint(10,200)
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()
		

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
