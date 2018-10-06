#!/usr/bin/env python

import rospy
from std_msgs.msg import String

"""
You must import rospy if you are writing a ROS Node
"""

def talker():
	"""
	pub declares that your node is publishing to chatter
	rospy.init_node() tells rospy the name of your node
	
	"""
	pub = rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('talker', anonymus=True)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass