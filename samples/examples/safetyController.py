import rospy
import numpy as np 
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

"""
Example of a safety controller from class

didn't work
"""

class SafetyController:

	def __init__(self, min_dist, angle_range):
		self.min_dist = min_dist
		self.angle_range = angle_range

		self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb, queue_size=1)
		self.pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=1)


	def laser_cb(self, msgg):
		mid_angle_idx = len(msg.ranges)/2
		min_angle_idx = int(mid_angle_idx - (self.angle_range/2.0) / msg.angle_increment)
		max_angle_idx = int(mid_angle_idx + (self.angle_range/2.0) / msg.angle_increment)

		min_angle_idx = max(0, min_angle_idx)
		max_angle_idx = min(len(msg.ranges)-1, max_angle_idx)

		too_close_count = 0
		for i in xrange(min_angle_idx, max_angle_idx):
			"""
			track how many scans are reporting too close
			"""
			if msg.ranges[i] < self.min_dist:
				too_close_count += 1

		if too_close_count / (max_angle_idx - min_angle_idx + 1) > 0.5:
			""" Come up with some threshold to be too close"""
			ads = AckermannDriveStamped()
			ads.header.stamp = rospy.Time.now()
			ads.header.frame_id = '/map'
			ads.drive.sterring_angle = 0.0 # go straight back
			ads.drive.speed = -2.0 # reverse 2 m/s backwards
			self.pub.publish(ads)


if __name__ == '__main__':
	min_dist = 0.1 # the closest to get to a wall
	angle_range = np.pi / 8 # width of angles to consider

	rospy.init_node("safety_controller", anonymous=True)
    min_dist = rospy.get_param("~min_dist", None)
    angle_range = rospy.get_param("~angle_range", None)

    sc = SafetyController(min_dist, angle_range)

    rospy.spin()
