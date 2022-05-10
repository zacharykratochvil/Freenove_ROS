#!/usr/bin/python
########################
# Copywrite Zachary Kratochvil 2022
# All rights reserved.
########################

# python imports
import sys
import time

# ros imports
import rospy

# local imports
from interfaces import Ultrasonic

# message imports
from freenove_ros.msg import Distance


class Ultrasonic_ROS():

	def __init__(self):

		# initialize publishers
		self.ultraPub = rospy.Publisher("ultrasonic_distance",
										Distance, queue_size=10)

		# initialize node
		rospy.init_node("ultrasonic_publisher", anonymous=False)

		# initialize sensor
		self.sensor = Ultrasonic.Ultrasonic()


	def start(self, rate):

		ros_rate = rospy.Rate(rate)
		
		# publish
		while not rospy.is_shutdown():
			
			start_time = time.time()
			distance = self.sensor.get_distance()
			end_time = time.time()
			avg_time = (start_time + end_time)/2

			timestamp = rospy.Time(avg_time)
			self.ultraPub.publish(cm=distance, timestamp=timestamp)

			ros_rate.sleep()

		

if __name__ == "__main__":

	parser = argparse.ArgumentParser(
		description="ROS interface to Ultrasonic sensor.")
	parser.add_argument("--rate", type=int, default=100,
		help='''rate to publish (Hz). Note actual rate will be
				limited by physical speed of sound:
				5m max distance from sensor, 5 measurements,
				means sound travels 2x5x5=50m. At 343m/s
				this takes .146 seconds.''')
	args = parser.parse_args(rospy.myargv()[1:])

	sensor = Ultrasonic_ROS()
	sensor.start(args.rate)



