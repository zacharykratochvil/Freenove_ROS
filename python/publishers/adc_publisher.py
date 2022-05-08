#!/usr/bin/python
########################
# Copywrite Zachary Kratochvil 2022
# All rights reserved.
########################
import rospy
import argparse
import sys

from freenove_ros.interfaces import ADC

from std_msgs.msg import Float32
from freenove_ros.msg import AmbientLight


class ADC_ROS():

	def __init__(self, battery=False, light=False):

		# store input
		self.battery = battery
		self.light = light

		# max voltage recordable by the ADS7830 ADC
		self.V_max = 3.6
		
		# initialize publishers
		self.lightPub = None; self.batteryPub = None
		if light:			
			self.lightPub = self.initPub("adc_light_sensor", AmbientLight)
		if battery:
			self.batteryPub = self.initPub("adc_battery_level", Float32)

		# initialize node
		rospy.init_node("adc_publisher", anonymous=False)

		# initialize ADC
		self.adc = ADC.Adc()


	def start(self, rate):

		ros_rate = rospy.Rate(rate)
		
		# publish
		while not rospy.is_shutdown():
			
			if self.light:
				left = self.adc.recvADC(0)/self.V_max
				right = self.adc.recvADC(1)/self.V_max
				self.lightPub.publish(left=left, right=right)
			
			if self.battery:
				level = self.adc.recvADC(2)*3
				self.batteryPub.publish(level)

			ros_rate.sleep()


	def initPub(self, name, msg_type):
		return rospy.Publisher(name, msg_type, queue_size=1)
		

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description="ROS interface to ADC. No arguments defaults to both --battery and --light.")
	parser.add_argument("--light", action="store_true", default=False,
		help="publish ambient light recordings")
	parser.add_argument("--battery", action="store_true", default=False,
		help="publish battery level recordings")
	parser.add_argument("--rate", type=int, default=60,
		help="rate to publish (Hz)")
	args = parser.parse_args()

	if not (args.light or args.battery):
		args.light = True
		args.battery = True

	adc = ADC_ROS(battery=args.battery, light=args.light)
	adc.start(args.rate)



