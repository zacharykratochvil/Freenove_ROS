#!/usr/bin/python

# used to conduct experiments to calculate coefficients
# of friction and damping
#
# run roslaunch freenove_ros freenove.launch in one terminal
# then run rosrun freenove_ros twist_experiments.py in another

import rospy
from collections import deque
import numpy as np

from freenove_ros.msg import DriveDuration
from std_msgs.msg import Float32

class Tests:
	def __init__(self):
		self.readings = deque()

		rospy.init_node("motor_experiments", anonymous=False)
		self.motors = rospy.Publisher("drive_motors", DriveDuration, queue_size = 1)
		self.battery = rospy.Subscriber("adc_battery_level", Float32, self.update_level)


	def run(self):
		print("ready? (q or x to quit, any other key to go)")
		while(input() not in {"q", "x", "Q", "quit", "exit"}):
			seconds = 1.5*(np.random.rand()+.25) #[.25 to 1.75]
			power_left = np.random.rand()*2 - 1 # [-1 to 1]
			power_right = np.random.rand()*2 - 1 # [-1 to 1]
			volts = np.mean(self.readings)
			print("power_left\tpower_right\tseconds\tbattery")
			print(f"{power_left}\t{power_right}\t{seconds}\t{volts}")

			self.motors.publish(power_left,power_right,
				power_left,power_right,seconds)
			print("ready?")


	def update_level(self, data):
		
		self.readings.append(data.data)

		if len(self.readings) > 10:
			self.readings.popleft()


if __name__ == "__main__":
	tests = Tests()
	tests.run()