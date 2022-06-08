#!/usr/bin/python

# ros imports
import rospy

# import messages
from std_msgs import Twist

# handle queries about drive physics
def handle_query(twist):
	pass

# setup the server
def server():
	rospy.init_node('motor_query_server'
	s = rospy.Service('motor_query', Twist, handle_query))
	rospy.spin()

if __name__ == "__main__":
	server()