#!/usr/bin/python
########################
# Copywrite Zachary Kratochvil 2022
# All rights reserved.
########################

import threading
import time
import rospy
import numpy as np
from interfaces.Motor import Motor

from freenove_ros.msg import DriveDuration, TwistDuration
from std_msgs.msg import Float32

# ROS implementation
class Motor_Sub:
    def __init__(self):
        self.MOTOR_MAX = 4096

        self.motors = Motor()
        self.timer = threading.Timer(0, None)

        self.battery_voltage = 7.5 # default guess until get messages

    def start(self):
        rospy.init_node('drive_motor_subscriber', anonymous=False)
        rospy.Subscriber("drive_motors", DriveDuration, self.set_power, queue_size=10)
        rospy.Subscriber("drive_twist", TwistDuration, self.calc_and_set_power, queue_size=10)
        rospy.Subscriber("adc_battery_level", Float32, self.update_voltage, queue_size=10)
        rospy.spin()

    def update_voltage(self, data):
        self.battery_voltage = data.data

    def calc_and_set_power(self, data):
        planar_velocity = [data.velocity.linear.y] # y is linear velociy in m/s +forward
        planar_velocity.append(data.velocity.angular.z) # z is angular velocity in degrees/s +left

        # using imperical model of speed vs. power
        speed_exponent = .3883
        speed_coefficient = .2 #.81
        desired_total_power = planar_velocity[0]**(1/speed_exponent)/speed_coefficient

        # using imperical model of power difference vs. angular speed
        '''
        angular_coefficient = 7 #5.4665
        angular_offset = -15 #5
        desired_power_difference = (planar_velocity[1] - angular_offset)/angular_coefficient
        '''
        m = 0.01116983*.85
        b = 0.01342751260826952
        scale = 15
        desired_power_difference = scale*np.tanh(m*planar_velocity[1] + b)

        desired_power_left = desired_total_power - desired_power_difference/2 # to turn left, left wheels should go in reverse
        desired_power_right = desired_total_power + desired_power_difference/2
        desired_power_model = {}
        desired_power_model["power_front_left"] = desired_power_left
        desired_power_model["power_front_right"] = desired_power_right
        desired_power_model["power_back_left"] = desired_power_left
        desired_power_model["power_back_right"] = desired_power_right

        # convert to percentage
        max_motoer_current = .5 # this is a guess
        available_power = self.battery_voltage*max_motoer_current
        power_percentage = {}
        for wheel, desired_power in desired_power_model.items():
            power_percentage[wheel] = desired_power/available_power

        message = DriveDuration()
        message.power_front_left = power_percentage["power_front_left"]
        message.power_front_right = power_percentage["power_front_right"]
        message.power_back_left = power_percentage["power_back_left"]
        message.power_back_right = power_percentage["power_back_right"]
        message.duration = data.duration
        self.set_power(message)


    def set_power(self, data):

        # validate inputs, note that in the freenove code in interfaces it must be > .12 to send any power
        def validate(number, channel):
            number = np.real(number)
            clipped_power = number
            if not (-1 <= number <= 1):
                clipped_power = min(1, max(-1, number))
                rospy.logwarn(f"Invalid DriveDuration message. Power {number} on channel {channel} must be between -1 and 1."
                    + f" Setting power to {clipped_power}.")
            return clipped_power

        data.power_front_left = validate(data.power_front_left, "front left")
        data.power_front_right = validate(data.power_front_right, "front right")
        data.power_back_left = validate(data.power_back_left, "back left")
        data.power_back_right = validate(data.power_back_right, "back right")

        # update state and apply power to motors 
        self.motors.setMotorModel(data.power_front_left*self.MOTOR_MAX,
                                data.power_back_left*self.MOTOR_MAX,
                                data.power_front_right*self.MOTOR_MAX,
                                data.power_back_right*self.MOTOR_MAX)

        # handle duration with new requests overriding
        self.timer.cancel()
        self.timer = threading.Timer(data.duration, self.zero)
        self.timer.start()

    def zero(self):
        self.motors.setMotorModel(0,0,0,0)        


if __name__ == "__main__":
    motors = None
    try:
        motors=Motor_Sub()
        motors.start()
    finally:
        motors.zero()




