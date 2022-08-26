#!/usr/bin/python
########################
# Copywrite Zachary Kratochvil 2022
# All rights reserved.
########################

import threading
import time
import rospy
from interfaces.Motor import Motor
from freenove_ros.msg import DriveDuration, TwistDuration

# ROS implementation
class Motor_Sub:
    def __init__(self):
        self.MOTOR_MAX = 4096

        self.motors = Motor()
        self.timer = threading.Timer(0, None)

    def start(self):
        rospy.init_node('drive_motor_subscriber', anonymous=False)
        rospy.Subscriber("drive_motors", DriveDuration, self.set_power, queue_size=10)
        rospy.Subscriber("drive_twist", TwistDuration, self.calc_and_set_power, queue_size=10)
        rospy.spin()

    def calc_and_set_power(self, data):
        pass


    def set_power(self, data):

        # validate inputs, note that in the freenove code in interfaces it must be > .12 to send any power
        def validate(number, channel):
            clipped_power = number
            if not (-1 <= number <= 1):
                clipped_power = min(1, max(-1, number))
                raise Warning(f"Invalid DriveDuration message. Power {number} on channel {channel} must be between -1 and 1."
                    + f" Setting power to {clipped_power}.")
            return clipped_power

        data.power_front_left = validate(data.power_front_left, "front left")
        data.power_front_right = validate(data.power_front_right, "front right")
        data.power_back_left = validate(data.power_back_left, "back left")
        data.power_back_right = validate(data.power_back_right, "back right")

        # update state and apply power to motors 
        self.motors.setMotorModel(-data.power_front_left*self.MOTOR_MAX,
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




