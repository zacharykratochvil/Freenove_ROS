#!/usr/bin/python
########################
# Copywrite Zachary Kratochvil 2022
# All rights reserved.
########################

import rospy
from interfaces.PCA9685 import PCA9685
from freenove_ros.msg import ServoAngle

# ROS implementation
class ServoSub:
    def __init__(self):
        self.pwm_servo = PCA9685(0x40, debug=True)
        self.pwm_servo.setPWMFreq(50)

    def start(self):
        rospy.init_node('servo_subscriber', anonymous=False)
        rospy.Subscriber("servos", ServoAngle, self.set_angle, queue_size=10)
        rospy.spin()

    def set_angle(self, data):

        if not (0 <= data.id <= 7):
            raise Exception(f"Invalid ServoAngle message. Id {data.id} must be between 0 and 7.")
        if not (0 <= data.angle <= 180):
            raise Exception(f"Invalid ServoAngle message. Angle {data.angle} must be between 0 and 180.")

        # update state and apply power to motors
        error = 10
        ch_offset = 8
        min_pulse = 500
        max_pulse = 2500
        self.pwm_servo.setServoPulse(data.id + ch_offset, min_pulse+int((data.angle+error)/0.09))


if __name__ == "__main__":
    servos=ServoSub()
    servos.start()




    



    

    
       



    
