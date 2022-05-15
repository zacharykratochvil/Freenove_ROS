#!/usr/bin/python
########################
# Copywrite Zachary Kratochvil 2022
# All rights reserved.
########################

import threading
import RPi.GPIO as GPIO
import rospy
from freenove_ros.msg import OnOffDuration


# original code from Freenove
class Buzzer:
    def __init__(self):
        GPIO.setwarnings(False)
        self.Buzzer_Pin = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Buzzer_Pin,GPIO.OUT)

    def run(self,command):
        if command!="0":
            GPIO.output(self.Buzzer_Pin,True)
        else:
            GPIO.output(self.Buzzer_Pin,False)
    

# ROS implementation
class Buzzer_Sub:
    def __init__(self):
        self.buzz = Buzzer()
        self.timer = threading.Timer(0, None)
        self.sleep = False

    def start(self):
        rospy.init_node('buzzer_subscriber', anonymous=False)
        rospy.Subscriber("buzzer", OnOffDuration, self.callback)
        rospy.spin()

    def callback(self, data):

        # turn buzzer on for specified duration
        if data.on and not self.sleep:
            self.buzz.run("1")

            # handle duration with new requests overriding
            self.timer.cancel()
            self.timer = threading.Timer(data.duration, self.callback,
                args=[OnOffDuration(False, 0)])
            self.timer.start()

        # suppress buzzer for specified duration
        elif not data.on and data.duration > 0:
            self.sleep = True

            self.timer.cancel()
            self.timer = threading.Timer(data.duration, self.end_sleep)
            self.timer.start()

        # turn off buzzer for now
        elif not data.on:
            self.buzz.run("0")

    
    def end_sleep(self):
        self.sleep = False


if __name__ == "__main__":
    buzzer = None
    try:
        buzzer=Buzzer_Sub()
        buzzer.start()
    finally:
        buzzer.buzz.run("0")




