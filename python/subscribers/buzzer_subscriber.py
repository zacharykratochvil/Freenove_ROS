import threading
import RPi.GPIO as GPIO
from Command import COMMAND as cmd
from freenove_ros.msg import OnOffDuration


class Buzzer:
    def __init__(self):
        GPIO.setwarnings(False)
        Buzzer_Pin = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(Buzzer_Pin,GPIO.OUT)

    def run(self,command):
        if command!="0":
            GPIO.output(Buzzer_Pin,True)
        else:
            GPIO.output(Buzzer_Pin,False)
    

class Buzzer_Sub:
    def __init__(self):
        self.buzz = Buzzer()

    def start(self):
        rospy.init_node('buzzer_subscriber', anonymous=False)
        rospy.Subscriber("buzzer", String, self.callback)
        rospy.spin()

    def callback(self, data):
        if data.on:
            self.buzz.run("1")

            # handle duration with new requests overriding
            self.timer.cancel()
            self.timer = threading.Timer(data.duration, self.callback,
                args=OnOffDuration(False, 0))
            self.timer.start()

        else:
            self.buzz.run("0")


if __name__ == "__main__":
    Buzzer_Sub().start()




