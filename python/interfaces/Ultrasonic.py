#!/usr/bin/python
# code partially from original Freenove code
# otherwise by Zachary Kratochvil

import time
import RPi.GPIO as GPIO
import numpy as np

class Ultrasonic:
    def __init__(self, smoothing=5):
        self.smoothing = smoothing

        # intialize GPIO
        GPIO.setwarnings(False)
        self.trigger_pin = 27
        self.echo_pin = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin,GPIO.OUT)
        GPIO.setup(self.echo_pin,GPIO.IN)

    def send_trigger_pulse(self):
        GPIO.output(self.trigger_pin,True)
        time.sleep(0.00015)
        GPIO.output(self.trigger_pin,False)
     
    # take median of self.smoothing many measurements
    def get_distance(self):
        
        distance_cm=-1*np.ones(self.smoothing)
        for i in range(len(distance_cm)):
            
            # take and store measurement
            self.send_trigger_pulse()
            
            timeout = GPIO.wait_for_edge(self.echo_pin, GPIO.RISING, timeout=35)
            if timeout is None:
                continue
            start = time.time()
            
            timeout = GPIO.wait_for_edge(self.echo_pin, GPIO.FALLING, timeout=35)
            if timeout is None:
                continue
            finish = time.time()

            pulse_len = finish-start
            distance_cm[i] = pulse_len/0.000058
        
        # not accurate above 5m
        valids = np.nonzero((distance_cm > 0) & (distance_cm < 500))[0]
        if len(valids) == 0:
            return -1
        else:
            distance_cm=sorted(distance_cm[valids])
            middle = len(distance_cm) // 2
            median = distance_cm[middle]
            return median
          
        
# Main program logic follows:
if __name__ == '__main__':

    print ('Program is starting ... ')
    ultrasonic=Ultrasonic()
    try:
        while True:
            print(ultrasonic.get_distance())
            time.sleep(.1)
    except KeyboardInterrupt:
        pass
