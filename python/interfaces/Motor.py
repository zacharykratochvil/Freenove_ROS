#!/usr/bin/python
# from original Freenove code with small modifications

import time
from .PCA9685 import PCA9685
class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.setMotorModel(0,0,0,0)

    def duty_range(self,duty1,duty2,duty3,duty4):

        def validate(duty):
            if duty>4095:
                duty=4095
            elif duty<-4095:
                duty=-4095        
            return int(duty)
        
        duty1 = validate(duty1)
        duty2 = validate(duty2)
        duty3 = validate(duty3)
        duty4 = validate(duty4)

        return duty1,duty2,duty3,duty4
        
    def left_Upper_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(0,0)
            self.pwm.setMotorPwm(1,duty)
        elif duty<0:
            self.pwm.setMotorPwm(1,0)
            self.pwm.setMotorPwm(0,abs(duty))
        else:
            self.pwm.setMotorPwm(0,0)
            self.pwm.setMotorPwm(1,0)
    def left_Lower_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(3,0)
            self.pwm.setMotorPwm(2,duty)
        elif duty<0:
            self.pwm.setMotorPwm(2,0)
            self.pwm.setMotorPwm(3,abs(duty))
        else:
            self.pwm.setMotorPwm(2,0)
            self.pwm.setMotorPwm(3,0)
    def right_Upper_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(6,0)
            self.pwm.setMotorPwm(7,duty)
        elif duty<0:
            self.pwm.setMotorPwm(7,0)
            self.pwm.setMotorPwm(6,abs(duty))
        else:
            self.pwm.setMotorPwm(6,0)
            self.pwm.setMotorPwm(7,0)
    def right_Lower_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(4,0)
            self.pwm.setMotorPwm(5,duty)
        elif duty<0:
            self.pwm.setMotorPwm(5,0)
            self.pwm.setMotorPwm(4,abs(duty))
        else:
            self.pwm.setMotorPwm(4,0)
            self.pwm.setMotorPwm(5,0)
            
 
    def setMotorModel(self,duty1,duty2,duty3,duty4):
        duty1,duty2,duty3,duty4=self.duty_range(duty1,duty2,duty3,duty4)
        self.left_Upper_Wheel(-duty1)
        self.left_Lower_Wheel(-duty2)
        self.right_Upper_Wheel(-duty3)
        self.right_Lower_Wheel(-duty4)
            

            
def loop(PWM): 
    PWM.setMotorModel(2000,2000,2000,2000)       #Forward
    time.sleep(3)
    PWM.setMotorModel(-2000,-2000,-2000,-2000)   #Back
    time.sleep(3)
    PWM.setMotorModel(-500,-500,2000,2000)       #Left 
    time.sleep(3)
    PWM.setMotorModel(2000,2000,-500,-500)       #Right    
    time.sleep(3)
    PWM.setMotorModel(0,0,0,0)                   #Stop
    
def destroy(PWM):
    PWM.setMotorModel(0,0,0,0)                   

if __name__=='__main__':
    PWM = Motor()
    try:
        loop(PWM)
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy(PWM)
