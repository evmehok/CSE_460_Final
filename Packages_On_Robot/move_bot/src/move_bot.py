#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import RPi.GPIO as GPIO

class Motors():
    
    def __init__(self):
        # motor_EN_A: Pin7  |  motor_EN_B: Pin11
        # motor_A:  Pin8,Pin10    |  motor_B: Pin13,Pin12
        self.__Motor_A_EN    = 4
        self.__Motor_B_EN    = 17

        self.__Motor_A_Pin1  = 26
        self.__Motor_A_Pin2  = 21
        self.__Motor_B_Pin1  = 27
        self.__Motor_B_Pin2  = 18

        self.Dir_forward   = 0
        self.Dir_backward  = 1

        self.left_forward  = 1
        self.left_backward = 0

        self.right_forward = 0
        self.right_backward= 1

        self.pwm_A = 0
        self.pwm_B = 0
        
        
    def motorStop(self): #Motor stops
        GPIO.output(self.__Motor_A_Pin1, GPIO.LOW)
        GPIO.output(self.__Motor_A_Pin2, GPIO.LOW)
        GPIO.output(self.__Motor_B_Pin1, GPIO.LOW)
        GPIO.output(self.__Motor_B_Pin2, GPIO.LOW)
        GPIO.output(self.__Motor_A_EN, GPIO.LOW)
        GPIO.output(self.__Motor_B_EN, GPIO.LOW)
        
        
    def setup(self):  #Motor initialization
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.__Motor_A_EN, GPIO.OUT)
        GPIO.setup(self.__Motor_B_EN, GPIO.OUT)
        GPIO.setup(self.__Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(self.__Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(self.__Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(self.__Motor_B_Pin2, GPIO.OUT)

        self.motorStop()
        try:
            self.pwm_A = GPIO.PWM(self.__Motor_A_EN, 1000)
            self.pwm_B = GPIO.PWM(self.__Motor_B_EN, 1000)
        except:
            pass
        
    def destroy(self):
        motorStop()
        GPIO.cleanup()             # Release resource
        
    def motor_left(self, status, direction, speed):#Motor 2 positive and negative rotation
        if status == 0: # stop
            GPIO.output(self.__Motor_B_Pin1, GPIO.LOW)
            GPIO.output(self.__Motor_B_Pin2, GPIO.LOW)
            GPIO.output(self.__Motor_B_EN, GPIO.LOW)
        else:
            if direction == self.Dir_backward:
                GPIO.output(self.__Motor_B_Pin1, GPIO.HIGH)
                GPIO.output(self.__Motor_B_Pin2, GPIO.LOW)
                self.pwm_B.start(100)
                self.pwm_B.ChangeDutyCycle(speed)
            elif direction == self.Dir_forward:
                GPIO.output(self.__Motor_B_Pin1, GPIO.LOW)
                GPIO.output(self.__Motor_B_Pin2, GPIO.HIGH)
                self.pwm_B.start(0)
                self.pwm_B.ChangeDutyCycle(speed)
                
    def motor_right(self, status, direction, speed):#Motor 1 positive and negative rotation
        if status == 0: # stop
            GPIO.output(self.__Motor_A_Pin1, GPIO.LOW)
            GPIO.output(self.__Motor_A_Pin2, GPIO.LOW)
            GPIO.output(self.__Motor_A_EN, GPIO.LOW)
        else:
            if direction == self.Dir_forward:#
                GPIO.output(self.__Motor_A_Pin1, GPIO.HIGH)
                GPIO.output(self.__Motor_A_Pin2, GPIO.LOW)
                self.pwm_A.start(100)
                self.pwm_A.ChangeDutyCycle(speed)
            elif direction == self.Dir_backward:
                GPIO.output(self.__Motor_A_Pin1, GPIO.LOW)
                GPIO.output(self.__Motor_A_Pin2, GPIO.HIGH)
                self.pwm_A.start(0)
                self.pwm_A.ChangeDutyCycle(speed)
        return direction
            
    
def move_motor(twist):
    global motor
    
    
    v_x = twist.linear.x * 100
    w_z = twist.angular.z * 100

    # Make sure speed isn't above 100 (or below), else error
    if v_x > 100:
        v_x = 100
    elif v_x < -100:
        v_x = -100
    
    # Make sure speed isn't above 100 (or below), else error
    if w_z > 100:
        w_z = 100
    elif w_z < -100:
        w_z = -100
        
    #######################################################
    
    # Move Forward, Left
    if v_x > 0 and w_z > 0:
        v_x = v_x / 2           
        if v_x - w_z >= 0:
            motor.motor_left(1,1, min([v_x - w_z, 100]))
        else:
            motor.motor_left(1,0, -1 * max([v_x - w_z, -100]))            
        motor.motor_right(1,0, min([v_x + w_z, 100]))
    #######################################################
    
    # Move Forward, Right
    elif v_x > 0 and w_z < 0:
        v_x = v_x / 2
        if v_x + w_z >= 0:
            motor.motor_right(1,0, min([v_x + w_z, 100]))
        else:
            motor.motor_right(1,1, -1 * max([v_x + w_z, -100]))
        motor.motor_left(1,1, min([v_x - w_z, 100]))
    #######################################################
    
    # Move Back, Left
    elif v_x < 0 and w_z > 0:
        v_x = v_x / 2
        if v_x + w_z >= 0:
            motor.motor_right(1,1, min([v_x + w_z, 100]))
        else:
            motor.motor_right(1,1, -1 * max([v_x + w_z, -100]))
        
        motor.motor_left(1,0, -1 * max([v_x - w_z, -100]))
    
    #######################################################
    
    # Move Back, Right
    elif v_x < 0 and w_z < 0:
        v_x = v_x / 2
        if v_x + w_z >= 0:
            motor.motor_left(1,0, min([v_x + w_z, 100]))
        else:
            motor.motor_left(1,1, -1 * max([v_x + w_z, -100]))
        motor.motor_right(1,1, -1 * max([v_x - w_z, -100]))
    
    #######################################################
    
    # Move Forward
    elif v_x > 0:
        motor.motor_left(1,1,v_x)
        motor.motor_right(1,0,v_x)
    
    # Move Backward
    elif v_x < 0:
        motor.motor_left(1,0,v_x * -1)
        motor.motor_right(1,1,v_x * -1)
        
    # Spin Left
    elif w_z > 0:
        motor.motor_left(1,0,w_z)
        motor.motor_right(1,0,w_z)
    
    # Spin Right
    elif w_z < 0:
        motor.motor_left(1,1,w_z * -1)
        motor.motor_right(1,1,w_z * -1)
    
    else:
        motor.motorStop()

motor = Motors()
motor.setup()
rospy.init_node('move_bot_node')

sub = rospy.Subscriber('/move_bot_info', Twist, move_motor)

rate = rospy.Rate(20)

# make sure rospy runs
rospy.spin()
