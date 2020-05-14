#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import RPi.GPIO as GPIO

# motor_EN_A: Pin7  |  motor_EN_B: Pin11
# motor_A:  Pin8,Pin10    |  motor_B: Pin13,Pin12

Motor_A_EN    = 4
Motor_B_EN    = 17

Motor_A_Pin1  = 26
Motor_A_Pin2  = 21
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

Dir_forward   = 0
Dir_backward  = 1

left_forward  = 1
left_backward = 0

right_forward = 0
right_backward= 1

pwn_A = 0
pwm_B = 0

def motorStop():#Motor stops
	GPIO.output(Motor_A_Pin1, GPIO.LOW)
	GPIO.output(Motor_A_Pin2, GPIO.LOW)
	GPIO.output(Motor_B_Pin1, GPIO.LOW)
	GPIO.output(Motor_B_Pin2, GPIO.LOW)
	GPIO.output(Motor_A_EN, GPIO.LOW)
	GPIO.output(Motor_B_EN, GPIO.LOW)


def setup():#Motor initialization
	global pwm_A, pwm_B
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(Motor_A_EN, GPIO.OUT)
	GPIO.setup(Motor_B_EN, GPIO.OUT)
	GPIO.setup(Motor_A_Pin1, GPIO.OUT)
	GPIO.setup(Motor_A_Pin2, GPIO.OUT)
	GPIO.setup(Motor_B_Pin1, GPIO.OUT)
	GPIO.setup(Motor_B_Pin2, GPIO.OUT)

	motorStop()
	try:
		pwm_A = GPIO.PWM(Motor_A_EN, 1000)
		pwm_B = GPIO.PWM(Motor_B_EN, 1000)
	except:
		pass


def destroy():
	motorStop()
	GPIO.cleanup()             # Release resource

def motor_left(status, direction, speed):#Motor 2 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_B_Pin1, GPIO.LOW)
		GPIO.output(Motor_B_Pin2, GPIO.LOW)
		GPIO.output(Motor_B_EN, GPIO.LOW)
	else:
		if direction == Dir_backward:
			GPIO.output(Motor_B_Pin1, GPIO.HIGH)
			GPIO.output(Motor_B_Pin2, GPIO.LOW)
			pwm_B.start(100)
			pwm_B.ChangeDutyCycle(speed)
		elif direction == Dir_forward:
			GPIO.output(Motor_B_Pin1, GPIO.LOW)
			GPIO.output(Motor_B_Pin2, GPIO.HIGH)
			pwm_B.start(0)
			pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed):#Motor 1 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_A_Pin1, GPIO.LOW)
		GPIO.output(Motor_A_Pin2, GPIO.LOW)
		GPIO.output(Motor_A_EN, GPIO.LOW)
	else:
		if direction == Dir_forward:#
			GPIO.output(Motor_A_Pin1, GPIO.HIGH)
			GPIO.output(Motor_A_Pin2, GPIO.LOW)
			pwm_A.start(100)
			pwm_A.ChangeDutyCycle(speed)
		elif direction == Dir_backward:
			GPIO.output(Motor_A_Pin1, GPIO.LOW)
			GPIO.output(Motor_A_Pin2, GPIO.HIGH)
			pwm_A.start(0)
			pwm_A.ChangeDutyCycle(speed)
	return direction


def move(speed, direction, turn, radius=0.6):   # 0 < radius <= 1
	#speed = 100
	if direction == 'forward':
		if turn == 'right':
			motor_left(0, left_backward, int(speed*radius))
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(0, right_backward, int(speed*radius))
		else:
			motor_left(1, left_forward, speed)
			motor_right(1, right_forward, speed)
	elif direction == 'backward':
		if turn == 'right':
			motor_left(0, left_forward, int(speed*radius))
			motor_right(1, right_backward, speed)
		elif turn == 'left':
			motor_left(1, left_backward, speed)
			motor_right(0, right_forward, int(speed*radius))
		else:
			motor_left(1, left_backward, speed)
			motor_right(1, right_backward, speed)
	elif direction == 'no':
		if turn == 'right':
			motor_left(1, left_backward, speed)
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(1, right_backward, speed)
		else:
			motorStop()
	else:
		pass




def move_motor(twist):
    setup()
    v_x = twist.linear.x
    w_z = twist.angular.z

    if v_x > 1:
        v_x = 1
    elif v_x < -1:
        v_x = -1
    if w_z > 1:
        w_z = 1
    elif w_z < -1:
        w_z = -1

    if v_x > 0 and w_z > 0:
        move(v_x*100, 'forward', 'left')
    elif v_x > 0 and w_z < 0:
        move(v_x*100, 'forward', 'right')
    elif v_x < 0 and w_z > 0:
        move(v_x*100, 'backward', 'left')
    elif v_x < 0 and w_z < 0:
        move(v_x*100, 'backwad', 'right')
    elif v_x > 0:
        move_direction("forward", v_x * 100)
    elif v_x < 0:
        move_direction("backward", v_x * 100 * (-1))
    elif w_z > 0:
        move_direction("left", w_z * 100)
    elif w_z < 0:
        move_direction("right", w_z*100*(-1))
    else:
        move_direction("NULL", 0)


def move_direction(direction, speed):
    if direction == "forward":
        motor_left(1,1,speed)
        motor_right(1,0,speed)
    elif direction == "backward":
        motor_left(1,0,speed)
        motor_right(1,1,speed)
    elif direction == "left":
        motor_left(1,0,speed)
        motor_right(1,0,speed)
    elif direction == "right":
        motor_left(1,1,speed)
        motor_right(1,1,speed)
    else:
        motorStop()


setup()

# Initiate a Node called 'cmd_vel_node'
rospy.init_node('move_bot_node')
rate = rospy.Rate(20)
sub = rospy.Subscriber('/move_bot_info', Twist, move_motor)

# make sure rospy runs
rospy.spin()
