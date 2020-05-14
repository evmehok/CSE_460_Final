#!/usr/bin/env python3
from __future__ import division
import rospy
import time
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import sys
import Adafruit_PCA9685


'''
change this form 1 to 0 to reverse servos
'''
look_direction = 1


pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

look_max = 310
look_min = 120

org_pos = 300

def ctrl_range(raw, max_genout, min_genout):
	if raw > max_genout:
		raw_output = max_genout
	elif raw < min_genout:
		raw_output = min_genout
	else:
		raw_output = raw
	return int(raw_output)


def camera_ang(direction, ang):
	global org_pos

	
	if ang == 'no':
		ang = 50
	if look_direction:
		if direction == 'lookdown':
			org_pos+=ang
			org_pos = ctrl_range(org_pos, look_max, look_min)
		elif direction == 'lookup':
			org_pos-=ang
			org_pos = ctrl_range(org_pos, look_max, look_min)
		elif direction == 'home':
			org_pos = 300
	else:
		if direction == 'lookdown':
			org_pos-=ang
			org_pos = ctrl_range(org_pos, look_max, look_min)
		elif direction == 'lookup':
			org_pos+=ang
			org_pos = ctrl_range(org_pos, look_max, look_min)
		elif direction == 'home':
			org_pos = 300	
    
    
	pwm.set_all_pwm(0,org_pos)


	

def clean_all():
	pwm.set_all_pwm(0, 0)


def move_servo(msg):
  degree = msg.data

  if degree == 0:
     camera_ang('home', 'no')
  elif degree > 0:
     camera_ang('lookup',degree)
  elif degree < 0:
     camera_ang('lookdown', -1*degree)
  else:
     return

  return


rospy.init_node('bot_servo_node')
rate = rospy.Rate(0.5)
sub = rospy.Subscriber('/bot_servo_info', Int32, move_servo)
pub = rospy.Publisher('bot_servo_get', Int32, queue_size=1)
servo_Pos = Int32()
servo_Pos.data = org_pos
pub.publish(servo_Pos)

# make sure rospy runs
rospy.spin()
