#!/usr/bin/python3

import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Float32

Tr = 11
Ec = 8

def checkdist():       #Reading distance
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(Ec, GPIO.IN)
    GPIO.output(Tr, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(Tr, GPIO.LOW)
    while not GPIO.input(Ec):
        if KeyboardInterrupt:
            break
        pass
    t1 = time.time()
	
    while GPIO.input(Ec):
        if KeyboardInterrupt:
            break
        pass
    t2 = time.time()
    dist = (t2-t1)*340/2
    return dist

rospy.init_node('bot_ultra_node')
pub = rospy.Publisher('/bot_ultra_info', Float32, queue_size = 1)
rate = rospy.Rate(20)
count = Float32()
count.data = 0

while not rospy.is_shutdown():
    try:
        dist = checkdist()
        pub.publish(dist)
        rate.sleep()
    
    except KeyboardInterrupt:
        break


