#!/usr/bin/python3

import time
import rospy
import os
import sys

#import LED2 as light
from bot_LED.msg import color

def changeColor(msg):
       
    os.system('sudo python3.7 ~/catkin_ws/src/robot_adeept/bot_led/src/LED2.py ' + str(msg.red) + ' ' + str(msg.blue) + ' ' + str(msg.green))

    

rospy.init_node('bot_led_node')
sub = rospy.Subscriber('/color', color, changeColor)

rospy.spin()


