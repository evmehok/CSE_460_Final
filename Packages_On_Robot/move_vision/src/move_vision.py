#! /usr/bin/env python3

import rospy, time, datetime, PID, sys
import numpy as np
from geometry_msgs.msg import Twist
from bot_LED.msg import color
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Vector2D

rospy.init_node('move_vision_node')

class Vision:
    
    def __init__(self):
        self.ultraData = 1.0
        self.servoPos = 300
        self.location = [0, 0]
        self.linearSpeed = 0.75
        self.angularSpeed = 0.9
        
        self.pubTwist = rospy.Publisher('/move_bot_info', Twist, queue_size=1)
        self.pubColor = rospy.Publisher('/color', color, queue_size=1)
        self.pubServo = rospy.Publisher('/bot_servo_info', Int32, queue_size=1)
        
        self.subUltra = rospy.Subscriber('/bot_ultra_info', Float32, self.ultra_callback)
        self.subLocation = rospy.Subscriber('/move_bot', Vector2D, self.location_callback)
        subServo = rospy.Subscriber('/bot_servo_get', Int32, self.servo_callback)

        
    def ultra_callback(self, msg):
        self.ultraData = msg.data
    
    def servo_callback(self, msg):
        self.servoPos = msg.data
        print(servoPos)
    
    def location_callback(self, data):
        self.location = [data.x, data.y]



vision = Vision()
twist = Twist()
servo = Int32()

pid = PID.PID()
pid.SetKp(0.5)
pid.SetKd(0)
pid.SetKi(0)

tor = 2
Y_lock = 0
X_lock = 0


while not rospy.is_shutdown():
    X = vision.location[0]
    Y = vision.location[1]
    ultraData = vision.ultraData
    
    if X == 0 and Y == 0:
        twist.linear.x = 0
        twist.angular.z = 0
        vision.pubTwist.publish(twist)
        #continue
        
    else:
        
        # Move 
        if vision.location[1] < (240-tor):
            error = (240-Y)/6
            servo.data = int(round((pid.GenOut(error)),0))
            #print(servo.data, "down")
            vision.pubServo.publish(servo)
            Y_lock = 1
        elif Y > (240+tor):
            error = (Y-240)/6
            servo.data = -1*int(round((pid.GenOut(error)),0))
            #print(servo.data, "up")
            vision.pubServo.publish(servo)
            Y_lock = 1
            
        else:
            Y_lock = 1


        if X < (320-tor*3):
            twist.linear.x = 0 * vision.linearSpeed
            twist.angular.z = 1 * vision.angularSpeed
    #                     move.move(70, 'no', 'right', 0.6)

            X_lock = 0
        elif X > (330+tor*3):
            twist.linear.x = 0 * vision.linearSpeed
            twist.angular.z = -1 * vision.angularSpeed
    #                     move.move(70, 'no', 'left', 0.6)
            X_lock = 0
        else:
            twist.linear.x = 0
            twist.angular.z = 0
            X_lock = 1

        if X_lock == 1 and Y_lock == 1:
            if ultraData > 0.5 or True:
                twist.linear.x = 1 * vision.linearSpeed
                twist.angular.z = 0 * vision.angularSpeed
    #                         move.move(70, 'forward', 'no', 0.6)
                #print(UltraData, "great")
            #elif UltraData < 0.4:
                #twist.linear.x = -1
                #twist.angular.z = 0
                    #move.move(70, 'backward', 'no', 0.6)
                #print(UltraData, "less")
            else:
                twist.linear.x = 0
                twist.angular.z = 0
        
        if ultraData < 0.3:
            twist.linear.x = 0 * vision.linearSpeed
            twist.angular.z = 0 * vision.angularSpeed
            #print(UltraData, "close")
        
        if ultraData < 0.2:
            twist.linear.x = -1 * vision.linearSpeed
            twist.angular.z = 0 * vision.angularSpeed
            
        #if servo_Pos < 150:
            #twist.linear.x = -1


        vision.pubTwist.publish(twist)
    rospy.sleep(2)
    



