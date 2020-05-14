#! /usr/bin/env python3

import rospy, cv2, time, datetime, PID, sys
import numpy as np

from picamera.array import PiRGBArray
from picamera import PiCamera

from geometry_msgs.msg import Twist
from bot_LED.msg import color
from std_msgs.msg import Int32
from std_msgs.msg import Float32


rospy.init_node('follow_color_node')


pid = PID.PID()
pid.SetKp(0.5)
pid.SetKd(0)
pid.SetKi(0)
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)
font = cv2.FONT_HERSHEY_SIMPLEX

colorUpper = (44,255,255)
colorLower = (24,100,100)
tor = 17
Y_lock = 0
X_lock = 0
UltraData = 1
servo_Pos = 300

def getUltra(msg):
    global UltraData
#     print (msg.data)
    UltraData = msg.data

def getServo(msg):
    global servo_Pos
    
    servo_Pos = msg.data





if __name__=="__main__":

    pubTwist = rospy.Publisher('/move_bot_info', Twist, queue_size=1)
    pubColor = rospy.Publisher('/color', color, queue_size=1)
    pubServo = rospy.Publisher('bot_servo_info', Int32, queue_size=1)
    subUltra = rospy.Subscriber('/bot_ultra_info', Float32, getUltra)
    subServo = rospy.Subscriber('/bot_servo_get', Int32, getServo)



    twist = Twist()
    servo = Int32()
   #  color = Int32()
#     servo = Float32()
#     color = color()
    try: 
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        

            frame_image = frame.array
            cv2.line(frame_image,(300,240),(340,240),(128,255,128),1)
            cv2.line(frame_image,(320,220),(320,260),(128,255,128),1)
            timestamp = datetime.datetime.now()
            mask = []


            ####>>>OpenCV Start<<<####
            hsv = cv2.cvtColor(frame_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, colorLower, colorUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            if len(cnts) > 0:
                cv2.putText(frame_image,'Target Detected',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA)
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                X = int(x)
                Y = int(y)
                if radius > 10:
                    cv2.rectangle(frame_image,(int(x-radius),int(y+radius)),(int(x+radius),int(y-radius)),(255,255,255),1)

                if Y < (240-tor):
                    error = (240-Y)/5
                    servo.data = int(round((pid.GenOut(error)),0))
                    pubServo.publish(servo)
    #                 servo.camera_ang('lookup',outv)
                    Y_lock = 1
                elif Y > (240+tor):
                    error = (Y-240)/5
                    servo.data = -1*int(round((pid.GenOut(error)),0))
                    pubServo.publish(servo)
    #                 servo.camera_ang('lookdown',outv)
                    Y_lock = 1
                else:
                    Y_lock = 1


                if X < (320-tor*3):
                    twist.linear.x = 0
                    twist.angular.z = 1
    #                     move.move(70, 'no', 'right', 0.6)
                    #print('turn right')
                    #time.sleep(0.1)
                    #move.motorStop()
                    X_lock = 0
                elif X > (330+tor*3):
                    twist.linear.x = 0
                    twist.angular.z = -1
    #                     move.move(70, 'no', 'left', 0.6)
                    #print('turn left')
                    #time.sleep(0.1)
    #                     move.motorStop()
                    X_lock = 0
                else:
                    twist.linear.x = 0
                    twist.angular.z = 0
                    X_lock = 1

                if X_lock == 1 and Y_lock == 1:
                    if UltraData > 0.5 or True:
                        twist.linear.x = 1
                        twist.angular.z = 0
    #                         move.move(70, 'forward', 'no', 0.6)
                        print(UltraData, "great")
                    #elif UltraData < 0.4:
                        #twist.linear.x = -1
                        #twist.angular.z = 0
                            #move.move(70, 'backward', 'no', 0.6)
                        #print(UltraData, "less")
                    else:
                        twist.linear.x = 0
                        twist.angular.z = 0
                
                if UltraData < 0.3:
                    twist.linear.x = 0
                    twist.angular.z = 0
                    print(UltraData, "close")
                
                if UltraData < 0.2:
                    twist.linear.x = -1
                    twist.angular.z = 0
                    
                if servo_Pos < 150:
                    twist.linear.x = -1


                pubTwist.publish(twist)

            else:
                cv2.putText(frame_image,'Target Detecting',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA)
                twist.linear.x = 0
                twist.angular.z = 0
                pubTwist.publish(twist)

            cv2.imshow("Frame", frame_image)              # Display image
            cv2.imshow("Mask", mask)
            key = cv2.waitKey(1) & 0xFF
            rawCapture.truncate(0)
            
    except KeyBoardInterrupt:
        sys.exit(0)
