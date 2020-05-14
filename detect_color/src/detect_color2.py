#! /usr/bin/env python3

import rospy, cv2, time, datetime,  sys, numpy as np, math as m
from cv_bridge import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

rospy.init_node('detect_color')

# allow the camera to warmup

font = cv2.FONT_HERSHEY_SIMPLEX

tor = 17
Y_lock = 0
X_lock = 0
UltraData = 1
servo_Pos = 300
photo = 255 * np.zeros(shape=[100, 100, 3], dtype=np.uint8)

    
def getImage(msg):
    global photo
    bridge = CvBridge()
    cv_image = bridge.compressed_imgmsg_to_cv2(msg)
    photo = cv_image


def getDirection(approx):
    x, y, dist = [], [], []
    index = -1
    if approx.shape[0] == 3:
        x.append(approx[0][0][0])
        y.append(approx[0][0][1])
        x.append(approx[1][0][0])
        y.append(approx[1][0][1])
        x.append(approx[2][0][0])
        y.append(approx[2][0][1])
        
        dist.append(int(m.sqrt((x[1]-x[0])**2 + (y[1]-y[0])**2)))
        dist.append(int(m.sqrt((x[2]-x[1])**2 + (y[2]-y[1])**2)))
        dist.append(int(m.sqrt((x[2]-x[0])**2 + (y[2]-y[0])**2)))
        
        if dist[0] > dist[1] and dist[0] > dist[2]: 
            index = 2
            x_end = x.pop(index)
            y_end = y.pop(index)
            x_begin = int((x.pop() + x.pop()) / 2)
            y_begin = int((y.pop() + y.pop()) / 2)
            
        elif dist[1] > dist[2] and dist[1] > dist[0]: 
            index = 0
            x_end = x.pop(index)
            y_end = y.pop(index)
            x_begin = int((x.pop() + x.pop()) / 2)
            y_begin = int((y.pop() + y.pop()) / 2)
                        
        elif dist[2] > dist[1] and dist[2] > dist[0]: 
            index = 1
            x_end = x.pop(index)
            y_end = y.pop(index)
            x_begin = int((x.pop() + x.pop()) / 2)
            y_begin = int((y.pop() + y.pop()) / 2)
                        
        else: 
            print("ERROR. Can't calculate for obtuse Triangle")
            x_end = -1
            y_end = -1
            x_begin = -1
            y_begin = -1
            
        return ([x_begin, y_begin], [x_end, y_end])

subCamera = rospy.Subscriber('/camera2/image_raw/compressed', CompressedImage, getImage)
cmd_vel_pub = rospy.Publisher('/move_bot_info', Twist, queue_size=1)
#pubLocation = rospy.Publisher('/move_bot', Vector2D, queue_size=1)

def publishTwist(twist):
    #cmd_vel_pub.publish(twist)
    pass


while(rospy.get_published_topics(namespace='/camera2/image_raw/compressed') == []):
    print('Waiting for camera to publish')
    time.sleep(1)
    falseStart = True

time.sleep(2)

while not rospy.is_shutdown():
    twist = Twist()

    frame = photo.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Color Range, Whiteness, Brightness (0 = black, 255 = white)
    lower_red = np.array([130, 60, 100])
    upper_red = np.array([200, 255, 255])
    
    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame,frame, mask= mask)
    
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)  
    
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    
    
    if len(cnts) > 0:
        cv2.putText(frame,'Target Detected',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA)
        c = max(cnts, key=cv2.contourArea)
        ratio = 1
        
        # Get shape and 
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        
        # If triangle detected, get line coordinates
        if approx.shape[0] == 3:
            line = getDirection(approx)
                
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            M = cv2.moments(c)
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)

            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(frame, [c], 0, (0, 255, 0), 3)
            
            # If coordinates exist, display them
            if sum(line[0]) >= 0:
                cv2.arrowedLine(frame, (line[0][0], line[0][1]), (line[1][0], line[1][1]), (0, 0, 0), 5)
                
                phi = 0
            
                theta = m.atan2(line[1][1] - line[0][1], line[1][0] - line[0][0])
                print(m.degrees(theta))
                
                p_diff = phi - theta
                
                # Proportional Controller for Angular velocity
                kpw = 1.5
                w = kpw * p_diff
                print(w)
                twist.angular.z = w * -1
                twist.angular.x = 0
                publishTwist(twist)
                
            else:
                twist.angular.z = 0
                twist.angular.x = 0
                publishTwist(twist)
        
        else:
            twist.angular.z = 0
            twist.angular.x = 0
            publishTwist(twist)
       
       
    else:
        cv2.putText(frame,'Target Detecting',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA)
        twist.angular.z = 0
        twist.angular.x = 0
        publishTwist(twist)


    
    # pubLocation.publish(coordinates)
    cv2.imshow('frame',frame)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    rospy.sleep(0.01)
