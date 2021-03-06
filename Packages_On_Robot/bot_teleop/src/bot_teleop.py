#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist
from bot_LED.msg import color
import sys, select, termios, tty

import tkinter as tk
import time

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 1
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

#def call_forward():
    #twist = Twist()
    #twist.linear.x = 1; twist.linear.y = 0; twist.linear.z = 0
    #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    #pub.publish(twist)

#def call_backward():
    #twist = Twist()
    #twist.linear.x = -1; twist.linear.y = 0; twist.linear.z = 0
    #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    #pub.publish(twist)


#wrapper(main)
rospy.init_node('bot_teleop_node')
prevKey = ""
firstRun = True

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)


    pub = rospy.Publisher('/move_bot_info', Twist, queue_size=1)
    pubColor = rospy.Publisher('/color', color, queue_size=1)
    colorVar = color()


    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    try:
        print (msg)
        print (vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0
                if speed > 1:
                    speed = 1
                if turn > 1:
                    turn = 1

                if (status == 14):
                    print (msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
                count = 0

            else:
                count = count + 1
                if count > 1:    # adjust this if your robot has a delayed start. has to do with keyboard settings when a key is held down
                    x = 0
                    th = 0

                    conrol_speed = 0
                    control_turn = 0
                if (key == '\x03'):

                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 1 )

            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 1 )

            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 1)
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)

            # set color of LED lights
            if key == 'i' or key == 'j' or key == 'l':
                colorVar.red = 255
                colorVar.green = 255
                colorVar.blue = 255
                key == 'i'

            elif key == 'j':
                colorVar.red = 0
                colorVar.green = 100
                colorVar.blue = 0
                key = 'i'

            elif key == 'l':
                colorVar.red = 0
                colorVar.green = 0
                colorVar.blue = 100
                key = 'i'

            elif key == ',':
                colorVar.red = 255
                colorVar.green = 0
                colorVar.blue = 0
            elif key == 'k':
                colorVar.red = 0
                colorVar.green = 0
                colorVar.blue = 0

            if prevKey != key and key != "":
                pubColor.publish(colorVar)
                prevKey = key

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        pass
        #print e

    finally:
        twist = Twist()
        colorVar = color()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        colorVar.red = 0
        colorVar.green = 0
        colorVar.blue = 0
        pubColor.publish(colorVar)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
