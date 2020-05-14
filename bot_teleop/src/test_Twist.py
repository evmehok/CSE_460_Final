#!/usr/bin/env python3


import rospy, numpy as np, math as m
from geometry_msgs.msg import Twist
from rospy.exceptions import ROSInterruptException


pubTwist = rospy.Publisher('/move_bot_info', Twist, queue_size=1)
var = False

if __name__=="__main__":
    
    ctrl_c = False
    twist = Twist()
    
    try:
        rospy.init_node('test_Twist')
            
        rate = rospy.Rate(20)
        
        def shutdownhook():
            # works better than the rospy.is_shut_down()
            twist.linear.x = 0
            twist.angular.z = 0
            pubTwist.publish(twist)
            rospy.loginfo("Received Shutdown Order")
            ctrl_c = True
            
        
        rospy.on_shutdown(shutdownhook)
                
        while not ctrl_c:
            
            if var == False:
                twist.linear.x = 0.4
                twist.angular.z = 0
                pubTwist.publish(twist)
                var = True
            else:
                twist.linear.x = 0.1
                twist.angular.z = 0
                pubTwist.publish(twist)
                var = False
            
            rospy.sleep(0.11)

    except ROSInterruptException:
        pass
