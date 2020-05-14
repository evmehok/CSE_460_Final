#!/usr/bin/env python
import rospy
import numpy as np, math as m
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from rospy.exceptions import ROSInterruptException
from tf.transformations import euler_from_quaternion
# from simple_pid import PID

class FollowPath(object):
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
        self.last_cmdvel_command = Twist()
        self.twist = Twist()
        self._cmdvel_pub_rate = rospy.Rate(10)
        self.pose_sub = rospy.Subscriber('/robot1/odom', Odometry, self.odom_callback)
        self.pose = None
        self.orient = None
        self.shutdown_detected = False
        self.t = 0
        self.loc = np.array([0,0])
        self.distance = 0
        # self.v_PID = PID(1, 0, 0, setpoint=1)
        # self.x_PID = PID(0.5, 0.1, 0.005, setpoint=1)

        # Takes form [a, b] where a <= t < b
        self.t_param = [
            [0, 7.45],
            [7.45, 10.5],
            [10.5, 13.55],
            [13.55, 18.5],
            [18.5, 25.5],
            [25.5, 31.5],
            [31.5, 37.5],
            [37.5, 43],
            [43, 46],
            [46, 52],
            [52, 55],
            [55, 61],
            [61, 67],
            [67, 77]]

        # Takes form [a, b] where x = a*t + b
        self.x_param = [
            [1, 0],
            [1, 0],
            [-1, 21],
            [-1, 21],
            [-1, 21],
            [-1, 21],
            [-1, 21],
            [-1, 21],
            [-1, 21],
            [0, -25],
            [1, -77],
            [1, -77],
            [1, -77],
            [1, -77]]

        # Takes form [a, b] where y = a*t + b
        self.y_param = [
            [0, 0],
            [0.9836, -7.3279],
            [0.9836, -7.3279],
            [0, 6],
            [0.857, -9.857],
            [0, 12],
            [-2, 75],
            [0, 0],
            [1, -43],
            [1, -43],
            [1, -43],
            [0, 12],
            [-2, 134],
            [0, 0]]

    def increment(self):
        self.t += 0.1
        # print "t = %2.2f" %(self.t)

    def odom_callback(self, odometry):
        if self.loc == []:
            self.twist.angular.x = 0
            self.twist.angular.z = 0
            self.move_robot(self.twist)
            return
        
        alt = 0

        # Offset from (0,0)
        x_off = -3.25
        y_off = 2.90

        x, y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
        orientation = odometry.pose.pose.orientation
        (rot_x, rot_y, phi) = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
       
        # Desired Point
        x_d = self.loc[0] + x_off
        y_d = self.loc[1] + y_off

        x_diff = x_d - x
        y_diff = y_d - y
        
        # Phi Desired
        phi_d = m.atan2(y_diff,  x_diff)
        p_diff = m.degrees(phi_d - phi)
        
        if p_diff < -180: 
            p_diff = 360 - abs(p_diff)
            alt = 4
        elif p_diff > 180:
            p_diff = -360 + abs(p_diff)
            alt = 5
        else:
            alt = 0
                
        # Distance to Desired Point
        self.distance = m.hypot(x_diff, y_diff)

        # Proportional Controller for linear velocity
        kpv = 0.5
        v = kpv * self.distance
        # print "d = %0.2f x =  %0.2f y =  %0.2f t = %0.1f pdiff = %0.2f phi_d = %0.2f phi = %0.2f xdiff = %0.2f ydiff = %0.2f alt = %s"%(self.distance, x_d, y_d, self.t, p_diff, m.degrees(phi_d), m.degrees(phi), x_diff, y_diff, alt)
        
        # Proportional Controller for Angular velocity
        kpw = 0.5
        w = kpw * m.radians(p_diff)

        if p_diff > 20 or p_diff < -20:
            self.twist.linear.x = v / 4
        # elif -2 < p_diff < 2:
        #     self.twist.linear.x = 4*v
        else:
            self.twist.linear.x = v
        
        if w > 2:
            w = 1

        self.twist.angular.z = w
        self.move_robot(self.twist)

    def move_robot(self, twist_object):
        self.cmd_vel_pub.publish(twist_object)
                                    
    def clean_class(self):
        # Stop Robot
        twist = Twist()
        twist.angular.z = 0.0
        twist.linear.x = 0.0
        self.move_robot(twist)
        self.shutdown_detected = True

    def update_location(self):
        if self.t < self.t_param[0][1]:
            self.loc = np.array([self.x_param[0][0] * self.t + self.x_param[0][1], 
            self.y_param[0][0] * self.t + self.y_param[0][1]])

        elif len(self.t_param) <= 1:
            self.loc = []
        else:
            self.t_param.pop(0)
            self.x_param.pop(0)
            self.y_param.pop(0)
            self.loc = np.array([self.x_param[0][0] * self.t + self.x_param[0][1], 
            self.y_param[0][0] * self.t + self.y_param[0][1]])
        
        # print "x = %0.2f y = %0.2f t = %0.2f" %(self.loc[0], self.loc[1], self.t)
        # print(self.x_param[0], self.y_param[0])
        return self.loc


def main():
    ctrl_c = False

    try:
        rospy.init_node('follow_path_node', anonymous=True)
            
        follow = FollowPath()
        rate = rospy.Rate(20)
        
        def shutdownhook():
            # works better than the rospy.is_shut_down()
            follow.clean_class()
            rospy.loginfo("Received Shutdown Order")
            ctrl_c = True
        
        rospy.on_shutdown(shutdownhook)
                
        while not ctrl_c:
            
            if follow.distance < 0.4:
                follow.increment()
                follow.update_location()

            if follow.loc == []:
                ctrl_c = True

            rate.sleep()

    except ROSInterruptException:
        pass

    
if __name__ == '__main__':
    main()
