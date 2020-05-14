#! /usr/bin/env python3

import rospy, time, datetime, sys
import numpy as np, math as m
from geometry_msgs.msg import Twist
from bot_LED.msg import color
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Vector2D
from rospy.exceptions import ROSInterruptException

class ROS_Params:
	def __init__(self):
		self.twist = Twist()
		self.servo = Int32()
		self.pose = ()
		self.loc = np.array([0, 0])
		self.theta = None
		self.x_dest = 0
		self.y_dest = 0
		self.kpv = 0.3
		self.kpw = 0.4
		self.t = 0
		self.stop = False
		self.distance = 0
		#self.pid = PID.PID()
		#self.setPID_params()
		self.color = color()
		self.ultraData = []
		self.servoPos = None
		self.location = None
		self.pubTwist = rospy.Publisher('/move_bot_info', Twist, queue_size=1)
		self.pubColor = rospy.Publisher('/color', color, queue_size=1)
		self.pubServo = rospy.Publisher('/bot_servo_info', Int32, queue_size=1)
		        
		self.subUltra = rospy.Subscriber('/bot_ultra_info', Float32, self.ultra_callback)
		self.subLocation = rospy.Subscriber('/move_bot', Vector2D, self.location_callback)
		self.subServo = rospy.Subscriber('/bot_servo_get', Int32, self.servo_callback)
		self.subPose = rospy.Subscriber('/botPose', Float32MultiArray, self.pose_callback)
		
		self.t_param = [
			[0, 30/100],
			[30/100, 50/100],
			[50/100, 80/100],
			[80/100, 1]]
		
		self.x_param = [
			[100, 20],
			[0, 50],
			[-100, 100],
			[0, 20]]

		self.y_param = [
			[0, 20],
			[100, -10],
			[0, 40],
			[-100, 120]]

	def setPID_params(self):
		self.pid.SetKp(0.5)
		self.pid.SetKd(0)
		self.pid.SetKi(0)
        
	def ultra_callback(self, msg):
		if len(self.ultraData) > 10:
			self.ultraData.pop(0)
		self.ultraData.append(msg.data*1000)
		#print(msg.data*1000)
	
	def pose_callback(self, msg):
		self.pose = msg.data
	
	def servo_callback(self, msg):
		self.servoPos = msg.data

	def location_callback(self, data):
		self.location = [data.x, data.y]

	def get_Velocity(self, pose):
		y_diff = self.y_dest - pose[1]
		x_diff = self.x_dest - pose[0]
		self.distance = m.hypot(x_diff, y_diff)
		phi = m.atan2(y_diff, x_diff)
		phi_d = phi - pose[2]
		print(m.degrees(phi), m.degrees(pose[2]), m.degrees(phi_d))
		print(pose[0], pose[1])
		print(self.x_dest, self.y_dest)
		# Proportional Controller for linear velocity
		v = self.kpv * self.distance 
		
		phi_d = m.degrees(phi_d)
		
		if phi_d < -180: 
			phi_d = 360 - abs(phi_d)
		elif phi_d > 180:
			phi_d = -360 + abs(phi_d)

		
		if phi_d > 40 or phi_d < -40:
			v = 0
			phi_d *= 1.2

		else:
			v = v
			
		phi_d = m.radians(phi_d)
		
		w = self.kpw * phi_d
		
		if v > 0.7:
			v = 0.7
		elif v < -0.7:
			v = -0.7
		
		if w > 0.9:
			w = 0.9
		elif w < -0.9:
			w = -0.9
			
		return v, w

	def update_location(self):
		if self.t < self.t_param[0][1]:
			self.loc = np.array([self.x_param[0][0] * self.t + self.x_param[0][1], 
			self.y_param[0][0] * self.t + self.y_param[0][1]])

		elif len(self.t_param) <= 1:
			self.loc = []
			self.x_dest = -1
			self.y_dest = -1
			self.stop = True
			return
			
		else:
			self.t_param.pop(0)
			self.x_param.pop(0)
			self.y_param.pop(0)
			self.loc = np.array([self.x_param[0][0] * self.t + self.x_param[0][1], 
			self.y_param[0][0] * self.t + self.y_param[0][1]])
		
		self.x_dest = self.loc[0]
		self.y_dest = self.loc[1]
		# print "x = %0.2f y = %0.2f t = %0.2f" %(self.loc[0], self.loc[1], self.t)
		# print(self.x_param[0], self.y_param[0])

	def increment(self):
		self.t += 0.01
		# print "t = %2.2f" %(self.t)

	def publishTwist(self, v, w):
		self.twist.linear.x = v
		self.twist.angular.z = w
		self.pubTwist.publish(self.twist)
		
def average(data):
	if len(data) == 0: return 0
	return sum(data) / len(data)
    


def main(): 
	ctrl_c = False

	try:
		rospy.init_node('search_cat')
			
		follow = ROS_Params()
		rate = rospy.Rate(20)
		
		def shutdownhook():
			# works better than the rospy.is_shut_down()
			#follow.clean_class()
			rospy.loginfo("Received Shutdown Order")
			ctrl_c = True
		
		rospy.on_shutdown(shutdownhook)
				
		
		while not ctrl_c:
			pose = follow.pose
			#print(pose)
			#print(pose == ())
			
			if not pose == ():
				if follow.distance < 6:
					follow.increment()
					follow.update_location()
					if follow.stop == True:
						break
					v, w = follow.get_Velocity(pose)
					follow.publishTwist(v, w)
					
				else:
					follow.update_location()
					if follow.stop == True:
						break
					v, w = follow.get_Velocity(pose)
					follow.publishTwist(v, w)
				
			else:
				follow.publishTwist(0, 0)
			
			rate.sleep()
			
		follow.publishTwist(0, 0)
	except ROSInterruptException:
		pass


if __name__ == '__main__':
	main()
	#tor = 2
	#Y_lock = 0
	#X_lock = 0
	#maxDist = 1.0
	#params = ROS_Params()
	#currentAction = "Forward"
	
	#while not rospy.is_shutdown():
		#print(params.ultraData)
		#rospy.sleep(1)
		#if currentAction == "Forward":
			
		
			#if average(params.ultraData) >= maxDist:
				#params.twist.x = 0.6
				#params.twist.z = 0.0
				#currentAction = "Forward"
				
			#elif average(params.ultraData) < params.ultraData[-1]:
				#params.twist.x = 0.0
				#params.twist.z = -0.7
				#currentAction = "Search Left"
				
				##rospy.sleep
