#!/usr/bin/env python
# coding: utf-8
import rospy
import tf
import math
import time
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from hsr_ros.srv import *

#indent = 1 tabs
# S:1000	(1000cm直進)
# T:90		(90度回転、反時計回り正回転)

###############################################################################

class OdomBaseController:
	def __init__(self):

		#初期化
		self.order_deg = 0
		self.order_cm = 0

		self.pub_twist = rospy.Publisher('/hsrb/opt_command_velocity', Twist, queue_size=10)
		self.sub_string = rospy.Service('base_ctrl', odom_base, self.string_callback)

		rospy.loginfo("Base_ctrl is OK.")

	def string_callback(self,req):
		check = self.Check_Command(req)
		if check == False:
			rospy.loginfo("The command is fail.")
			return False
		else:
			if "T" in req.req_str:
				self.order_deg = self.Read_Value(req)
				rospy.loginfo("Base_ctrl: Turn: %f(deg)" % (self.order_deg))
				send_twist = Twist()
				if self.order_deg > 0:
					send_twist.angular.z = 0.174533#deg2rad(10)
				else:
					send_twist.angular.z = -0.174533#deg2rad(-10)
				wait_time = float(abs(self.order_deg) / 10)
				start_time = rospy.Time.now()
				while (start_time + rospy.Duration(wait_time)) > rospy.Time.now():
					self.pub_twist.publish(send_twist)
					rospy.sleep(0.1)
				#stop
				send_twist.angular.z = 0.0
				self.pub_twist.publish(send_twist)
			elif "X" in req.req_str:
				self.order_cm = self.Read_Value(req)#cm
				rospy.loginfo("Base_ctrl: Straight: %f(cm)" % (self.order_cm))
				send_twist = Twist()
				if self.order_cm > 0:
					send_twist.linear.x = 0.10
				else:
					send_twist.linear.x = -0.10
				wait_time = float(abs(self.order_cm) / 10)
				start_time = rospy.Time.now()
				while (start_time + rospy.Duration(wait_time)) > rospy.Time.now():
					self.pub_twist.publish(send_twist)
					rospy.sleep(0.1)
				#stop
				send_twist.linear.x = 0.0
				self.pub_twist.publish(send_twist)
			elif "Y" in req.req_str:
				self.order_cm = self.Read_Value(req)#cm
				rospy.loginfo("Base_ctrl: Sideways: %f(cm)" % (self.order_cm))
				send_twist = Twist()
				if self.order_cm > 0:
					send_twist.linear.y = 0.10
				else:
					send_twist.linear.y = -0.10
				wait_time = float(abs(self.order_cm) / 10)
				start_time = rospy.Time.now()
				while (start_time + rospy.Duration(wait_time)) > rospy.Time.now():
					self.pub_twist.publish(send_twist)
					rospy.sleep(0.1)
				#stop
				send_twist.linear.y = 0.0
				self.pub_twist.publish(send_twist)

		rospy.loginfo("Moving Finished")
		return str(req.req_str) + " Finished"

	def Check_Command(self, line):
		cmd_line = line.req_str[0:2]
		value_line = line.req_str[2:len(line.req_str)]
		for i in range(len(cmd_line)):
			key = cmd_line[i]
			if key == 'T' or key == 'X' or key == 'Y' or key == ':':
				pass
			else:
				rospy.loginfo("check_command cmd error")
				rospy.loginfo(line.req_str)
				return False
		for i in range(len(value_line)):
			key = value_line[i]
			if key >= '0' and key <= '9' or key == '-' or key == '.':
				pass
			else:
				rospy.loginfo("check_command cmd error")
				rospy.loginfo(line.req_str)
				return False
		return True

	def Read_Value(self, line):
		value_str = line.req_str[2:len(line.req_str)]
		value = float(value_str)
		return value

if __name__ == '__main__':

	rospy.init_node('odom_base_controller')

	obc = OdomBaseController()
	rospy.spin()
