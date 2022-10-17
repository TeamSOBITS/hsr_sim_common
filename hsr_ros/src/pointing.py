#!/usr/bin/env python3
# coding: utf-8
import sys
import copy
import rospy
import tf
from trajectory_msgs.msg import *
import std_msgs.msg
import math
import numpy
from hsr_ros.srv import *

class Pointing:
	def __init__(self):

		self.pub_arm_traj = rospy.Publisher("/hsrb/arm_trajectory_controller/command", JointTrajectory, queue_size=1)
		self.pub_head_traj = rospy.Publisher("/hsrb/head_trajectory_controller/command", JointTrajectory, queue_size=1)
		self.pub_gripper_traj = rospy.Publisher("/hsrb/gripper_trajectory_controller/command", JointTrajectory, queue_size=1)
		self.servise = rospy.Service('motion_ctrl', robot_motion, self.Robot_motion)
		self.listener = tf.TransformListener()

		# objectの座標値調整のパラメーター(cm)
		self.param_object_x = 0.0# +-前後
		self.param_object_y = 0.0# +-左右
		self.param_object_z = 0.0# +-上下
		self.param_grasp_depth = 5.0# 手で物体を握る深さ

		#物体把持時に使用する変数
		self.arm_lift_val = 0.0
		self.arm_flex_val = 0.0
		self.arm_flexup_val = 0.0
		self.arm_roll_val = 0.0
		self.wrist_flex_val = 0.0
		self.move_x = 0.0
		self.move_y = 0.0

		rospy.loginfo("Body_ctrl is OK.")

	def Robot_motion(self, srv_msg):
		target_motion = srv_msg.motion_type
		rospy.loginfo("Robot_motion [%s]", target_motion)

		if target_motion == 'INITIAL_POSE':
			self.motion_initial_pose()
		elif target_motion == 'POINTING_POSE':
			self.motion_pointing_pose()
		elif target_motion == 'RESET':
			self.motion_reset()

		return robot_motionResponse(True)

	def motion_pointing_pose(self):
		self.move_hand_open(False)
		self.move_head('head_pan_joint', 0.0)
		self.move_head('head_tilt_joint', 0.0)
		self.move_arm('arm_roll_joint', 0.0)
		self.move_arm('arm_lift_joint', 0.25)
		self.move_arm('arm_flex_joint', -0.6)
		self.move_arm('wrist_flex_joint', -0.6)

	def motion_initial_pose(self):
		self.move_hand_open(False)
		self.move_head('head_pan_joint', 0.0)
		self.move_head('head_tilt_joint', 0.0)
		self.move_arm('arm_lift_joint', 0.0)
		self.move_arm('arm_flex_joint', 0.0)
		self.move_arm('arm_roll_joint', -1.5708) #deg2rad(-90)
		self.move_arm('wrist_flex_joint', -1.5708) #deg2rad(-90)

	def motion_reset(self):
		self.move_hand_open(False)
		self.move_head('head_pan_joint', 0.0)
		self.move_head('head_tilt_joint', 0.0)
		self.move_arm('arm_lift_joint', 0.0)
		self.move_arm('arm_flex_joint', 0.0)
		self.move_arm('arm_roll_joint', 0.0) #deg2rad(-90)
		self.move_arm('wrist_flex_joint', 0.0) #deg2rad(-90)

	def move_head(self, joint_name, value):
		point = JointTrajectoryPoint()
		point.time_from_start = rospy.Duration(2)
		point.positions.append(value)

		send_data = JointTrajectory()
		send_data.joint_names.append(joint_name)
		send_data.points.append(point)
		#send_dataを送信
		self.pub_head_traj.publish(send_data)

	def move_arm(self, joint_name, value):
		point = JointTrajectoryPoint()
		point.time_from_start = rospy.Duration(2)
		point.positions.append(value)

		send_data = JointTrajectory()
		send_data.joint_names.append(joint_name)
		send_data.points.append(point)
		#send_dataを送信
		self.pub_arm_traj.publish(send_data)

	def move_hand_open(self, msg):
		point = JointTrajectoryPoint()
		point.time_from_start = rospy.Duration(2)
		if msg == True:
			point.positions = [+0.611, -0.611]
		else:
			point.positions = [-0.05, +0.05]

if __name__ == '__main__':
	rospy.init_node('pointing_node')
	point = Pointing()
	rospy.spin()
