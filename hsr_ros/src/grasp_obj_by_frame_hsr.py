#!/usr/bin/env python3
# coding: utf-8
import sys
import copy
import rospy
import tf
import geometry_msgs.msg
from trajectory_msgs.msg import *
import std_msgs.msg
import math
import numpy
from hsr_ros.srv import *

class Grasping:
	def __init__(self):

		self.pub_arm_traj = rospy.Publisher("/hsrb/arm_trajectory_controller/command", JointTrajectory, queue_size=10)
		self.pub_head_traj = rospy.Publisher("/hsrb/head_trajectory_controller/command", JointTrajectory, queue_size=10)
		self.pub_gripper_traj = rospy.Publisher("/hsrb/gripper_trajectory_controller/command", JointTrajectory, queue_size=10)
		self.servise = rospy.Service('grasp_ctrl', grasp_ctrl, self.Grasp_by_frame)
		self.servise = rospy.Service('motion_ctrl', robot_motion, self.Robot_motion)
		self.servise = rospy.Service('arm_height', arm_height, self.Arm_height)
		self.servise = rospy.Service('put_ctrl', put_ctrl, self.Put_ctrl)
		self.servise = rospy.Service('detect_ctrl', detect_ctrl, self.Detect_ctrl)
		self.servise = rospy.Service('object_reach_ctrl', grasp_ctrl, self.Object_raech)
		self.servise = rospy.Service('watch_motion', watch_motion, self.Wath_motion)
		self.servise = rospy.Service('gripper_move', gripper_move, self.Gripper_move)
		self.servise = rospy.Service('gripper_open', gripper_ctrl, self.Gripper_open)
		self.servise = rospy.Service('gripper_close', gripper_ctrl, self.Gripper_close)
		self.listener = tf.TransformListener()

		# objectの座標値調整のパラメーター(cm)
		self.param_object_x = 0.0# +-前後
		self.param_object_y = 0.0# +-左右
		self.param_object_z = 0.0# +-上下
		self.param_grasp_depth = 3.0# 手で物体を握る深さ

		#物体把持時に使用する変数
		self.arm_lift_val = 0.0
		self.arm_flex_val = 0.0
		self.arm_flexup_val = 0.0
		self.arm_roll_val = 0.0
		self.wrist_flex_val = 0.0
		self.move_x = 0.0
		self.move_y = 0.0

	def Wath_motion(self, srv_msg):
		try:
			print (srv_msg.req_str)
			(trans, rot) = self.listener.lookupTransform('/base_footprint',srv_msg.req_str, rospy.Time(0))
			x = trans[0] - 0.060
			print (x)
			y = trans[1] + 0.022
			print (y)
			z = 0.967 - trans[2]
			print (z)
			if x < 0:
				rospy.loginfo('向くことができません。')
				return watch_motionResponse(False)
			rad_pan = math.atan(y / x)
			rad_tilt = - math.atan(z / math.sqrt(x ** 2 + y ** 2))
			print ('head_pan_joint    :', rad_pan)
			print ('head_tilt_joint    :', rad_tilt)
			self.move_head('head_pan_joint', rad_pan)
			self.move_head('head_tilt_joint', rad_tilt)
			return watch_motionResponse(True)
		except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logerr("watch_motion -> TF lookup error")
			return watch_motionResponse(False)
	def Arm_height(self, srv_msg):
		try:
			(arm_trans, arm_rot) = self.listener.lookupTransform('/base_footprint','/hand_l_distal_link', rospy.Time(0))
			rospy.loginfo("arm_height --> %f", arm_trans[2])
			return arm_heightResponse(arm_trans[2])
		except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logerr("arm_height -> TF lookup error")
			return -1

	def Detect_ctrl(self, srv_msg):
		detect_height = (0.173205 + srv_msg.req_height - 0.752) * 2 #0.752を通常のbase_footprintからcameraのz軸距離、0.173205は家具まで高さが30cm時、見る角度が-30度の時のz軸
		if detect_height < 0.2:
			detect_height = 0.2
		print ('DETECTING_POSEの高さ: ', detect_height)
		self.move_head('head_pan_joint', 0.0)
		self.move_head('head_tilt_joint', -0.5233)#-30度
		self.move_arm('arm_lift_joint', 0.6)
		rospy.sleep(2)
		self.move_arm('arm_flex_joint', -2.62)
		self.move_arm('arm_roll_joint', -1.5708) #deg2rad(-90)
		self.move_arm('wrist_flex_joint', -1.5708) #deg2rad(-90)
		rospy.sleep(2)
		self.move_arm('arm_lift_joint', detect_height)
		if detect_height == 0.2:
			return detect_ctrlResponse(False)
		return detect_ctrlResponse(True)

	def Put_ctrl(self, srv_msg):
		height_designation = srv_msg.req_height * 100
		self.target_position = srv_msg.req_str
		if 34 <= height_designation and height_designation <= 103:# 腕を水平にして把持できる高さ範囲
			self.arm_lift_val = (height_designation - 34) * 0.01 #m単位に変換
			self.arm_flex_val = -1.5708 #deg2rad(-90)
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = 0.0
			self.move_x = self.param_grasp_depth - 71.5 #71.5はbase_footprintからfingerまでのx軸距離
			#把持開始
			self.put_motion()

		elif height_designation < 34:
			temp_val = (34 - height_designation) / 34.5#armの角度算出
			flex_angle = math.asin(temp_val)
			base_finger_len = 37 + (34.5 * math.cos(flex_angle))
			self.arm_lift_val = 0.0
			self.arm_flex_val = -(1.5708 + flex_angle)
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = flex_angle
			self.move_x = self.param_grasp_depth - base_finger_len
			if self.arm_flexup_val > 0:
				self.arm_flexup_val = 0
			#把持開始
			self.put_motion()

		elif height_designation > 103:
			temp_val = 34.5 / (height_designation - 103)#armの角度算出
			flex_angle = math.asin(temp_val)
			base_finger_len = 37 + (34.5 * math.cos(flex_angle))
			self.arm_lift_val = 0.69
			self.arm_flex_val = -1.5708 + flex_angle
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = -flex_angle
			self.move_x = self.param_grasp_depth - base_finger_len
			if self.arm_flexup_val > 0:
				self.arm_flexup_val = 0
			#把持開始
			self.put_motion()
		return put_ctrlResponse(True)

	def put_motion(self):

		self.move_head('head_pan_joint', 0.0)
		self.move_head('head_tilt_joint', 0.0)
		#arm移動
		self.move_arm('arm_lift_joint', self.arm_lift_val)
		rospy.sleep(3)
		self.move_arm('arm_flex_joint', self.arm_flex_val)
		self.move_arm('arm_roll_joint', self.arm_roll_val)
		self.move_arm('wrist_flex_joint', self.wrist_flex_val)
		rospy.sleep(3)
		#直進して物体把持
		self.base_ctrl_call('X:20')
		rospy.sleep(0.5)
		self.move_hand_open(True)
		rospy.sleep(3)
		#self.move_arm('arm_flex_joint', self.arm_flexup_val)
		#rospy.sleep(2)
		self.base_ctrl_call('X:-30')
		rospy.sleep(2)

		self.motion_initial_pose()


	def Robot_motion(self, srv_msg):
		target_motion = srv_msg.motion_type
		rospy.loginfo("Robot_motion [%s]", target_motion)

		if target_motion == 'INITIAL_POSE':
			self.motion_initial_pose()
		elif target_motion == 'LIFT_HIGH_POSE':
			self.motion_high_pose()
		elif target_motion == 'LIFT_MIDDLE_POSE':
			self.motion_middle_pose()
		elif target_motion == 'LIFT_JUST_POSE':
			self.motion_just_pose()
		elif target_motion == 'PUT_OBJECT_MOTION':
			self.motion_put_object()
		elif target_motion == 'DETECTING_POSE':
			self.motion_detect_pose()
		elif target_motion == 'POINTING_POSE':
			self.motion_pointing_pose()
		elif target_motion == 'MESUREMENT_POSE':
			self.motion_mesurement_pose()

		elif target_motion == 'CAMERA_TILT_UP':
			self.move_head('head_tilt_joint', 0.25)
		elif target_motion == 'CAMERA_TILT_CENTER':
			self.move_head('head_tilt_joint', 0.0)
		elif target_motion == 'CAMERA_TILT_DOWN':
			self.move_head('head_tilt_joint', -0.25)
		elif target_motion == 'CAMERA_TILT_DEEP_DOWN':
			self.move_head('head_tilt_joint', -0.5)

		elif target_motion == 'CAMERA_PAN_LEFT':
			self.move_head('head_pan_joint', 0.3)
		elif target_motion == 'CAMERA_PAN_CENTER':
			self.move_head('head_pan_joint', 0.0)
		elif target_motion == 'CAMERA_PAN_RIGHT':
			self.move_head('head_pan_joint', -0.3)

		elif target_motion == 'HAND_OPEN':
			self.move_hand_open(True)
		elif target_motion == 'HAND_CLOSE':
			self.move_hand_open(False)

		return robot_motionResponse(True)


	def Grasp_by_frame(self, srv_msg):
		target_object = srv_msg.req_str
		key = self.listener.canTransform('/base_footprint', target_object, rospy.Time(0))# 座標変換の可否判定
		if key == False:
			rospy.logerr("Grasp_ctrl Can't Transform [%s]", target_object)
			return False

		#物体把持の処理開始
		rospy.loginfo("Grasp_ctrl Target_object [%s]", target_object)
		(trans,rot) = self.listener.lookupTransform('/base_footprint', target_object, rospy.Time(0))
		print (trans)
		# 物体座標(cm)
		object_x = (trans[0] * 100) + self.param_object_x
		object_y = (trans[1] * 100) + self.param_object_y
		object_z = (trans[2] * 100) + self.param_object_z

		print (object_x)
		print (object_y)
		print (object_z)

		if 34 <= object_z and object_z <= 103:# 腕を水平にして把持できる高さ範囲
			self.arm_lift_val = (object_z - 34) * 0.01 #m単位に変換
			self.arm_flex_val = -1.5708 #deg2rad(-90)
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = 0.0
			self.move_x = (object_x + self.param_grasp_depth - 71.5) #71.5はbase_footprintからfingerまでのx軸距離
			self.move_y = (object_y - 7.8) #7.8はbase_footprintからfingerまでのy軸距離
			#把持開始
			self.grasp_motion()

		elif object_z < 34:
			temp_val = (34 - object_z) / 34.5#armの角度算出
			flex_angle = math.asin(temp_val)
			base_finger_len = 37 + (34.5 * math.cos(flex_angle))
			self.arm_lift_val = 0.0
			self.arm_flex_val = -(1.5708 + flex_angle)
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = flex_angle
			self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
			self.move_y = (object_y - 7.8) #7.8はbase_footprintからfingerまでのy軸距離
			if self.arm_flexup_val > 0:
				self.arm_flexup_val = 0
			#把持開始
			self.grasp_motion()

		elif object_z > 103:
			temp_val = (object_z - 103) / 34.5 #armの角度算出
			flex_angle = math.asin(temp_val)
			base_finger_len = 37 + (34.5 * math.cos(flex_angle))
			self.arm_lift_val = 0.69
			self.arm_flex_val = -1.5708 + flex_angle
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = -flex_angle
			self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
			self.move_y = (object_y - 7.8) #7.8はbase_footprintからfingerまでのy軸距離
			if self.arm_flexup_val > 0:
				self.arm_flexup_val = 0
			#把持開始
			self.grasp_motion()
		return grasp_ctrlResponse(True)

	def grasp_motion(self):
		#手を開いて横移動
		#self.move_hand_open(True)
		#self.move_arm('arm_lift_joint', 0.5)
		if abs(self.move_x) > 1:#1cmより大きい場合のみ動く
			self.base_ctrl_call('X:' + str(self.move_x))
		if abs(self.move_y) > 1:#1cmより大きい場合のみ動く
			self.base_ctrl_call('Y:' + str(self.move_y))
		#arm移動
		self.move_arm('arm_lift_joint', self.arm_lift_val)
		self.move_arm('arm_flex_joint', self.arm_flex_val)
		self.move_arm('arm_roll_joint', self.arm_roll_val)
		self.move_arm('wrist_flex_joint', self.wrist_flex_val)
		rospy.sleep(3)
		#直進して物体把持
		#self.base_ctrl_call('X:20')

	def Object_raech(self, srv_msg):

		target_object = srv_msg.req_str
		key = self.listener.canTransform('/base_footprint', target_object, rospy.Time(0))# 座標変換の可否判定
		if key == False:
			rospy.logerr("Grasp_ctrl Can't Transform [%s]", target_object)
			return False

		#物体把持の処理開始
		rospy.loginfo("Grasp_ctrl Target_object [%s]", target_object)
		(trans,rot) = self.listener.lookupTransform('/base_footprint', target_object, rospy.Time(0))
		print (trans)
		# 物体座標(cm)
		object_x = (trans[0] * 100) + self.param_object_x
		object_y = (trans[1] * 100) + self.param_object_y
		object_z = (trans[2] * 100) + self.param_object_z

		print (object_x)
		print (object_y)
		print (object_z)

		if 34 <= object_z and object_z <= 103:# 腕を水平にして把持できる高さ範囲
			object_z += 5
			object_x -= 5
			self.arm_lift_val = (object_z - 34) * 0.01 #m単位に変換
			self.arm_flex_val = -1.5708 #deg2rad(-90)
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = 0.0
			self.move_x = (object_x + self.param_grasp_depth - 71.5) #71.5はbase_footprintからfingerまでのx軸距離
			self.move_y = (object_y - 7.8) #7.8はbase_footprintからfingerまでのy軸距離
			#把持開始
			self.reach_motion()

		elif object_z < 34:
			object_x -= 5
			if object_z < 5.0:
				object_z = 5.0
			object_y = 0
			print ("object_z: " , object_z)
			temp_val = (34 - object_z) / 34.5 #armの角度算出
			print ("temp_val : " , temp_val)
			flex_angle = math.asin(temp_val)
			base_finger_len = 37 + (34.5 * math.cos(flex_angle))
			self.arm_lift_val = 0.0
			self.arm_flex_val = -(1.5708 + flex_angle)
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = flex_angle
			self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
			self.move_y = (object_y - 7.8) #7.8はbase_footprintからfingerまでのy軸距離
			if self.arm_flexup_val > 0:
				self.arm_flexup_val = 0
			#把持開始
			self.reach_motion()

		elif object_z > 103:
			print ("object_z: " , object_z)
			temp_val = (object_z - 103) / 34.5 #armの角度算出
			print ("temp_val : " , temp_val)
			flex_angle = math.asin(temp_val)
			base_finger_len = 37 + (34.5 * math.cos(flex_angle))
			self.arm_lift_val = 0.69
			self.arm_flex_val = -1.5708 + flex_angle
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = -flex_angle
			self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
			self.move_y = (object_y - 7.8) #7.8はbase_footprintからfingerまでのy軸距離
			if self.arm_flexup_val > 0:
				self.arm_flexup_val = 0
			#把持開始
			self.reach_motion()
		return grasp_ctrlResponse(True)

	def Gripper_move(self, srv_msg):
		target_object = srv_msg.target_name
		key = self.listener.canTransform('/base_footprint', target_object, rospy.Time(0))# 座標変換の可否判定
		if key == False:
			rospy.logerr("Gripper_move Can't Transform [%s]", target_object)
			return gripper_moveResponse(False)
		#物体把持の処理開始
		rospy.loginfo("Gripper_move Target_object [%s]", target_object)
		(trans,rot) = self.listener.lookupTransform('/base_footprint', target_object, rospy.Time(0))
		print (trans)
		object_x = (trans[0] * 100) + srv_msg.distance.x * 100
		object_y = (trans[1] * 100) + srv_msg.distance.y * 100
		object_z = (trans[2] * 100) + srv_msg.distance.z * 100
		if 34 <= object_z and object_z <= 103:# 腕を水平にして把持できる高さ範囲
			self.arm_lift_val = (object_z - 34) * 0.01 #m単位に変換
			self.arm_flex_val = -1.5708 #deg2rad(-90)
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = 0.0
			self.move_x = (object_x + self.param_grasp_depth - 71.5) #71.5はbase_footprintからfingerまでのx軸距離
			self.move_y = (object_y - 7.8) #7.8はbase_footprintからfingerまでのy軸距離
			#把持開始
			self.grasp_motion()
		elif object_z < 34:
			temp_val = (34 - object_z) / 34.5#armの角度算出
			flex_angle = math.asin(temp_val)
			base_finger_len = 37 + (34.5 * math.cos(flex_angle))
			self.arm_lift_val = 0.0
			self.arm_flex_val = -(1.5708 + flex_angle)
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = flex_angle
			self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
			self.move_y = (object_y - 7.8) #7.8はbase_footprintからfingerまでのy軸距離
			if self.arm_flexup_val > 0:
				self.arm_flexup_val = 0
			#把持開始
			self.grasp_motion()
		elif object_z > 103:
			temp_val = (object_z - 103) / 34.5 #armの角度算出
			flex_angle = math.asin(temp_val)
			base_finger_len = 37 + (34.5 * math.cos(flex_angle))
			self.arm_lift_val = 0.69
			self.arm_flex_val = -1.5708 + flex_angle
			self.arm_flexup_val = self.arm_flex_val + 0.2
			self.wrist_flex_val = -flex_angle
			self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
			self.move_y = (object_y - 7.8) #7.8はbase_footprintからfingerまでのy軸距離
			if self.arm_flexup_val > 0:
				self.arm_flexup_val = 0
			#把持開始
			self.grasp_motion()
		return gripper_moveResponse(True)

	def Gripper_open(self, srv_msg):
		point = JointTrajectoryPoint()
		point.time_from_start = rospy.Duration(1)
		point.positions = [+0.611, -0.611]

		send_data = JointTrajectory()
		send_data.joint_names = ['hand_l_proximal_joint', 'hand_r_proximal_joint']
		send_data.points.append(point)
		#send_dataを送信
		self.pub_gripper_traj.publish(send_data)
		return True

	def Gripper_close(self, srv_msg):
		point = JointTrajectoryPoint()
		point.time_from_start = rospy.Duration(1)
		point.positions = [-0.05, +0.05]

		send_data = JointTrajectory()
		send_data.joint_names = ['hand_l_proximal_joint', 'hand_r_proximal_joint']
		send_data.points.append(point)
		#send_dataを送信
		self.pub_gripper_traj.publish(send_data)
		return True

	def reach_motion(self):
		#手を開いて横移動
		self.move_hand_open(True)
		self.move_arm('arm_lift_joint', 0.5)
		self.base_ctrl_call('X:' + str(self.move_x - 20))
		self.base_ctrl_call('Y:' + str(self.move_y))
		#arm移動
		self.move_arm('arm_lift_joint', self.arm_lift_val)
		rospy.sleep(1)
		self.move_arm('arm_flex_joint', self.arm_flex_val)
		self.move_arm('arm_roll_joint', self.arm_roll_val)
		self.move_arm('wrist_flex_joint', self.wrist_flex_val)
		rospy.sleep(3)
		#直進して物体把持
		self.base_ctrl_call('X:20')
		self.move_hand_open(False)

	def motion_mesurement_pose(self):
		self.move_arm('arm_roll_joint', 3.14)
		self.move_head('head_tilt_joint', -0.2)
		self.move_arm('arm_lift_joint', 0.0)
		self.move_arm('arm_flex_joint', -1.0)
		self.move_arm('wrist_flex_joint', -0.8)

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

	def motion_high_pose(self):
		self.move_head('head_pan_joint', 0.0)
		rospy.sleep(1)
		self.move_arm('arm_lift_joint', 0.8)
		rospy.sleep(1)
		self.move_arm('arm_roll_joint', -1.5708) #deg2rad(-90)

	def motion_middle_pose(self):
		self.move_head('head_pan_joint', 0.0)
		rospy.sleep(1)
		self.move_arm('arm_lift_joint', 0.2)
		rospy.sleep(1)
		self.move_arm('arm_roll_joint', -1.5708) #deg2rad(-90)

	def motion_just_pose(self):
		rospy.sleep(1)
		self.move_head('head_tilt_joint', -0.2)
		rospy.sleep(1)
		self.move_arm('arm_lift_joint', 0.1)
		rospy.sleep(1)
		self.move_arm('arm_flex_joint', 0.0)
		rospy.sleep(1)
		self.move_arm('arm_roll_joint', -1.5708) #deg2rad(-90)
		rospy.sleep(1)
		self.move_arm('wrist_flex_joint', -1.5708) #deg2rad(-90)

	def motion_put_object(self):
		self.move_head('head_pan_joint', 0.0)
		self.move_head('head_tilt_joint', 0.0)
		self.move_arm('arm_lift_joint', 0.6)
		rospy.sleep(2)
		self.move_arm('arm_flex_joint', -1.3)
		self.move_arm('arm_roll_joint', 0) #deg2rad(-90)
		self.move_arm('wrist_flex_joint', -0.25) #deg2rad(-90)
		rospy.sleep(2)
		self.move_hand_open(True)
		rospy.sleep(2)
		self.move_arm('arm_flex_joint', 0.0)
		self.motion_initial_pose()

	def motion_detect_pose(self):
		self.move_head('head_pan_joint', -0.50)
		self.move_head('head_tilt_joint', -0.58)
		self.move_arm('arm_lift_joint', 0.35)
		self.move_arm('arm_flex_joint', -0.23)
		self.move_arm('arm_roll_joint', 1.5708) #deg2rad(-90)
		self.move_arm('wrist_flex_joint', -1.5708) #deg2rad(-90)
		self.move_arm('wrist_roll_joint', 0.0) #deg2rad(-90)
		rospy.sleep(2)

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

		send_data = JointTrajectory()
		send_data.joint_names = ['hand_l_proximal_joint', 'hand_r_proximal_joint']
		send_data.points.append(point)
		#send_dataを送信
		self.pub_gripper_traj.publish(send_data)

	def base_ctrl_call(self, str_x):# HSRの位置調整
		rospy.wait_for_service('/robot_ctrl/base_ctrl')
		try:
			service_name = rospy.ServiceProxy('/robot_ctrl/base_ctrl', odom_base)
			resp = service_name(str_x)
			return resp.res_str
		except rospy.ServiceException as e:
			print ("Service call failed: %s"%e)

if __name__ == '__main__':
	rospy.init_node('grasp_obj_by_frame_hsr')
	gs = Grasping()
	rospy.spin()
