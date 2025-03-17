#!/usr/bin/env python3
# coding: utf-8

import math
import time

import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer
import tf_transformations

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from hsr_sim_common_interfaces.srv import (
    GraspCtrl,
    RobotMotion,
    ArmHeight,
    PutCtrl,
    DetectCtrl,
    WatchMotion,
    GripperMove,
    GripperCtrl,
    OdomBase
)

class Grasping(Node):
    def __init__(self):
        super().__init__("grasp_obj_by_frame_hsr")

        self.pub_arm_traj = self.create_publisher(JointTrajectory, "/hsrb/arm_trajectory_controller/command", 10)
        self.pub_head_traj = self.create_publisher(JointTrajectory, "/hsrb/head_trajectory_controller/command", 10)
        self.pub_gripper_traj = self.create_publisher(JointTrajectory, "/hsrb/gripper_trajectory_controller/command", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.srv_grasp_ctrl = self.create_service(GraspCtrl, 'grasp_ctrl', self.Grasp_by_frame)
        self.srv_motion_ctrl = self.create_service(RobotMotion, 'motion_ctrl', self.Robot_motion)
        self.srv_arm_height = self.create_service(ArmHeight, 'arm_height', self.Arm_height)
        self.srv_put_ctrl = self.create_service(PutCtrl, 'put_ctrl', self.Put_ctrl)
        self.srv_detect_ctrl = self.create_service(DetectCtrl, 'detect_ctrl', self.Detect_ctrl)
        self.srv_object_reach_ctrl = self.create_service(GraspCtrl, 'object_reach_ctrl', self.Object_raech)
        self.srv_watch_motion = self.create_service(WatchMotion, 'watch_motion', self.Wath_motion)
        self.srv_gripper_move = self.create_service(GripperMove, 'gripper_move', self.Gripper_move)
        self.srv_gripper_open = self.create_service(GripperCtrl, 'gripper_open', self.Gripper_open)
        self.srv_gripper_close = self.create_service(GripperCtrl, 'gripper_close', self.Gripper_close)

        self.param_object_x = 0.0   # +/- front-back
        self.param_object_y = 0.0   # +/- left-right
        self.param_object_z = 0.0   # +/- up-down
        self.param_grasp_depth = 3.0  # additional offset into the grasp

        self.arm_lift_val = 0.0
        self.arm_flex_val = 0.0
        self.arm_flexup_val = 0.0
        self.arm_roll_val = 0.0
        self.wrist_flex_val = 0.0
        self.move_x = 0.0
        self.move_y = 0.0

        self.get_logger().info("Grasping node started.")

    def Wath_motion(self, request: WatchMotion.Request, response: WatchMotion.Response = None):
        """
        Rotate head (pan/tilt) to look at a frame relative to /base_footprint.
        """
        target_frame = request.req_str
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_footprint", target_frame, rclpy.time.Time()
            )
            trans = transform.transform.translation

            x = trans.x - 0.060
            y = trans.y + 0.022
            z = 0.967 - trans.z

            if x < 0:
                self.get_logger().info("Cannot turn to face that location.")
                response.success = False
                return response

            rad_pan = math.atan2(y, x)  # improved usage: atan2
            distance_xy = math.sqrt(x**2 + y**2)
            rad_tilt = -math.atan2(z, distance_xy)

            self.get_logger().info(f"head_pan_joint: {rad_pan:.3f}")
            self.get_logger().info(f"head_tilt_joint: {rad_tilt:.3f}")

            self.move_head("head_pan_joint", rad_pan)
            self.move_head("head_tilt_joint", rad_tilt)
            response.success = True
            return response

        except Exception as e:
            self.get_logger().error(f"watch_motion -> TF lookup error: {e}")
            response.success = False
            return response

    def Arm_height(self, request: ArmHeight.Request, response: ArmHeight.Response = None):
        """
        Return the current height of /hand_l_distal_link relative to /base_footprint
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_footprint", "hand_l_distal_link", rclpy.time.Time()
            )
            trans = transform.transform.translation
            self.get_logger().info(f"arm_height --> {trans.z:.3f}")
            response.height = trans.z
            return response
        except Exception as e:
            self.get_logger().error(f"arm_height -> TF lookup error: {e}")
            response.height = -1.0
            return response

    def Detect_ctrl(self, request: DetectCtrl.Request, response: DetectCtrl.Response = None):
        """
        Some custom detection pose logic from your original code.
        """
        detect_height = (0.173205 + request.req_height - 0.752) * 2
        if detect_height < 0.2:
            detect_height = 0.2
        self.get_logger().info(f"[Detect_ctrl] detect_height: {detect_height:.3f}")

        self.move_head("head_pan_joint", 0.0)
        self.move_head("head_tilt_joint", -0.5233)  # ~ -30 deg
        self.move_arm("arm_lift_joint", 0.6)
        time.sleep(2)

        self.move_arm("arm_flex_joint", -2.62)
        self.move_arm("arm_roll_joint", -1.5708)  # -90 deg
        self.move_arm("wrist_flex_joint", -1.5708)
        time.sleep(2)

        self.move_arm("arm_lift_joint", detect_height)
        if abs(detect_height - 0.2) < 1e-6:
            response.success = False
            return response
        response.success = True 
        return response

    def Put_ctrl(self, request: PutCtrl.Request, response: PutCtrl.Response = None):
        """
        Place an object at a specified height (request.req_height) after approaching or adjusting the arm.
        """
        height_cm = request.req_height * 100.0
        self.target_position = request.req_str

        if 34 <= height_cm <= 103:
            # Arm horizontal
            self.arm_lift_val = (height_cm - 34) * 0.01
            self.arm_flex_val = -1.5708  # ~ -90 deg
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = 0.0
            self.move_x = self.param_grasp_depth - 71.5
            self.put_motion()

        elif height_cm < 34:
            # Lower
            temp_val = (34 - height_cm) / 34.5
            flex_angle = math.asin(temp_val)
            base_finger_len = 37 + (34.5 * math.cos(flex_angle))
            self.arm_lift_val = 0.0
            self.arm_flex_val = -(1.5708 + flex_angle)
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = flex_angle
            self.move_x = self.param_grasp_depth - base_finger_len
            if self.arm_flexup_val > 0:
                self.arm_flexup_val = 0
            self.put_motion()

        else:
            # height_cm > 103
            temp_val = 34.5 / (height_cm - 103)
            flex_angle = math.asin(temp_val)
            base_finger_len = 37 + (34.5 * math.cos(flex_angle))
            self.arm_lift_val = 0.69
            self.arm_flex_val = -1.5708 + flex_angle
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = -flex_angle
            self.move_x = self.param_grasp_depth - base_finger_len
            if self.arm_flexup_val > 0:
                self.arm_flexup_val = 0
            self.put_motion()
        response.success = True
        return response

    def Robot_motion(self, request: RobotMotion.Request, response: RobotMotion.Response = None):
        """
        Execute various named motions (INITIAL_POSE, etc.).
        """
        target_motion = request.motion_type
        self.get_logger().info(f"Robot_motion [{target_motion}]")

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
        response.success = True
        return response

    def Grasp_by_frame(self, request: GraspCtrl.Request, response: GraspCtrl.Response = None):
        """
        Grasp an object by TF frame name.
        """
        target_object = request.req_str
        try:
            transform = self.tf_buffer.lookup_transform("base_footprint", target_object, rclpy.time.Time())
        except Exception:
            self.get_logger().error(f"Grasp_ctrl can't transform [{target_object}]")
            response.success = False
            return response

        self.get_logger().info(f"Grasp_ctrl Target_object [{target_object}]")
        trans = transform.transform.translation

        object_x = (trans.x * 100.0) + self.param_object_x
        object_y = (trans.y * 100.0) + self.param_object_y
        object_z = (trans.z * 100.0) + self.param_object_z

        self.get_logger().info(f"Object (x,y,z) [cm]: ({object_x:.2f}, {object_y:.2f}, {object_z:.2f})")

        if 34 <= object_z <= 103:
            self.arm_lift_val = (object_z - 34) * 0.01
            self.arm_flex_val = -1.5708
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = 0.0
            self.move_x = (object_x + self.param_grasp_depth - 71.5)
            self.move_y = (object_y - 7.8)
            self.grasp_motion()
        elif object_z < 34:
            temp_val = (34 - object_z) / 34.5
            flex_angle = math.asin(temp_val)
            base_finger_len = 37 + (34.5 * math.cos(flex_angle))
            self.arm_lift_val = 0.0
            self.arm_flex_val = -(1.5708 + flex_angle)
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = flex_angle
            self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
            self.move_y = (object_y - 7.8)
            if self.arm_flexup_val > 0:
                self.arm_flexup_val = 0
            self.grasp_motion()
        else:
            # object_z > 103
            temp_val = (object_z - 103) / 34.5
            flex_angle = math.asin(temp_val)
            base_finger_len = 37 + (34.5 * math.cos(flex_angle))
            self.arm_lift_val = 0.69
            self.arm_flex_val = -1.5708 + flex_angle
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = -flex_angle
            self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
            self.move_y = (object_y - 7.8)
            if self.arm_flexup_val > 0:
                self.arm_flexup_val = 0
            self.grasp_motion()
        response.success = True 
        return response

    def Object_raech(self, request: GraspCtrl.Request, response: GraspCtrl.Response = None):
        """
        Another variant of “reach” to an object by TF name, from your code.
        """
        target_object = request.req_str
        # Attempt transform
        try:
            transform = self.tf_buffer.lookup_transform("base_footprint", target_object, rclpy.time.Time())
        except Exception:
            self.get_logger().error(f"Object_reach_ctrl can't transform [{target_object}]")
            response.success = False
            return response

        self.get_logger().info(f"Object_reach_ctrl Target_object [{target_object}]")
        trans = transform.transform.translation

        object_x = (trans.x * 100.0) + self.param_object_x
        object_y = (trans.y * 100.0) + self.param_object_y
        object_z = (trans.z * 100.0) + self.param_object_z

        self.get_logger().info(f"Object (x,y,z) [cm]: ({object_x:.2f}, {object_y:.2f}, {object_z:.2f})")

        if 34 <= object_z <= 103:
            object_z += 5
            object_x -= 5
            self.arm_lift_val = (object_z - 34) * 0.01
            self.arm_flex_val = -1.5708
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = 0.0
            self.move_x = (object_x + self.param_grasp_depth - 71.5)
            self.move_y = (object_y - 7.8)
            self.reach_motion()

        elif object_z < 34:
            object_x -= 5
            if object_z < 5.0:
                object_z = 5.0
            object_y = 0.0  # per your code
            temp_val = (34 - object_z) / 34.5
            flex_angle = math.asin(temp_val)
            base_finger_len = 37 + (34.5 * math.cos(flex_angle))
            self.arm_lift_val = 0.0
            self.arm_flex_val = -(1.5708 + flex_angle)
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = flex_angle
            self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
            self.move_y = (object_y - 7.8)
            if self.arm_flexup_val > 0:
                self.arm_flexup_val = 0
            self.reach_motion()

        else:  # object_z > 103
            temp_val = (object_z - 103) / 34.5
            flex_angle = math.asin(temp_val)
            base_finger_len = 37 + (34.5 * math.cos(flex_angle))
            self.arm_lift_val = 0.69
            self.arm_flex_val = -1.5708 + flex_angle
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = -flex_angle
            self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
            self.move_y = (object_y - 7.8)
            if self.arm_flexup_val > 0:
                self.arm_flexup_val = 0
            self.reach_motion()
        response.success = True
        return response

    def Gripper_move(self, request: GripperMove.Request, response: GripperMove.Response = None):
        """
        Move the gripper to a specific offset from the target frame.
        """
        target_object = request.target_name
        try:
            transform = self.tf_buffer.lookup_transform("base_footprint", target_object, rclpy.time.Time())
        except Exception:
            self.get_logger().error(f"Gripper_move can't transform [{target_object}]")
            response.success = False
            return response

        trans = transform.transform.translation

        # Incorporate requested offset
        dx = request.distance.x * 100.0
        dy = request.distance.y * 100.0
        dz = request.distance.z * 100.0

        object_x = (trans.x * 100.0) + dx
        object_y = (trans.y * 100.0) + dy
        object_z = (trans.z * 100.0) + dz

        if 34 <= object_z <= 103:
            self.arm_lift_val = (object_z - 34) * 0.01
            self.arm_flex_val = -1.5708
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = 0.0
            self.move_x = (object_x + self.param_grasp_depth - 71.5)
            self.move_y = (object_y - 7.8)
            self.grasp_motion()

        elif object_z < 34:
            temp_val = (34 - object_z) / 34.5
            flex_angle = math.asin(temp_val)
            base_finger_len = 37 + (34.5 * math.cos(flex_angle))
            self.arm_lift_val = 0.0
            self.arm_flex_val = -(1.5708 + flex_angle)
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = flex_angle
            self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
            self.move_y = (object_y - 7.8)
            if self.arm_flexup_val > 0:
                self.arm_flexup_val = 0
            self.grasp_motion()

        else:  # object_z > 103
            temp_val = (object_z - 103) / 34.5
            flex_angle = math.asin(temp_val)
            base_finger_len = 37 + (34.5 * math.cos(flex_angle))
            self.arm_lift_val = 0.69
            self.arm_flex_val = -1.5708 + flex_angle
            self.arm_flexup_val = self.arm_flex_val + 0.2
            self.wrist_flex_val = -flex_angle
            self.move_x = (object_x + self.param_grasp_depth - base_finger_len)
            self.move_y = (object_y - 7.8)
            if self.arm_flexup_val > 0:
                self.arm_flexup_val = 0
            self.grasp_motion()
        response.success = True
        return response

    def Gripper_open(self, request: GripperCtrl.Request, response: GripperCtrl.Response = None):
        """
        Open the gripper.
        """
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=1)
        # positions for opening
        point.positions = [+0.611, -0.611]

        send_data = JointTrajectory()
        send_data.joint_names = ['hand_l_proximal_joint', 'hand_r_proximal_joint']
        send_data.points.append(point)

        self.pub_gripper_traj.publish(send_data)
        response.success = True
        return response

    def Gripper_close(self, request: GripperCtrl.Request, response: GripperCtrl.Response = None):
        """
        Close the gripper.
        """
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=1)
        point.positions = [-0.05, +0.05]

        send_data = JointTrajectory()
        send_data.joint_names = ['hand_l_proximal_joint', 'hand_r_proximal_joint']
        send_data.points.append(point)

        self.pub_gripper_traj.publish(send_data)
        response.success = True
        return response

    def put_motion(self):
        """
        Sequence for placing an object at a certain pose.
        """
        self.move_head('head_pan_joint', 0.0)
        self.move_head('head_tilt_joint', 0.0)
        self.move_arm('arm_lift_joint', self.arm_lift_val)
        time.sleep(3)
        self.move_arm('arm_flex_joint', self.arm_flex_val)
        self.move_arm('arm_roll_joint', self.arm_roll_val)
        self.move_arm('wrist_flex_joint', self.wrist_flex_val)
        time.sleep(3)

        self.base_ctrl_call('X:20')
        time.sleep(0.5)
        self.move_hand_open(True)
        time.sleep(3)
        self.base_ctrl_call('X:-30')
        time.sleep(2)

        self.motion_initial_pose()

    def grasp_motion(self):
        """
        Moves base + arm to grasp an object.
        """
        if abs(self.move_x) > 1.0:
            self.base_ctrl_call(f"X:{self.move_x}")
        if abs(self.move_y) > 1.0:
            self.base_ctrl_call(f"Y:{self.move_y}")

        self.move_arm('arm_lift_joint', self.arm_lift_val)
        self.move_arm('arm_flex_joint', self.arm_flex_val)
        self.move_arm('arm_roll_joint', self.arm_roll_val)
        self.move_arm('wrist_flex_joint', self.wrist_flex_val)
        time.sleep(3)

        # (Your code suggests you might want to do final base approach or hand close here)
        # self.base_ctrl_call('X:20')
        # self.move_hand_open(False)

    def reach_motion(self):
        """
        Variation of grasp approach with partial motions first.
        """
        self.move_hand_open(True)
        self.move_arm('arm_lift_joint', 0.5)

        self.base_ctrl_call(f"X:{self.move_x - 20}")
        self.base_ctrl_call(f"Y:{self.move_y}")

        self.move_arm('arm_lift_joint', self.arm_lift_val)
        time.sleep(1)
        self.move_arm('arm_flex_joint', self.arm_flex_val)
        self.move_arm('arm_roll_joint', self.arm_roll_val)
        self.move_arm('wrist_flex_joint', self.wrist_flex_val)
        time.sleep(3)

        self.base_ctrl_call("X:20")
        self.move_hand_open(False)

    def motion_initial_pose(self):
        self.move_hand_open(False)
        self.move_head('head_pan_joint', 0.0)
        self.move_head('head_tilt_joint', 0.0)
        self.move_arm('arm_lift_joint', 0.0)
        self.move_arm('arm_flex_joint', 0.0)
        self.move_arm('arm_roll_joint', -1.5708)
        self.move_arm('wrist_flex_joint', -1.5708)

    def motion_high_pose(self):
        self.move_head('head_pan_joint', 0.0)
        time.sleep(1)
        self.move_arm('arm_lift_joint', 0.8)
        time.sleep(1)
        self.move_arm('arm_roll_joint', -1.5708)

    def motion_middle_pose(self):
        self.move_head('head_pan_joint', 0.0)
        time.sleep(1)
        self.move_arm('arm_lift_joint', 0.2)
        time.sleep(1)
        self.move_arm('arm_roll_joint', -1.5708)

    def motion_just_pose(self):
        time.sleep(1)
        self.move_head('head_tilt_joint', -0.2)
        time.sleep(1)
        self.move_arm('arm_lift_joint', 0.1)
        time.sleep(1)
        self.move_arm('arm_flex_joint', 0.0)
        time.sleep(1)
        self.move_arm('arm_roll_joint', -1.5708)
        time.sleep(1)
        self.move_arm('wrist_flex_joint', -1.5708)

    def motion_put_object(self):
        self.move_head('head_pan_joint', 0.0)
        self.move_head('head_tilt_joint', 0.0)
        self.move_arm('arm_lift_joint', 0.6)
        time.sleep(2)
        self.move_arm('arm_flex_joint', -1.3)
        self.move_arm('arm_roll_joint', 0.0)
        self.move_arm('wrist_flex_joint', -0.25)
        time.sleep(2)
        self.move_hand_open(True)
        time.sleep(2)
        self.move_arm('arm_flex_joint', 0.0)
        self.motion_initial_pose()

    def motion_detect_pose(self):
        self.move_head('head_pan_joint', -0.50)
        self.move_head('head_tilt_joint', -0.58)
        self.move_arm('arm_lift_joint', 0.35)
        self.move_arm('arm_flex_joint', -0.23)
        self.move_arm('arm_roll_joint', 1.5708)
        self.move_arm('wrist_flex_joint', -1.5708)
        self.move_arm('wrist_roll_joint', 0.0)
        time.sleep(2)


    def move_head(self, joint_name: str, value: float):
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=2)
        point.positions.append(value)

        msg = JointTrajectory()
        msg.joint_names.append(joint_name)
        msg.points.append(point)

        self.pub_head_traj.publish(msg)

    def move_arm(self, joint_name: str, value: float):
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=2)
        point.positions.append(value)

        msg = JointTrajectory()
        msg.joint_names.append(joint_name)
        msg.points.append(point)

        self.pub_arm_traj.publish(msg)

    def move_hand_open(self, open_hand: bool):
        """
        True => open, False => close
        """
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=2)

        if open_hand:
            point.positions = [+0.611, -0.611]
        else:
            point.positions = [-0.05, +0.05]

        msg = JointTrajectory()
        msg.joint_names = ['hand_l_proximal_joint', 'hand_r_proximal_joint']
        msg.points.append(point)

        self.pub_gripper_traj.publish(msg)

    def base_ctrl_call(self, str_command: str):
        """
        Calls /robot_ctrl/base_ctrl to adjust the HSR base, passing something
        like "X:20" or "Y:-10" or "T:30" as the string.
        """
        client = self.create_client(OdomBase, '/robot_ctrl/base_ctrl')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /robot_ctrl/base_ctrl not available.")
            return "FAILED"

        request = OdomBase.Request()
        request.req_str = str_command

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            return future.result().res_str
        else:
            self.get_logger().error("Service call to base_ctrl failed.")
            return "FAILED"


def main(args=None):
    rclpy.init(args=args)
    node = Grasping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
