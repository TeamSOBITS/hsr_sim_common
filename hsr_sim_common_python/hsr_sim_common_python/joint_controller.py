#!/usr/bin/env python3
# coding: utf-8
import math
import time

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import tf_transformations

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from hsr_sim_common_interfaces.srv import RobotMotion
from hsr_sim_common_interfaces.srv import OdomBase
from hsr_sim_common_interfaces.srv import GripperMove
from hsr_sim_common_interfaces.srv import GripperCtrl
from hsr_sim_common_interfaces.srv import IsGrasped


class JointController(Node):

    def __init__(self):
        super().__init__("joint_controller")

        # Publishers
        self.pub_arm_control = self.create_publisher(JointTrajectory, "/hsrb/arm_trajectory_controller/command", 10)
        self.pub_head_control = self.create_publisher(JointTrajectory, "/hsrb/head_trajectory_controller/command", 10)
        self.pub_gripper_control = self.create_publisher(JointTrajectory, "/hsrb/gripper_controller/command", 10)

        # Services
        self.srv_gripper_ctrl = self.create_service(
            GripperCtrl, "gripper_open_and_close", self.open_and_close_gripper_server
        )
        self.srv_gripper_move = self.create_service(
            GripperMove, "gripper_move_to_target", self.move_gripper_to_target_server
        )
        self.srv_robot_motion = self.create_service(
            RobotMotion, "motion_ctrl", self.move_to_registered_motion_server
        )
        self.srv_is_grasped = self.create_service(
            IsGrasped, "is_grasped", self.is_grasped_server
        )

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Tracking for joint trajectory messages
        self.arm_control_data = JointTrajectory()
        self.head_control_data = JointTrajectory()
        self.gripper_control_data = JointTrajectory()
        # Pre-populate a single-point list
        self.arm_control_data.points = [JointTrajectoryPoint()]
        self.head_control_data.points = [JointTrajectoryPoint()]
        self.gripper_control_data.points = [JointTrajectoryPoint()]

        self.has_getted_data = False
        self.hand_l_spring_proximal_joint_state = 0.0

        self.arm_lift_joint_now = 0.0
        self.arm_flex_joint_now = 0.0
        self.arm_roll_joint_now = -1.5708
        self.wrist_flex_joint_now = -1.5708
        self.wrist_roll_joint_now = 0.0

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState, "/hsrb/joint_states", self.joint_state_callback, 10
        )

        self.get_logger().info("JointController node has been initialized.")
        
    def check_publishers_connection(self, publisher):
        loop_rate_to_check_connection = rospy.Rate(1)
        while (publisher.get_num_connections() == 0 and not rospy.is_shutdown()):
            try:
                loop_rate_to_check_connection.sleep()
            except rospy.ROSInterruptException:
                pass

    def add_arm_control_data_to_storage(self, joint_name, rad):
        self.arm_control_data.joint_names.append(joint_name)
        self.arm_control_data.points[0].positions.append(rad)
        self.arm_control_data.points[0].velocities.append(0.0)
        self.arm_control_data.points[0].accelerations.append(0.0)
        self.arm_control_data.points[0].effort.append(0.0)

    def add_head_control_data_to_storage(self, joint_name, rad):
        self.head_control_data.joint_names.append(joint_name)
        self.head_control_data.points[0].positions.append(rad)
        self.head_control_data.points[0].velocities.append(0.0)
        self.head_control_data.points[0].accelerations.append(0.0)
        self.head_control_data.points[0].effort.append(0.0)

    def add_gripper_control_data_to_storage(self, joint_name, rad):
        self.gripper_control_data.joint_names.append(joint_name)
        self.gripper_control_data.points[0].positions.append(rad)
        self.gripper_control_data.points[0].velocities.append(0.0)
        self.gripper_control_data.points[0].accelerations.append(0.0)
        self.gripper_control_data.points[0].effort.append(0.0)

    def publish_arm_control_data(self, time_from_start_sec):
        self.arm_control_data.points[0].time_from_start = Duration(sec=int(time_from_start_sec))
        self.pub_arm_control.publish(self.arm_control_data)
        self.arm_control_data = JointTrajectory()
        self.arm_control_data.points = [JointTrajectoryPoint()]

    def publish_head_control_data(self, time_from_start_sec):
        self.head_control_data.points[0].time_from_start = Duration(sec=int(time_from_start_sec))
        self.pub_head_control.publish(self.head_control_data)
        self.head_control_data = JointTrajectory()
        self.head_control_data.points = [JointTrajectoryPoint()]

    def publish_gripper_control_data(self, time_from_start_sec):
        self.gripper_control_data.points[0].time_from_start = Duration(sec=int(time_from_start_sec))
        self.pub_gripper_control.publish(self.gripper_control_data)
        self.gripper_control_data = JointTrajectory()
        self.gripper_control_data.points = [JointTrajectoryPoint()]

    def joint_state_callback(self, msg: JointState):
        """
        Keep track of the 'hand_l_spring_proximal_joint' to detect whether an object is grasped.
        """
        for i, name in enumerate(msg.name):
            if name == "hand_l_spring_proximal_joint":
                self.hand_l_spring_proximal_joint_state = msg.position[i]
                self.has_getted_data = True

    def move_gripper_to_target_server(self, req: GripperMove.Request, resp: GripperMove.Response = None):
        """
        Move hand_motor_link to the TF specified by req.target_name, with an additional shift of req.shift.
        """
        target_object = req.target_name
        try:
            transform = self.tf_buffer.lookup_transform(
                "head_pan_link", target_object, rclpy.time.Time()
            )
            transform_base = self.tf_buffer.lookup_transform(
                "base_footprint", "head_pan_link", rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().error(f"TF lookup error: {e}")
            resp.success = False
            return resp

        trans = transform.transform.translation
        rot = transform_base.transform.rotation

        object_x = trans.x + req.shift.x
        object_y = trans.y + req.shift.y
        object_z = trans.z + req.shift.z + 0.8269999

        quaternion = (rot.x, rot.y, rot.z, rot.w)
        (_, _, object_yaw_rad) = tf_transformations.euler_from_quaternion(quaternion)
        object_yaw_deg = math.degrees(object_yaw_rad)

        object_x_cm = object_x * 100.0
        object_y_cm = object_y * 100.0
        object_z_cm = object_z * 100.0

        self.get_logger().info(f"TRANS: {trans.x}, {trans.y}, {trans.z}")
        self.get_logger().info(f"OBJECT XYZ (cm): {object_x_cm}, {object_y_cm}, {object_z_cm}")
        self.get_logger().info(f"SHIFT HAND: {req.shift.x}, {req.shift.y}, {req.shift.z}")

        arm_lift_joint_m = 0.0
        arm_flex_joint_rad = 0.0
        arm_roll_joint_rad = 0.0
        wrist_flex_joint_rad = 0.0
        wrist_roll_joint_rad = 0.0
        move_x_cm = 0.0
        move_y_cm = 0.0

        if 34 <= object_z_cm <= 103:
            self.get_logger().info("Within horizontal-grasp range [34,103] cm.")
            arm_lift_joint_m = (object_z_cm - 34) * 0.01
            arm_flex_joint_rad = -math.radians(90)
            arm_roll_joint_rad = 0.0
            wrist_roll_joint_rad = 0.0
            wrist_flex_joint_rad = 0.0
            move_x_cm = object_x_cm - 71.5
            move_y_cm = object_y_cm - 7.8
        elif object_z_cm < 34:
            self.get_logger().info("object_z_cm < 34")
            arm_lift_joint_m = 0.0
            wrist_flex_joint_rad = math.asin((34 - object_z_cm) / 34.5)
            arm_flex_joint_rad = -(math.radians(90) + wrist_flex_joint_rad)
            arm_roll_joint_rad = 0.0
            wrist_roll_joint_rad = 0.0
            base_to_finger_cm = 37 + (34.5 * math.cos(wrist_flex_joint_rad))
            move_x_cm = object_x_cm - base_to_finger_cm
            move_y_cm = object_y_cm - 7.8
        else:
            self.get_logger().info("object_z_cm > 103")
            tmp_wrist_flex_joint_rad = math.asin((object_z_cm - 103) / 34.5)
            base_to_finger_cm = 37 + (34.5 * math.cos(tmp_wrist_flex_joint_rad))
            arm_lift_joint_m = 0.69
            arm_flex_joint_rad = -math.radians(90) + tmp_wrist_flex_joint_rad
            arm_roll_joint_rad = 0.0
            wrist_flex_joint_rad = -tmp_wrist_flex_joint_rad
            wrist_roll_joint_rad = 0.0
            move_x_cm = object_x_cm - base_to_finger_cm
            move_y_cm = object_y_cm - 7.8

        if abs(object_yaw_deg) > 1.0:
            self.move_wheel(f"T:{object_yaw_deg}")
        if abs(move_x_cm) > 1.0:
            self.move_wheel(f"X:{move_x_cm}")
        if abs(move_y_cm) > 1.0:
            self.move_wheel(f"Y:{move_y_cm}")

        self.add_arm_control_data_to_storage("arm_lift_joint", arm_lift_joint_m)
        self.add_arm_control_data_to_storage("arm_flex_joint", arm_flex_joint_rad)
        self.add_arm_control_data_to_storage("arm_roll_joint", arm_roll_joint_rad)
        self.add_arm_control_data_to_storage("wrist_flex_joint", wrist_flex_joint_rad)
        self.add_arm_control_data_to_storage("wrist_roll_joint", wrist_roll_joint_rad)

        self.add_head_control_data_to_storage("head_pan_joint", 0.0)
        self.add_head_control_data_to_storage("head_tilt_joint", -0.35)

        self.publish_arm_control_data(2.0)
        self.publish_head_control_data(2.0)
        time.sleep(2.0)
        resp.success = True
        return resp

    def open_and_close_gripper_server(self, req: GripperCtrl.Request, resp: GripperCtrl.Response = None):
        """
        Open/close the gripper by setting hand_motor_joint to the requested angle.
        """
        hand_motor_joint_rad = req.rad
        time_from_start_sec = 1.0
        self.add_gripper_control_data_to_storage("hand_motor_joint", hand_motor_joint_rad)
        self.publish_gripper_control_data(time_from_start_sec)
        time.sleep(time_from_start_sec)
        resp.success = True
        return resp

    def is_grasped_server(self, req: IsGrasped.Request, resp: IsGrasped.Response = None):
        """
        Checks if something is grasped by reading the hand_l_spring_proximal_joint.
        You already subscribe to /hsrb/joint_states. So if that joint moves away from 0,
        we interpret that as 'grasped'.
        """
        self.get_logger().info("Checking if object is grasped...")

        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while (self.get_clock().now().seconds_nanoseconds()[0] - start_time) < 5.0:
            if self.has_getted_data:
                if not (abs(self.hand_l_spring_proximal_joint_state) < 1e-6):
                    return IsGraspedResponse(grasped=True)
                else:
                    return IsGraspedResponse(grasped=False)
            else:
                time.sleep(0.5)
        resp.grapsed = False
        return resp

    def move_to_registered_motion_server(self, req: RobotMotion.Request, resp: RobotMotion.Response = None):
        motion_type = req.motion_type
        if motion_type == "INITIAL_POSE":
            self.move_to_initial_pose()
        elif motion_type == "DETECTING_POSE":
            self.move_to_detecting_pose()
        elif motion_type == "LOW_DETECTING_POSE":
            self.move_to_low_detecting_pose()
        elif motion_type == "LOWEST_DETECTING_POSE":
            self.move_to_lowest_detecting_pose()
        elif motion_type == "LIFT_UP_HAND":
            self.move_to_lift_up_hand()
        elif motion_type == "MEASUREMENT_POSE":
            self.move_to_measurement_pose()
        elif motion_type == "DETECTING_BOX_POSE":
            self.move_to_detecting_box_pose()
        else:
            pass
        resp.success = True
        return resp

    def move_wheel(self, str_distance):
        """Call /robot_ctrl/base_ctrl service with the given command string."""
        client = self.create_client(OdomBase, "/robot_ctrl/base_ctrl")

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /robot_ctrl/base_ctrl not available.")
            return "FAILED"

        request = OdomBase.Request()
        request.req_str = str_distance
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().res_str
        else:
            self.get_logger().error("Service call failed.")
            return "FAILED"

    def move_to_initial_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage("head_pan_joint", 0.0)
        self.add_head_control_data_to_storage("head_tilt_joint", 0.0)
        self.add_arm_control_data_to_storage("arm_lift_joint", 0.0)
        self.add_arm_control_data_to_storage("arm_flex_joint", 0.0)
        self.add_arm_control_data_to_storage("arm_roll_joint", -1.5708)
        self.add_arm_control_data_to_storage("wrist_flex_joint", -1.5708)
        self.add_arm_control_data_to_storage("wrist_roll_joint", 0.0)
        self.publish_head_control_data(time_from_start_sec)
        self.publish_arm_control_data(time_from_start_sec)
        time.sleep(1.0)

    def move_to_detecting_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage("head_pan_joint", -0.52)
        self.add_head_control_data_to_storage("head_tilt_joint", -0.30)
        self.add_arm_control_data_to_storage("arm_lift_joint", 0.20)
        self.add_arm_control_data_to_storage("arm_flex_joint", 0.0)
        self.add_arm_control_data_to_storage("arm_roll_joint", 1.5708)
        self.add_arm_control_data_to_storage("wrist_flex_joint", -1.35)
        self.add_arm_control_data_to_storage("wrist_roll_joint", 0.0)
        self.publish_head_control_data(time_from_start_sec)
        self.publish_arm_control_data(time_from_start_sec)
        self.move_wheel("T:29")
        time.sleep(1.0)

    def move_to_low_detecting_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage("head_pan_joint", -0.52)
        self.add_head_control_data_to_storage("head_tilt_joint", -0.55)
        self.add_arm_control_data_to_storage("arm_lift_joint", 0.15)
        self.add_arm_control_data_to_storage("arm_flex_joint", 0.0)
        self.add_arm_control_data_to_storage("arm_roll_joint", 1.5708)
        self.add_arm_control_data_to_storage("wrist_flex_joint", -1.35)
        self.add_arm_control_data_to_storage("wrist_roll_joint", 0.0)
        self.publish_head_control_data(time_from_start_sec)
        self.publish_arm_control_data(time_from_start_sec)
        time.sleep(1.0)

    def move_to_lowest_detecting_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage("head_pan_joint", -0.52)
        self.add_head_control_data_to_storage("head_tilt_joint", -0.89)
        self.add_arm_control_data_to_storage("arm_lift_joint", 0.15)
        self.add_arm_control_data_to_storage("arm_flex_joint", 0.0)
        self.add_arm_control_data_to_storage("arm_roll_joint", 1.5708)
        self.add_arm_control_data_to_storage("wrist_flex_joint", -1.35)
        self.add_arm_control_data_to_storage("wrist_roll_joint", 0.0)
        self.publish_head_control_data(time_from_start_sec)
        self.publish_arm_control_data(time_from_start_sec)
        time.sleep(1.0)

    def move_to_lift_up_hand(self):
        time_from_start_sec = 1.0
        self.arm_lift_joint_now += 0.03
        self.add_arm_control_data_to_storage("arm_lift_joint", self.arm_lift_joint_now)
        self.add_arm_control_data_to_storage("arm_flex_joint", self.arm_flex_joint_now)
        self.add_arm_control_data_to_storage("arm_roll_joint", self.arm_roll_joint_now)
        self.add_arm_control_data_to_storage("wrist_flex_joint", self.wrist_flex_joint_now)
        self.add_arm_control_data_to_storage("wrist_roll_joint", self.wrist_roll_joint_now)
        self.publish_arm_control_data(time_from_start_sec)
        time.sleep(1.0)

    def move_to_measurement_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage("head_pan_joint", 0.0)
        self.add_head_control_data_to_storage("head_tilt_joint", -0.2)
        self.add_arm_control_data_to_storage("arm_lift_joint", 0.0)
        self.add_arm_control_data_to_storage("arm_flex_joint", -1.0)
        self.add_arm_control_data_to_storage("arm_roll_joint", 3.14)
        self.add_arm_control_data_to_storage("wrist_flex_joint", -0.8)
        self.add_arm_control_data_to_storage("wrist_roll_joint", 0.0)
        self.publish_head_control_data(time_from_start_sec)
        self.publish_arm_control_data(time_from_start_sec)
        time.sleep(1.0)

    def move_to_detecting_box_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage("head_pan_joint", 0.0)
        self.add_head_control_data_to_storage("head_tilt_joint", -0.55)
        self.add_arm_control_data_to_storage("arm_lift_joint", 0.15)
        self.add_arm_control_data_to_storage("arm_flex_joint", 0.0)
        self.add_arm_control_data_to_storage("arm_roll_joint", 1.5708)
        self.add_arm_control_data_to_storage("wrist_flex_joint", -1.35)
        self.add_arm_control_data_to_storage("wrist_roll_joint", 0.0)
        self.publish_head_control_data(time_from_start_sec)
        self.publish_arm_control_data(time_from_start_sec)
        time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()