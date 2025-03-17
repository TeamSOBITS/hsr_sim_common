#!/usr/bin/env python3
# coding: utf-8

import math
import time

import rclpy
from rclpy.node import Node

# Assuming JointController is a ROS2-compatible Python module you've already ported
from joint_controller import JointController

# Import your service definitions from ROS2
# For example, if `gripper_ctrl` is a service named GripperCtrl in your hsr_sim_common interface:
from hsr_sim_common.srv import GripperCtrl
# If you also need GripperMoveResponse, import or define it here.
# from hsr_sim_common.srv import GripperMove, GripperMoveResponse

class TestGraspNode(Node):
    def __init__(self):
        super().__init__('test_grasp_node')

        # Instantiate the JointController (which we assume is already ported to ROS2)
        self.jc = JointController()

        # Create a service client to open/close the gripper
        self.gripper_client = self.create_client(GripperCtrl, '/robot_ctrl/gripper_open_and_close')

        # Wait up to e.g. 5 seconds for the gripper service to be available
        if not self.gripper_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /robot_ctrl/gripper_open_and_close not available.")
        else:
            self.get_logger().info("TestGraspNode is ready. Services are available.")

    def run_test_grasp(self):
        """
        Main routine that attempts a grasp at a fixed coordinate (50,50,50).
        This is the ROS2 equivalent of the original 'test_grasp()' logic.
        """
        object_x_cm = 50
        object_y_cm = 50
        object_z_cm = 50

        # Decide how to position the arm based on the object's z-height
        if 34 <= object_z_cm <= 103:  # Arm horizontal range
            self.get_logger().info("腕を水平にして把持できる高さ範囲")
            arm_lift_joint_m = (object_z_cm - 34) * 0.01
            arm_flex_joint_rad = -math.radians(90)
            arm_roll_joint_rad = 0.0
            wrist_flex_joint_rad = 0.0
            wrist_roll_joint_rad = 0.0
            move_x_cm = (object_x_cm - 71.5)  # 71.5 = base_footprint -> finger distance in X
            move_y_cm = (object_y_cm - 7.8)   # 7.8 = base_footprint -> finger distance in Y

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

        else:  # object_z_cm > 103
            self.get_logger().info("object_z_cm > 103")
            tmp_wrist_flex_joint_rad = math.asin((object_z_cm - 103) / 34.5)
            arm_lift_joint_m = 0.69
            arm_flex_joint_rad = -math.radians(90) + tmp_wrist_flex_joint_rad
            arm_roll_joint_rad = 0.0
            wrist_flex_joint_rad = -tmp_wrist_flex_joint_rad
            wrist_roll_joint_rad = 0.0
            # If we need base_to_finger_cm here:
            base_to_finger_cm = 37 + (34.5 * math.cos(tmp_wrist_flex_joint_rad))
            move_x_cm = object_x_cm - base_to_finger_cm
            move_y_cm = object_y_cm - 7.8

        self.get_logger().info("把持処理を開始します...")

        # Move to some known initial pose
        self.jc.move_to_initial_pose()

        # Open the gripper
        self.open_gripper(0.92)

        # Move in X
        if abs(move_x_cm) > 1:
            self.jc.move_wheel("X:" + str(move_x_cm))

        # Move in Y
        if abs(move_y_cm) > 1:
            self.jc.move_wheel("Y:" + str(move_y_cm))

        # Publish the arm positions
        self.jc.add_arm_control_data_to_storage('arm_lift_joint', arm_lift_joint_m)
        self.jc.add_arm_control_data_to_storage('arm_flex_joint', arm_flex_joint_rad)
        self.jc.add_arm_control_data_to_storage('arm_roll_joint', arm_roll_joint_rad)
        self.jc.add_arm_control_data_to_storage('wrist_flex_joint', wrist_flex_joint_rad)
        self.jc.add_arm_control_data_to_storage('wrist_roll_joint', wrist_roll_joint_rad)

        # Optionally tilt the head for a better view
        self.jc.add_head_control_data_to_storage('head_pan_joint', 0.0)
        self.jc.add_head_control_data_to_storage('head_tilt_joint', -0.35)

        self.jc.publish_arm_control_data(2.0)
        self.jc.publish_head_control_data(2.0)
        time.sleep(2.0)

        # Close gripper
        self.close_gripper(0.00)
        time.sleep(2.0)

        # Return to initial pose
        self.jc.move_to_initial_pose()

        self.get_logger().info("Grasp test completed.")
        # If you need to return a GripperMoveResponse or similar, 
        # you can do that. Otherwise just finishing is enough.
        # return GripperMoveResponse(True)

    def open_gripper(self, open_position: float) -> bool:
        """
        Calls the gripper service to open the gripper to `open_position`.
        open_position is presumably a joint angle, e.g. 0.92 for fully open.
        """
        if not self.gripper_client.service_is_ready():
            # Attempt to wait again
            self.get_logger().warn("Waiting for /robot_ctrl/gripper_open_and_close service again...")
            if not self.gripper_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error("Gripper service not available.")
                return False

        request = GripperCtrl.Request()
        request.rad = open_position

        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().is_moved
        else:
            self.get_logger().error("open_gripper() service call failed.")
            return False

    def close_gripper(self, close_position: float) -> bool:
        """
        Calls the gripper service to close the gripper to `close_position`.
        close_position is presumably a joint angle, e.g. 0.00 for fully closed.
        """
        if not self.gripper_client.service_is_ready():
            self.get_logger().warn("Waiting for /robot_ctrl/gripper_open_and_close service again...")
            if not self.gripper_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error("Gripper service not available.")
                return False

        request = GripperCtrl.Request()
        request.rad = close_position

        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().is_moved
        else:
            self.get_logger().error("close_gripper() service call failed.")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = TestGraspNode()

    try:
        node.run_test_grasp()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
