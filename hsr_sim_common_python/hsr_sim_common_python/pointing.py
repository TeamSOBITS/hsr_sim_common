#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node

# Messages
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Services (adjust import paths to your actual ROS2 package)
from hsr_sim_common_interfaces.srv import RobotMotion


class Pointing(Node):
    def __init__(self):
        super().__init__('pointing_node')

        # Publishers
        self.pub_arm_traj = self.create_publisher(JointTrajectory, "/hsrb/arm_trajectory_controller/command", 10)
        self.pub_head_traj = self.create_publisher(JointTrajectory, "/hsrb/head_trajectory_controller/command", 10)
        self.pub_gripper_traj = self.create_publisher(JointTrajectory, "/hsrb/gripper_trajectory_controller/command", 10)

        # Service for controlling robot motion
        self.srv_motion_ctrl = self.create_service(
            RobotMotion, 
            'motion_ctrl', 
            self.robot_motion_callback
        )

        # Some internal parameters if needed
        self.arm_lift_val = 0.0
        self.arm_flex_val = 0.0
        self.arm_flexup_val = 0.0
        self.arm_roll_val = 0.0
        self.wrist_flex_val = 0.0
        self.move_x = 0.0
        self.move_y = 0.0

        self.get_logger().info("Body_ctrl (pointing_node) is ready.")

    def robot_motion_callback(self, request: RobotMotion.Request, response: RobotMotion.Response):
        """
        Called when the 'motion_ctrl' service is invoked.
        request.motion_type: string specifying which motion to perform.
        """
        target_motion = request.motion_type
        self.get_logger().info(f"Robot_motion [{target_motion}]")

        if target_motion == 'INITIAL_POSE':
            self.motion_initial_pose()
        elif target_motion == 'POINTING_POSE':
            self.motion_pointing_pose()
        elif target_motion == 'RESET':
            self.motion_reset()
        else:
            self.get_logger().info("Unknown motion type requested.")

        # Indicate success in the response
        response.success = True
        return response

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
        self.move_arm('arm_roll_joint', -1.5708)     # ~ -90 deg
        self.move_arm('wrist_flex_joint', -1.5708)  # ~ -90 deg

    def motion_reset(self):
        self.move_hand_open(False)
        self.move_head('head_pan_joint', 0.0)
        self.move_head('head_tilt_joint', 0.0)
        self.move_arm('arm_lift_joint', 0.0)
        self.move_arm('arm_flex_joint', 0.0)
        self.move_arm('arm_roll_joint', 0.0)
        self.move_arm('wrist_flex_joint', 0.0)

    def move_head(self, joint_name, value):
        """
        Publishes a JointTrajectory command for a single head joint.
        """
        # Create a single trajectory point
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=2)
        point.positions.append(value)

        msg = JointTrajectory()
        msg.joint_names.append(joint_name)
        msg.points.append(point)

        # Publish
        self.pub_head_traj.publish(msg)

    def move_arm(self, joint_name, value):
        """
        Publishes a JointTrajectory command for a single arm joint.
        """
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=2)
        point.positions.append(value)

        msg = JointTrajectory()
        msg.joint_names.append(joint_name)
        msg.points.append(point)

        # Publish
        self.pub_arm_traj.publish(msg)

    def move_hand_open(self, is_open: bool):
        """
        Publishes a JointTrajectory command to open/close the gripper.
        True => open, False => close
        """
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=2)

        if is_open:
            point.positions = [0.611, -0.611]
        else:
            point.positions = [-0.05, 0.05]

        msg = JointTrajectory()
        msg.joint_names = ['hand_l_proximal_joint', 'hand_r_proximal_joint']
        msg.points.append(point)

        # Publish
        self.pub_gripper_traj.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Pointing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
