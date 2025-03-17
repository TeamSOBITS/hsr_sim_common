#!/usr/bin/env python3
# coding: utf-8

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
# Adjust import based on where `odom_base.srv` resides in your ROS2 setup
from hsr_sim_common_interfaces.srv import OdomBase


class OdomBaseController(Node):

    def __init__(self):
        super().__init__('odom_base_controller')

        # Create a publisher for Twist messages
        self.pub_twist = self.create_publisher(Twist, '/hsrb/command_velocity', 10)

        # Create the service server
        self.srv = self.create_service(OdomBase, 'base_ctrl', self.base_control_callback)

        self.get_logger().info("Base_ctrl service is up.")

    def base_control_callback(self, request, response):
        """
        Callback for the 'base_ctrl' service. Parses a string command (e.g., 'T:90', 'X:20', 'Y:-10'), 
        then drives the robot accordingly.
        """
        if not self.check_command(request.req_str):
            self.get_logger().info("The command is invalid.")
            response.res_str = "failed"
            return response

        if "T" in request.req_str:
            # Turn command
            order_deg = self.read_value(request.req_str)
            self.get_logger().info(f"Base_ctrl: Turn: {order_deg} (deg)")
            self.execute_turn(order_deg)

        elif "X" in request.req_str:
            # Straight command
            order_cm = self.read_value(request.req_str)
            self.get_logger().info(f"Base_ctrl: Straight: {order_cm} (cm)")
            self.execute_linear_x(order_cm)

        elif "Y" in request.req_str:
            # Sideways command
            order_cm = self.read_value(request.req_str)
            self.get_logger().info(f"Base_ctrl: Sideways: {order_cm} (cm)")
            self.execute_linear_y(order_cm)

        self.get_logger().info("Moving Finished")
        response.res_str = f"{request.req_str} Finished"
        return response

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def check_command(self, req_str: str) -> bool:
        """
        Quick validation that the command starts with T/X/Y, followed by a colon,
        then a numeric value (possibly negative, possibly float).
        """
        # We expect something like 'T:90' or 'X:-10.5', etc.
        # Example parse approach:
        if len(req_str) < 3:
            return False
        if req_str[1] != ':':
            return False
        cmd = req_str[0]
        if cmd not in ['T', 'X', 'Y']:
            return False
        value_part = req_str[2:]
        # Check if the remainder is a float (possibly negative)
        try:
            float(value_part)
        except ValueError:
            return False
        return True

    def read_value(self, req_str: str) -> float:
        """
        Extract the numeric portion from the command string, e.g. from 'T:90' -> 90.0
        """
        return float(req_str[2:])

    def execute_turn(self, degrees: float):
        """
        Publish a Twist command that rotates the base for a given number of degrees.
        Positive => counterclockwise, Negative => clockwise.
        10 deg/sec as a fixed speed in this example.
        """
        twist = Twist()

        # 10 deg/s -> convert to rad/s
        speed_rad_s = math.radians(10.0)
        if degrees >= 0.0:
            twist.angular.z = speed_rad_s
        else:
            twist.angular.z = -speed_rad_s

        # Determine how long to spin
        wait_time = abs(degrees) / 10.0  # 10 deg/s => X seconds
        start_time = time.time()
        while time.time() - start_time < wait_time:
            self.pub_twist.publish(twist)
            time.sleep(0.1)

        # stop
        twist.angular.z = 0.0
        self.pub_twist.publish(twist)

    def execute_linear_x(self, cm: float):
        """
        Publish a Twist command that moves the base forward/backward by cm.
        Speed is 10 cm/s.
        """
        twist = Twist()
        speed_m_s = 0.10  # 10 cm/s
        if cm >= 0.0:
            twist.linear.x = speed_m_s
        else:
            twist.linear.x = -speed_m_s

        wait_time = abs(cm) / 10.0  # distance / speed => time
        start_time = time.time()
        while time.time() - start_time < wait_time:
            self.pub_twist.publish(twist)
            time.sleep(0.1)

        # stop
        twist.linear.x = 0.0
        self.pub_twist.publish(twist)

    def execute_linear_y(self, cm: float):
        """
        Publish a Twist command that moves the base sideways by cm.
        Speed is 10 cm/s sideways.
        """
        twist = Twist()
        speed_m_s = 0.10  # 10 cm/s
        if cm >= 0.0:
            twist.linear.y = speed_m_s
        else:
            twist.linear.y = -speed_m_s

        wait_time = abs(cm) / 10.0
        start_time = time.time()
        while time.time() - start_time < wait_time:
            self.pub_twist.publish(twist)
            time.sleep(0.1)

        # stop
        twist.linear.y = 0.0
        self.pub_twist.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = OdomBaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
