#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

# ROS2 message types
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

class PotentialDirection(Node):
    def __init__(self):
        super().__init__('potential')

        # ------------------------------------------------
        # Parameters / Constants
        # ------------------------------------------------
        self.turn_speed_deg = 20        # [degrees]
        self.straight_speed_m_s = 0.3   # used for linear.x if desired (in your original code it looked like 0.3 was cm, but let's keep logic consistent)
        self.turtlebot_r = 0.19         # [m]
        self.order_angle = 0.0          # [rad], "desired" angle (the "influence" angle)
        self.potential_angle = 0.0      # [rad]
        self.potential_scara_result = 0.0
        self.DISTANCE_MAX = 1.0         # [m] - max distance of interest
        self.INRYOKU_KEISUU = 700.0     # coefficient for attraction
        self.SEKIRYOKU_KEISUU = 1300.0  # coefficient for repulsion
        self.PI = math.pi
        self.m2pix = 250               # scaling factor for drawing
        self.urg_scan_using = False
        self.do_flag = False           # whether to start processing
        self.urg_scan = LaserScan()    # store the last LaserScan

        # Image size for debug drawing
        # (2*m + 1) in each dimension so we can map +/- DISTANCE_MAX.
        self.img_size = (
            int(self.DISTANCE_MAX * self.m2pix * 2 + 1),
            int(self.DISTANCE_MAX * self.m2pix * 2 + 1),
            3
        )

        # ------------------------------------------------
        # Publishers & Subscribers
        # ------------------------------------------------
        # For controlling the robot's velocity
        self.pub_twist = self.create_publisher(Twist, 'cmd_vel_mux/input/teleop', 10)

        # Subscribe to laser data
        self.sub_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Subscribe to the desired angle (Float32 in rad)
        self.sub_order_angle = self.create_subscription(
            Float32,
            'potential_order_angle',
            self.order_angle_callback,
            10
        )

        # Subscribe to the do_flag (whether to process)
        self.sub_do_flag = self.create_subscription(
            Bool,
            'potential_do_flag',
            self.do_flag_callback,
            10
        )

        self.get_logger().info("PotentialDirection node initialized.")

    # ------------------------------------------------
    # Callbacks
    # ------------------------------------------------
    def laser_callback(self, data: LaserScan):
        """
        Process laser scans and compute a potential-field-based heading.
        """
        self.urg_scan = data

        # If not active, do nothing
        if not self.do_flag:
            return

        # Summation variables for repulsion + attraction
        order_x = math.sin(self.order_angle) * self.INRYOKU_KEISUU
        order_y = math.cos(self.order_angle) * self.INRYOKU_KEISUU
        potential_x = 0.0
        potential_y = 0.0

        # For drawing
        rgb_color = (0, 255, 0)  # BGR -> green in OpenCV
        map_img = np.zeros(self.img_size, dtype=np.uint8)

        # Mark the center
        center_pix = int(self.DISTANCE_MAX * self.m2pix)
        cv2.circle(map_img, (center_pix, center_pix), 3, (0, 255, 0), -1)

        object_count = 0
        self.urg_scan_using = True

        # Laser angles range from [-pi/2..+pi/2], or - we interpret them in a custom way:
        # The original code tries something like i - len/2 => rad. We keep the logic the same.
        num_ranges = len(data.ranges)
        for i in range(num_ranges):
            dist = data.ranges[i]
            if dist < self.DISTANCE_MAX:
                # Calculate angle
                # The code is: radian = (i - len(...) * 0.5) / len(...) * pi
                radian = float(i - num_ranges * 0.5) / float(num_ranges) * self.PI

                # Weighted distance for repulsion
                # (the closer the obstacle, the larger the repulsion)
                distance = (self.DISTANCE_MAX - dist) * self.SEKIRYOKU_KEISUU

                # Convert to x,y in potential sense
                object_x = distance * math.sin(radian)
                object_y = distance * math.cos(radian)

                # For drawing in the image, we mark the obstacle (roughly)
                # Use the raw dist + angle to find pixel coords
                #   x_4map = (DISTANCE_MAX - dist*sin(radian))*m2pix
                #   y_4map = ...
                # We'll just replicate your logic:
                obj_x_map = int((self.DISTANCE_MAX - dist * math.sin(radian)) * self.m2pix)
                obj_y_map = int((self.DISTANCE_MAX - dist * math.cos(radian)) * self.m2pix)
                if 0 <= obj_x_map < self.img_size[1] and 0 <= obj_y_map < self.img_size[0]:
                    map_img[obj_y_map][obj_x_map] = (0, 255, 0)

                # Summation of potential
                potential_x += (order_x - object_x)
                potential_y += (order_y - object_y)
                object_count += 1

        self.urg_scan_using = False

        # Compute final potential-based angle
        if object_count == 0:
            # No obstacles => go in order_angle direction
            potential_result_x = math.sin(self.order_angle)
            potential_result_y = math.cos(self.order_angle)
            potential_angle = self.order_angle
        else:
            potential_result_x = potential_x / object_count
            potential_result_y = potential_y / object_count
            potential_angle = math.atan2(potential_result_x, potential_result_y)

        # Log info
        deg_angle = (potential_angle / self.PI) * 180.0
        self.get_logger().info(f"potential_angle [deg] = {deg_angle:.1f}")

        # Magnitude of resultant
        scara = math.hypot(potential_result_x, potential_result_y)
        if object_count == 0:
            scara = self.INRYOKU_KEISUU  # no obstacles => purely attraction
        # Cap scara if it exceeds the attraction constant
        scara = min(scara, self.INRYOKU_KEISUU)

        self.potential_scara_result = (scara / self.INRYOKU_KEISUU) * 100.0
        self.get_logger().info(f"potential_scara_result = {self.potential_scara_result:.2f}")

        # Draw final potential vector in red
        px_end = center_pix - self.potential_scara_result * math.sin(potential_angle) * 5
        py_end = center_pix - self.potential_scara_result * math.cos(potential_angle) * 5
        cv2.line(
            map_img,
            (center_pix, center_pix),
            (int(px_end), int(py_end)),
            (0, 0, 255),
            3
        )

        # Draw the original order vector in blue
        ox_end = (self.DISTANCE_MAX - math.sin(self.order_angle)) * self.m2pix
        oy_end = (self.DISTANCE_MAX - math.cos(self.order_angle)) * self.m2pix
        cv2.line(
            map_img,
            (center_pix, center_pix),
            (int(ox_end), int(oy_end)),
            (255, 0, 0),
            2
        )

        # Draw a circle representing the robot
        cv2.circle(
            map_img,
            (center_pix, center_pix + int(self.turtlebot_r * self.m2pix)),
            int(self.turtlebot_r * self.m2pix),
            (0, 100, 0),
            1
        )

        # Show the map in a debug window
        cv2.namedWindow("map_img", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("map_img", map_img)
        cv2.moveWindow("map_img", 100, 100)
        cv2.waitKey(23)

        # -----------------------------------------------------
        # Publish the Twist command to drive the robot
        # The original code sets:
        #   twist.angular.z = potential_angle
        #   twist.linear.x  = self.stlight_speed * (self.potential_scara_result*0.01)
        # That sets an angle in rad directly to angular.z, which is unusual, but we'll keep the logic the same.
        # stlight_speed is 0.3 [m/s?], then multiplied by self.potential_scara_result*0.01 => scaled by 0..1
        # If you need to convert angle => rate, you'd do something else, but let's keep your approach.
        # -----------------------------------------------------
        send_cmd = Twist()
        send_cmd.angular.z = potential_angle
        send_cmd.linear.x = self.straight_speed_m_s * (self.potential_scara_result * 0.01)
        self.pub_twist.publish(send_cmd)

    def order_angle_callback(self, data: Float32):
        self.order_angle = data.data
        self.get_logger().info(f"Received new order_angle [rad] = {self.order_angle:.3f}")

    def do_flag_callback(self, data: Bool):
        self.do_flag = data.data
        if self.do_flag:
            self.get_logger().info("PotentialDirection: START processing.")
        else:
            self.get_logger().info("PotentialDirection: STOP processing.")


def main(args=None):
    rclpy.init(args=args)
    node = PotentialDirection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
