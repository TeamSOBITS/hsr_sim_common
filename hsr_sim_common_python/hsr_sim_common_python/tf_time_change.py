#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class SigverseTfTimeChanger(Node):
    def __init__(self):
        super().__init__('sigverse_tf_time_changer')

        # Create a publisher for the /tf topic
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 10)

        # Create a subscriber to /tf_sigverse
        self.sub_tf_sigverse = self.create_subscription(
            TFMessage,
            '/tf_sigverse',
            self.callback_tf_sigverse,
            10
        )

        self.get_logger().info("sigverse_tf_time_changer node started. Listening on /tf_sigverse.")

    def callback_tf_sigverse(self, msg: TFMessage):
        """
        This callback receives TFMessage from /tf_sigverse, updates each transform
        to have the current time, and republishes to /tf.
        """
        # Create a new TFMessage to publish
        out_msg = TFMessage()

        # Current ROS2 time
        now_stamp = self.get_clock().now().to_msg()

        for transform_stamped in msg.transforms:
            # Copy the transform
            new_transform = TransformStamped()
            new_transform.header.stamp = now_stamp
            new_transform.header.frame_id = transform_stamped.header.frame_id
            new_transform.child_frame_id = transform_stamped.child_frame_id
            new_transform.transform = transform_stamped.transform

            out_msg.transforms.append(new_transform)

        # Publish the updated TFMessage
        self.pub_tf.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SigverseTfTimeChanger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class SigverseTfTimeChanger(Node):
    def __init__(self):
        super().__init__('sigverse_tf_time_changer')

        # Create a publisher for the /tf topic
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 10)

        # Create a subscriber to /tf_sigverse
        self.sub_tf_sigverse = self.create_subscription(
            TFMessage,
            '/tf_sigverse',
            self.callback_tf_sigverse,
            10
        )

        self.get_logger().info("sigverse_tf_time_changer node started. Listening on /tf_sigverse.")

    def callback_tf_sigverse(self, msg: TFMessage):
        """
        This callback receives TFMessage from /tf_sigverse, updates each transform
        to have the current time, and republishes to /tf.
        """
        # Create a new TFMessage to publish
        out_msg = TFMessage()

        # Current ROS2 time
        now_stamp = self.get_clock().now().to_msg()

        for transform_stamped in msg.transforms:
            # Copy the transform
            new_transform = TransformStamped()
            new_transform.header.stamp = now_stamp
            new_transform.header.frame_id = transform_stamped.header.frame_id
            new_transform.child_frame_id = transform_stamped.child_frame_id
            new_transform.transform = transform_stamped.transform

            out_msg.transforms.append(new_transform)

        # Publish the updated TFMessage
        self.pub_tf.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SigverseTfTimeChanger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
