#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class th_ph_node(Node):
    def __init__(self):
        super().__init__('pose_to_tf')

        # ---------- Parameters ----------
        self.declare_parameter('pose_topic',   '/robot1/pose')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame',  'base_link')

        pose_topic  = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.parent = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child  = self.get_parameter('child_frame').get_parameter_value().string_value

        # ---------- TF broadcaster ----------
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- Subscriber ----------
        self.subscription = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, 10
        )

        self.get_logger().info(
            f"Subscribed to '{pose_topic}' → broadcasting TF "
            f"'{self.parent}' → '{self.child}'"
        )

    # ------------------------------------------------------------------ #
    def pose_callback(self, msg: PoseStamped):

        t = TransformStamped()

        t.header.stamp = msg.header.stamp

        t.header.frame_id = self.parent
        t.child_frame_id  = self.child

        # Translation
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = th_ph_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()