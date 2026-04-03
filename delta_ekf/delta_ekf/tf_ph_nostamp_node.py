#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, TransformStamped  # <-- CAMBIO
from tf2_ros import TransformBroadcaster


class tf_ph_nostamp_node(Node):
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
            Pose, pose_topic, self.pose_callback, 10   # <-- CAMBIO
        )

        self.get_logger().info(
            f"Subscribed to '{pose_topic}' (Pose) → broadcasting TF "
            f"'{self.parent}' → '{self.child}'"
        )
        
        self.position=Pose()  # Variable para almacenar la última pose recibida

        self.create_timer(0.1, self.timer_callback)  # Timer para publicar TF periódicamente
    
    def timer_callback(self):
        # Publicar el último TF aunque no haya llegado un nuevo mensaje de Pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id  = self.child
        # Aquí podrías almacenar la última pose recibida y usarla para actualizar la transformación
        # Por simplicidad, este ejemplo no lo hace, pero en una implementación real deberías hacerlo.
        # Translation
        t.transform.translation.x = self.position.position.x/1000   # <-- CAMBIO
        t.transform.translation.y = self.position.position.y/1000
        t.transform.translation.z = self.position.position.z/1000

        # Rotation
        t.transform.rotation = self.position.orientation
        self.tf_broadcaster.sendTransform(t)
    # ------------------------------------------------------------------ #
    def pose_callback(self, msg: Pose):   # <-- CAMBIO

        
        self.position = msg  # Almacenar la última pose recibida


def main(args=None):
    rclpy.init(args=args)
    node = tf_ph_nostamp_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()