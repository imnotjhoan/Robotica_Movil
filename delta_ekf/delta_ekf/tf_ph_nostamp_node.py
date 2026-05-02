#!/usr/bin/env python3
"""
odometry_node.py
Integra posición (encoder) + orientación (Madgwick) y publica:
  - nav_msgs/Odometry  →  slam_toolbox / cartographer
  - TF odom → base_link
"""
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # ── Parámetros ──────────────────────────────────────────────────
        self.declare_parameter('encoder_topic',     '/encoder/vel')
        self.declare_parameter('madgwick_topic',    '/madgwick/pose')
        self.declare_parameter('odom_topic',        '/odom')
        self.declare_parameter('parent_frame',      'odom')
        self.declare_parameter('child_frame',       'base_link')
        self.declare_parameter('deadband',          0.02)

        enc_topic  = self.get_parameter('encoder_topic').value
        mad_topic  = self.get_parameter('madgwick_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.parent     = self.get_parameter('parent_frame').value
        self.child      = self.get_parameter('child_frame').value
        self.DEADBAND   = self.get_parameter('deadband').value

        # ── Estado ──────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0          # viene del Madgwick correcto
        self.orientation_q = np.array([1.0, 0.0, 0.0, 0.0])  # w,x,y,z
        self.last_enc_time = None

        # ── Publicadores ────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_br    = TransformBroadcaster(self)

        # ── Suscriptores ────────────────────────────────────────────────
        from geometry_msgs.msg import TwistStamped
        self.create_subscription(
            TwistStamped, enc_topic, self.encoder_cb, 10)
        self.create_subscription(
            Pose, mad_topic, self.madgwick_cb, 10)

        self.get_logger().info(
            f"Encoder ← '{enc_topic}' | Madgwick ← '{mad_topic}'\n"
            f"Odometry → '{odom_topic}' | TF: '{self.parent}'→'{self.child}'"
        )

    # ── Callback orientación (Madgwick) ─────────────────────────────────
    def madgwick_cb(self, msg: Pose):
        """
        Recibe el cuaternión limpio de madgwick_node.
        Sin conjugados ni inversiones: publicar el quaternión directamente.
        """
        q = msg.orientation
        self.orientation_q = np.array([q.w, q.x, q.y, q.z])

        # Extraemos yaw para la integración de posición
        qw, qx, qy, qz = self.orientation_q
        self.yaw = np.arctan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy**2 + qz**2)
        )

    # ── Callback encoder ────────────────────────────────────────────────
    def encoder_cb(self, msg):
        current_time = (msg.header.stamp.sec
                        + msg.header.stamp.nanosec * 1e-9)

        if self.last_enc_time is None:
            self.last_enc_time = current_time
            return

        dt = current_time - self.last_enc_time
        self.last_enc_time = current_time

        if dt <= 0.0 or dt > 2.0:
            return

        v = msg.twist.linear.x
        if abs(v) < self.DEADBAND:
            v = 0.0

        # ── Integración 2D ──────────────────────────────────────────────
        self.x += v * np.cos(self.yaw) * dt
        self.y += v * np.sin(self.yaw) * dt

        now = self.get_clock().now().to_msg()
        qw, qx, qy, qz = self.orientation_q

        # ── Publicar Odometry ───────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.parent
        odom.child_frame_id  = self.child

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.w = float(qw)
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)

        # Covarianza diagonal (ajusta según tu sensor)
        odom.pose.covariance[0]  = 0.05   # σ²_x
        odom.pose.covariance[7]  = 0.05   # σ²_y
        odom.pose.covariance[35] = 0.02   # σ²_yaw

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = 0.0  # si tienes ω del encoder/IMU ponla aquí
        odom.twist.covariance[0]   = 0.01
        odom.twist.covariance[35]  = 0.05

        self.odom_pub.publish(odom)

        # ── Publicar TF ─────────────────────────────────────────────────
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = self.parent
        tf.child_frame_id  = self.child

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w    = float(qw)
        tf.transform.rotation.x    = float(qx)
        tf.transform.rotation.y    = float(qy)
        tf.transform.rotation.z    = float(qz)

        self.tf_br.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()