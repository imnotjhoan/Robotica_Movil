#!/usr/bin/env python3
# coding=utf-8

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import TwistStamped, Pose


class MadgwickAHRS:
    def __init__(self, beta=0.1):
        self.beta = beta
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.sample_period = 0.02

    def update(self, gyro, accel, mag):
        q = self.quaternion
        gx, gy, gz = gyro
        ax, ay, az = accel
        mx, my, mz = mag

        norm_acc = np.linalg.norm([ax, ay, az])
        if norm_acc == 0.0:
            return
        ax, ay, az = ax/norm_acc, ay/norm_acc, az/norm_acc

        norm_mag = np.linalg.norm([mx, my, mz])
        if norm_mag > 0.0:
            mx, my, mz = mx/norm_mag, my/norm_mag, mz/norm_mag

        # (Gradiente simplificado)
        s = np.array([0.0, 0.0, 0.0, 0.0])

        q_dot = 0.5*np.array([
            -q[1]*gx - q[2]*gy - q[3]*gz,
             q[0]*gx + q[2]*gz - q[3]*gy,
             q[0]*gy - q[1]*gz + q[3]*gx,
             q[0]*gz + q[1]*gy - q[2]*gx
        ]) - self.beta * s

        q = q + q_dot * self.sample_period
        self.quaternion = q / np.linalg.norm(q)

    def get_yaw(self):
        q = self.quaternion
        return np.arctan2(
            2.0*(q[0]*q[3] + q[1]*q[2]),
            1.0 - 2.0*(q[2]**2 + q[3]**2)
        )


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # Estado
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = None
        self.last_imu_time = None
        self.last_mag = np.zeros(3)

        self.DEADBAND = 0.02

        self.filter = MadgwickAHRS()

        # Subs
        self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.create_subscription(MagneticField, '/imu/mag', self.mag_cb, 10)
        self.create_subscription(TwistStamped, '/encoder/vel', self.encoder_cb, 10)

        self.pub = self.create_publisher(Pose, '/robot1/pose', 10)

    def mag_cb(self, msg):
        self.last_mag = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ])

    def imu_cb(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return

        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time

        if dt <= 0 or dt > 1.0:
            return

        self.filter.sample_period = dt

        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        self.filter.update(gyro, accel, self.last_mag)
        self.yaw = self.filter.get_yaw()

    def encoder_cb(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = msg.twist.linear.x

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0 or dt > 2.0:
            return

        # Deadband
        if abs(v) < self.DEADBAND:
            v = 0.0

        # 🔥 Integración correcta 2D
        self.x += v * np.cos(self.yaw) * dt
        self.y += v * np.sin(self.yaw) * dt

        # Cuaternión desde yaw
        q = np.array([
            np.cos(self.yaw/2),
            0.0,
            0.0,
            np.sin(self.yaw/2)
        ])

        pose = Pose()
        pose.position.x = float(self.x)
        pose.position.y = float(self.y)
        pose.position.z = 0.0

        pose.orientation.w = float(q[0])
        pose.orientation.x = float(q[1])
        pose.orientation.y = float(q[2])
        pose.orientation.z = float(q[3])

        self.pub.publish(pose)


def main():
    rclpy.init()
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()