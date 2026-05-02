#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Pose


class MadgwickAHRS:
    def __init__(self, sample_period=0.02, beta=0.1):
        self.sample_period = sample_period   # ✅ FIX CRÍTICO
        self.beta = beta
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    def update(self, gyroscope, accelerometer, magnetometer):
        q = self.quaternion
        gx, gy, gz = gyroscope
        ax, ay, az = accelerometer
        mx, my, mz = magnetometer

        norm_acc = np.linalg.norm([ax, ay, az])
        if norm_acc == 0.0:
            return
        ax, ay, az = ax/norm_acc, ay/norm_acc, az/norm_acc

        norm_mag = np.linalg.norm([mx, my, mz])
        use_mag = norm_mag > 0.0
        if use_mag:
            mx, my, mz = mx/norm_mag, my/norm_mag, mz/norm_mag

        _2q1=2.*q[0]; _2q2=2.*q[1]; _2q3=2.*q[2]; _2q4=2.*q[3]
        _4q1=4.*q[0]; _4q2=4.*q[1]; _4q3=4.*q[2]
        _8q2=8.*q[1]; _8q3=8.*q[2]
        q1q1=q[0]*q[0]; q2q2=q[1]*q[1]; q3q3=q[2]*q[2]; q4q4=q[3]*q[3]

        s1 = _4q1*q3q3 + _2q3*ax + _4q1*q2q2 - _2q2*ay
        s2 = (_4q2*q4q4 - _2q4*ax + 4.*q1q1*q[1] - _2q1*ay
              - _4q2 + _8q2*q2q2 + _8q2*q3q3 + _4q2*az)
        s3 = (4.*q1q1*q[2] + _2q1*ax + _4q3*q4q4 - _2q4*ay
              - _4q3 + _8q3*q2q2 + _8q3*q3q3 + _4q3*az)
        s4 = 4.*q2q2*q[3] - _2q2*ax + 4.*q3q3*q[3] - _2q3*ay

        norm_s = np.linalg.norm([s1, s2, s3, s4])
        if norm_s == 0.0:
            return
        s1,s2,s3,s4 = s1/norm_s, s2/norm_s, s3/norm_s, s4/norm_s

        q_dot = 0.5*np.array([
            -q[1]*gx - q[2]*gy - q[3]*gz,
             q[0]*gx + q[2]*gz - q[3]*gy,
             q[0]*gy - q[1]*gz + q[3]*gx,
             q[0]*gz + q[1]*gy - q[2]*gx
        ]) - self.beta*np.array([s1, s2, s3, s4])

        q = q + q_dot * self.sample_period
        self.quaternion = q / np.linalg.norm(q)

    def get_yaw(self):
        q = self.quaternion
        return np.arctan2(
            2.0*(q[0]*q[3] + q[1]*q[2]),
            1.0 - 2.0*(q[2]**2 + q[3]**2)
        )


def yaw_to_quaternion(yaw):
    h = yaw * 0.5
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)])


def conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


class MadgwickNode(Node):
    def __init__(self):
        super().__init__('madgwick_node')

        self.filter = MadgwickAHRS()
        self.last_mag = np.zeros(3)

        self.pub = self.create_publisher(Pose, '/madgwick/pose', 10)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.create_subscription(MagneticField, '/imu/mag', self.mag_cb, 10)

    def mag_cb(self, msg):
        self.last_mag = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ])

    def imu_cb(self, msg):
        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])

        accel = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])

        self.filter.update(gyro, accel, self.last_mag)

        yaw = self.filter.get_yaw()
        q = yaw_to_quaternion(yaw)

        pose = Pose()
        pose.orientation.w = float(q[0])
        pose.orientation.x = float(q[1])
        pose.orientation.y = float(q[2])
        pose.orientation.z = float(q[3])
        self.pub.publish(pose)


def main():
    rclpy.init()
    node = MadgwickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()