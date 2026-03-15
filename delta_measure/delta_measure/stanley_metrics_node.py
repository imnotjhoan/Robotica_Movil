#!/usr/bin/env python3

import os
import csv
import math
import re

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from rosgraph_msgs.msg import Clock


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class StanleyLogger(Node):

    def __init__(self):
        super().__init__('stanley_logger')

        self.declare_parameter(
            'output_dir',
            '/home/jhoan/mrad_ws_2601_delta2/src/delta_measure/data_path_tracking'
        )

        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('save_stanley_errors', True)

        output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value
        )

        path_topic = self.get_parameter(
            'path_topic').get_parameter_value().string_value

        self.save_errors = self.get_parameter(
            'save_stanley_errors').value

        os.makedirs(output_dir, exist_ok=True)

        pattern = re.compile(r'^(\d+)_experiment\.csv$')

        max_n = max(
            (int(m.group(1)) for f in os.listdir(output_dir)
             if (m := pattern.match(f))),
            default=0
        )

        run = max_n + 1
        csv_path = os.path.join(output_dir, f'{run} path_tracking_data.csv')

        self.get_logger().info(f'Run {run}')
        self.get_logger().info(f'CSV → {csv_path}')

        self.fields = [
            "timestamp_s",
            "elapsed_s",
            "robot_x",
            "robot_y",
            "robot_yaw",
            "path_x",
            "path_y",
        ]

        if self.save_errors:
            self.fields += [
                "delta",
                "cross_track_error",
                "heading_error",
            ]

        self._file = open(csv_path, 'w', newline='', encoding='utf-8')

        self.writer = csv.DictWriter(
            self._file,
            fieldnames=self.fields
        )

        self.writer.writeheader()
        self._file.flush()

        self.recording = False
        self.start_time = None
        self.last_clock = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.delta = 0.0
        self.cross_track_error = 0.0
        self.heading_error = 0.0

        self.path_saved = False

        self.create_subscription(
            Bool, '/start', self._start_cb, 10)

        self.create_subscription(
            Clock, '/clock', self._clock_cb, 10)

        self.create_subscription(
            Pose, '/ground_truth_pose', self._pose_cb, 10)

        self.create_subscription(
            Path, path_topic, self._path_cb, 10)

        if self.save_errors:

            self.create_subscription(
                Float64,
                '/stanley/delta',
                self._delta_cb,
                10
            )

            self.create_subscription(
                Float64,
                '/stanley/cross_track_error',
                self._cte_cb,
                10
            )

            self.create_subscription(
                Float64,
                '/stanley/heading_error',
                self._he_cb,
                10
            )

        self.get_logger().info("Esperando /start...")

    # control

    def _start_cb(self, msg):

        if msg.data and not self.recording:

            self.recording = True
            self.start_time = self.last_clock

            self.get_logger().info("Grabación iniciada")

    def _clock_cb(self, msg):

        self.last_clock = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.recording and self.start_time is None:
            self.start_time = self.last_clock

    # robot pose

    def _pose_cb(self, msg: Pose):

        self.x = msg.position.x
        self.y = msg.position.y

        q = msg.orientation
        self.yaw = quaternion_to_yaw(q)

        if not self.recording or self.last_clock is None:
            return

        row = {
            "timestamp_s": self.last_clock,
            "elapsed_s": self.last_clock - self.start_time,
            "robot_x": self.x,
            "robot_y": self.y,
            "robot_yaw": self.yaw,
            "path_x": None,
            "path_y": None,
        }

        if self.save_errors:

            row.update({
                "delta": self.delta,
                "cross_track_error": self.cross_track_error,
                "heading_error": self.heading_error
            })

        self.writer.writerow(row)
        self._file.flush()

    # stanley

    def _delta_cb(self, msg):
        self.delta = msg.data

    def _cte_cb(self, msg):
        self.cross_track_error = msg.data

    def _he_cb(self, msg):
        self.heading_error = msg.data

    # path

    def _path_cb(self, msg: Path):

        if self.path_saved:
            return

        if not msg.poses:
            return

        self.get_logger().info(
            f'Guardando {len(msg.poses)} waypoints del path'
        )

        for pose in msg.poses:

            row = {
                "timestamp_s": None,
                "elapsed_s": None,
                "robot_x": None,
                "robot_y": None,
                "robot_yaw": None,
                "path_x": pose.pose.position.x,
                "path_y": pose.pose.position.y,
            }

            if self.save_errors:

                row.update({
                    "delta": None,
                    "cross_track_error": None,
                    "heading_error": None
                })

            self.writer.writerow(row)

        self._file.flush()

        self.path_saved = True

    # cierre

    def _close(self):

        self._file.flush()
        self._file.close()

        self.get_logger().info("CSV cerrado")


def main():

    rclpy.init()

    node = StanleyLogger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node._close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()