"""
ttc_gap_control.launch.py

    Launch the three-node autonomous navigation stack:
      1. break_ttc_directional  — TTC computation + emergency braking
      2. rc_gap_logger          — Gap detection + steering angle
      3. rc_controller          — Signal smoothing + velocity command

    Assumptions:
    - LiDAR topic:         /scan           (sensor_msgs/LaserScan)
    - External cmd:        /cmd_vel        (geometry_msgs/TwistStamped)
    - Start signal:        /start          (std_msgs/Bool)
    - All nodes live in package: <your_pkg>  ← change PKG_NAME below

    Put this file in: <your_pkg>/launch/ttc_gap_control.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PKG_NAME = "car_followgap_pkg"  # ← Replace with your actual package name


def generate_launch_description():

    # ------------------------------------------------------------------ #
    #  Launch arguments                                                    #
    # ------------------------------------------------------------------ #

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    forward_velocity_arg = DeclareLaunchArgument(
        "forward_velocity",
        default_value="2.1",
        description="Base forward velocity for the controller (m/s)",
    )

    ttc_threshold_arg = DeclareLaunchArgument(
        "ttc_threshold",
        default_value="0.2",
        description="TTC threshold for emergency braking (seconds)",
    )

    min_distance_threshold_arg = DeclareLaunchArgument(
        "min_distance_threshold",
        default_value="0.10",
        description="Minimum distance to obstacle before braking (meters)",
    )

    pub_logger_arg = DeclareLaunchArgument(
        "pub_logger",
        default_value="true",
        description="Enable logging publishers on gap_logger and controller",
    )

    start_flag_arg = DeclareLaunchArgument(
        "start_flag",
        default_value="false",
        description="Start the controller immediately without waiting for /start signal",
    )

    # Convenience references
    use_sim_time       = LaunchConfiguration("use_sim_time")
    forward_velocity   = LaunchConfiguration("forward_velocity")
    ttc_threshold      = LaunchConfiguration("ttc_threshold")
    min_distance_thr   = LaunchConfiguration("min_distance_threshold")
    pub_logger         = LaunchConfiguration("pub_logger")
    start_flag         = LaunchConfiguration("start_flag")

    # ------------------------------------------------------------------ #
    #  Node 1 — break_ttc_directional                                     #
    #  Reads /scan and /cmd_vel, publishes /ttc_values, /cmd_ttc,         #
    #  /brake_active                                                       #
    # ------------------------------------------------------------------ #

    break_ttc_node = Node(
        package=PKG_NAME,
        executable="break_ttc_directional",
        name="break_ttc_directional",
        output="screen",
        parameters=[{
            "use_sim_time":           use_sim_time,
            "publish_rate":           20.0,         # Hz
            "ttc_threshold":          ttc_threshold,
            "min_distance_threshold": min_distance_thr,
            "forward_angle_range":    10.0,           # degrees
            "rear_angle_range":       10.0,           # degrees
            "min_range":              0.1,            # meters
            "max_range":              1.3,            # meters
        }],
        remappings=[
            # Uncomment to remap topics if your setup differs:
            # ("/scan",    "/lidar/scan"),
            # ("/cmd_vel", "/nav/cmd_vel"),
        ],
    )

    # ------------------------------------------------------------------ #
    #  Node 2 — rc_gap_logger                                             #
    #  Reads /scan, /ttc_values, /error.                                  #
    #  Publishes /cmd_ang_tcc, /gap_marker                                #
    # ------------------------------------------------------------------ #

    gap_logger_node = Node(
        package=PKG_NAME,
        executable="rc_gap_logger",
        name="rc_gap_logger",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "pub_logger":   pub_logger,
        }],
        remappings=[
            # ("/scan", "/lidar/scan"),
        ],
    )

    # ------------------------------------------------------------------ #
    #  Node 3 — rc_controller                                             #
    #  Reads /cmd_ang_tcc, /start.                                        #
    #  Publishes /cmd_vel_ttc_gap                                         #
    # ------------------------------------------------------------------ #

    controller_node = Node(
        package=PKG_NAME,
        executable="rc_controller",
        name="rc_controller",
        output="screen",
        parameters=[{
            "use_sim_time":     use_sim_time,
            "forward_velocity": forward_velocity,
            "start_flag":       start_flag,
            "pub_logger":       pub_logger,
        }],
        remappings=[
            # ("/cmd_vel_ttc_gap", "/robot/cmd_vel"),
        ],
    )

    # ------------------------------------------------------------------ #
    #  Launch description                                                  #
    # ------------------------------------------------------------------ #

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        forward_velocity_arg,
        ttc_threshold_arg,
        min_distance_threshold_arg,
        pub_logger_arg,
        start_flag_arg,

        # Node 1 — starts immediately (safety layer should be up first)
        break_ttc_node,

        # Node 2 — slight delay so /ttc_values is already being published
        TimerAction(period=1.0, actions=[gap_logger_node]),

        # Node 3 — starts last; depends on /cmd_ang_tcc from node 2
        TimerAction(period=2.0, actions=[controller_node]),
    ])