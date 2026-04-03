import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition  

import xacro

ack = False
maze = ["empty_world.sdf", "j_maze.sdf", "melgui_maze.sdf", "DemoRaceTrack.sdf", "RaceTrack.sdf", "RaceTrackObs.sdf" , "walls_world2.sdf"]
xacro_model = "ackerman_robot.urdf.xacro" if ack else "robot_ph.urdf.xacro"
gz_world = maze[5] # Change the index to select a different world

def generate_launch_description():
    gazebo_pkg_name = "delta_gazebo"
    bringup_pkg_name = "delta_bringup"
    description_pkg_name = "delta_description"

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    # --- Robot description (xacro -> URDF XML string) ---
    xacro_file = os.path.join(get_package_share_directory(description_pkg_name), "diffdrive_urdf", xacro_model)
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": False}],
    )

 

    return LaunchDescription([
       
      
        rsp,
     
    ])