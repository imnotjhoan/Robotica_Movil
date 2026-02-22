# DELTA - Autonomous Mobile Robot Platform

A ROS 2-based autonomous mobile robot system with multiple reactive navigation methods including wall-following, gap-distance following, and time-to-collision (TTC) gap navigation.

## Repository Architecture

The workspace is organized into several ROS 2 packages, each handling specific aspects of the robot's operation:

```
delta_ws/
├── delta_bringup          # Robot spawning and simulation setup
├── delta_control          # Control layer abstractions
├── delta_description      # Robot URDF/Xacro models and kinematics
├── delta_ekf              # Extended Kalman Filter for localization
├── delta_follow_the_gap   # Gap-following reactive navigation methods
├── delta_gazebo           # Gazebo simulation environment and worlds
├── delta_measure          # Performance metrics and data logging
├── delta_nav              # Navigation utilities and start controller
└── delta_wall_following   # Wall-following reactive navigation method
```

## Package Descriptions

### **delta_bringup**
Handles robot instantiation and simulation environment setup. Launches the robot state publisher, Gazebo simulator, and controller spawning for initial simulation setup.

**Nodes:**
- `robot_state_publisher`: Publishes robot model state and TF transforms from URDF description to `odom` and `base_link` frames.

**Launch Files:**
- `gz_spawn.launch.py`: Launches Gazebo with the selected world, spawns the robot, and starts the robot state publisher. Configure the world by changing the `gz_world` variable (options: empty_world.sdf, j_maze.sdf, melgui_maze.sdf, DemoRaceTrack.sdf, RaceTrack.sdf).
- `rsp.launch.py`: Launches only the robot state publisher for offline URDF testing without Gazebo.

### **delta_control**
C++ package containing low-level motor control implementations. Currently a placeholder for hardware abstraction layer.

**Nodes:**
- None active (legacy)

### **delta_description**
Contains the robot's URDF/Xacro models defining the kinematic structure, sensors (LIDAR, IMU, camera), visual appearance, and simulation parameters.

**Nodes:**
- None (configuration files only)

**Key Files:**
- `robot.urdf.xacro`: Main differential-drive robot model (base configuration)
- `ackerman_robot.urdf.xacro`: Ackermann steering variant
- `ros2_control.xacro`: Hardware interface definitions for controllers
- `lidar.xacro`, `imu.xacro`, `depth_camera.xacro`: Sensor definitions

### **delta_ekf**
Implements an Extended Kalman Filter for robot localization and state estimation. Fuses wheel odometry with IMU data to provide robust pose estimation.

**Nodes:**
- `ekf_node`: Estimates robot pose (x, y, yaw) by fusing differential-drive wheel odometry and IMU angular velocity. Publishes filtered odometry and TF transforms between `odom` and `base_link` frames.

**Launch Files:**
- `ekf.launch.py`: Starts the EKF node with configuration from `config/ekf.yaml`. Includes optional visualization of odometry data.

### **delta_follow_the_gap**
Implements follow-the-gap navigation algorithms with multiple reactive methods for obstacle avoidance based on LIDAR data.

**Nodes:**
- `gap_distance_node`: Detects the largest navigable gap in LIDAR data using distance-based scoring. Publishes target angular command to `/cmd_ang_tcc` topic.
- `gap_distance_controller`: Receives distance-based gap commands and converts them to velocity commands with PD control. Publishes to `/cmd_vel_distance_gap`.
- `control_gap_ttc`: Implements controller for TTC-based gap following. Receives gap commands and publishes velocity commands to `/cmd_vel_ttc_gap` with dynamic forward velocity.
- `ttc_gap_logger_node`: Computes Time-to-Collision (TTC) for all LIDAR measurements and identifies the best navigable gap. Publishes TTC values and angular error commands.
- `ttc_break_gap_node`: Emergency braking node for gap-following methods. Monitors TTC and distance to obstacles, triggers emergency stop if thresholds exceeded.

**Launch Files:**
- `distance_gap_follower.launch.py`: Launches the distance-based gap following pipeline (gap_distance_node + gap_distance_controller + ttc_break_gap_node from delta_nav).
- `distance_gap_follower_obs.launch.py`: Variant with obstacle detection and visualization.
- `gap_follower.launch.py`: Launches the TTC-based gap following pipeline (ttc_gap_logger_node + control_gap_ttc + ttc_break_gap_node).

### **delta_gazebo**
Simulation environment definitions with multiple world configurations for testing.

**Nodes:**
- None (Gazebo simulator)

**Worlds:**
- `empty_world.sdf`: Featureless environment for basic testing
- `j_maze.sdf`, `melgui_maze.sdf`: Maze environments for navigation testing
- `DemoRaceTrack.sdf`, `RaceTrack.sdf`, `RaceTrackObs.sdf`: Racing circuit configurations

### **delta_measure**
Data logging and performance metrics collection for autonomous navigation experiments.

**Nodes:**
- `metrics_node`: Listens for robot velocity commands, pose, and brake signals. Records trajectory, velocity, and performance metrics to a timestamped CSV file. Logs start on `/start` signal and stop on `Ctrl+C`.

### **delta_nav**
Navigation utilities and session control for coordinating reactive method execution.

**Nodes:**
- `start_controller`: Manages experiment start/stop signals via keyboard input (press 's' then 'enter') or joystick command. Publishes `/start` signal to coordinate all reactive method nodes.
- `ttc_break_node`: Emergency braking monitor for wall-following methods. Computes Time-to-Collision from LIDAR and distance thresholds to trigger emergency stop.

### **delta_wall_following**
Wall-following reactive navigation method for autonomous corridor and wall-following tasks.

**Nodes:**
- `dist_finder`: Analyzes LIDAR scan to extract wall distance and orientation. Computes target steering angle based on desired wall distance using geometric distance estimation. Publishes angular command.
- `control`: PD controller for wall-following steering. Receives distance-based angle command and converts to velocity commands with forward velocity modulation. Publishes to `/cmd_vel`.

**Launch Files:**
- `wall_follower.launch.py`: Launches the complete wall-following pipeline (dist_finder + control + ttc_break_node from delta_nav) with safety braking enabled.

---

## Running Reactive Methods

This section provides step-by-step instructions to run any of the three reactive navigation methods.

### Prerequisites
All terminals should be in the workspace root directory:
```bash
cd ~/mrad_ws_2601_delta
source install/setup.bash
```

### Steps

**Step 1: Configure World Selection**
Edit `/src/delta_bringup/launch/gz_spawn.launch.py` and change the `gz_world` variable to select the desired world:
```python
maze = ["empty_world.sdf", "j_maze.sdf", "melgui_maze.sdf", "DemoRaceTrack.sdf", "RaceTrack.sdf", "RaceTrackObs.sdf"]
gz_world = maze[0]  # Change index to select different world
```

**Step 2: Launch Gazebo Simulation**
In **Terminal 1**, launch the simulation environment with the robot:
```bash
ros2 launch delta_bringup gz_spawn.launch.py use_sim_time:=true 
```


**Step 3: Launch Desired Reactive Method**
In **Terminal 2**, launch one of the following reactive navigation methods:

**Wall Following:**
```bash
ros2 launch delta_wall_following wall_follower.launch.py
```

**Distance-Based Gap Following: Optimized for Map without Obstacles**
```bash
ros2 launch delta_follow_the_gap distance_gap_follower.launch.py
```
**Distance-Based Gap Following: Optimized for Map with Obstacles**
```bash
ros2 launch delta_follow_the_gap distance_gap_follower_obs.launch.py
```

**TTC-Based Gap Following:**
```bash
ros2 launch delta_follow_the_gap gap_follower.launch.py
```

**Step 4: Start Navigation**
In **Terminal 3**, launch the start controller and begin navigation:
```bash
ros2 run delta_nav start_controller
```
When ready, press **'s'** followed by **'enter'** to signal the start. The reactive method will begin autonomous navigation.

**Step 5 (Optional): Collect Performance Metrics**
In **Terminal 4**, start the metrics logging node to record trajectory and performance data:
```bash
ros2 run delta_measure metrics_node
```
This will create a CSV file in the hard-coded directory (make sure to change it to your desired path) with timestamped data including:
- Robot velocity commands (linear and angular)
- Robot pose estimates (x, y position)
- Accumulated distance traveled
- Brake state and events

The metrics node will record data from the moment the start signal is received (Step 4) until you stop it with **Ctrl+C**.

### Example Complete Workflow
```bash
# Terminal 1: Simulate
ros2 launch delta_bringup gz_spawn.launch.py 

# Terminal 2: Wall Following
ros2 launch delta_wall_following wall_follower.launch.py

# Terminal 3: Start Navigation (press 's' + enter when ready)
ros2 run delta_nav start_controller

# Terminal 4: Log Metrics (optional)
ros2 run delta_measure metrics_node
# Stop with Ctrl+C when done
```
