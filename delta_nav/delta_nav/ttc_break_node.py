import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
from std_msgs.msg import Bool, Int32
from std_msgs.msg import Float32MultiArray



class TTCBreakNode(Node):
    def __init__(self):
        super().__init__('ttc_break_node')
        
        # ---- Parameters ----
        self.declare_parameter('publish_rate', 100.0)           # Hz - rate to check TTC and publish commands
        self.declare_parameter('ttc_threshold', 0.5)            # seconds - TTC threshold for emergency braking
        self.declare_parameter('min_distance_threshold', 0.6)   # meters - minimum distance to obstacle for braking
        self.declare_parameter('forward_angle_range', 8.0)      # degrees - angle range in front of robot to consider
        self.declare_parameter('min_range', 0.01)                # meters - ignore measurements closer than this
        self.declare_parameter('max_range', 12.0)               # meters - ignore measurements farther than this
        self.declare_parameter("safety_bubble", True)           # If true, consider all measurements within min_distance_threshold as braking threats regardless of TTC
        self.declare_parameter("heartbeat_rate_hz", 1.0)
        self.declare_parameter("enable_narrow_section_detection", True)
        self.declare_parameter("narrow_section_topic", "/narrow_section_active")
        self.declare_parameter("narrow_side_angle_window", 20.0)   # degrees around +/-90 used to sense side walls
        self.declare_parameter("narrow_corridor_width_threshold", 1.5)  # meters - activate if left+right width <= this
        self.declare_parameter("narrow_corridor_width_clear_threshold", 1.7)  # meters - clear hysteresis threshold
        self.declare_parameter("narrow_min_points_per_side", 3)   # min valid side-wall points on each side
        self.declare_parameter("narrow_hold_sec", 0.6)          # keep active for this time after last positive detection

        # Deprecated compatibility params accepted from older YAML files; ignored by current detector.
        self.declare_parameter("narrow_angle_range", 25.0)
        self.declare_parameter("narrow_distance_threshold", 0.85)
        self.declare_parameter("narrow_min_points", 10)
        self.declare_parameter("narrow_ratio_activate", 0.50)
        self.declare_parameter("narrow_ratio_clear", 0.35)


        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.ttc_threshold = float(self.get_parameter('ttc_threshold').value)
        self.min_distance_threshold = float(self.get_parameter('min_distance_threshold').value)
        self.forward_angle_range = float(self.get_parameter('forward_angle_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.safety_bubble = bool(self.get_parameter('safety_bubble').value)
        self.heartbeat_rate_hz = float(self.get_parameter('heartbeat_rate_hz').value)
        self.enable_narrow_section_detection = bool(self.get_parameter('enable_narrow_section_detection').value)
        self.narrow_section_topic = str(self.get_parameter('narrow_section_topic').value)
        self.narrow_side_angle_window = max(float(self.get_parameter('narrow_side_angle_window').value), 0.0)
        self.narrow_corridor_width_threshold = max(
            float(self.get_parameter('narrow_corridor_width_threshold').value),
            0.0,
        )
        self.narrow_corridor_width_clear_threshold = max(
            float(self.get_parameter('narrow_corridor_width_clear_threshold').value),
            self.narrow_corridor_width_threshold,
        )
        self.narrow_min_points_per_side = max(int(self.get_parameter('narrow_min_points_per_side').value), 1)
        self.narrow_hold_sec = max(float(self.get_parameter('narrow_hold_sec').value), 0.0)

        # Deprecated params are intentionally not used by the width-based narrow detector.

        # ---- State ----
        self.current_cmd_vel = TwistStamped()    # Last received command velocity
        self.last_laser_scan = None              # Last received laser scan
        self.min_ttc = float('inf')              # Minimum TTC from all forward objects
        self.min_distance = float('inf')         # Minimum distance from all obstacles in relevant zone
        self.should_brake = False                # Flag indicating if emergency brake is needed
        self.brake_trigger_angle = None          # Angle of the measurement that triggered brake
        self.is_stopped = None                   # Last published /brake_active state
        self.is_warned = None                    # Last published /brake_warn state
        self.narrow_section_active = None
        self.last_narrow_positive_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_narrow_corridor_width = float('inf')
        
        # ---- Pub/Sub ----
        # QoS profile compatible with sensor publishers (RELIABLE, VOLATILE)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to laser scan data
        self.create_subscription(LaserScan, '/scan', self._laser_scan_callback, qos)
        
        # Subscribe to commanded velocity (so we can intercept forward commands)
        self.create_subscription(TwistStamped, '/diffdrive_controller/cmd_vel', self._cmd_vel_callback, qos)
        
        # Publisher for safety-override velocity commands
        self.cmd_ttc_publisher = self.create_publisher(TwistStamped, '/cmd_ttc', qos)
         # Publisher for safety-override velocity commands
        self.brake_pub = self.create_publisher(Bool, '/brake_active', 10)
        # Publisher for brake direction (0=right, 1=left, 3=indeterminate)
        self.dir_brake_pub = self.create_publisher(Int32, '/dir_brake', 10)
        self.narrow_section_pub = self.create_publisher(Bool, self.narrow_section_topic, 10)

        if self.safety_bubble:
            self.brake_warn_pub = self.create_publisher(Bool, '/brake_warn', 10)

        self.ttc_array_pub = self.create_publisher(Float32MultiArray, '/ttc_values', 10)
        
        # ---- Timer ----
        # Timer to periodically check TTC and issue commands if needed
        self.dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.dt, self._check_ttc_and_publish)

        # heartbeat timer to log that node is alive
        # self.heartbeat_timer = self.create_timer(1.0 / self.heartbeat_rate_hz, self._heartbeat_callback)

    def _heartbeat_callback(self):
        # Publish the current state of only the brake state
        if self.is_stopped is not None:
            brake_msg = Bool()
            brake_msg.data = self.is_stopped
            self.brake_pub.publish(brake_msg)


    def _laser_scan_callback(self, msg: LaserScan):
        """Store the latest laser scan message."""
        self.last_laser_scan = msg
    
    def _cmd_vel_callback(self, msg: TwistStamped):
        """Store the latest commanded velocity from the controller."""
        self.current_cmd_vel = msg

    def _publish_brake_state_if_changed(self, brake_state: bool):
        """Publish /brake_active only when state changes."""
        if self.is_stopped == brake_state:
            return

        brake_msg = Bool()
        brake_msg.data = brake_state
        self.brake_pub.publish(brake_msg)
        self.is_stopped = brake_state

    def _publish_warn_state_if_changed(self, warn_state: bool):
        """Publish /brake_warn only when state changes."""
        if not self.safety_bubble:
            return
        if self.is_warned == warn_state:
            return
        
        if warn_state:
            self.get_logger().warn(
                f"SAFETY BUBBLE ALERT: Obstacle at {warn_state:.2f}m within safety bubble "
                f"(threshold: {self.min_distance_threshold}m)."
                f"Issuing brake warning!"
            )
        else:
            self.get_logger().info(
                "SAFETY BUBBLE CLEAR: No obstacles within safety bubble. Clearing brake warning!"
            )

        brake_msg = Bool()
        brake_msg.data = warn_state
        self.brake_warn_pub.publish(brake_msg)
        self.is_warned = warn_state

    def _publish_narrow_state_if_changed(self, narrow_state: bool):
        """Publish /narrow_section_active only on transitions."""
        if self.narrow_section_active == narrow_state:
            return

        msg = Bool()
        msg.data = narrow_state
        self.narrow_section_pub.publish(msg)
        self.narrow_section_active = narrow_state

        if narrow_state:
            if math.isfinite(self.last_narrow_corridor_width):
                self.get_logger().warn(
                    "NARROW SECTION detected. "
                    f"Estimated corridor width={self.last_narrow_corridor_width:.2f}m "
                    f"<= {self.narrow_corridor_width_threshold:.2f}m."
                )
            else:
                self.get_logger().warn("NARROW SECTION detected. Publishing speed-reduction hint.")
        else:
            if math.isfinite(self.last_narrow_corridor_width):
                self.get_logger().info(
                    "NARROW SECTION cleared. "
                    f"Estimated corridor width={self.last_narrow_corridor_width:.2f}m "
                    f">= {self.narrow_corridor_width_clear_threshold:.2f}m."
                )
            else:
                self.get_logger().info("NARROW SECTION cleared.")

    def _min_side_distance(self, values):
        """Robust side-wall distance estimate from the smallest few lateral samples."""
        if not values:
            return float('inf')

        ordered = sorted(values)
        k = min(3, len(ordered))
        return sum(ordered[:k]) / float(k)

    def _detect_narrow_section(self, scan_msg: LaserScan) -> bool:
        """
        Detect narrow sections from side-wall corridor width with hysteresis and hold time.
        """
        if not self.enable_narrow_section_detection:
            return False

        side_window_rad = math.radians(self.narrow_side_angle_window)
        left_samples = []
        right_samples = []
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        for i, range_val in enumerate(scan_msg.ranges):
            if not math.isfinite(range_val):
                continue
            if range_val < self.min_range or range_val > self.max_range:
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Convert to lateral distance from robot centerline.
            lateral = abs(range_val * math.sin(angle))

            # Side-wall windows centered at +/- 90 deg.
            if abs(angle - (math.pi / 2.0)) <= side_window_rad:
                left_samples.append(lateral)
            elif abs(angle + (math.pi / 2.0)) <= side_window_rad:
                right_samples.append(lateral)

        is_active = bool(self.narrow_section_active)
        if (
            len(left_samples) < self.narrow_min_points_per_side
            or len(right_samples) < self.narrow_min_points_per_side
        ):
            self.last_narrow_corridor_width = float('inf')
            if is_active and (now_sec - self.last_narrow_positive_time) < self.narrow_hold_sec:
                return True
            return False

        left_dist = self._min_side_distance(left_samples)
        right_dist = self._min_side_distance(right_samples)
        corridor_width = left_dist + right_dist
        self.last_narrow_corridor_width = corridor_width

        activate = corridor_width <= self.narrow_corridor_width_threshold
        clear = corridor_width >= self.narrow_corridor_width_clear_threshold

        if activate:
            self.last_narrow_positive_time = now_sec
            return True

        if is_active:
            if (now_sec - self.last_narrow_positive_time) < self.narrow_hold_sec:
                return True
            if not clear:
                return True

        return False
    
    def _calculate_ttc_for_measurement(self, range_val: float, angle: float, 
                                        cmd_linear_x: float) -> float:
        """
        Calculate Time-To-Collision (TTC) for a single laser measurement.
        
        Args:
            range_val: Distance to object (meters)
            angle: Angle of measurement relative to robot forward (radians)
            cmd_linear_x: Commanded linear velocity in x direction (m/s)
        
        Returns:
            TTC in seconds, or infinity if no collision threat
        """
        # Filter out invalid measurements
        if range_val < self.min_range or range_val > self.max_range:
            return float('inf')
        
        # Calculate ri_dot: projection of relative velocity onto line-of-sight
        # ri_dot = V_x * cos(theta)
        ri_dot = cmd_linear_x * math.cos(angle)
        
        # TTC = ri / ri_dot
        ri_dot = max(ri_dot, 1e-5)  # Avoid division by zero
        ttc = range_val / ri_dot
        
        # Only consider positive TTC values
        if ttc > 0.0:
            return ttc
        else:
            return float('inf')
    
    def _determine_brake_direction(self) -> int:
        """
        Determine the direction of the obstacle that triggered braking.
        
        Returns:
            0 if obstacle is on the right side (angle < 0)
            1 if obstacle is on the left side (angle > 0)
            3 if cannot be determined or indeterminate (angle ≈ 0)
        """
        if self.brake_trigger_angle is None:
            return 3
        
        # Threshold to classify as "straight ahead" (indeterminate)
        straight_threshold = math.radians(self.forward_angle_range*0.03) # 3% of forward angle range
        
        if abs(self.brake_trigger_angle) < straight_threshold:
            return 3  # Straight ahead, indeterminate
        elif self.brake_trigger_angle < 0:
            return 0  # Right side
        else:
            return 1  # Left side
        
    def _determine_brake_direction_ang(self, angle) -> int:
        """
        Determine the direction of the obstacle that triggered braking.
        
        Returns:
            0 if obstacle is on the right side (angle < 0)
            1 if obstacle is on the left side (angle > 0)
            3 if cannot be determined or indeterminate (angle ≈ 0)
        """
        if angle is None:
            return 3
        
        # Threshold to classify as "straight ahead" (indeterminate)
        straight_threshold = math.radians(self.forward_angle_range*0.03) # 3% of forward angle range
        
        if abs(angle) < straight_threshold:
            return 3  # Straight ahead, indeterminate
        elif angle < 0:
            return 0  # Right side
        else:
            return 1  # Left side
    
    def _compute_directional_ttc_array(self, scan_msg, cmd_linear_x):
        """
        Compute TTC for the front 180° ([-90°, +90°]).
        All other rays are set to inf.

        Note: this node is forward-only by design.
        Returns:
            List of TTC values (same size as scan.ranges)
        """

        ttc_array = []

        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

            # Half plane = 180 degrees
        half_plane_rad = math.pi / 2.0

        for i, range_val in enumerate(scan_msg.ranges):

                # Default value
            ttc = float('inf')

                # Skip invalid ranges
            if not math.isfinite(range_val):
                    ttc_array.append(ttc)
                    continue

            angle = angle_min + i * angle_increment

                # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Front 180° only.
            in_direction = abs(angle) <= half_plane_rad

            if in_direction:
                ttc = self._calculate_ttc_for_measurement(
                    range_val, angle, cmd_linear_x
                 )

            ttc_array.append(float(ttc))

        return ttc_array

    
    def _check_ttc_and_publish(self):
        """
        Periodically check TTC for forward obstacles and publish safety override if needed.
        """
        scan_msg = self.last_laser_scan
        ttc_array = []
        # Early exit if no recent laser scan data
        if self.last_laser_scan is None:
            return

        narrow_active = self._detect_narrow_section(scan_msg)
        self._publish_narrow_state_if_changed(narrow_active)
        
        # Reset minimum TTC for this cycle
        self.min_ttc = float('inf')
        self.min_distance = float('inf')
        self.brake_trigger_angle = None
        
        # Get the current forward velocity command
        cmd_linear_x = self.current_cmd_vel.twist.linear.x
        
        # Forward-only mode: treat non-forward commands as stopped.
        if cmd_linear_x <= 0.0:
            # Create array of inf values for all laser rays
            directional_ttc_array = [float('inf')] * len(scan_msg.ranges)
            # Add direction flag (0.0 for forward by default)
            directional_ttc_array.append(0.0)
            
            ttc_msg = Float32MultiArray()
            ttc_msg.data = directional_ttc_array
            self.ttc_array_pub.publish(ttc_msg)
            
            # Keep existing behavior: publish True on /brake_active while stopped.
            self._publish_brake_state_if_changed(True)
            return
        
        # Process laser scan measurements
        scan_msg = self.last_laser_scan
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        
        # Convert forward angle range to radians
        forward_range_rad = math.radians(self.forward_angle_range)
        bubble_warn_active = False
        bubble_warn_distance = float('inf')
        bubble_warn_angle = None
        
        # Iterate through measurements
        for i, range_val in enumerate(scan_msg.ranges):
            # Skip invalid measurements (inf or nan)
            ttc = float('inf')
            if not math.isfinite(range_val):
                continue
            
            # Calculate the angle of this measurement
            angle = angle_min + i * angle_increment
            
            # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # If the safety bubble parameter is enabled, consider any measurement within min_distance_threshold as a threat regardless of TTC
            if self.safety_bubble and range_val < self.min_distance_threshold:
                bubble_warn_active = True
                if range_val < bubble_warn_distance:
                    bubble_warn_distance = range_val
                    bubble_warn_angle = angle
                continue
            
            # Forward-only: check only front cone (around angle 0)
            in_relevant_zone = abs(angle) <= forward_range_rad
            
            if in_relevant_zone:
                ttc = self._calculate_ttc_for_measurement(range_val, angle, cmd_linear_x)
                
                # Track minimum TTC
                if ttc < self.min_ttc:
                    self.min_ttc = ttc
                    self.brake_trigger_angle = angle
                
                # Track minimum distance
                if range_val < self.min_distance:
                    self.min_distance = range_val
                    if self.min_distance < self.min_distance_threshold:
                        self.brake_trigger_angle = angle

            # Append TTC for this ray (inf if not relevant or invalid)
            ttc_array.append(float(ttc))
            
        # Compute full directional TTC array (180° depending on motion)
        directional_ttc_array = self._compute_directional_ttc_array(
            scan_msg, cmd_linear_x
        )

        # Publish safety-bubble warning only on state transitions.
        if self.safety_bubble:
            self._publish_warn_state_if_changed(bubble_warn_active)
            if bubble_warn_active and not self.should_brake:
                dir_msg = Int32()
                dir_msg.data = self._determine_brake_direction_ang(bubble_warn_angle)
                self.dir_brake_pub.publish(dir_msg)
                self.get_logger().warn(
                    f"(threshold: {self.min_distance_threshold}m). Direction {dir_msg.data}."
                )

        # Append direction flag as forward-only mode marker.
        directional_ttc_array.append(0.0)

        ttc_msg = Float32MultiArray()
        ttc_msg.data = directional_ttc_array
        self.ttc_array_pub.publish(ttc_msg)
        total_values = len(ttc_msg.data)
        finite_values = sum(1 for v in ttc_msg.data if math.isfinite(v))

        # self.get_logger().info(
        #     f"TTC array size: {total_values} | Finite TTC values: {finite_values}"
        # )



        # Determine if we should brake
        was_braking = self.should_brake
        # Brake if TTC is below threshold OR if any obstacle is too close (within min_distance_threshold)
        self.should_brake = ((self.min_ttc < self.ttc_threshold and self.min_ttc != float('inf')) or 
                             (self.min_distance < self.min_distance_threshold))
        
        # Publish brake state only on state transitions.
        self._publish_brake_state_if_changed(self.should_brake)
        
        # Publish brake direction
        if self.should_brake:
            dir_msg = Int32()
            dir_msg.data = self._determine_brake_direction()
            self.dir_brake_pub.publish(dir_msg)

        # Log state changes
        if self.should_brake != was_braking:
            if self.should_brake:
                self.get_logger().warn(
                    f"COLLISION ALERT: TTC = {self.min_ttc:.2f}s (threshold: {self.ttc_threshold}s), "
                    f"Distance = {self.min_distance:.2f}m (threshold: {self.min_distance_threshold}m). "
                    f"Issuing emergency brake!"
                )
            else:
                self.get_logger().info(
                    f"Collision cleared. TTC = {self.min_ttc:.2f}s, Distance = {self.min_distance:.2f}m"
                )
        
        # Publish safety override command if braking is needed
        if self.should_brake:
            # Create a zero-velocity command to override forward motion
            '''For a differential drive robot, we typically want to zero out the forward 
            linear velocity (x) but allow rotation and lateral motion if needed.'''
            safety_cmd = TwistStamped()
            safety_cmd.twist.linear.x = 0.0      # Zero forward velocity
            safety_cmd.twist.linear.y = 0.0      # No lateral motion
            safety_cmd.twist.linear.z = 0.0      # No vertical motion
            # Allow rotation while braking (do not constrain angular components).
            safety_cmd.twist.angular.x = 0.0    # No pitch
            safety_cmd.twist.angular.y = 0.0      # No roll
            safety_cmd.twist.angular.z = self.current_cmd_vel.twist.angular.z  # Allow rotation if needed
            
            self.cmd_ttc_publisher.publish(safety_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TTCBreakNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()