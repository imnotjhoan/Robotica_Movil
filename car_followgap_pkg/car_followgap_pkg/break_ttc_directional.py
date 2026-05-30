import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

class break_ttc_directional(Node):
    def __init__(self):
        super().__init__('break_ttc_directional')
        
        # ---- Parameters ----
        self.declare_parameter('publish_rate', 10.0)           
        self.declare_parameter('ttc_threshold', 1.0)            
        self.declare_parameter('min_distance_threshold', 0.5)   
        self.declare_parameter('forward_angle_range', 20.0)     
        self.declare_parameter('rear_angle_range', 10.0)        
        self.declare_parameter('min_range', 0.01)                
        self.declare_parameter('max_range', 1.3)               
        self.declare_parameter('pub_logger', True) # Nueva opción

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.ttc_threshold = float(self.get_parameter('ttc_threshold').value)
        self.min_distance_threshold = float(self.get_parameter('min_distance_threshold').value)
        self.forward_angle_range = float(self.get_parameter('forward_angle_range').value)
        self.rear_angle_range = float(self.get_parameter('rear_angle_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.pub_logger = bool(self.get_parameter('pub_logger').value)
        
        # ---- State ----
        self.current_cmd_vel = TwistStamped()    
        self.last_laser_scan = None              
        self.min_ttc = float('inf')              
        self.min_distance = float('inf')         
        self.should_brake = False                
        
        # Variables para el Logger (Persistentes)
        self.log_finite_ttc_count = 0
        self.log_total_ttc_values = 0

        # ---- Pub/Sub ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(LaserScan, '/scan', self._laser_scan_callback, qos)
        self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_vel_callback, qos)
        self.cmd_ttc_publisher = self.create_publisher(TwistStamped, '/cmd_ttc', qos)
        self.brake_pub = self.create_publisher(Bool, '/brake_active', 10)
        self.ttc_array_pub = self.create_publisher(Float32MultiArray, '/ttc_values', 10)
        
        # ---- Timers ----
        self.dt = 1.0 / self.publish_rate
        # Timer principal de control (10Hz)
        self.timer = self.create_timer(self.dt, self._check_ttc_and_publish)
        # Timer de log (0.5Hz = cada 2 segundos)
        self.timer_log = self.create_timer(2.0, self._log_timer_cb)

        self.get_logger().info('break_ttc_directional ready.')

    def _log_timer_cb(self):
        """Muestra el estado de seguridad cada 2 segundos."""
        if self.pub_logger:
            status = "BRAKING" if self.should_brake else "CLEAR"
            self.get_logger().info(
                f"[{status}] | TTC min: {self.min_ttc:.2f}s | "
                f"Dist min: {self.min_distance:.2f}m | "
                f"Finite TTCs: {self.log_finite_ttc_count}/{self.log_total_ttc_values}"
            )

    def _laser_scan_callback(self, msg: LaserScan):
        self.last_laser_scan = msg
    
    def _cmd_vel_callback(self, msg: TwistStamped):
        self.current_cmd_vel = msg
    
    def _calculate_ttc_for_measurement(self, range_val: float, angle: float, cmd_linear_x: float) -> float:
        if range_val < self.min_range or range_val > self.max_range:
            return float('inf')
        
        ri_dot = cmd_linear_x * math.cos(angle)
        ri_dot = max(ri_dot, 1e-5) 
        ttc = range_val / ri_dot
        
        return ttc if ttc > 0.0 else float('inf')
    
    def _compute_directional_ttc_array(self, scan_msg, cmd_linear_x):
        ttc_array = []
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        half_plane_rad = math.pi / 2.0
        is_moving_forward = cmd_linear_x > 0.0

        for i, range_val in enumerate(scan_msg.ranges):
            ttc = float('inf')
            if not math.isfinite(range_val):
                ttc_array.append(ttc)
                continue

            angle = angle_min + i * angle_increment
            # Normalize angle
            angle = (angle + math.pi) % (2 * math.pi) - math.pi

            if is_moving_forward:
                in_direction = abs(angle) <= half_plane_rad
            else:
                in_direction = abs(angle) >= half_plane_rad

            if in_direction:
                ttc = self._calculate_ttc_for_measurement(range_val, angle, cmd_linear_x)

            ttc_array.append(float(ttc))
        return ttc_array

    def _check_ttc_and_publish(self):
        if self.last_laser_scan is None:
            return
        
        self.min_ttc = float('inf')
        self.min_distance = float('inf')
        cmd_linear_x = self.current_cmd_vel.twist.linear.x

        if cmd_linear_x == 0.0:
            # Publicar freno falso si no hay movimiento
            brake_msg = Bool(); brake_msg.data = False
            self.brake_pub.publish(brake_msg)
            return
        
        scan_msg = self.last_laser_scan
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        is_moving_forward = cmd_linear_x > 0.0
        
        forward_range_rad = math.radians(self.forward_angle_range)
        rear_range_rad = math.radians(self.rear_angle_range)
        
        for i, range_val in enumerate(scan_msg.ranges):
            if not math.isfinite(range_val):
                continue
            
            angle = angle_min + i * angle_increment
            angle = (angle + math.pi) % (2 * math.pi) - math.pi
            
            in_relevant_zone = False
            if is_moving_forward:
                in_relevant_zone = abs(angle) <= forward_range_rad
            else:
                in_relevant_zone = abs(angle) >= (math.pi - rear_range_rad)
            
            if in_relevant_zone:
                ttc = self._calculate_ttc_for_measurement(range_val, angle, cmd_linear_x)
                if ttc < self.min_ttc: self.min_ttc = ttc
                if range_val < self.min_distance: self.min_distance = range_val

        # Array de TTC para visualización/debug
        directional_ttc_array = self._compute_directional_ttc_array(scan_msg, cmd_linear_x)
        if cmd_linear_x > 0.0: directional_ttc_array.append(0.0)
        elif cmd_linear_x < 0.0: directional_ttc_array.append(1.0)
        
        ttc_msg = Float32MultiArray()
        ttc_msg.data = directional_ttc_array
        self.ttc_array_pub.publish(ttc_msg)

        # Guardar para el timer del log
        self.log_total_ttc_values = len(ttc_msg.data)
        self.log_finite_ttc_count = sum(1 for v in ttc_msg.data if math.isfinite(v))

        # Lógica de frenado
        was_braking = self.should_brake
        self.should_brake = ((self.min_ttc < self.ttc_threshold and self.min_ttc != float('inf')) or 
                             (self.min_distance < self.min_distance_threshold))
        
        brake_msg = Bool()
        brake_msg.data = self.should_brake
        self.brake_pub.publish(brake_msg)

        # Advertencias inmediatas (esto se queda fuera del timer porque son alertas críticas)
        if self.should_brake != was_braking:
            if self.should_brake:
                self.get_logger().warn(f"!!! FRENO DE EMERGENCIA !!! TTC: {self.min_ttc:.2f}s")
            else:
                self.get_logger().info("Camino despejado.")
        
        if self.should_brake:
            safety_cmd = TwistStamped()
            safety_cmd.header.stamp = self.get_clock().now().to_msg()
            safety_cmd.twist.linear.x = 0.0
            safety_cmd.twist.angular.z = self.current_cmd_vel.twist.angular.z
            self.cmd_ttc_publisher.publish(safety_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = break_ttc_directional()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()