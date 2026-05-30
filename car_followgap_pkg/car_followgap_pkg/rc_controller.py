import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool


# ---------------------------------------------------------------------------
# Steering model  (signal ↔ physical angle in degrees)
# ---------------------------------------------------------------------------
STEERING_MODEL_A = 10.422533
STEERING_MODEL_B = 1.026880

SIGNAL_LIMIT     = 3.0
ANGLE_LIMIT_DEG  = 27.0


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def angle_deg_to_signal(target_deg: float) -> float:
    target_deg = _clamp(target_deg, -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG)
    if target_deg == 0.0:
        return 0.0
    raw = math.copysign(
        (abs(target_deg) / STEERING_MODEL_A) ** (1.0 / STEERING_MODEL_B),
        target_deg,
    )
    return _clamp(raw, -SIGNAL_LIMIT, SIGNAL_LIMIT)


# ---------------------------------------------------------------------------
# Velocity multiplier table  { alert_code: multiplier }
# ---------------------------------------------------------------------------
_VEL_MULTIPLIER: dict[int, float] = {
    0:  1.0,
    1:  1.0,
    2:  1.0,
    3:  1.0,
    4:  1.0,
    5:  1.0,
    6:  1.0,
    7:  1.0,
    8:  1.0,
    9:  1.0,
    10: 1.0,
    11: 1.0,
    12: 1.0,
    13: 1.0,
}


# ---------------------------------------------------------------------------
# Controller node
# ---------------------------------------------------------------------------
class rc_controller(Node):

    def __init__(self):
        super().__init__('rc_controller')

        # ---- Parameters ----
        self.declare_parameter('forward_velocity', 2.2)
        self.declare_parameter('start_flag',       False)
        self.declare_parameter('pub_logger',       True)

        self.forward_vel = float(self.get_parameter('forward_velocity').value)
        self.start_flag  = bool(self.get_parameter('start_flag').value)
        self.pub_logger  = bool(self.get_parameter('pub_logger').value)

        # ---- Brake state ----
        self.brake_active = False

        # ---- Filter / rate-limiter state  (all in DEGREES) ----
        self.alpha          = 0.8    # low-pass weight
        self.max_delta_deg  = 7.0    # max steering change per step (degrees)
        self.angle_filtered = 0.0    # filter memory
        self.prev_angle_deg = 0.0    # previous rate-limited output

        # ---- Logger snapshot (updated each steering callback) ----
        self.log_alert        = 0
        self.log_angle_deg    = 0.0
        self.log_filtered_deg = 0.0
        self.log_final_deg    = 0.0
        self.log_hw_signal    = 0.0

        # ---- Current command ----
        self.current_cmd = TwistStamped()
        self.current_cmd.twist.linear.x  = 0.0
        self.current_cmd.twist.angular.z = 0.0

        # ---- Subscribers ----
        self.create_subscription(Bool,  '/start',        self._start_cb, 10)
        self.create_subscription(Twist, '/cmd_ang_tcc',  self._error_cb, 10)
        self.create_subscription(Bool,  '/brake_active', self._brake_cb, 10)

        # ---- Publisher ----
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_ttc_gap', 10)

        # ---- Timers ----
        self.create_timer(0.1, self._heartbeat)
        self.create_timer(2.0, self._log_timer_cb)

        self.get_logger().info('rc_controller ready.')

    # ------------------------------------------------------------------
    # Brake callback
    # ------------------------------------------------------------------
    def _brake_cb(self, msg: Bool):
        prev = self.brake_active
        self.brake_active = msg.data

        if self.brake_active and not prev:
            self.get_logger().warn("EMERGENCY BRAKE ACTIVATED")
            self.current_cmd.twist.linear.x = 0.0
            self.cmd_pub.publish(self.current_cmd)

        elif not self.brake_active and prev:
            self.get_logger().info("Emergency brake released.")

    # ------------------------------------------------------------------
    # Heartbeat — keeps downstream alive; enforces brake at 10 Hz
    # ------------------------------------------------------------------
    def _heartbeat(self):
        if self.brake_active:
            self.current_cmd.twist.linear.x = 0.0
        self.cmd_pub.publish(self.current_cmd)

    # ------------------------------------------------------------------
    # Periodic logger
    # ------------------------------------------------------------------
    def _log_timer_cb(self):
        if not self.pub_logger:
            return
        state = "BRAKE" if self.brake_active else "RUN  "
        self.get_logger().info(
            f"[{state}] alert={self.log_alert:2d} | "
            f"req={self.log_angle_deg:+6.2f}° | "
            f"filt={self.log_filtered_deg:+6.2f}° | "
            f"out={self.log_final_deg:+6.2f}° | "
            f"signal={self.log_hw_signal:+5.3f} | "
            f"vel={self.current_cmd.twist.linear.x:.2f} m/s"
        )

    # ------------------------------------------------------------------
    # Start signal
    # ------------------------------------------------------------------
    def _start_cb(self, msg: Bool):
        if msg.data and not self.start_flag:
            self.get_logger().info("Start signal received — enabling control.")
            self.start_flag = True
            if not self.brake_active:
                self.current_cmd.twist.linear.x = self.forward_vel
                self.cmd_pub.publish(self.current_cmd)

    # ------------------------------------------------------------------
    # Main steering callback
    # ------------------------------------------------------------------
    def _error_cb(self, msg: Twist):
        if not self.start_flag:
            return

        # 1. Convert incoming radian angle → degrees and clamp
        angle_deg = _clamp(
            math.degrees(float(msg.angular.z)),
            -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG
        )
        alert = int(round(float(msg.angular.y)))

        # 2. Low-pass filter
        self.angle_filtered = (
            self.alpha * angle_deg
            + (1.0 - self.alpha) * self.angle_filtered
        )
        filtered_deg = self.angle_filtered

        # 3. Rate limiter — cap how fast the wheel turns per step
        delta        = _clamp(
            filtered_deg - self.prev_angle_deg,
            -self.max_delta_deg, self.max_delta_deg
        )
        filtered_deg        = self.prev_angle_deg + delta
        self.prev_angle_deg = filtered_deg

        # 4. Final clamp + hardware conversion
        final_deg = _clamp(filtered_deg, -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG)
        hw_signal = angle_deg_to_signal(final_deg)

        # 5. Velocity
        velocity = 0.0 if self.brake_active else _VEL_MULTIPLIER.get(alert, 1.0) * self.forward_vel

        # 6. Update and publish command
        self.current_cmd.header.stamp    = self.get_clock().now().to_msg()
        self.current_cmd.twist.linear.x  = velocity
        self.current_cmd.twist.angular.z = hw_signal
        self.cmd_pub.publish(self.current_cmd)

        # 7. Save snapshot for periodic logger
        self.log_alert        = alert
        self.log_angle_deg    = angle_deg
        self.log_filtered_deg = filtered_deg
        self.log_final_deg    = final_deg
        self.log_hw_signal    = hw_signal


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = rc_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()