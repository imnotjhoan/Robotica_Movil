import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# Constants (tune here, not buried in the callback)
# ---------------------------------------------------------------------------
CLEARANCE_ANGLE_DEG = 34.0
FRONT_IDX_LO        = 533
FRONT_IDX_HI        = 548
LAT_RIGHT_IDX       = 270
LAT_LEFT_IDX        = 810

# Corridor-mode thresholds
MIN_FRONT_CLEAR    = 3.0
MIN_LAT_CLEAR      = 0.60
MIN_DIAG_CLEAR     = 1.00
CORRIDOR_WIDTH_TOL = 0.18
CENTERING_DEAD_BAND = 0.15
ANGLE_DEAD_BAND    = 0.15
CENTERING_CMD      = 0.05

# Safety thresholds
CURB_WARN  = 0.25
DIAG_SOFT  = 0.45
DIAG_HARD  = 0.35
EXT_SOFT   = 0.45
EXT_HARD   = 0.30

# Safety bubble
SAFETY_BUBBLE_M = 0.30  # m — radio de burbuja alrededor de cada obstáculo


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class rc_gap_logger(Node):

    def __init__(self):
        super().__init__('rc_gap_logger')
        self.declare_parameter('pub_logger', True)
        self.pub_logger = bool(self.get_parameter('pub_logger').value)

        self.last_scan: LaserScan | None = None
        self.tot_suma = 1.5

        self._scan_cached   = False
        self._i_start       = 0
        self._i_end         = 0
        self._forward_idx   = None
        self._backward_idx  = None
        self._clearance_idx = []

        self.create_subscription(LaserScan,         '/scan',       self._scan_cb, 10)
        self.create_subscription(Float32MultiArray, '/ttc_values', self._ttc_cb,  10)

        self.marker_pub  = self.create_publisher(Marker, '/gap_marker',  10)
        self.ang_err_pub = self.create_publisher(Twist,  '/cmd_ang_tcc', 10)

        self.get_logger().info("rc_gap_logger ready.")

    # ------------------------------------------------------------------
    # Scan geometry cache — computed once on first scan message
    # ------------------------------------------------------------------
    def _cache_scan_geometry(self, msg: LaserScan):
        N         = len(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        i_start = math.ceil((-math.pi / 2 - angle_min) / angle_inc)
        i_end   = math.floor(( math.pi / 2 - angle_min) / angle_inc)
        i_start = max(0, min(N - 1, i_start))
        i_end   = max(0, min(N - 1, i_end))

        self._i_start      = i_start
        self._i_end        = i_end
        self._forward_idx  = np.arange(i_start, i_end + 1, dtype=np.int32)
        self._backward_idx = np.concatenate([
            np.arange(i_end + 1, N, dtype=np.int32),
            np.arange(0, i_start,  dtype=np.int32),
        ])

        offset = int(CLEARANCE_ANGLE_DEG * 3)
        self._clearance_idx = [
            offset          % 1080,
            (540 - offset)  % 1080,
            (540 + offset)  % 1080,
            (1080 - offset) % 1080,
        ]
        self._scan_cached = True

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg
        if not self._scan_cached:
            self._cache_scan_geometry(msg)

    def _ttc_cb(self, msg: Float32MultiArray):
        if self.last_scan is None or not self._scan_cached:
            return

        data = msg.data
        if len(data) < 1081:
            return

        ttc_arr        = np.asarray(data[:1080], dtype=np.float32)
        direction_flag = data[1080]

        # ---- Safety bubble: inflate every obstacle before gap search ----
        ttc_arr = self._apply_safety_bubble(ttc_arr)

        # ---- Clearance measurements (on raw ranges, not ttc) ----
        ranges = self.last_scan.ranges
        clr = [ranges[i] for i in self._clearance_idx]
        clr.append(min(ranges[FRONT_IDX_LO:FRONT_IDX_HI]))
        clr.append(ranges[LAT_RIGHT_IDX])
        clr.append(ranges[LAT_LEFT_IDX])

        # ---- Direction indices ----
        indices = self._forward_idx if abs(direction_flag) < 1e-6 else self._backward_idx

        # ---- Largest inf-gap ----
        angle_rad = self._largest_gap_angle(ttc_arr, indices)
        if angle_rad is None:
            return

        self._publish_gap_marker(angle_rad)

        angle_rad, mark_alert = self._compute_steering(angle_rad, clr)

        self.get_logger().info(
            f"fr: {clr[4]:.2f} | dr: {clr[5]:.2f} | iz: {clr[6]:.2f} | alert: {mark_alert:.0f}"
        )

        cmd = Twist()
        cmd.angular.z = angle_rad
        cmd.angular.y = mark_alert
        self.ang_err_pub.publish(cmd)

    # ------------------------------------------------------------------
    # Safety bubble
    # ------------------------------------------------------------------
    def _apply_safety_bubble(self, ttc_arr: np.ndarray) -> np.ndarray:
        """
        Infla cada obstáculo detectado por SAFETY_BUBBLE_M metros.

        Para cada rayo i con ttc finito (obstáculo real a distancia d_i):
          θ_bubble = arcsin(r_bubble / d_i)
          n_rayos  = ceil(θ_bubble / angle_increment)
          → ttc[i-n .. i+n] = 0.0   (bloquea rayos vecinos)

        Los gaps de inf se encogen en sus bordes, forzando al robot
        a apuntar al centro del espacio libre y no rozar las paredes.
        """
        result    = ttc_arr.copy()
        N         = len(ttc_arr)
        angle_inc = self.last_scan.angle_increment
        ranges    = self.last_scan.ranges

        obstacle_indices = np.where(np.isfinite(ttc_arr))[0]

        for i in obstacle_indices:
            d = float(ranges[i])

            if d <= SAFETY_BUBBLE_M:
                # Obstáculo dentro de la burbuja → bloquea todo el array
                result[:] = 0.0
                return result

            theta_bubble = math.asin(min(SAFETY_BUBBLE_M / d, 1.0))
            n_rays = math.ceil(theta_bubble / angle_inc)

            lo = max(0, i - n_rays)
            hi = min(N - 1, i + n_rays)
            result[lo : hi + 1] = 0.0

        return result

    # ------------------------------------------------------------------
    # Gap detection — numpy vectorised
    # ------------------------------------------------------------------
    def _largest_gap_angle(self, ttc_arr: np.ndarray, indices: np.ndarray):
        """Return center angle (rad) of the widest inf-gap, or None."""
        is_inf = np.isinf(ttc_arr[indices])

        if not is_inf.any():
            return None

        padded   = np.concatenate([[False], is_inf, [False]])
        starts_k = np.where(~padded[:-1] &  padded[1:])[0]
        ends_k   = np.where( padded[:-1] & ~padded[1:])[0]

        lengths  = ends_k - starts_k
        best     = np.argmax(lengths)
        center_k = (starts_k[best] + ends_k[best] - 1) // 2

        center_scan_idx = int(indices[center_k])
        angle_rad = (self.last_scan.angle_min
                     + center_scan_idx * self.last_scan.angle_increment)

        return (angle_rad + math.pi) % (2 * math.pi) - math.pi

    # ------------------------------------------------------------------
    # Steering correction hierarchy
    # ------------------------------------------------------------------
    def _compute_steering(self, angle_rad: float, clr: list) -> tuple[float, float]:
        """
        Layered safety corrections.
        clr: [0] diag-front-right  [1] diag-front-left
             [2] diag-rear-right   [3] diag-rear-left
             [4] front-min  [5] lateral-right  [6] lateral-left
        """
        mark_alert = 0.0

        # Corridor / straight-line mode
        if (clr[4] > MIN_FRONT_CLEAR
                and clr[5] > MIN_LAT_CLEAR
                and clr[6] > MIN_LAT_CLEAR
                and clr[2] > MIN_DIAG_CLEAR
                and clr[1] > MIN_DIAG_CLEAR):

            if abs((clr[5] + clr[6]) - self.tot_suma) < CORRIDOR_WIDTH_TOL:
                if clr[5] > clr[6] + CENTERING_DEAD_BAND:
                    angle_rad, mark_alert = -CENTERING_CMD, 1.0
                elif clr[6] > clr[5] + CENTERING_DEAD_BAND:
                    angle_rad, mark_alert =  CENTERING_CMD, 2.0
                else:
                    angle_rad  = 0.0 if abs(angle_rad) < ANGLE_DEAD_BAND else 0.5 * angle_rad
                    mark_alert = 2.0

            self.tot_suma = clr[5] + clr[6]
            return angle_rad, mark_alert

        # Anti-curb
        if angle_rad > 0.0 and clr[6] < CURB_WARN:
            return -0.3 * angle_rad, 4.0
        if angle_rad < 0.0 and clr[5] < CURB_WARN:
            return -0.3 * angle_rad, 5.0

        # Diagonal guard — inner quadrant
        if angle_rad > 0.0 and clr[2] < DIAG_SOFT:
            return (-1.0 if clr[2] < DIAG_HARD else -abs(angle_rad)), (6.0 if clr[2] < DIAG_HARD else 7.0)
        if angle_rad < 0.0 and clr[1] < DIAG_SOFT:
            return ( 1.0 if clr[1] < DIAG_HARD else  abs(angle_rad)), (8.0 if clr[1] < DIAG_HARD else 9.0)

        # Outer-radius guard
        if angle_rad < 0.0 and clr[2] < EXT_SOFT:
            return (-1.0 if clr[2] < EXT_HARD else -abs(angle_rad)), (10.0 if clr[2] < EXT_HARD else 11.0)
        if angle_rad > 0.0 and clr[1] < EXT_SOFT:
            return ( 1.0 if clr[1] < EXT_HARD else  abs(angle_rad)), (12.0 if clr[1] < EXT_HARD else 13.0)

        return angle_rad, mark_alert

    # ------------------------------------------------------------------
    # Marker publisher
    # ------------------------------------------------------------------
    def _publish_gap_marker(self, angle_rad: float):
        m = Marker()
        m.header.frame_id = self.last_scan.header.frame_id
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns     = "gap_direction"
        m.id     = 0
        m.type   = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = math.sin(angle_rad / 2.0)
        m.pose.orientation.w = math.cos(angle_rad / 2.0)
        m.scale.x = 1.0
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color.a = 1.0
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = rc_gap_logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()