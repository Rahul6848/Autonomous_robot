"""
mission_controller.py
─────────────────────
ROS 2 node that:
  • Sits idle at Point A until a terminal command arrives on /robot_command
  • On "go check B" → drives the robot from A to B using both LiDAR-based
    obstacle avoidance AND camera lane-following simultaneously
  • On "stop" / "return" → stops the robot or brings it back to A
  • Publishes mission status on /mission_status

Run the commander separately:
    ros2 run my_robot_controller mission_commander
"""

import math
import time
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


# ──────────────────────────────────────────────────────────────────────────────
# World geometry – match road_world.world spawn / turning point
# ──────────────────────────────────────────────────────────────────────────────
POINT_A = (-8.0, 0.75)   # spawn position
POINT_B = (4.5,  1.5)    # turning / destination

WAYPOINTS_A_TO_B: List[Tuple[float, float]] = [
    (-8.0, 0.75),
    (-5.0, 0.75),
    (-2.0, 0.75),
    ( 1.0, 0.75),
    ( 3.2, 0.75),
    ( 4.1, 0.90),
    ( 4.5, 1.35),
]

WAYPOINTS_B_TO_A: List[Tuple[float, float]] = [
    ( 4.5, 1.35),
    ( 4.1, 0.90),
    ( 3.2, 0.75),
    ( 1.0, 0.75),
    (-2.0, 0.75),
    (-5.0, 0.75),
    (-8.0, 0.75),
]

GOAL_TOLERANCE    = 0.35   # m – close enough to a waypoint
LOOKAHEAD         = 0.75   # m – pure-pursuit lookahead

# Motion limits
MAX_LINEAR        = 0.24   # m/s
SLOW_LINEAR       = 0.12
MAX_ANGULAR       = 1.0    # rad/s

# LiDAR safety
FRONT_SAFE_DIST   = 1.20   # m
CRIT_FRONT_DIST   = 0.45
SAFE_SIDE_DIST    = 0.60
CRIT_SIDE_DIST    = 0.32
FRONT_HALF_DEG    = 30.0
SIDE_START_DEG    = 55.0
SIDE_END_DEG      = 110.0
SELF_FILTER       = 0.22
SCAN_MAX          = 5.0
MIN_PTS           = 6

# Camera lane-follow blend weight (0 = pure LiDAR, 1 = pure camera)
CAMERA_WEIGHT     = 0.35


class MissionState(Enum):
    IDLE       = auto()
    GOING_TO_B = auto()
    AT_B       = auto()
    RETURNING  = auto()
    STOPPED    = auto()


# ──────────────────────────────────────────────────────────────────────────────
class MissionController(Node):
    def __init__(self) -> None:
        super().__init__('mission_controller')

        # ── publishers ────────────────────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)

        # ── subscribers ───────────────────────────────────────────────────────
        self.create_subscription(String,    '/robot_command', self._cmd_cb,  10)
        self.create_subscription(Odometry,  '/odom',          self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan',          self._scan_cb, 10)
        if CV_AVAILABLE:
            self.create_subscription(Image, '/camera/image_raw', self._img_cb, 10)

        # ── control timer (10 Hz) ─────────────────────────────────────────────
        self.create_timer(0.10, self._control_loop)

        # ── state ─────────────────────────────────────────────────────────────
        self.state   : MissionState          = MissionState.IDLE
        self.waypoints: List[Tuple[float,float]] = []
        self.wp_idx  : int                   = 0

        self.robot_x : Optional[float] = None
        self.robot_y : Optional[float] = None
        self.robot_yaw: float          = 0.0

        self.latest_scan : Optional[LaserScan] = None
        self.latest_frame: Optional[object]    = None   # numpy array

        self.bridge = CvBridge() if CV_AVAILABLE else None

        # camera-derived angular correction (updated each frame)
        self.cam_angular: float = 0.0

        self._prev_state = None
        self._paused_state: Optional[MissionState] = None
        self.get_logger().info('Mission Controller ready. Waiting for command on /robot_command')
        self._publish_status('IDLE – send "go check B" to start')

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────
    def _cmd_cb(self, msg: String) -> None:
        text = msg.data.strip().lower()
        self.get_logger().info(f'[CMD] received: "{msg.data}"')

        if any(k in text for k in ('start', 'resume', 'continue')):
            self._resume_mission()
            return

        if any(k in text for k in ('go check b', 'go to b', 'move to b', 'go')):
            if self.state in (MissionState.IDLE, MissionState.AT_B,
                              MissionState.STOPPED):
                self._start_mission(WAYPOINTS_A_TO_B, MissionState.GOING_TO_B,
                                    'GOING_TO_B')
        elif any(k in text for k in ('return', 'come back', 'go to a',
                                     'go home', 'back')):
            if self.state in (MissionState.AT_B, MissionState.STOPPED,
                              MissionState.GOING_TO_B):
                self._start_mission(WAYPOINTS_B_TO_A, MissionState.RETURNING,
                                    'RETURNING')
        elif 'stop' in text or 'halt' in text:
            if self.state in (MissionState.GOING_TO_B, MissionState.RETURNING):
                self._paused_state = self.state
            else:
                self._paused_state = None
            self._stop_robot()
            self.state = MissionState.STOPPED
            if self._paused_state is not None:
                self._publish_status('STOPPED – mission paused. Send "start" to resume.')
            else:
                self._publish_status('STOPPED – robot halted by command')

    def _odom_cb(self, msg: Odometry) -> None:
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _img_cb(self, msg: Image) -> None:
        if not CV_AVAILABLE or self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = frame
            self.cam_angular  = self._compute_lane_angular(frame)
        except Exception as e:
            self.get_logger().warning(f'Image conversion failed: {e}',
                                      throttle_duration_sec=2.0)

    # ──────────────────────────────────────────────────────────────────────────
    # Control loop
    # ──────────────────────────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        # Log state transitions
        if self.state != self._prev_state:
            self.get_logger().info(f'State → {self.state.name}')
            self._prev_state = self.state

        if self.state in (MissionState.IDLE, MissionState.STOPPED,
                          MissionState.AT_B):
            self._stop_robot()
            return

        if self.robot_x is None or self.latest_scan is None:
            return  # wait for sensor data

        # ── advance waypoint pointer ───────────────────────────────────────
        self._advance_waypoint()

        if self.wp_idx >= len(self.waypoints):
            self._on_mission_complete()
            return

        # ── compute blended command ────────────────────────────────────────
        cmd = self._compute_blended_command()
        self.cmd_pub.publish(cmd)

    # ──────────────────────────────────────────────────────────────────────────
    # Waypoint helpers
    # ──────────────────────────────────────────────────────────────────────────
    def _advance_waypoint(self) -> None:
        """Skip waypoints that are within lookahead distance."""
        while self.wp_idx < len(self.waypoints) - 1:
            tx, ty = self.waypoints[self.wp_idx]
            if math.hypot(tx - self.robot_x, ty - self.robot_y) > LOOKAHEAD:
                break
            self.wp_idx += 1

    def _on_mission_complete(self) -> None:
        if self.state == MissionState.GOING_TO_B:
            self.state = MissionState.AT_B
            self._stop_robot()
            self._publish_status('AT_B – reached destination B. Send "return" to go back.')
        elif self.state == MissionState.RETURNING:
            self.state = MissionState.IDLE
            self._stop_robot()
            self._publish_status('IDLE – returned to A. Ready for next command.')

    # ──────────────────────────────────────────────────────────────────────────
    # Blended motion command (LiDAR + camera + pure-pursuit)
    # ──────────────────────────────────────────────────────────────────────────
    def _compute_blended_command(self) -> Twist:
        cmd = Twist()

        # 1) Pure-pursuit heading to current waypoint
        tx, ty = self.waypoints[self.wp_idx]
        heading = math.atan2(ty - self.robot_y, tx - self.robot_x)
        err     = self._norm_angle(heading - self.robot_yaw)
        pursuit_angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, 1.8 * err))

        # 2) LiDAR obstacle correction
        lidar_angular = self._lidar_angular_correction()
        lidar_blocked = self._lidar_front_blocked()

        # 3) Camera lane-follow correction
        cam_angular = self.cam_angular  # updated asynchronously

        # 4) Blend: pursuit dominant, camera adds lane-centering, LiDAR safety
        if lidar_blocked:
            # Hard override – turn away from obstacle
            combined_angular = lidar_angular
            cmd.linear.x = 0.05
        else:
            pursuit_w = 0.55
            cam_w     = CAMERA_WEIGHT
            lidar_w   = 1.0 - pursuit_w - cam_w   # remaining (~0.10)
            combined_angular = (pursuit_w * pursuit_angular +
                                cam_w     * cam_angular      +
                                lidar_w   * lidar_angular)
            combined_angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, combined_angular))

            # Speed – slow down if turning hard or near goal
            dist_to_wp = math.hypot(tx - self.robot_x, ty - self.robot_y)
            turn_factor = max(0.3, 1.0 - abs(combined_angular) / MAX_ANGULAR)
            base_speed  = SLOW_LINEAR + (MAX_LINEAR - SLOW_LINEAR) * turn_factor
            cmd.linear.x = min(base_speed, max(0.08, dist_to_wp * 0.5))

        cmd.angular.z = combined_angular
        return cmd

    # ──────────────────────────────────────────────────────────────────────────
    # LiDAR helpers
    # ──────────────────────────────────────────────────────────────────────────
    def _get_sector(self, min_deg: float, max_deg: float) -> List[float]:
        if self.latest_scan is None:
            return []
        scan = self.latest_scan
        vals: List[float] = []
        for i, d in enumerate(scan.ranges):
            if math.isnan(d):
                continue
            ang_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            if not (min_deg <= ang_deg <= max_deg):
                continue
            if math.isinf(d) or d > scan.range_max:
                vals.append(SCAN_MAX)
                continue
            if d < max(scan.range_min, SELF_FILTER):
                continue
            vals.append(min(d, SCAN_MAX))
        return vals

    def _safe_min(self, vals: List[float]) -> float:
        return min(vals) if vals else float('inf')

    def _lidar_front_blocked(self) -> bool:
        front = self._get_sector(-FRONT_HALF_DEG, FRONT_HALF_DEG)
        return len(front) >= MIN_PTS and self._safe_min(front) < CRIT_FRONT_DIST

    def _lidar_angular_correction(self) -> float:
        left  = self._get_sector( SIDE_START_DEG,  SIDE_END_DEG)
        right = self._get_sector(-SIDE_END_DEG,   -SIDE_START_DEG)
        front = self._get_sector(-FRONT_HALF_DEG,  FRONT_HALF_DEG)

        lmin = self._safe_min(left)
        rmin = self._safe_min(right)
        fmin = self._safe_min(front)

        angular = 0.0
        # Push away from close walls
        if lmin < CRIT_SIDE_DIST:
            angular -= 1.2
        elif lmin < SAFE_SIDE_DIST:
            angular -= 0.7 * (SAFE_SIDE_DIST - lmin) / SAFE_SIDE_DIST

        if rmin < CRIT_SIDE_DIST:
            angular += 1.2
        elif rmin < SAFE_SIDE_DIST:
            angular += 0.7 * (SAFE_SIDE_DIST - rmin) / SAFE_SIDE_DIST

        # Front avoidance (prefer the more open side)
        if fmin < FRONT_SAFE_DIST:
            front_l = self._get_sector(10.0, 70.0)
            front_r = self._get_sector(-70.0, -10.0)
            if self._safe_min(front_l) >= self._safe_min(front_r):
                angular += 0.6 * (FRONT_SAFE_DIST - fmin) / FRONT_SAFE_DIST
            else:
                angular -= 0.6 * (FRONT_SAFE_DIST - fmin) / FRONT_SAFE_DIST

        return max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

    # ──────────────────────────────────────────────────────────────────────────
    # Camera lane-follow helpers
    # ──────────────────────────────────────────────────────────────────────────
    def _compute_lane_angular(self, frame: 'np.ndarray') -> float:  # type: ignore
        """Return angular correction in rad/s from camera lane detection."""
        if not CV_AVAILABLE:
            return 0.0
        try:
            h, w = frame.shape[:2]
            roi  = frame[int(h * 0.55):, :]

            hsv        = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lane_mask  = cv2.inRange(hsv, (0, 0, 150), (180, 80, 255))
            road_mask  = cv2.inRange(hsv, (0, 0, 20),  (180, 80, 110))
            kernel     = np.ones((5, 5), np.uint8)
            lane_mask  = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
            road_mask  = cv2.morphologyEx(road_mask, cv2.MORPH_OPEN, kernel)

            target_x = self._lane_center_x(lane_mask, road_mask, w)
            if target_x is None:
                return 0.0

            error = (target_x - w / 2.0) / (w / 2.0)   # -1 … +1
            return max(-MAX_ANGULAR, min(MAX_ANGULAR, -0.8 * error))
        except Exception:
            return 0.0

    def _lane_center_x(self, lane_mask, road_mask, w: int) -> Optional[float]:
        # Try lane markings first
        pts = cv2.findNonZero(lane_mask)
        if pts is not None:
            xs = pts[:, 0, 0]
            left_xs = xs[xs < w * 0.6]
            if len(left_xs) >= 40:
                return max(0.0, float(np.median(left_xs)) - w * 0.18)

        # Fall back to road contour
        contours, _ = cv2.findContours(road_mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        x, _, cw, _ = cv2.boundingRect(c)
        if cw < w * 0.25:
            return None
        return x + cw / 2.0 - cw * 0.22

    # ──────────────────────────────────────────────────────────────────────────
    # Utilities
    # ──────────────────────────────────────────────────────────────────────────
    def _start_mission(self, wps: List[Tuple[float, float]],
                       new_state: MissionState, label: str) -> None:
        self.waypoints = list(wps)
        self.wp_idx    = 0
        self.state     = new_state
        self._paused_state = None
        self._publish_status(f'{label} – moving…')

    def _resume_mission(self) -> None:
        if self.state != MissionState.STOPPED or self._paused_state is None:
            self._publish_status('IDLE – no paused mission is available to resume.')
            return

        self.state = self._paused_state
        resumed_state = self._paused_state
        self._paused_state = None
        self._publish_status(f'{resumed_state.name} – resuming from the stopped position.')

    def _stop_robot(self) -> None:
        self.cmd_pub.publish(Twist())

    def _publish_status(self, text: str) -> None:
        msg      = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f'[STATUS] {text}')

    @staticmethod
    def _norm_angle(a: float) -> float:
        while a >  math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
