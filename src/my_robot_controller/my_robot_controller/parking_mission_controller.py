import math
import time
import re
from enum import Enum, auto
from typing import Any, Dict, List, Optional, Tuple

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


IDLE_POSE = (0.0, 0.0, 0.0)
SCAN_SETTLE_SEC = 0.45
SCAN_SAMPLE_SEC = 1.20
DEBUG_WINDOW_NAME = 'Parking Vision Debug'
PARKING_ROI_TOP_RATIO = 0.25
MIN_SLOT_AREA = 280.0
SCAN_OCCUPIED_FRONT_DISTANCE = 1.75
OCCUPIED_SLOT_REJECT_DISTANCE = 0.95
SLOT_CLEARANCE_SECTOR_DEG = 12.0
PARKING_STOP_DISTANCE = 0.20
PARKING_STRAIGHTEN_Y_NORM = 0.70
PARKING_FINAL_Y_NORM = 0.82
PARKING_FINAL_HEIGHT_RATIO = 0.26
PARKING_FINAL_FRONT_DISTANCE = 0.34
POINT_ONLY_MODE = True
SEARCH_TIMEOUT_SEC = 2.0

PARKING_TARGETS: Dict[str, Dict[str, Any]] = {
    'a': {
        'label': 'A',
        'approach_waypoints': [
            (0.0, 0.0),
            (1.6, -0.8),
            (2.5, -1.55),
            (3.2, -2.2),
        ],
        'staging_pose': (3.2, -2.2, -math.pi / 2.0),
        'slot_centers': {
            'left': (4.2, -4.0),
            'right': (2.2, -4.0),
        },
    },
    'b': {
        'label': 'B',
        'approach_waypoints': [
            (0.0, 0.0),
            (-1.6, 0.8),
            (-2.5, 1.55),
            (-3.2, 2.2),
        ],
        'staging_pose': (-3.2, 2.2, math.pi / 2.0),
        'slot_centers': {
            'left': (-4.2, 4.0),
            'right': (-2.2, 4.0),
        },
    },
    'c': {
        'label': 'C',
        'approach_waypoints': [
            (0.0, 0.0),
            (1.6, 0.8),
            (2.5, 1.55),
            (3.2, 2.2),
        ],
        'staging_pose': (3.2, 2.2, math.pi / 2.0),
        'slot_centers': {
            'left': (2.2, 4.0),
            'right': (4.2, 4.0),
        },
    },
    'd': {
        'label': 'D',
        'approach_waypoints': [
            (0.0, 0.0),
            (-1.6, -0.8),
            (-2.5, -1.55),
            (-3.2, -2.2),
        ],
        'staging_pose': (-3.2, -2.2, -math.pi / 2.0),
        'slot_centers': {
            'left': (-2.2, -4.0),
            'right': (-4.2, -4.0),
        },
    },
}


class ParkingState(Enum):
    IDLE = auto()
    APPROACHING_TARGET = auto()
    SCAN_LEFT = auto()
    SCAN_RIGHT = auto()
    TURN_TO_SLOT = auto()
    VISION_PARK = auto()
    RETURNING_TO_IDLE = auto()
    PARKED = auto()
    STOPPED = auto()


class ParkingMissionController(Node):
    def __init__(self) -> None:
        super().__init__('parking_mission_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)

        self.create_subscription(String, '/robot_command', self._command_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)

        self.bridge = CvBridge() if CV_AVAILABLE else None
        if CV_AVAILABLE:
            self.create_subscription(Image, '/camera/image_raw', self._image_callback, 10)

        self.create_timer(0.1, self._control_loop)

        self.state = ParkingState.IDLE
        self._previous_state: Optional[ParkingState] = None
        self._paused_state: Optional[ParkingState] = None

        self.robot_x: Optional[float] = None
        self.robot_y: Optional[float] = None
        self.robot_yaw = 0.0
        self.latest_scan: Optional[LaserScan] = None
        self.latest_detection: Optional[Dict[str, Any]] = None
        self.last_detection_time = 0.0

        self.active_target_key: Optional[str] = None
        self.active_target: Optional[Dict[str, Any]] = None
        self.waypoint_index = 0
        self.target_queue: List[str] = []

        self.scan_center_yaw: Optional[float] = None
        self.scan_started_at = 0.0
        self.scan_best_detection: Optional[Dict[str, Any]] = None
        self.scan_results: Dict[str, Optional[Dict[str, Any]]] = {
            'left': None,
            'right': None,
        }
        self.scan_front_distances: Dict[str, float] = {
            'left': float('inf'),
            'right': float('inf'),
        }
        self.scan_best_front_distance = float('inf')
        self.scan_clearance_samples: Dict[str, List[float]] = {
            'left': [],
            'right': [],
        }
        self.blocked_sides: set[str] = set()
        self.selected_side: Optional[str] = None
        self.selected_slot_yaw: Optional[float] = None
        self.vision_started_at = 0.0
        self.search_state_started_at = 0.0
        self.queued_command_text: Optional[str] = None

        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.debug_window_enabled = CV_AVAILABLE
        self._debug_window_failed = False

        if CV_AVAILABLE:
            self.get_logger().info(
                'Parking mission controller ready. Vision-based parking debug window is enabled.'
            )
        else:
            self.get_logger().warning(
                'OpenCV/cv_bridge import failed. Vision-based parking cannot start.'
            )

        self._publish_status(
            'IDLE - commands: "park at A/B/C/D", multi-point routes, "stop", "return", or "idle". Point-only mode is active.'
        )

    def _command_callback(self, msg: String) -> None:
        text = msg.data.strip().lower()
        self.get_logger().info(f'[CMD] received: "{msg.data}"')

        if any(keyword in text for keyword in ('start', 'resume', 'continue')):
            self._resume_paused_mission()
            return

        if 'stop' in text or 'halt' in text:
            self.queued_command_text = None
            if self.state in (
                ParkingState.APPROACHING_TARGET,
                ParkingState.SCAN_LEFT,
                ParkingState.SCAN_RIGHT,
                ParkingState.TURN_TO_SLOT,
                ParkingState.VISION_PARK,
                ParkingState.RETURNING_TO_IDLE,
            ):
                self._paused_state = self.state
            else:
                self._paused_state = None
            self._stop_robot()
            self.state = ParkingState.STOPPED
            if self._paused_state is not None:
                self._publish_status('STOPPED - mission paused. Send "start" to resume.')
            else:
                self._publish_status('STOPPED - robot halted by command.')
            return

        if any(keyword in text for keyword in ('return', 'idle', 'go back', 'home')):
            if self._is_search_state(self.state):
                self.queued_command_text = 'return'
                self._publish_status(
                    f'QUEUED - will return to idle if parking search lasts more than {SEARCH_TIMEOUT_SEC:.1f} s.'
                )
                return
            self._start_return_to_idle()
            return

        route_targets = self._extract_route_targets(text)
        if not route_targets:
            return

        if self._is_search_state(self.state):
            self.queued_command_text = self._format_route_command(route_targets)
            self._publish_status(
                f'QUEUED - will switch to route {self._format_route_labels(route_targets)} if parking search lasts more than {SEARCH_TIMEOUT_SEC:.1f} s.'
            )
            return

        self._start_parking_route(route_targets)

    def _start_parking_route(self, route_targets: List[str]) -> None:
        if not route_targets:
            self._publish_status('IDLE - no parking route was provided.')
            return

        self.target_queue = list(route_targets)
        self._start_next_target_in_queue()

    def _start_next_target_in_queue(self) -> None:
        if not self.target_queue:
            self.active_target_key = None
            self.active_target = None
            self.state = ParkingState.PARKED
            self._publish_status('PARKED - route queue is complete and the robot is stopped.')
            return

        next_target_key = self.target_queue.pop(0)
        self._start_parking_mission(next_target_key)

    def _start_parking_mission(self, target_key: str) -> None:
        self.active_target_key = target_key
        self.active_target = PARKING_TARGETS[target_key]
        self.waypoint_index = self._select_retarget_waypoint_index(self.active_target)
        self.scan_center_yaw = None
        self.scan_started_at = 0.0
        self.scan_best_detection = None
        self.scan_results = {'left': None, 'right': None}
        self.scan_front_distances = {'left': float('inf'), 'right': float('inf')}
        self.scan_best_front_distance = float('inf')
        self.scan_clearance_samples = {'left': [], 'right': []}
        self.blocked_sides = set()
        self.selected_side = None
        self.selected_slot_yaw = None
        self.vision_started_at = 0.0
        self._paused_state = None
        self.queued_command_text = None
        self.state = ParkingState.APPROACHING_TARGET
        waypoint_count = len(self.active_target['approach_waypoints'])
        if self.waypoint_index < waypoint_count:
            next_waypoint = self.waypoint_index + 1
            route_hint = f' via approach waypoint {next_waypoint}/{waypoint_count}'
        else:
            route_hint = ' via the staging pose'
        self._publish_status(
            f'APPROACHING_{self.active_target["label"]} - driving to Point {self.active_target["label"]}{route_hint}.'
        )

    def _start_return_to_idle(self) -> None:
        self.target_queue = []
        self.scan_started_at = 0.0
        self.scan_best_detection = None
        self.scan_results = {'left': None, 'right': None}
        self.scan_front_distances = {'left': float('inf'), 'right': float('inf')}
        self.scan_best_front_distance = float('inf')
        self.scan_clearance_samples = {'left': [], 'right': []}
        self.blocked_sides = set()
        self.selected_side = None
        self.selected_slot_yaw = None
        self._paused_state = None
        self.queued_command_text = None
        self.state = ParkingState.RETURNING_TO_IDLE
        self._publish_status('RETURNING_TO_IDLE - driving back to the spawn pose.')

    def _resume_paused_mission(self) -> None:
        if self.state != ParkingState.STOPPED or self._paused_state is None:
            self._publish_status('IDLE - no paused parking mission is available to resume.')
            return

        self.state = self._paused_state
        resumed_state = self._paused_state
        self._paused_state = None
        self._publish_status(
            f'{resumed_state.name} - resuming from the stopped position.'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _image_callback(self, msg: Image) -> None:
        if not CV_AVAILABLE or self.bridge is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to convert image frame: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        detection = self._detect_yellow_slot(frame)
        self.latest_detection = detection
        if detection is not None:
            self.last_detection_time = time.time()

        self._render_debug_view(frame, detection)

    def _control_loop(self) -> None:
        if self.state != self._previous_state:
            self.get_logger().info(f'State -> {self.state.name}')
            if self._is_search_state(self.state):
                self.search_state_started_at = time.time()
            else:
                self.search_state_started_at = 0.0
            self._previous_state = self.state

        if self.state in (ParkingState.IDLE, ParkingState.STOPPED, ParkingState.PARKED):
            self._stop_robot()
            return

        if self._is_search_state(self.state):
            if self._search_timeout_exceeded():
                self._abort_search_and_process_queue()
                return

        if self.robot_x is None or self.robot_y is None:
            return

        if self.state == ParkingState.APPROACHING_TARGET:
            self._run_approach_to_target()
            return

        if self.state == ParkingState.SCAN_LEFT:
            self._run_scan('left')
            return

        if self.state == ParkingState.SCAN_RIGHT:
            self._run_scan('right')
            return

        if self.state == ParkingState.TURN_TO_SLOT:
            self._run_turn_to_selected_slot()
            return

        if self.state == ParkingState.VISION_PARK:
            self._run_vision_park()
            return

        if self.state == ParkingState.RETURNING_TO_IDLE:
            self._run_return_to_idle()
            return

    def _run_approach_to_target(self) -> None:
        if self.active_target is None:
            self.state = ParkingState.STOPPED
            self._publish_status('STOPPED - no active parking target is set.')
            return

        waypoints: List[Tuple[float, float]] = self.active_target['approach_waypoints']
        if self.waypoint_index < len(waypoints):
            target_x, target_y = waypoints[self.waypoint_index]
            reached = self._drive_to_pose(target_x, target_y, None, goal_tolerance=0.22)
            if reached:
                self.waypoint_index += 1
                if self.waypoint_index >= len(waypoints):
                    self._finish_at_target()
            return

        self._finish_at_target()

    def _run_scan(self, side: str) -> None:
        if self._search_timeout_exceeded():
            self._abort_search_and_process_queue()
            return

        if self.active_target is None:
            self.state = ParkingState.STOPPED
            self._publish_status(
                f'STOPPED - active target is unavailable at Point {self.active_target["label"] if self.active_target else "?"}.'
            )
            return

        now = time.time()
        self._stop_robot()
        if self.scan_started_at == 0.0:
            self.scan_started_at = now
            self.scan_best_detection = None
            self.scan_best_front_distance = float('inf')
            self.scan_clearance_samples[side] = []
            return

        sample_start = self.scan_started_at + SCAN_SETTLE_SEC
        sample_end = sample_start + SCAN_SAMPLE_SEC
        if sample_start <= now <= sample_end:
            clearance = self._slot_clearance(side)
            if math.isfinite(clearance):
                self.scan_clearance_samples[side].append(clearance)
                self.scan_best_front_distance = min(
                    self.scan_best_front_distance,
                    clearance,
                )
            if self._detection_is_fresh():
                candidate = self._clone_detection(self.latest_detection)
                if candidate is not None and (
                    self.scan_best_detection is None
                    or candidate['score'] > self.scan_best_detection['score']
                ):
                    self.scan_best_detection = candidate

        if now < sample_end:
            return

        self.scan_results[side] = self._clone_detection(self.scan_best_detection)
        representative_clearance = self._representative_clearance(
            self.scan_clearance_samples[side]
        )
        self.scan_front_distances[side] = representative_clearance
        best_score = 0.0 if self.scan_best_detection is None else self.scan_best_detection['score']
        occupied = representative_clearance <= SCAN_OCCUPIED_FRONT_DISTANCE
        if occupied:
            self.blocked_sides.add(side)
        self.get_logger().info(
            f'[SCAN] {side} best score: {best_score:.1f}, '
            f'slot clearance: {representative_clearance:.2f} m, '
            f'occupied={occupied}'
        )

        self.scan_started_at = 0.0
        self.scan_best_detection = None
        self.scan_best_front_distance = float('inf')
        self.scan_clearance_samples[side] = []

        if side == 'left':
            self.state = ParkingState.SCAN_RIGHT
            self._publish_status(
                'SCAN_RIGHT - left scan complete. Holding position and measuring right-slot clearance.'
            )
            return

        self._select_scanned_slot()

    def _select_scanned_slot(self) -> None:
        if self.active_target is None:
            self.state = ParkingState.STOPPED
            self._publish_status('STOPPED - no active parking target is set.')
            return

        candidates = [
            (side, self.scan_front_distances.get(side, float('inf')))
            for side in ('left', 'right')
            if math.isfinite(self.scan_front_distances.get(side, float('inf')))
        ]
        if not candidates:
            self.state = ParkingState.STOPPED
            self._publish_status(
                f'STOPPED - no usable LiDAR clearance was measured for either slot at Point {self.active_target["label"]}.'
            )
            return

        free_candidates = [
            (side, clearance)
            for side, clearance in candidates
            if clearance > SCAN_OCCUPIED_FRONT_DISTANCE and side not in self.blocked_sides
        ]
        if free_candidates:
            self.selected_side, best_clearance = max(
                free_candidates,
                key=lambda item: item[1],
            )
            status_reason = (
                f'selected the free {self.selected_side} slot at Point '
                f'{self.active_target["label"]} (clearance {best_clearance:.2f} m)'
            )
        else:
            self.selected_side, best_clearance = max(
                candidates,
                key=lambda item: item[1],
            )
            status_reason = (
                f'no clearly free slot was found at Point {self.active_target["label"]}; '
                f'falling back to the {self.selected_side} side with the largest clearance '
                f'({best_clearance:.2f} m)'
            )

        self.selected_slot_yaw = self._yaw_to_side(self.selected_side)
        self.state = ParkingState.TURN_TO_SLOT
        self._publish_status(
            f'TURN_TO_SLOT - {status_reason}. Rotating to align for vision parking.'
        )

    def _run_turn_to_selected_slot(self) -> None:
        if self._search_timeout_exceeded():
            self._abort_search_and_process_queue()
            return

        if self.selected_slot_yaw is None:
            self.state = ParkingState.STOPPED
            self._publish_status('STOPPED - no selected slot yaw is available.')
            return

        if self._rotate_to_yaw(self.selected_slot_yaw, tolerance=0.05):
            self.vision_started_at = time.time()
            self.state = ParkingState.VISION_PARK
            self._publish_status(
                'VISION_PARK - rectangle locked. Steering into the parking area from camera measurements.'
            )

    def _run_vision_park(self) -> None:
        if self._search_timeout_exceeded():
            self._abort_search_and_process_queue()
            return

        detection = self.latest_detection
        center_y_norm = 0.0
        height_ratio = 0.0
        center_error = 0.0
        rect_angle_deg = 0.0
        image_width = 1.0
        image_height = 1.0
        area_ratio = 0.0
        if detection is not None:
            image_width = detection['image_width']
            image_height = detection['image_height']
            center_error = (detection['center_x'] - (image_width / 2.0)) / (image_width / 2.0)
            center_y_norm = detection['center_y'] / image_height
            height_ratio = detection['height'] / image_height
            area_ratio = detection['area_ratio']
            rect_angle_deg = detection['rect_angle_deg']

        front_distance = self._front_distance()
        if front_distance < OCCUPIED_SLOT_REJECT_DISTANCE and center_y_norm < 0.68:
            if self._switch_to_alternate_slot('occupied slot detected ahead'):
                return

        if front_distance < PARKING_STOP_DISTANCE:
            self._stop_robot()
            self.state = ParkingState.STOPPED
            self._publish_status('STOPPED - obstacle too close during vision parking.')
            return

        if not self._detection_is_fresh():
            now = time.time()
            if (
                now - self.last_detection_time > 2.5
                and now - self.vision_started_at > 2.5
            ):
                self._stop_robot()
                self.state = ParkingState.STOPPED
                self._publish_status('STOPPED - lost visual contact with the parking rectangle.')
                return

            cmd = Twist()
            cmd.linear.x = 0.02
            if self.selected_side == 'left':
                cmd.angular.z = 0.08
            elif self.selected_side == 'right':
                cmd.angular.z = -0.08
            self._publish_cmd(cmd)
            return

        if detection is None:
            return

        if (
            (
                center_y_norm >= PARKING_FINAL_Y_NORM
                and height_ratio >= PARKING_FINAL_HEIGHT_RATIO
            )
            or (
                front_distance <= PARKING_FINAL_FRONT_DISTANCE
                and center_y_norm >= 0.80
                and abs(center_error) <= 0.12
            )
        ):
            self._stop_robot()
            self.state = ParkingState.PARKED
            self._publish_status(
                f'PARKED - the robot tracked the yellow rectangle into Point {self.active_target["label"] if self.active_target else "?"} and stopped inside it.'
            )
            return

        cmd = Twist()
        if center_y_norm >= PARKING_STRAIGHTEN_Y_NORM and height_ratio >= 0.20:
            cmd.linear.x = 0.04
            cmd.angular.z = 0.0
            self._publish_cmd(cmd)
            return

        if abs(center_error) > 0.22 and area_ratio < 0.10:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = 0.07
            if abs(center_error) > 0.10:
                cmd.linear.x = 0.05
            if area_ratio > 0.10 or center_y_norm > 0.76:
                cmd.linear.x = min(cmd.linear.x, 0.04)
            if abs(center_error) <= 0.06 and abs(rect_angle_deg) <= 7.0:
                cmd.linear.x = min(cmd.linear.x, 0.05)

        angular = (-1.30 * center_error) - (0.018 * rect_angle_deg)
        cmd.angular.z = max(-0.48, min(0.48, angular))
        self._publish_cmd(cmd)

    def _run_return_to_idle(self) -> None:
        reached_pose = self._drive_to_pose(
            IDLE_POSE[0],
            IDLE_POSE[1],
            IDLE_POSE[2],
            goal_tolerance=0.12,
            final_yaw_tolerance=0.08,
        )
        if reached_pose:
            self.active_target_key = None
            self.active_target = None
            self.target_queue = []
            self.state = ParkingState.IDLE
            self._publish_status('IDLE - returned to the spawn pose and waiting for the next command.')

    def _drive_to_pose(
        self,
        target_x: float,
        target_y: float,
        target_yaw: Optional[float],
        goal_tolerance: float,
        final_yaw_tolerance: float = 0.15,
        max_linear: float = 0.22,
    ) -> bool:
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.hypot(dx, dy)
        cmd = Twist()

        if distance <= goal_tolerance:
            if target_yaw is None:
                self._stop_robot()
                return True

            yaw_error = self._normalize_angle(target_yaw - self.robot_yaw)
            if abs(yaw_error) <= final_yaw_tolerance:
                self._stop_robot()
                return True

            cmd.angular.z = max(-0.5, min(0.5, 1.6 * yaw_error))
            self._publish_cmd(cmd)
            return False

        target_heading = math.atan2(dy, dx)
        heading_error = self._normalize_angle(target_heading - self.robot_yaw)
        yaw_error = 0.0 if target_yaw is None else self._normalize_angle(target_yaw - self.robot_yaw)

        if self._front_distance() < 0.40 and distance > 0.18:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-0.5, min(0.5, 1.4 * heading_error))
            self._publish_cmd(cmd)
            return False

        if abs(heading_error) > 0.8:
            cmd.linear.x = 0.02
        else:
            cmd.linear.x = min(max_linear, 0.08 + 0.18 * distance)

        angular_command = 1.8 * heading_error + 0.35 * yaw_error
        cmd.angular.z = max(-0.8, min(0.8, angular_command))
        self._publish_cmd(cmd)
        return False

    def _rotate_to_yaw(self, target_yaw: float, tolerance: float = 0.08) -> bool:
        yaw_error = self._normalize_angle(target_yaw - self.robot_yaw)
        if abs(yaw_error) <= tolerance:
            self._stop_robot()
            return True

        cmd = Twist()
        cmd.angular.z = max(-0.45, min(0.45, 1.6 * yaw_error))
        self._publish_cmd(cmd)
        return False

    def _detect_yellow_slot(self, frame: 'np.ndarray') -> Optional[Dict[str, Any]]:
        height, width = frame.shape[:2]
        roi_top = int(height * PARKING_ROI_TOP_RATIO)
        roi = frame[roi_top:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower = np.array([15, 55, 55], dtype=np.uint8)
        upper = np.array([45, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_detection: Optional[Dict[str, Any]] = None
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < MIN_SLOT_AREA:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w < 28 or h < 18:
                continue

            rect = cv2.minAreaRect(contour)
            (_, _), (rect_w, rect_h), _ = rect
            rect_area = max(float(rect_w * rect_h), 1.0)
            rectangularity = area / rect_area
            if rectangularity < 0.58:
                continue

            aspect_ratio = max(rect_w, rect_h) / max(min(rect_w, rect_h), 1.0)
            if aspect_ratio < 1.0 or aspect_ratio > 6.0:
                continue

            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.035 * perimeter, True)

            box = cv2.boxPoints(rect)
            box[:, 1] += roi_top

            contour_shifted = contour.copy()
            contour_shifted[:, 0, 1] += roi_top

            approx_shifted = approx.copy()
            approx_shifted[:, 0, 1] += roi_top

            rect_angle_deg = self._compute_box_angle_deg(box)
            score = area * rectangularity * min(aspect_ratio, 2.6)

            detection = {
                'center_x': float(x + (w / 2.0)),
                'center_y': float(roi_top + y + (h / 2.0)),
                'width': float(w),
                'height': float(h),
                'area': area,
                'area_ratio': area / float(width * height),
                'image_width': float(width),
                'image_height': float(height),
                'bottom_y': float(roi_top + y + h),
                'rectangularity': rectangularity,
                'aspect_ratio': aspect_ratio,
                'approx_vertices': int(len(approx)),
                'rect_angle_deg': rect_angle_deg,
                'score': score,
                'box_points': np.int32(box),
                'contour': contour_shifted,
                'approx': approx_shifted,
            }

            if best_detection is None or detection['score'] > best_detection['score']:
                best_detection = detection

        return best_detection

    def _render_debug_view(self, frame: 'np.ndarray', detection: Optional[Dict[str, Any]]) -> None:
        if not self.debug_window_enabled:
            return

        try:
            view = frame.copy()
            height, width = view.shape[:2]
            roi_top = int(height * PARKING_ROI_TOP_RATIO)

            cv2.line(view, (0, roi_top), (width, roi_top), (160, 160, 160), 1)
            cv2.line(view, (width // 2, 0), (width // 2, height), (255, 0, 255), 1)
            cv2.rectangle(
                view,
                (int(width * 0.44), 0),
                (int(width * 0.56), height),
                (255, 180, 255),
                1,
            )

            if detection is not None:
                cv2.drawContours(view, [detection['contour']], -1, (0, 255, 0), 2)
                cv2.drawContours(view, [detection['approx']], -1, (255, 255, 0), 2)
                cv2.drawContours(view, [detection['box_points']], -1, (0, 165, 255), 2)

                center = (int(detection['center_x']), int(detection['center_y']))
                cv2.circle(view, center, 5, (0, 0, 255), -1)
                cv2.line(view, (width // 2, center[1]), center, (0, 0, 255), 2)

                info_lines = [
                    f'center_err: {(detection["center_x"] - (width / 2.0)) / (width / 2.0):+.3f}',
                    f'rect_angle: {detection["rect_angle_deg"]:+.1f} deg',
                    f'area_ratio: {detection["area_ratio"]:.3f}',
                    f'rectangularity: {detection["rectangularity"]:.2f}',
                    f'corners: {detection["approx_vertices"]}',
                ]
                for index, text in enumerate(info_lines):
                    cv2.putText(
                        view,
                        text,
                        (10, height - 110 + (22 * index)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

            left_score = 0.0 if self.scan_results['left'] is None else self.scan_results['left']['score']
            right_score = 0.0 if self.scan_results['right'] is None else self.scan_results['right']['score']
            left_clearance = self.scan_front_distances['left']
            right_clearance = self.scan_front_distances['right']
            header_lines = [
                f'state: {self.state.name}',
                f'target: {self.active_target["label"] if self.active_target else "-"}',
                f'selected_side: {self.selected_side or "-"}',
                f'cmd: v={self.last_cmd_linear:+.2f} m/s  w={self.last_cmd_angular:+.2f} rad/s',
                f'scan_scores: left={left_score:.1f} right={right_score:.1f}',
                f'scan_clearance: left={left_clearance:.2f}m right={right_clearance:.2f}m',
            ]
            for index, text in enumerate(header_lines):
                cv2.putText(
                    view,
                    text,
                    (10, 24 + (24 * index)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (20, 20, 20),
                    3,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    view,
                    text,
                    (10, 24 + (24 * index)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            steer_tip_x = int((width / 2.0) - (self.last_cmd_angular * 180.0))
            steer_tip_y = height - 30
            cv2.arrowedLine(
                view,
                (width // 2, height - 10),
                (steer_tip_x, steer_tip_y),
                (0, 0, 255),
                3,
                tipLength=0.2,
            )

            cv2.imshow(DEBUG_WINDOW_NAME, view)
            cv2.waitKey(1)
        except cv2.error as exc:
            if not self._debug_window_failed:
                self._debug_window_failed = True
                self.debug_window_enabled = False
                self.get_logger().warning(
                    f'OpenCV debug window disabled: {exc}',
                    throttle_duration_sec=2.0,
                )

    def _extract_target_key(self, text: str) -> Optional[str]:
        route_targets = self._extract_route_targets(text)
        if not route_targets:
            return None
        return route_targets[0]

    def _extract_route_targets(self, text: str) -> List[str]:
        compact = re.sub(r'[^a-z0-9]+', ' ', text.lower()).strip()
        if not compact:
            return []

        if any(
            phrase in compact
            for phrase in (
                'all points',
                'all point',
                'every point',
                'all parking points',
                'all parking point',
                'every parking point',
            )
        ):
            return ['a', 'b', 'c', 'd']

        matches = re.findall(r'\b([abcd])\b', compact)
        if not matches:
            return []

        has_parking_context = any(
            phrase in compact
            for phrase in (
                'park',
                'parking',
                'point',
                'zone',
                'slot',
                'route',
                'go to',
                'visit',
            )
        )
        if len(matches) == 1 and not has_parking_context:
            return []

        route_targets: List[str] = []
        for key in matches:
            if key not in route_targets:
                route_targets.append(key)
        return route_targets

    def _search_timeout_exceeded(self) -> bool:
        return (
            self.search_state_started_at > 0.0
            and (time.time() - self.search_state_started_at) >= SEARCH_TIMEOUT_SEC
        )

    def _abort_search_and_process_queue(self) -> None:
        self._stop_robot()
        queued_command = self.queued_command_text
        self.queued_command_text = None

        if queued_command == 'return':
            self._publish_status(
                f'TIMEOUT - parking search exceeded {SEARCH_TIMEOUT_SEC:.1f} s. Returning to idle.'
            )
            self._start_return_to_idle()
            return

        if queued_command is not None:
            route_targets = self._extract_route_targets(queued_command)
            if route_targets:
                self._publish_status(
                    f'TIMEOUT - parking search exceeded {SEARCH_TIMEOUT_SEC:.1f} s. Switching to route {self._format_route_labels(route_targets)}.'
                )
                self._start_parking_route(route_targets)
                return

        self.state = ParkingState.STOPPED
        self._publish_status(
            f'STOPPED - parking search exceeded {SEARCH_TIMEOUT_SEC:.1f} s with no queued command.'
        )

    def _finish_at_target(self) -> None:
        reached_label = self.active_target["label"] if self.active_target else '?'
        self._stop_robot()
        self.scan_started_at = 0.0
        self.scan_best_detection = None
        self.scan_results = {'left': None, 'right': None}
        self.scan_front_distances = {'left': float('inf'), 'right': float('inf')}
        self.scan_best_front_distance = float('inf')
        self.scan_clearance_samples = {'left': [], 'right': []}
        self.blocked_sides = set()
        self.selected_side = None
        self.selected_slot_yaw = None
        self.vision_started_at = 0.0
        self.search_state_started_at = 0.0
        if self.target_queue:
            next_label = PARKING_TARGETS[self.target_queue[0]]['label']
            self._publish_status(
                f'REACHED_{reached_label} - reached Point {reached_label}. Continuing to Point {next_label}.'
            )
            self._start_next_target_in_queue()
            return

        self.state = ParkingState.PARKED
        if self.active_target is None:
            self._publish_status('PARKED - point target reached and robot stopped.')
            return
        self._publish_status(
            f'REACHED_{reached_label} - reached Point {reached_label} and stopped there.'
        )

    @staticmethod
    def _format_route_command(route_targets: List[str]) -> str:
        if len(route_targets) == 1:
            return f'park at {route_targets[0].upper()}'
        return 'park route ' + ' '.join(target.upper() for target in route_targets)

    @staticmethod
    def _format_route_labels(route_targets: List[str]) -> str:
        return ' -> '.join(target.upper() for target in route_targets)

    @staticmethod
    def _is_search_state(state: ParkingState) -> bool:
        return state in (
            ParkingState.SCAN_LEFT,
            ParkingState.SCAN_RIGHT,
            ParkingState.TURN_TO_SLOT,
            ParkingState.VISION_PARK,
        )

    def _select_retarget_waypoint_index(self, target: Dict[str, Any]) -> int:
        waypoints: List[Tuple[float, float]] = target['approach_waypoints']
        if not waypoints:
            return 0

        # The first waypoint mirrors the idle pose. Skip it so a new parking
        # target redirects from the robot's current position instead of
        # replaying a return-to-idle hop.
        candidate_indices = [
            index
            for index, waypoint in enumerate(waypoints)
            if waypoint != IDLE_POSE[:2]
        ]
        if not candidate_indices:
            return 0

        if self.robot_x is None or self.robot_y is None:
            return candidate_indices[0]

        return min(
            candidate_indices,
            key=lambda index: math.hypot(
                waypoints[index][0] - self.robot_x,
                waypoints[index][1] - self.robot_y,
            ),
        )

    def _publish_cmd(self, cmd: Twist) -> None:
        self.last_cmd_linear = float(cmd.linear.x)
        self.last_cmd_angular = float(cmd.angular.z)
        self.cmd_pub.publish(cmd)

    def _yaw_to_side(self, side: str) -> float:
        if self.active_target is None:
            return self.robot_yaw

        slot_x, slot_y = self.active_target['slot_centers'][side]
        robot_x = self.robot_x if self.robot_x is not None else self.active_target['staging_pose'][0]
        robot_y = self.robot_y if self.robot_y is not None else self.active_target['staging_pose'][1]
        return self._normalize_angle(math.atan2(slot_y - robot_y, slot_x - robot_x))

    def _switch_to_alternate_slot(self, reason: str) -> bool:
        if self.selected_side is None:
            return False

        self.blocked_sides.add(self.selected_side)
        alternate_side = self._opposite_side(self.selected_side)
        if alternate_side in self.blocked_sides:
            return False

        self.selected_side = alternate_side
        self.selected_slot_yaw = self._yaw_to_side(alternate_side)
        self.state = ParkingState.TURN_TO_SLOT
        self._publish_status(
            f'TURN_TO_SLOT - {reason}. Switching to the {alternate_side} parking slot.'
        )
        return True

    @staticmethod
    def _opposite_side(side: str) -> str:
        return 'right' if side == 'left' else 'left'

    def _detection_is_fresh(self) -> bool:
        return (time.time() - self.last_detection_time) <= 0.6

    def _front_distance(self) -> float:
        if self.latest_scan is None:
            return float('inf')

        valid_ranges: List[float] = []
        for index, value in enumerate(self.latest_scan.ranges):
            if not math.isfinite(value) or value <= 0.12:
                continue
            angle = self.latest_scan.angle_min + (index * self.latest_scan.angle_increment)
            angle_deg = math.degrees(angle)
            if -18.0 <= angle_deg <= 18.0:
                valid_ranges.append(float(value))

        return min(valid_ranges) if valid_ranges else float('inf')

    def _slot_clearance(self, side: str) -> float:
        if self.latest_scan is None or self.active_target is None:
            return float('inf')

        slot_x, slot_y = self.active_target['slot_centers'][side]
        robot_x = self.robot_x if self.robot_x is not None else self.active_target['staging_pose'][0]
        robot_y = self.robot_y if self.robot_y is not None else self.active_target['staging_pose'][1]
        slot_bearing = self._normalize_angle(
            math.atan2(slot_y - robot_y, slot_x - robot_x) - self.robot_yaw
        )
        sector_half_width = math.radians(SLOT_CLEARANCE_SECTOR_DEG)

        valid_ranges: List[float] = []
        for index, value in enumerate(self.latest_scan.ranges):
            if not math.isfinite(value) or value <= 0.12:
                continue
            angle = self.latest_scan.angle_min + (index * self.latest_scan.angle_increment)
            angle_error = self._normalize_angle(angle - slot_bearing)
            if abs(angle_error) <= sector_half_width:
                valid_ranges.append(float(value))

        return min(valid_ranges) if valid_ranges else float('inf')

    @staticmethod
    def _representative_clearance(samples: List[float]) -> float:
        if not samples:
            return float('inf')

        ordered = sorted(samples)
        return ordered[len(ordered) // 2]

    def _stop_robot(self) -> None:
        self._publish_cmd(Twist())

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.get_logger().info(f'[STATUS] {text}')

    def destroy_node(self) -> bool:
        if CV_AVAILABLE and self.debug_window_enabled:
            try:
                cv2.destroyWindow(DEBUG_WINDOW_NAME)
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        return super().destroy_node()

    @staticmethod
    def _clone_detection(detection: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if detection is None:
            return None

        clone: Dict[str, Any] = {}
        for key, value in detection.items():
            if hasattr(value, 'copy'):
                clone[key] = value.copy()
            else:
                clone[key] = value
        return clone

    @staticmethod
    def _compute_box_angle_deg(box_points: 'np.ndarray') -> float:
        max_length = -1.0
        best_angle = 0.0
        for index in range(4):
            p1 = box_points[index]
            p2 = box_points[(index + 1) % 4]
            dx = float(p2[0] - p1[0])
            dy = float(p2[1] - p1[1])
            length = math.hypot(dx, dy)
            if length <= max_length:
                continue
            max_length = length
            angle = math.degrees(math.atan2(dy, dx))
            while angle > 90.0:
                angle -= 180.0
            while angle < -90.0:
                angle += 180.0
            best_angle = angle
        return best_angle

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingMissionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
