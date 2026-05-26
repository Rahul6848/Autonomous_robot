import math
import time
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


POINT_A_POSE = (4.2, -2.2, 0.0)
APPROACH_WAYPOINTS: List[Tuple[float, float]] = [
    (0.0, 0.0),
    (1.6, -0.8),
    (3.1, -1.55),
    (POINT_A_POSE[0], POINT_A_POSE[1]),
]

SCAN_SETTLE_SEC = 0.45
SCAN_SAMPLE_SEC = 1.20
DEBUG_WINDOW_NAME = 'Parking Vision Debug'


class ParkingState(Enum):
    IDLE = auto()
    APPROACHING_A = auto()
    SCAN_LEFT = auto()
    SCAN_RIGHT = auto()
    TURN_TO_SLOT = auto()
    VISION_PARK = auto()
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
        self.robot_x: Optional[float] = None
        self.robot_y: Optional[float] = None
        self.robot_yaw = 0.0
        self.latest_scan: Optional[LaserScan] = None
        self.latest_detection: Optional[Dict[str, Any]] = None
        self.last_detection_time = 0.0
        self.waypoint_index = 0

        self.scan_center_yaw: Optional[float] = None
        self.scan_started_at = 0.0
        self.scan_best_detection: Optional[Dict[str, Any]] = None
        self.scan_results: Dict[str, Optional[Dict[str, Any]]] = {
            'left': None,
            'right': None,
        }
        self.selected_side: Optional[str] = None
        self.selected_slot_yaw: Optional[float] = None
        self.vision_started_at = 0.0

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
            'IDLE - send "park at a" to drive to Point A, scan left/right, detect a yellow parking rectangle, and park with vision.'
        )

    def _command_callback(self, msg: String) -> None:
        text = msg.data.strip().lower()
        self.get_logger().info(f'[CMD] received: "{msg.data}"')

        if 'stop' in text or 'halt' in text:
            self._stop_robot()
            self.state = ParkingState.STOPPED
            self._publish_status('STOPPED - robot halted by command.')
            return

        start_requested = any(
            keyword in text
            for keyword in (
                'park',
                'parking',
                'go to a',
                'point a',
                'yellow slot',
            )
        )
        if start_requested and self.state in (
            ParkingState.IDLE,
            ParkingState.STOPPED,
            ParkingState.PARKED,
        ):
            if not CV_AVAILABLE:
                self._publish_status(
                    'FAILED - OpenCV/cv_bridge is unavailable, so vision parking cannot start.'
                )
                return
            self.waypoint_index = 0
            self.scan_center_yaw = None
            self.scan_started_at = 0.0
            self.scan_best_detection = None
            self.scan_results = {'left': None, 'right': None}
            self.selected_side = None
            self.selected_slot_yaw = None
            self.vision_started_at = 0.0
            self.state = ParkingState.APPROACHING_A
            self._publish_status('APPROACHING_A - driving from the spawn point to Point A.')

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
            self._previous_state = self.state

        if self.state in (ParkingState.IDLE, ParkingState.STOPPED, ParkingState.PARKED):
            self._stop_robot()
            return

        if self.robot_x is None or self.robot_y is None:
            return

        if self.state == ParkingState.APPROACHING_A:
            self._run_approach_to_point_a()
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

    def _run_approach_to_point_a(self) -> None:
        if self.waypoint_index < len(APPROACH_WAYPOINTS):
            target_x, target_y = APPROACH_WAYPOINTS[self.waypoint_index]
            reached = self._drive_to_pose(target_x, target_y, None, goal_tolerance=0.22)
            if reached:
                self.waypoint_index += 1
            return

        reached_pose = self._drive_to_pose(
            POINT_A_POSE[0],
            POINT_A_POSE[1],
            POINT_A_POSE[2],
            goal_tolerance=0.12,
            final_yaw_tolerance=0.08,
        )
        if reached_pose:
            self.scan_center_yaw = self.robot_yaw
            self.scan_started_at = 0.0
            self.scan_best_detection = None
            self.state = ParkingState.SCAN_LEFT
            self._publish_status(
                'SCAN_LEFT - reached Point A. Rotating left by 90 degrees to inspect for a yellow parking rectangle.'
            )

    def _run_scan(self, side: str) -> None:
        if self.scan_center_yaw is None:
            self.state = ParkingState.STOPPED
            self._publish_status('STOPPED - scan center yaw is unavailable at Point A.')
            return

        yaw_offset = math.pi / 2.0 if side == 'left' else -math.pi / 2.0
        target_yaw = self._normalize_angle(self.scan_center_yaw + yaw_offset)
        if not self._rotate_to_yaw(target_yaw, tolerance=0.06):
            return

        now = time.time()
        self._stop_robot()
        if self.scan_started_at == 0.0:
            self.scan_started_at = now
            self.scan_best_detection = None
            return

        sample_start = self.scan_started_at + SCAN_SETTLE_SEC
        sample_end = sample_start + SCAN_SAMPLE_SEC
        if sample_start <= now <= sample_end and self._detection_is_fresh():
            candidate = self._clone_detection(self.latest_detection)
            if candidate is not None and (
                self.scan_best_detection is None
                or candidate['score'] > self.scan_best_detection['score']
            ):
                self.scan_best_detection = candidate

        if now < sample_end:
            return

        self.scan_results[side] = self._clone_detection(self.scan_best_detection)
        best_score = 0.0 if self.scan_best_detection is None else self.scan_best_detection['score']
        self.get_logger().info(f'[SCAN] {side} best score: {best_score:.1f}')

        self.scan_started_at = 0.0
        self.scan_best_detection = None

        if side == 'left':
            self.state = ParkingState.SCAN_RIGHT
            self._publish_status(
                'SCAN_RIGHT - left scan complete. Rotating right by 90 degrees to inspect the opposite side.'
            )
            return

        self._select_scanned_slot()

    def _select_scanned_slot(self) -> None:
        candidates = [
            (side, detection)
            for side, detection in self.scan_results.items()
            if detection is not None
        ]
        if not candidates:
            self.state = ParkingState.STOPPED
            self._publish_status(
                'STOPPED - no yellow rectangular parking area was detected on either side of Point A.'
            )
            return

        self.selected_side, best_detection = max(candidates, key=lambda item: item[1]['score'])
        yaw_offset = math.pi / 2.0 if self.selected_side == 'left' else -math.pi / 2.0
        self.selected_slot_yaw = self._normalize_angle(self.scan_center_yaw + yaw_offset)
        self.state = ParkingState.TURN_TO_SLOT
        self._publish_status(
            f'TURN_TO_SLOT - selected the {self.selected_side} parking rectangle (score {best_detection["score"]:.1f}). Rotating back to align for vision parking.'
        )

    def _run_turn_to_selected_slot(self) -> None:
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
        front_distance = self._front_distance()
        if front_distance < 0.28:
            self._stop_robot()
            self.state = ParkingState.STOPPED
            self._publish_status('STOPPED - obstacle too close during vision parking.')
            return

        if not self._detection_is_fresh():
            now = time.time()
            if (
                now - self.last_detection_time > 1.2
                and now - self.vision_started_at > 1.2
            ):
                self._stop_robot()
                self.state = ParkingState.STOPPED
                self._publish_status('STOPPED - lost visual contact with the parking rectangle.')
                return

            cmd = Twist()
            cmd.angular.z = 0.20 if self.selected_side == 'left' else -0.20
            self._publish_cmd(cmd)
            return

        detection = self.latest_detection
        if detection is None:
            return

        image_width = detection['image_width']
        image_height = detection['image_height']
        center_error = (detection['center_x'] - (image_width / 2.0)) / (image_width / 2.0)
        center_y_norm = detection['center_y'] / image_height
        height_ratio = detection['height'] / image_height
        area_ratio = detection['area_ratio']
        rect_angle_deg = detection['rect_angle_deg']

        if center_y_norm >= 0.83 and height_ratio >= 0.30:
            self._stop_robot()
            self.state = ParkingState.PARKED
            self._publish_status(
                'PARKED - the robot tracked the yellow rectangle into the parking area and stopped inside it.'
            )
            return

        cmd = Twist()
        if abs(center_error) > 0.24 and area_ratio < 0.09:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = 0.08
            if abs(center_error) > 0.12:
                cmd.linear.x = 0.05
            if area_ratio > 0.12:
                cmd.linear.x = min(cmd.linear.x, 0.04)

        angular = (-1.05 * center_error) - (0.012 * rect_angle_deg)
        cmd.angular.z = max(-0.55, min(0.55, angular))
        self._publish_cmd(cmd)

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
        roi_top = int(height * 0.35)
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
            if area < 500.0:
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
            roi_top = int(height * 0.35)

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
            header_lines = [
                f'state: {self.state.name}',
                f'selected_side: {self.selected_side or "-"}',
                f'cmd: v={self.last_cmd_linear:+.2f} m/s  w={self.last_cmd_angular:+.2f} rad/s',
                f'scan_scores: left={left_score:.1f} right={right_score:.1f}',
                'drawn: green=contour  cyan=polygon  orange=minAreaRect',
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

    def _publish_cmd(self, cmd: Twist) -> None:
        self.last_cmd_linear = float(cmd.linear.x)
        self.last_cmd_angular = float(cmd.angular.z)
        self.cmd_pub.publish(cmd)

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
