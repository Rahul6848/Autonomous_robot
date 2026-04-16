import math
import os
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class PerceptionControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('perception_controller')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('use_webcam', False)
        self.declare_parameter('webcam_index', 0)
        self.declare_parameter('webcam_fps', 10.0)
        self.declare_parameter('show_debug_window', True)
        self.declare_parameter('model_path', '')
        self.declare_parameter('labels_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('stop_duration_sec', 2.5)
        self.declare_parameter('command_hold_sec', 3.0)
        self.declare_parameter('turn_intent_hold_sec', 8.0)
        self.declare_parameter('turn_execute_sec', 1.8)
        self.declare_parameter('intersection_min_road_width_ratio', 0.58)
        self.declare_parameter('intersection_max_left_edge_ratio', 0.12)
        self.declare_parameter('intersection_min_left_opening_ratio', 0.18)
        self.declare_parameter('turn_trigger_x_min', 3.9)
        self.declare_parameter('turn_trigger_y_min', 0.2)

        self.image_topic = self.get_parameter('image_topic').value
        self.use_webcam = bool(self.get_parameter('use_webcam').value)
        self.webcam_index = int(self.get_parameter('webcam_index').value)
        self.webcam_fps = float(self.get_parameter('webcam_fps').value)
        self.show_debug_window = bool(self.get_parameter('show_debug_window').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.labels_path = str(self.get_parameter('labels_path').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.stop_duration_sec = float(self.get_parameter('stop_duration_sec').value)
        self.command_hold_sec = float(self.get_parameter('command_hold_sec').value)
        self.turn_intent_hold_sec = float(self.get_parameter('turn_intent_hold_sec').value)
        self.turn_execute_sec = float(self.get_parameter('turn_execute_sec').value)
        self.intersection_min_road_width_ratio = float(
            self.get_parameter('intersection_min_road_width_ratio').value
        )
        self.intersection_max_left_edge_ratio = float(
            self.get_parameter('intersection_max_left_edge_ratio').value
        )
        self.intersection_min_left_opening_ratio = float(
            self.get_parameter('intersection_min_left_opening_ratio').value
        )
        self.turn_trigger_x_min = float(self.get_parameter('turn_trigger_x_min').value)
        self.turn_trigger_y_min = float(self.get_parameter('turn_trigger_y_min').value)

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_subscription = None
        self.webcam_capture = None
        self.webcam_timer = None
        if self.use_webcam:
            self.initialize_webcam()
        else:
            self.image_subscription = self.create_subscription(
                Image, self.image_topic, self.image_callback, 10
            )
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.debug_window_name = 'robot_perception'
        self.yolo_model = None
        self.class_names: List[str] = []
        self.active_command = 'forward'
        self.command_until = 0.0
        self.stop_until = 0.0
        self.pending_turn: Optional[str] = None
        self.turn_intent_until = 0.0
        self.turn_execute_until = 0.0
        self.left_turn_authorized = False
        self.robot_x: Optional[float] = None
        self.robot_y: Optional[float] = None
        self.yaw = 0.0
        self.goal_tolerance = 0.2
        self.lookahead_distance = 0.75
        self.max_linear_speed = 0.24
        self.max_angular_speed = 1.0
        self.approach_waypoints = [
            (-8.0, 0.75),
            (-5.0, 0.75),
            (-2.0, 0.75),
            (1.0, 0.75),
            (3.2, 0.75),
            (4.1, 0.9),
        ]
        self.turn_waypoints = [
            (4.6, 1.35),
            (4.55, 2.4),
            (4.35, 4.0),
            (4.25, 6.0),
            (4.25, 8.2),
        ]
        self.approach_waypoint_index = 0
        self.turn_waypoint_index = 0

        self.load_detector()
        self.get_logger().info(
            f'Perception controller using {"webcam" if self.use_webcam else self.image_topic}. '
            f'Debug window: {self.show_debug_window}'
        )

    def initialize_webcam(self) -> None:
        self.webcam_capture = cv2.VideoCapture(self.webcam_index)
        if not self.webcam_capture.isOpened():
            self.get_logger().error(
                f'Failed to open webcam index {self.webcam_index}.'
            )
            self.webcam_capture = None
            return

        period = 1.0 / max(1.0, self.webcam_fps)
        self.webcam_timer = self.create_timer(period, self.webcam_timer_callback)

    def webcam_timer_callback(self) -> None:
        if self.webcam_capture is None:
            return

        success, frame = self.webcam_capture.read()
        if not success or frame is None:
            self.get_logger().warning('Webcam frame capture failed.', throttle_duration_sec=2.0)
            return

        self.process_frame(frame)

    def load_detector(self) -> None:
        if not self.model_path:
            self.get_logger().warning(
                'No YOLO model_path provided. Sign control is disabled; lane following only.'
            )
            return

        if not os.path.exists(self.model_path):
            self.get_logger().warning(
                f'Model file not found at {self.model_path}. Sign control is disabled.'
            )
            return

        try:
            self.yolo_model = YOLO(self.model_path)
        except Exception as exc:
            self.get_logger().error(f'Failed to load YOLO model: {exc}')
            self.yolo_model = None
            return

        if self.labels_path and os.path.exists(self.labels_path):
            with open(self.labels_path, 'r', encoding='utf-8') as handle:
                self.class_names = [line.strip() for line in handle if line.strip()]
        elif hasattr(self.yolo_model, 'names'):
            names = self.yolo_model.names
            if isinstance(names, dict):
                self.class_names = [str(names[idx]) for idx in sorted(names)]
            else:
                self.class_names = [str(name) for name in names]
        else:
            self.class_names = ['forward', 'left', 'right', 'stop']

        self.get_logger().info(f'Loaded YOLO model from {self.model_path}')

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_frame(frame)

    def process_frame(self, frame: np.ndarray) -> None:
        annotated = frame.copy()

        detections = self.detect_signs(frame) if self.yolo_model is not None else []
        lane_result = self.compute_lane_command(frame)
        self.update_command_from_detections(detections, lane_result)
        cmd = self.compose_motion_command(lane_result)
        self.publisher_.publish(cmd)

        self.draw_debug(annotated, detections, lane_result, cmd)
        if self.show_debug_window:
            cv2.imshow(self.debug_window_name, annotated)
            cv2.waitKey(1)

    def odom_callback(self, msg: Odometry) -> None:
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def detect_signs(self, frame: np.ndarray) -> List[Tuple[str, float, Tuple[int, int, int, int]]]:
        results = self.yolo_model.predict(
            source=frame,
            verbose=False,
            conf=self.confidence_threshold,
            imgsz=640,
        )
        detections = []
        if not results:
            return detections

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                confidence = float(box.conf[0])
                if confidence < self.confidence_threshold:
                    continue

                class_id = int(box.cls[0])
                xyxy = box.xyxy[0].tolist()
                x1, y1, x2, y2 = [int(value) for value in xyxy]
                label = (
                    self.class_names[class_id]
                    if class_id < len(self.class_names)
                    else f'class_{class_id}'
                )
                detections.append((label.lower(), confidence, (x1, y1, x2 - x1, y2 - y1)))

        return detections

    def update_command_from_detections(
        self,
        detections: List[Tuple[str, float, Tuple[int, int, int, int]]],
        lane_result: dict,
    ) -> None:
        now = time.time()
        best_detection = None

        if self.pending_turn is not None and now > self.turn_intent_until:
            self.pending_turn = None

        if now > self.turn_execute_until and self.active_command != 'stop':
            self.active_command = 'forward'

        for label, confidence, box in detections:
            normalized = self.normalize_sign_label(label)
            if normalized is None:
                continue
            if best_detection is None or confidence > best_detection[1]:
                best_detection = (normalized, confidence, box)

        if best_detection is None:
            if now > self.command_until and now > self.turn_execute_until:
                self.active_command = 'forward'
            self.promote_pending_turn_if_ready(lane_result)
            return

        label = best_detection[0]
        if label == 'stop':
            self.stop_until = now + self.stop_duration_sec
            self.pending_turn = None
        else:
            if label == 'left':
                self.pending_turn = 'left'
                self.turn_intent_until = now + self.turn_intent_hold_sec
                self.left_turn_authorized = True
                self.promote_pending_turn_if_ready(lane_result)
            else:
                self.active_command = label
                self.command_until = now + self.command_hold_sec

    def promote_pending_turn_if_ready(self, lane_result: dict) -> None:
        now = time.time()

        if self.pending_turn != 'left':
            return

        if now >= self.turn_execute_until and lane_result['left_turn_ready']:
            self.active_command = 'left'
            self.turn_execute_until = now + self.turn_execute_sec
            self.command_until = self.turn_execute_until
            self.pending_turn = None

    def compute_lane_command(self, frame: np.ndarray) -> dict:
        height, width = frame.shape[:2]
        roi = frame[int(height * 0.55):, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lane_mask = cv2.inRange(hsv, (0, 0, 150), (180, 80, 255))
        road_mask = cv2.inRange(hsv, (0, 0, 20), (180, 80, 110))

        kernel = np.ones((5, 5), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_OPEN, kernel)

        lane_center_x = self.find_lane_center_from_marks(lane_mask, width)
        if lane_center_x is None:
            lane_center_x = self.find_lane_center_from_road(road_mask, width)

        if lane_center_x is None:
            lane_center_x = width * 0.35

        target_x = int(lane_center_x)
        error = (target_x - width / 2.0) / (width / 2.0)
        road_features = self.extract_road_features(road_mask)
        left_turn_ready = self.is_left_turn_ready(road_features, width)
        odom_turn_ready = self.is_odom_turn_ready()

        return {
            'roi_top': int(height * 0.55),
            'target_x': target_x,
            'error': error,
            'lane_mask': lane_mask,
            'road_mask': road_mask,
            'road_features': road_features,
            'left_turn_ready': left_turn_ready and odom_turn_ready,
            'visual_turn_ready': left_turn_ready,
            'odom_turn_ready': odom_turn_ready,
        }

    def extract_road_features(self, road_mask: np.ndarray) -> dict:
        mask_height, mask_width = road_mask.shape[:2]
        contours, _ = cv2.findContours(road_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return {
                'left_edge': mask_width,
                'road_width': 0,
                'road_width_ratio': 0.0,
                'left_opening_ratio': 0.0,
            }

        contour = max(contours, key=cv2.contourArea)
        x, _, w, _ = cv2.boundingRect(contour)

        lower_start = int(mask_height * 0.45)
        left_limit = max(1, int(mask_width * 0.35))
        lower_left_zone = road_mask[lower_start:, :left_limit]
        left_opening_ratio = float(np.count_nonzero(lower_left_zone)) / float(lower_left_zone.size)

        return {
            'left_edge': x,
            'road_width': w,
            'road_width_ratio': float(w) / float(mask_width),
            'left_opening_ratio': left_opening_ratio,
        }

    def is_left_turn_ready(self, road_features: dict, image_width: int) -> bool:
        return (
            road_features['road_width_ratio'] >= self.intersection_min_road_width_ratio
            and road_features['left_edge'] <= image_width * self.intersection_max_left_edge_ratio
            and road_features['left_opening_ratio'] >= self.intersection_min_left_opening_ratio
        )

    def is_odom_turn_ready(self) -> bool:
        if self.robot_x is None or self.robot_y is None:
            return False

        return self.robot_x >= self.turn_trigger_x_min and self.robot_y >= self.turn_trigger_y_min

    def find_lane_center_from_marks(
        self, lane_mask: np.ndarray, image_width: int
    ) -> Optional[float]:
        points = cv2.findNonZero(lane_mask)
        if points is None:
            return None

        xs = points[:, 0, 0]
        left_half = xs[xs < image_width * 0.6]
        if len(left_half) < 40:
            return None

        centerline_x = float(np.median(left_half))
        return max(0.0, centerline_x - image_width * 0.18)

    def find_lane_center_from_road(
        self, road_mask: np.ndarray, image_width: int
    ) -> Optional[float]:
        road_features = self.extract_road_features(road_mask)
        if road_features['road_width'] <= 0:
            return None

        x = road_features['left_edge']
        w = road_features['road_width']
        if w < image_width * 0.25:
            return None

        road_center = x + w / 2.0
        return road_center - w * 0.22

    def compose_motion_command(self, lane_result: dict) -> Twist:
        now = time.time()
        cmd = Twist()

        if now < self.stop_until:
            return cmd

        path_cmd = self.compose_path_command()
        if path_cmd is None:
            self.active_command = 'waiting_at_turn'
            return cmd

        return path_cmd

    def compose_path_command(self) -> Optional[Twist]:
        if self.robot_x is None or self.robot_y is None:
            return None

        if self.should_follow_turn_path():
            self.active_command = 'left'
            target, self.turn_waypoint_index, reached_goal = self.advance_waypoint_index(
                self.turn_waypoints, self.turn_waypoint_index
            )
        else:
            self.active_command = 'forward'
            target, self.approach_waypoint_index, reached_goal = self.advance_waypoint_index(
                self.approach_waypoints, self.approach_waypoint_index
            )

            if reached_goal and not self.left_turn_authorized:
                return None

        if target is None:
            return Twist()

        target_x, target_y = target
        heading_to_target = math.atan2(target_y - self.robot_y, target_x - self.robot_x)
        heading_error = self.normalize_angle(heading_to_target - self.yaw)

        cmd = Twist()
        cmd.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, 1.8 * heading_error),
        )

        turn_scale = max(0.25, 1.0 - min(abs(heading_error), 1.2) / 1.2)
        cmd.linear.x = min(self.max_linear_speed, 0.12 + 0.14 * turn_scale)

        if abs(heading_error) > 0.9:
            cmd.linear.x = 0.08

        return cmd

    def should_follow_turn_path(self) -> bool:
        if not self.left_turn_authorized:
            return False

        return self.robot_x is not None and self.robot_x >= self.turn_trigger_x_min

    def advance_waypoint_index(
        self, waypoints: List[Tuple[float, float]], index: int
    ) -> Tuple[Optional[Tuple[float, float]], int, bool]:
        robot_x = self.robot_x
        robot_y = self.robot_y
        if robot_x is None or robot_y is None:
            return None, index, False

        if index >= len(waypoints):
            return None, index, True

        while index < len(waypoints) - 1:
            target_x, target_y = waypoints[index]
            if math.hypot(target_x - robot_x, target_y - robot_y) > self.lookahead_distance:
                break
            index += 1

        target = waypoints[index]
        distance = math.hypot(target[0] - robot_x, target[1] - robot_y)
        reached_goal = index == len(waypoints) - 1 and distance < self.goal_tolerance
        return target, index, reached_goal

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def draw_debug(
        self,
        frame: np.ndarray,
        detections: List[Tuple[str, float, Tuple[int, int, int, int]]],
        lane_result: dict,
        cmd: Twist,
    ) -> None:
        for label, confidence, (x, y, w, h) in detections:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (40, 220, 40), 2)
            cv2.putText(
                frame,
                f'{label} {confidence:.2f}',
                (x, max(25, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (40, 220, 40),
                2,
            )

        roi_top = lane_result['roi_top']
        target_x = lane_result['target_x']
        cv2.line(frame, (target_x, roi_top), (target_x, frame.shape[0] - 1), (0, 255, 255), 2)
        cv2.line(
            frame,
            (frame.shape[1] // 2, roi_top),
            (frame.shape[1] // 2, frame.shape[0] - 1),
            (255, 255, 0),
            1,
        )

        road_features = lane_result['road_features']
        status = (
            f'cmd={self.active_command} pending={self.pending_turn or "-"} '
            f'left_auth={self.left_turn_authorized} '
            f'left_ready={lane_result["left_turn_ready"]} '
            f'visual={lane_result["visual_turn_ready"]} '
            f'odom={lane_result["odom_turn_ready"]} '
            f'width={road_features["road_width_ratio"]:.2f} '
            f'open={road_features["left_opening_ratio"]:.2f} '
            f'v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}'
        )
        cv2.putText(
            frame, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
        )

    @staticmethod
    def normalize_sign_label(label: str) -> Optional[str]:
        text = label.strip().lower().replace(' ', '_')
        if 'stop' in text:
            return 'stop'
        if 'cell_phone' in text or 'cellphone' in text or 'mobile_phone' in text:
            return 'left'
        if 'left' in text:
            return 'left'
        if 'right' in text:
            return 'right'
        if 'forward' in text or 'straight' in text:
            return 'forward'
        return None

    def destroy_node(self):
        if self.webcam_capture is not None:
            self.webcam_capture.release()
        if self.show_debug_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
