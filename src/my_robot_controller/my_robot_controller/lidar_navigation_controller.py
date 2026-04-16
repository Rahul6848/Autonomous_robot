import math
import time
from typing import Iterable, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarNavigationController(Node):
    def __init__(self) -> None:
        super().__init__('lidar_navigation_controller')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('forward_speed', 0.18)
        self.declare_parameter('slow_speed', 0.08)
        self.declare_parameter('turn_speed', 1.0)
        self.declare_parameter('scan_max_range', 5.0)
        self.declare_parameter('safe_side_distance', 0.6)
        self.declare_parameter('critical_side_distance', 0.32)
        self.declare_parameter('front_safe_distance', 1.2)
        self.declare_parameter('critical_front_distance', 0.45)
        self.declare_parameter('front_sector_deg', 30.0)
        self.declare_parameter('front_side_sector_deg', 70.0)
        self.declare_parameter('side_sector_start_deg', 55.0)
        self.declare_parameter('side_sector_end_deg', 110.0)
        self.declare_parameter('left_turn_bias', 0.2)
        self.declare_parameter('clear_path_threshold', 1.5)
        self.declare_parameter('self_filter_distance', 0.22)
        self.declare_parameter('min_points_per_sector', 8)
        self.declare_parameter('startup_hold_sec', 1.5)
        self.declare_parameter('corridor_width_threshold', 0.9)
        self.declare_parameter('corridor_center_gain', 1.4)
        self.declare_parameter('junction_opening_threshold', 3.0)
        self.declare_parameter('turn_commit_sec', 1.8)
        self.declare_parameter('committed_turn_speed', 0.75)
        self.declare_parameter('committed_turn_linear_speed', 0.12)
        self.declare_parameter('corridor_confirm_frames', 4)
        self.declare_parameter('opening_confirm_frames', 4)
        self.declare_parameter('junction_arm_x', 2.5)
        self.declare_parameter('junction_arm_y', 0.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.slow_speed = float(self.get_parameter('slow_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.scan_max_range = float(self.get_parameter('scan_max_range').value)
        self.safe_side_distance = float(
            self.get_parameter('safe_side_distance').value
        )
        self.critical_side_distance = float(
            self.get_parameter('critical_side_distance').value
        )
        self.front_safe_distance = float(
            self.get_parameter('front_safe_distance').value
        )
        self.critical_front_distance = float(
            self.get_parameter('critical_front_distance').value
        )
        self.front_sector_deg = float(self.get_parameter('front_sector_deg').value)
        self.front_side_sector_deg = float(
            self.get_parameter('front_side_sector_deg').value
        )
        self.side_sector_start_deg = float(
            self.get_parameter('side_sector_start_deg').value
        )
        self.side_sector_end_deg = float(
            self.get_parameter('side_sector_end_deg').value
        )
        self.left_turn_bias = float(self.get_parameter('left_turn_bias').value)
        self.clear_path_threshold = float(
            self.get_parameter('clear_path_threshold').value
        )
        self.self_filter_distance = float(
            self.get_parameter('self_filter_distance').value
        )
        self.min_points_per_sector = int(
            self.get_parameter('min_points_per_sector').value
        )
        self.startup_hold_sec = float(self.get_parameter('startup_hold_sec').value)
        self.corridor_width_threshold = float(
            self.get_parameter('corridor_width_threshold').value
        )
        self.corridor_center_gain = float(
            self.get_parameter('corridor_center_gain').value
        )
        self.junction_opening_threshold = float(
            self.get_parameter('junction_opening_threshold').value
        )
        self.turn_commit_sec = float(self.get_parameter('turn_commit_sec').value)
        self.committed_turn_speed = float(
            self.get_parameter('committed_turn_speed').value
        )
        self.committed_turn_linear_speed = float(
            self.get_parameter('committed_turn_linear_speed').value
        )
        self.corridor_confirm_frames = int(
            self.get_parameter('corridor_confirm_frames').value
        )
        self.opening_confirm_frames = int(
            self.get_parameter('opening_confirm_frames').value
        )
        self.junction_arm_x = float(self.get_parameter('junction_arm_x').value)
        self.junction_arm_y = float(self.get_parameter('junction_arm_y').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1.0),
            self.control_loop,
        )

        self.latest_scan: Optional[LaserScan] = None
        self.last_state = 'waiting_for_scan'
        self.started_at = time.time()
        self.turn_commit_until = 0.0
        self.turn_commit_direction: Optional[str] = None
        self.corridor_stable_count = 0
        self.left_opening_count = 0
        self.robot_x: Optional[float] = None
        self.robot_y: Optional[float] = None

        self.get_logger().info(f'LiDAR navigation controller listening on {self.scan_topic}')

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)

    def control_loop(self) -> None:
        cmd = Twist()
        now = time.time()

        if self.latest_scan is None:
            self.cmd_pub.publish(cmd)
            return

        if now - self.started_at < self.startup_hold_sec:
            if self.last_state != 'startup_hold':
                self.get_logger().info('state=startup_hold waiting for stable LiDAR data')
                self.last_state = 'startup_hold'
            self.cmd_pub.publish(cmd)
            return

        front_ranges = self.get_sector_ranges(self.latest_scan, -self.front_sector_deg, self.front_sector_deg)
        front_left_ranges = self.get_sector_ranges(
            self.latest_scan, 10.0, self.front_side_sector_deg
        )
        front_right_ranges = self.get_sector_ranges(
            self.latest_scan, -self.front_side_sector_deg, -10.0
        )
        left_ranges = self.get_sector_ranges(
            self.latest_scan, self.side_sector_start_deg, self.side_sector_end_deg
        )
        right_ranges = self.get_sector_ranges(
            self.latest_scan, -self.side_sector_end_deg, -self.side_sector_start_deg
        )

        front_min = self.safe_min(front_ranges)
        front_left_min = self.safe_min(front_left_ranges)
        front_right_min = self.safe_min(front_right_ranges)
        left_min = self.safe_min(left_ranges)
        right_min = self.safe_min(right_ranges)
        front_valid = len(front_ranges) >= self.min_points_per_sector
        front_left_valid = len(front_left_ranges) >= self.min_points_per_sector
        front_right_valid = len(front_right_ranges) >= self.min_points_per_sector
        left_valid = len(left_ranges) >= self.min_points_per_sector
        right_valid = len(right_ranges) >= self.min_points_per_sector

        front_score = self.clearance_score(front_ranges)
        front_left_score = self.clearance_score(front_left_ranges)
        front_right_score = self.clearance_score(front_right_ranges)
        left_score = self.clearance_score(left_ranges)
        right_score = self.clearance_score(right_ranges)
        corridor_width = left_min + right_min
        junction_armed = (
            self.robot_x is not None
            and self.robot_y is not None
            and self.robot_x >= self.junction_arm_x
            and self.robot_y >= self.junction_arm_y
        )
        left_opening_detected = (
            junction_armed
            and
            left_score >= self.junction_opening_threshold
            and front_left_score >= self.junction_opening_threshold
            and right_min <= self.safe_side_distance
        )

        in_corridor = corridor_width < self.corridor_width_threshold
        if in_corridor:
            self.corridor_stable_count = min(
                self.corridor_stable_count + 1,
                self.corridor_confirm_frames + 2,
            )
            self.left_opening_count = 0
        elif left_opening_detected and self.corridor_stable_count >= self.corridor_confirm_frames:
            self.left_opening_count = min(
                self.left_opening_count + 1,
                self.opening_confirm_frames + 2,
            )
        else:
            self.left_opening_count = 0

        state = 'forward'
        left_bias_score = max(front_left_score, left_score) + self.left_turn_bias
        right_bias_score = max(front_right_score, right_score)
        left_turn_ready = (
            self.left_opening_count >= self.opening_confirm_frames
            and (
                front_min <= self.front_safe_distance * 1.15
                or front_left_score > front_score + 0.3
                or left_score > right_score + 0.5
            )
        )

        if not (front_valid and front_left_valid and front_right_valid and left_valid and right_valid):
            state = 'insufficient_scan_data'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.corridor_stable_count < self.corridor_confirm_frames:
            state = 'acquire_corridor'
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif now < self.turn_commit_until and self.turn_commit_direction is not None:
            state = f'commit_{self.turn_commit_direction}_turn'
            cmd.linear.x = self.committed_turn_linear_speed
            cmd.angular.z = (
                self.committed_turn_speed
                if self.turn_commit_direction == 'left'
                else -self.committed_turn_speed
            )
        elif front_min < self.critical_front_distance:
            if left_bias_score >= right_bias_score and left_min > self.critical_side_distance:
                state = 'critical_turn_left'
                cmd.linear.x = 0.02
                cmd.angular.z = self.turn_speed
            elif right_min > self.critical_side_distance:
                state = 'critical_turn_right'
                cmd.linear.x = 0.02
                cmd.angular.z = -self.turn_speed
            else:
                state = 'critical_stop'
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        else:
            angular = 0.0

            if left_min < self.critical_side_distance:
                angular -= 1.2
            elif left_min < self.safe_side_distance:
                angular -= 0.7 * (self.safe_side_distance - left_min) / self.safe_side_distance

            if right_min < self.critical_side_distance:
                angular += 1.2
            elif right_min < self.safe_side_distance:
                angular += 0.7 * (self.safe_side_distance - right_min) / self.safe_side_distance

            if left_turn_ready:
                state = 'left_junction_detected'
                self.turn_commit_direction = 'left'
                self.turn_commit_until = now + self.turn_commit_sec
                cmd.linear.x = self.committed_turn_linear_speed
                cmd.angular.z = self.committed_turn_speed
            elif in_corridor:
                state = 'corridor_centering'
                angular += self.corridor_center_gain * (left_min - right_min)
                cmd.angular.z = max(-self.turn_speed, min(self.turn_speed, angular))
                cmd.linear.x = 0.14
            elif front_min < self.front_safe_distance:
                if left_bias_score >= right_bias_score:
                    state = 'turn_left_for_clearance'
                    angular += self.turn_speed * self.turn_strength(front_min, self.front_safe_distance)
                else:
                    state = 'turn_right_for_clearance'
                    angular -= self.turn_speed * self.turn_strength(front_min, self.front_safe_distance)
                cmd.angular.z = max(-self.turn_speed, min(self.turn_speed, angular))
            else:
                state = 'follow_open_space'
                angular += 0.4 * (left_score - right_score)
                cmd.angular.z = max(-self.turn_speed, min(self.turn_speed, angular))

            if state not in ('left_junction_detected', 'corridor_centering'):
                if front_score > self.clear_path_threshold and front_min > self.front_safe_distance:
                    cmd.linear.x = self.forward_speed
                elif front_min > self.critical_front_distance:
                    cmd.linear.x = self.slow_speed + 0.08 * min(front_score / self.scan_max_range, 1.0)
                else:
                    cmd.linear.x = 0.03

            if abs(cmd.angular.z) > 0.65:
                cmd.linear.x = min(cmd.linear.x, self.slow_speed)

        if state != self.last_state:
            self.get_logger().info(
                f'state={state} front={front_min:.2f} front_left={front_left_min:.2f} '
                f'front_right={front_right_min:.2f} left_min={left_min:.2f} '
                f'right_min={right_min:.2f} front_score={front_score:.2f} '
                f'left_score={left_score:.2f} right_score={right_score:.2f} '
                f'fl_score={front_left_score:.2f} fr_score={front_right_score:.2f} '
                f'corridor={corridor_width:.2f} '
                f'openL={left_opening_detected} corridorN={self.corridor_stable_count} '
                f'openLN={self.left_opening_count} turnReady={left_turn_ready} '
                f'valid={len(front_ranges)}/{len(front_left_ranges)}/{len(front_right_ranges)}/'
                f'{len(left_ranges)}/{len(right_ranges)} '
                f'cmd_v={cmd.linear.x:.2f} cmd_w={cmd.angular.z:.2f}'
            )
            self.last_state = state

        self.cmd_pub.publish(cmd)

    def get_sector_ranges(
        self,
        scan: LaserScan,
        min_deg: float,
        max_deg: float,
    ) -> List[float]:
        values: List[float] = []
        for index, distance in enumerate(scan.ranges):
            if math.isnan(distance):
                continue

            angle = scan.angle_min + index * scan.angle_increment
            angle_deg = math.degrees(angle)
            if not (min_deg <= angle_deg <= max_deg):
                continue

            if math.isinf(distance) or distance > scan.range_max:
                values.append(self.scan_max_range)
                continue

            if distance < max(scan.range_min, self.self_filter_distance):
                continue

            values.append(min(distance, self.scan_max_range))

        return values

    @staticmethod
    def safe_min(values: Iterable[float]) -> float:
        values = list(values)
        return min(values) if values else float('inf')

    @staticmethod
    def safe_mean(values: Iterable[float]) -> float:
        values = list(values)
        return sum(values) / len(values) if values else 5.0

    @staticmethod
    def turn_strength(distance: float, safe_distance: float) -> float:
        deficit = max(0.0, safe_distance - distance)
        return max(0.25, min(1.0, deficit / safe_distance))

    def clearance_score(self, values: Iterable[float]) -> float:
        values = list(values)
        if not values:
            return self.scan_max_range

        min_value = min(values)
        mean_value = sum(values) / len(values)
        return 0.65 * min_value + 0.35 * mean_value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarNavigationController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
