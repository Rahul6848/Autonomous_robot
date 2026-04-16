import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MoveRobotNode(Node):
    def __init__(self) -> None:
        super().__init__('move_robot_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        # Left-lane centerline for the current world geometry.
        self.path_points = [
            (-8.0, 0.75),
            (-5.0, 0.75),
            (-2.0, 0.75),
            (1.0, 0.75),
            (3.2, 0.75),
            (4.1, 0.9),
            (4.6, 1.35),
            (4.55, 2.4),
            (4.35, 4.0),
            (4.25, 6.0),
            (4.25, 8.2),
        ]
        self.current_waypoint_index = 0
        self.position = None
        self.yaw = 0.0
        self.goal_tolerance = 0.2
        self.lookahead_distance = 0.75
        self.max_linear_speed = 0.28
        self.max_angular_speed = 1.1
        self.get_logger().info('Following the left lane using odometry feedback.')

    def odom_callback(self, msg: Odometry) -> None:
        self.position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self) -> None:
        if self.position is None:
            return

        if self.current_waypoint_index >= len(self.path_points):
            self.stop_robot('Reached the end of the left lane path.')
            return

        robot_x = self.position.x
        robot_y = self.position.y

        while self.current_waypoint_index < len(self.path_points) - 1:
            target_x, target_y = self.path_points[self.current_waypoint_index]
            if math.hypot(target_x - robot_x, target_y - robot_y) > self.lookahead_distance:
                break
            self.current_waypoint_index += 1

        target_x, target_y = self.path_points[self.current_waypoint_index]
        distance = math.hypot(target_x - robot_x, target_y - robot_y)

        if (
            self.current_waypoint_index == len(self.path_points) - 1
            and distance < self.goal_tolerance
        ):
            self.stop_robot('Reached the final left-lane waypoint.')
            return

        heading_to_target = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_error = self.normalize_angle(heading_to_target - self.yaw)

        cmd = Twist()
        cmd.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, 1.8 * heading_error),
        )

        turn_scale = max(0.25, 1.0 - min(abs(heading_error), 1.2) / 1.2)
        cmd.linear.x = min(self.max_linear_speed, 0.12 + 0.18 * turn_scale)

        if abs(heading_error) > 0.9:
            cmd.linear.x = 0.08

        self.publisher_.publish(cmd)

    def stop_robot(self, message: str) -> None:
        self.publisher_.publish(Twist())
        self.get_logger().info(message)
        self.destroy_timer(self.timer_)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveRobotNode()

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
