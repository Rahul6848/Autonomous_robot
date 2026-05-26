"""
mission_commander.py
─────────────────────
Interactive terminal node – type natural-language commands and
they are published on /robot_command for the mission_controller to consume.

Usage:
    ros2 run my_robot_controller mission_commander

Supported commands (case-insensitive, partial matches work):
    go check B      → robot moves from A to B
    park at a/b/c/d → start parking at the requested lot pair
    go / move       → same as above
    stop / halt     → robot stops immediately
    start / resume  → continue a paused mission from the stopped position
    return / idle   → return to idle pose in parking mode, or return to A in road mode
    status          → print last /mission_status message
    quit / exit     → exit the commander
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

BANNER = """
╔══════════════════════════════════════════════════════╗
║           ROBOT MISSION COMMANDER                   ║
╠══════════════════════════════════════════════════════╣
║  Commands:                                          ║
║   • go check B  – send robot to Point B             ║
║   • park at A/B/C/D – park in requested zone        ║
║   • start / resume – continue paused mission        ║
║   • return / idle – go back to idle or home         ║
║   • stop        – halt robot immediately            ║
║   • status      – print current mission status      ║
║   • quit        – exit commander                    ║
╚══════════════════════════════════════════════════════╝
"""


class MissionCommander(Node):
    def __init__(self) -> None:
        super().__init__('mission_commander')
        self.pub = self.create_publisher(String, '/robot_command', 10)
        self.create_subscription(String, '/mission_status', self._status_cb, 10)
        self.last_status = '(no status received yet)'

    def _status_cb(self, msg: String) -> None:
        self.last_status = msg.data
        print(f'\n  [STATUS] {msg.data}\n> ', end='', flush=True)

    def send(self, text: str) -> None:
        msg      = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f'Published command: "{text}"')


def input_loop(node: MissionCommander) -> None:
    print(BANNER)
    while rclpy.ok():
        try:
            raw = input('> ').strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not raw:
            continue

        lower = raw.lower()

        if lower in ('quit', 'exit', 'q'):
            print('Exiting commander.')
            break
        elif lower == 'status':
            print(f'  Last status: {node.last_status}')
        else:
            node.send(raw)

    rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionCommander()

    # Spin ROS in background thread so input() doesn't block it
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    input_loop(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
