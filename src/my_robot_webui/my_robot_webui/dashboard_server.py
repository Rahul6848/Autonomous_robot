import argparse
import json
import os
import re
import shlex
import signal
import subprocess
import threading
import time
import urllib.error
import urllib.request
import webbrowser
from collections import deque
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Deque, Dict, List, Optional
from urllib.parse import urlparse

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import cv2
    from cv_bridge import CvBridge

    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

DEFAULT_HOST = '0.0.0.0'
DEFAULT_PORT = 8080
DEFAULT_VNC_PORT = 5901
DEFAULT_NOVNC_PORT = 6080
DEFAULT_STACK_MODE = 'parking'
DEFAULT_COMMAND_MODEL = 'gpt-5.4-mini'
DEFAULT_COMMAND_API_URL = 'https://api.openai.com/v1/responses'
DEFAULT_COMMAND_TIMEOUT_SEC = 12.0
LOCAL_ENV_FILENAME = '.my_robot_webui.env'
LOG_TAIL = 250
JPEG_QUALITY = 80
CAMERA_TOPIC = '/camera/image_raw'
TOP_VIEW_CAMERA_TOPIC = '/top_view_camera/image_raw'
COMMAND_TOPIC = '/robot_command'
STATUS_TOPIC = '/mission_status'
GUI_ENV_KEYS = [
    'DISPLAY',
    'WAYLAND_DISPLAY',
    'XAUTHORITY',
    'XDG_RUNTIME_DIR',
    'DBUS_SESSION_BUS_ADDRESS',
]

STACK_COMMANDS = {
    'road': ['ros2', 'launch', 'my_robot_bringup', 'my_robot_mission.launch.xml'],
    'parking': ['ros2', 'launch', 'my_robot_bringup', 'parking_mission.launch.xml'],
}

ALLOWED_ROBOT_COMMANDS = {
    'road': ['resume', 'stop', 'return', 'go check B'],
    'parking': ['resume', 'stop', 'return', 'park at A', 'park at B', 'park at C', 'park at D', 'park route A B C D'],
}


def resolve_static_dir() -> Path:
    module_static_dir = Path(__file__).resolve().parent / 'static'
    if module_static_dir.is_dir():
        return module_static_dir

    try:
        share_static_dir = Path(get_package_share_directory('my_robot_webui')) / 'static'
    except PackageNotFoundError:
        return module_static_dir

    if share_static_dir.is_dir():
        return share_static_dir
    return module_static_dir


STATIC_DIR = resolve_static_dir()


def find_workspace_root() -> Path:
    current = Path(__file__).resolve()
    for parent in [current] + list(current.parents):
        if (parent / 'src').is_dir() and (parent / 'install').is_dir():
            return parent
    return Path.cwd()


WORKSPACE_ROOT = find_workspace_root()


def load_local_env_file() -> str:
    env_file = WORKSPACE_ROOT / LOCAL_ENV_FILENAME
    if not env_file.is_file():
        return ''

    try:
        for raw_line in env_file.read_text(encoding='utf-8').splitlines():
            line = raw_line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, value = line.split('=', 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key and key not in os.environ:
                os.environ[key] = value
    except OSError:
        return ''

    return str(env_file)


@dataclass
class CommandResult:
    ok: bool
    message: str


@dataclass
class CommandResolution:
    ok: bool
    command: str
    message: str


class ManagedProcess:
    def __init__(self, name: str) -> None:
        self.name = name
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._logs: Deque[str] = deque(maxlen=LOG_TAIL)
        self._started_at = 0.0
        self._command: List[str] = []
        self._last_error = ''
        self._mode = ''

    def is_running(self) -> bool:
        with self._lock:
            return self._process is not None and self._process.poll() is None

    def mode(self) -> str:
        with self._lock:
            return self._mode

    def start(self, command: List[str], mode: str = '', env_overrides: Optional[Dict[str, str]] = None) -> CommandResult:
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                return CommandResult(False, f'{self.name} is already running.')

            env = os.environ.copy()
            env.setdefault('RCUTILS_COLORIZED_OUTPUT', '1')
            if env_overrides:
                env.update(env_overrides)

            try:
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    text=True,
                    bufsize=1,
                    env=env,
                    preexec_fn=os.setsid,
                )
            except FileNotFoundError as exc:
                self._last_error = str(exc)
                return CommandResult(False, f'Failed to start {self.name}: {exc}')

            self._process = process
            self._started_at = time.time()
            self._command = command
            self._mode = mode
            self._last_error = ''
            self._logs.clear()
            self._logs.append(f'$ {" ".join(shlex.quote(part) for part in command)}')
            self._reader_thread = threading.Thread(
                target=self._capture_output,
                args=(process,),
                daemon=True,
                name=f'{self.name}-log-reader',
            )
            self._reader_thread.start()
            return CommandResult(True, f'Started {self.name}.')

    def stop(self) -> CommandResult:
        with self._lock:
            process = self._process

        if process is None or process.poll() is not None:
            return CommandResult(True, f'{self.name} is not running.')

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=3)
        finally:
            with self._lock:
                self._process = None
                self._mode = ''
        return CommandResult(True, f'Stopped {self.name}.')

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            process = self._process
            command = list(self._command)
            started_at = self._started_at
            mode = self._mode
            last_error = self._last_error
            logs = list(self._logs)

        running = process is not None and process.poll() is None
        exit_code = None if running or process is None else process.poll()
        return {
            'name': self.name,
            'running': running,
            'mode': mode,
            'command': command,
            'started_at': started_at,
            'uptime_sec': max(0.0, time.time() - started_at) if started_at else 0.0,
            'exit_code': exit_code,
            'last_error': last_error,
            'logs': logs,
        }

    def _capture_output(self, process: subprocess.Popen) -> None:
        if process.stdout is None:
            return

        for line in process.stdout:
            cleaned = line.rstrip()
            if not cleaned:
                continue
            with self._lock:
                self._logs.append(cleaned)

        with self._lock:
            if process.poll() not in (None, 0):
                self._last_error = f'{self.name} exited with code {process.poll()}.'
            if self._process is process and process.poll() is not None:
                self._process = None
                self._mode = ''


class CommandInterpreter:
    def __init__(self) -> None:
        self.api_key = os.environ.get('OPENAI_API_KEY', '').strip()
        self.model = os.environ.get('OPENAI_COMMAND_MODEL', DEFAULT_COMMAND_MODEL).strip() or DEFAULT_COMMAND_MODEL
        self.api_url = os.environ.get('OPENAI_RESPONSES_URL', DEFAULT_COMMAND_API_URL).strip() or DEFAULT_COMMAND_API_URL
        timeout_raw = os.environ.get('OPENAI_COMMAND_TIMEOUT_SEC', str(DEFAULT_COMMAND_TIMEOUT_SEC)).strip()
        try:
            self.timeout_sec = max(3.0, float(timeout_raw))
        except ValueError:
            self.timeout_sec = DEFAULT_COMMAND_TIMEOUT_SEC

    def resolve(self, raw_text: str, stack_mode: str, mission_status: str) -> CommandResolution:
        user_text = ' '.join(raw_text.split()).strip()
        if not user_text:
            return CommandResolution(False, '', 'Command is empty.')

        mode = stack_mode if stack_mode in ALLOWED_ROBOT_COMMANDS else DEFAULT_STACK_MODE
        direct_command = self._normalize_direct_command(user_text, mode)
        if direct_command:
            return CommandResolution(True, direct_command, self._success_message(direct_command, user_text))

        if not self.api_key:
            if mode == 'parking':
                supported = 'resume, stop, return, park at A/B/C/D, or park route A B C D'
            else:
                supported = ', '.join(ALLOWED_ROBOT_COMMANDS.get(mode, []))
            return CommandResolution(
                False,
                '',
                f'Natural-language command routing needs OPENAI_API_KEY. Supported direct commands for {mode} mode: {supported}.',
            )

        llm_result = self._resolve_with_openai(user_text, mode, mission_status)
        if not llm_result.ok:
            return llm_result

        normalized = self._normalize_direct_command(llm_result.command, mode)
        if not normalized:
            return CommandResolution(False, '', 'The LLM returned an unsupported command. Please try again.')

        return CommandResolution(True, normalized, self._success_message(normalized, user_text))

    def _normalize_direct_command(self, text: str, mode: str) -> str:
        compact = re.sub(r'[^a-z0-9]+', ' ', text.lower()).strip()
        if not compact:
            return ''

        if mode == 'parking':
            route_targets = self._extract_parking_targets(compact)
            if route_targets:
                return self._format_parking_command(route_targets)

        if mode == 'road' and any(
            phrase in compact for phrase in ('go check b', 'go to b', 'move to b', 'check b', 'point b')
        ):
            return 'go check B'

        if any(phrase in compact for phrase in ('return', 'go back', 'come back', 'go home', 'home', 'idle')):
            return 'return'

        if any(phrase in compact for phrase in ('stop', 'halt', 'pause')):
            return 'stop'

        if any(phrase in compact for phrase in ('resume', 'continue', 'continue mission', 'start again', 'restart motion')):
            return 'resume'

        if compact == 'start':
            return 'resume'

        allowed = ALLOWED_ROBOT_COMMANDS.get(mode, [])
        for command in allowed:
            if compact == re.sub(r'[^a-z0-9]+', ' ', command.lower()).strip():
                return command

        return ''

    def _extract_parking_target(self, compact: str) -> str:
        targets = self._extract_parking_targets(compact)
        return targets[0] if targets else ''

    def _extract_parking_targets(self, compact: str) -> List[str]:
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
        if len(matches) == 1:
            patterns = (
                r'\bpark(?:ing)?(?: at| in| to)?\s*([abcd])\b',
                r'\bslot\s*([abcd])\b',
                r'\bpoint\s*([abcd])\b',
                r'\bzone\s*([abcd])\b',
            )
            for pattern in patterns:
                match = re.search(pattern, compact)
                if match:
                    return [match.group(1)]
            if not has_parking_context:
                return []

        route_targets: List[str] = []
        for target in matches:
            if target not in route_targets:
                route_targets.append(target)
        return route_targets

    def _format_parking_command(self, route_targets: List[str]) -> str:
        if len(route_targets) == 1:
            return f'park at {route_targets[0].upper()}'
        return 'park route ' + ' '.join(target.upper() for target in route_targets)

    def _resolve_with_openai(self, user_text: str, mode: str, mission_status: str) -> CommandResolution:
        if mode == 'parking':
            schema = {
                'type': 'object',
                'additionalProperties': False,
                'properties': {
                    'command': {
                        'type': 'string',
                    },
                    'reply': {
                        'type': 'string',
                    },
                },
                'required': ['command', 'reply'],
            }
            instructions = (
                'You translate robot operator instructions into exactly one safe robot command. '
                'The current mission mode is "parking". '
                'Valid outputs are: "resume", "stop", "return", "park at A", "park at B", '
                '"park at C", "park at D", or a multi-point route in the exact format '
                '"park route A B C D" using one or more unique points in the requested order. '
                'If the user requests all points, return "park route A B C D". '
                'If the request is ambiguous or unsupported, return command="clarify". '
                'If the user wants to continue after a stop or pause, choose "resume". '
                'Do not invent any other command wording.'
            )
        else:
            allowed_commands = ALLOWED_ROBOT_COMMANDS.get(mode, [])
            schema = {
                'type': 'object',
                'additionalProperties': False,
                'properties': {
                    'command': {
                        'type': 'string',
                        'enum': allowed_commands + ['clarify'],
                    },
                    'reply': {
                        'type': 'string',
                    },
                },
                'required': ['command', 'reply'],
            }
            instructions = (
                'You translate robot operator instructions into exactly one safe robot command. '
                f'The current mission mode is "{mode}". '
                f'The only valid commands are: {", ".join(allowed_commands)}. '
                'If the request is ambiguous, asks for unsupported behavior, or does not match the current mission mode, '
                'return command="clarify". '
                'If the user wants to continue after a stop or pause, choose "resume". '
                'Do not invent new commands.'
            )

        payload = {
            'model': self.model,
            'instructions': instructions,
            'input': [
                {
                    'role': 'user',
                    'content': (
                        f'Current mission status: {mission_status or "unknown"}\n'
                        f'Operator request: {user_text}'
                    ),
                }
            ],
            'text': {
                'format': {
                    'type': 'json_schema',
                    'name': 'robot_command_router',
                    'strict': True,
                    'schema': schema,
                }
            },
        }

        request = urllib.request.Request(
            self.api_url,
            data=json.dumps(payload).encode('utf-8'),
            headers={
                'Authorization': f'Bearer {self.api_key}',
                'Content-Type': 'application/json',
            },
            method='POST',
        )

        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                response_payload = json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            error_body = exc.read().decode('utf-8', errors='replace')
            return CommandResolution(False, '', f'OpenAI request failed ({exc.code}): {error_body}')
        except Exception as exc:
            return CommandResolution(False, '', f'OpenAI request failed: {exc}')

        output_text = self._extract_output_text(response_payload)
        if not output_text:
            return CommandResolution(False, '', 'OpenAI did not return a command.')

        try:
            parsed = json.loads(output_text)
        except json.JSONDecodeError:
            return CommandResolution(False, '', f'OpenAI returned invalid JSON: {output_text}')

        command = str(parsed.get('command', '')).strip()
        reply = str(parsed.get('reply', '')).strip() or 'Please clarify your command.'
        if command == 'clarify':
            return CommandResolution(False, '', reply)
        return CommandResolution(True, command, reply)

    def _extract_output_text(self, payload: Dict[str, object]) -> str:
        for item in payload.get('output', []):
            if not isinstance(item, dict) or item.get('type') != 'message':
                continue
            for content in item.get('content', []):
                if isinstance(content, dict) and content.get('type') == 'output_text':
                    return str(content.get('text', ''))
        return ''

    def _success_message(self, command: str, raw_text: str) -> str:
        normalized_raw = re.sub(r'\s+', ' ', raw_text).strip()
        if normalized_raw.lower() == command.lower():
            return f'Sent command: {command}'
        return f'Interpreted "{normalized_raw}" as "{command}" and sent it.'


class DesktopStreamManager:
    def __init__(self, vnc_port: int, novnc_port: int) -> None:
        self.vnc_port = vnc_port
        self.novnc_port = novnc_port
        self.vnc = ManagedProcess('x11vnc')
        self.novnc = ManagedProcess('noVNC')

    def start(self) -> CommandResult:
        display = os.environ.get('DISPLAY')
        if not display:
            return CommandResult(False, 'DISPLAY is not set. Start the dashboard from a desktop session.')
        x11vnc_path = resolve_tool_path('x11vnc')
        if not x11vnc_path:
            return CommandResult(False, 'x11vnc is not installed.')
        novnc_command = build_novnc_command(self.novnc_port, self.vnc_port)
        if not novnc_command:
            return CommandResult(False, 'noVNC tools are not installed.')

        vnc_result = self.vnc.start(
            [
                x11vnc_path,
                '-display',
                display,
                '-auth',
                'guess',
                '-nopw',
                '-forever',
                '-shared',
                '-noxdamage',
                '-rfbport',
                str(self.vnc_port),
            ]
        )
        if not vnc_result.ok and not self.vnc.is_running():
            return vnc_result

        time.sleep(0.8)
        if not self.vnc.is_running():
            vnc_snapshot = self.vnc.snapshot()
            recent_logs = '\n'.join(vnc_snapshot.get('logs', [])[-8:])
            details = f' x11vnc logs:\n{recent_logs}' if recent_logs else ''
            return CommandResult(False, f'x11vnc failed to stay running.{details}')

        novnc_result = self.novnc.start(
            novnc_command
        )
        if not novnc_result.ok and not self.novnc.is_running():
            return novnc_result

        return CommandResult(True, 'Desktop streaming is ready.')

    def stop(self) -> CommandResult:
        novnc_result = self.novnc.stop()
        vnc_result = self.vnc.stop()
        if not novnc_result.ok:
            return novnc_result
        return vnc_result

    def snapshot(self, host: str) -> Dict[str, object]:
        hostname = host.split(':', 1)[0] or 'localhost'
        novnc_url = f'http://{hostname}:{self.novnc_port}/vnc.html?autoconnect=true&resize=scale'
        return {
            'display': os.environ.get('DISPLAY', ''),
            'display_available': bool(os.environ.get('DISPLAY')),
            'session_type': os.environ.get('XDG_SESSION_TYPE', ''),
            'x11vnc_installed': bool(resolve_tool_path('x11vnc')),
            'novnc_installed': novnc_available(),
            'vnc': self.vnc.snapshot(),
            'novnc': self.novnc.snapshot(),
            'novnc_url': novnc_url,
        }


class DashboardRosBridge(Node):
    def __init__(self) -> None:
        super().__init__('my_robot_web_dashboard')
        self._lock = threading.Lock()
        self._bridge = CvBridge() if CV_AVAILABLE else None
        self._camera_frame: Optional[bytes] = None
        self._camera_updated_at = 0.0
        self._top_camera_frame: Optional[bytes] = None
        self._top_camera_updated_at = 0.0
        self._last_status = 'Waiting for mission status...'
        self._status_updated_at = 0.0
        self._status_sequence = 0
        self._last_command = ''
        self._command_updated_at = 0.0
        self._status_history: Deque[str] = deque(maxlen=40)
        self._status_events: Deque[Dict[str, object]] = deque(maxlen=40)

        self.command_pub = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.create_subscription(String, STATUS_TOPIC, self._status_callback, 10)
        self.create_subscription(Image, CAMERA_TOPIC, self._image_callback, 10)
        self.create_subscription(Image, TOP_VIEW_CAMERA_TOPIC, self._top_image_callback, 10)

    def send_command(self, text: str) -> CommandResult:
        command = text.strip()
        if not command:
            return CommandResult(False, 'Command is empty.')

        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        with self._lock:
            self._last_command = command
            self._command_updated_at = time.time()
        self.get_logger().info(f'Published dashboard command: "{command}"')
        return CommandResult(True, f'Sent command: {command}')

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            camera_age = time.time() - self._camera_updated_at if self._camera_updated_at else None
            top_camera_age = time.time() - self._top_camera_updated_at if self._top_camera_updated_at else None
            return {
                'last_status': self._last_status,
                'status_updated_at': self._status_updated_at,
                'last_command': self._last_command,
                'command_updated_at': self._command_updated_at,
                'camera_available': self._camera_frame is not None and camera_age is not None and camera_age < 2.5,
                'camera_updated_at': self._camera_updated_at,
                'top_camera_available': self._top_camera_frame is not None and top_camera_age is not None and top_camera_age < 2.5,
                'top_camera_updated_at': self._top_camera_updated_at,
                'status_sequence': self._status_sequence,
                'status_history': list(self._status_history),
                'status_events': list(self._status_events),
            }

    def camera_frame(self) -> Optional[bytes]:
        with self._lock:
            return self._camera_frame

    def top_camera_frame(self) -> Optional[bytes]:
        with self._lock:
            return self._top_camera_frame

    def _status_callback(self, msg: String) -> None:
        with self._lock:
            self._status_sequence += 1
            self._last_status = msg.data
            self._status_updated_at = time.time()
            self._status_history.appendleft(msg.data)
            self._status_events.appendleft(
                {
                    'seq': self._status_sequence,
                    'text': msg.data,
                }
            )

    def _image_callback(self, msg: Image) -> None:
        encoded = self._encode_image(msg)
        if encoded is None:
            return

        with self._lock:
            self._camera_frame = encoded
            self._camera_updated_at = time.time()

    def _top_image_callback(self, msg: Image) -> None:
        encoded = self._encode_image(msg)
        if encoded is None:
            return

        with self._lock:
            self._top_camera_frame = encoded
            self._top_camera_updated_at = time.time()

    def _encode_image(self, msg: Image) -> Optional[bytes]:
        if not CV_AVAILABLE or self._bridge is None:
            return None
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if not ok:
                return None
        except Exception as exc:
            self.get_logger().warning(f'Camera frame conversion failed: {exc}', throttle_duration_sec=2.0)
            return None
        return encoded.tobytes()


class DashboardApplication:
    def __init__(self, host: str, port: int, vnc_port: int, novnc_port: int, open_browser: bool, auto_stack_mode: str) -> None:
        self.host = host
        self.port = port
        self.open_browser = open_browser
        self.auto_stack_mode = auto_stack_mode
        self.bridge = DashboardRosBridge()
        self.command_interpreter = CommandInterpreter()
        self.stack = ManagedProcess('robot stack')
        self.desktop = DesktopStreamManager(vnc_port=vnc_port, novnc_port=novnc_port)
        self.server: Optional[ThreadingHTTPServer] = None
        self._auto_launch_result = CommandResult(False, 'Automatic mission launch has not started yet.')
        self._auto_launch_attempted = False
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True, name='ros-spin')
        self.ros_thread.start()

    def start_stack(self, mode: str) -> CommandResult:
        launch_command = STACK_COMMANDS.get(mode)
        if launch_command is None:
            return CommandResult(False, f'Unsupported mode: {mode}')

        if not os.environ.get('DISPLAY'):
            return CommandResult(
                False,
                'DISPLAY is not set for the dashboard process. Start the dashboard from your Linux desktop session so Gazebo and RViz can open as native windows.',
            )

        launch_wrapper = build_stack_command(launch_command)
        result = self.stack.start(
            launch_wrapper,
            mode=mode,
            env_overrides=gui_environment(),
        )
        if result.ok:
            self.desktop.start()
        return result

    def stop_stack(self) -> CommandResult:
        return self.stack.stop()

    def dispatch_command(self, raw_text: str) -> CommandResult:
        stack_snapshot = self.stack.snapshot()
        ros_snapshot = self.bridge.snapshot()
        active_mode = str(stack_snapshot.get('mode') or self.auto_stack_mode or DEFAULT_STACK_MODE)
        mission_status = str(ros_snapshot.get('last_status') or '')

        resolution = self.command_interpreter.resolve(raw_text, active_mode, mission_status)
        if not resolution.ok:
            return CommandResult(False, resolution.message)

        publish_result = self.bridge.send_command(resolution.command)
        if not publish_result.ok:
            return publish_result
        return CommandResult(True, resolution.message)

    def state_snapshot(self, host: str) -> Dict[str, object]:
        return {
            'stack': self.stack.snapshot(),
            'desktop': self.desktop.snapshot(host),
            'ros': self.bridge.snapshot(),
            'auto_launch': {
                'attempted': self._auto_launch_attempted,
                'mode': self.auto_stack_mode,
                'ok': self._auto_launch_result.ok,
                'message': self._auto_launch_result.message,
            },
            'available_modes': list(STACK_COMMANDS.keys()),
            'camera_stream_url': '/stream/camera.mjpg',
            'top_camera_stream_url': '/stream/top_camera.mjpg',
            'gui_env': gui_environment(),
        }

    def serve_forever(self) -> None:
        handler = self._handler_factory()
        self.server = ThreadingHTTPServer((self.host, self.port), handler)
        self._autostart_stack()
        self.bridge.get_logger().info(f'Dashboard listening on http://{self.host}:{self.port}')
        if self.open_browser:
            threading.Thread(target=self._open_browser_once, daemon=True).start()

        try:
            self.server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None
        self.desktop.stop()
        self.stack.stop()
        try:
            self.bridge.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    def _spin_ros(self) -> None:
        try:
            rclpy.spin(self.bridge)
        except ExternalShutdownException:
            pass

    def _open_browser_once(self) -> None:
        time.sleep(1.0)
        webbrowser.open(f'http://localhost:{self.port}')

    def _autostart_stack(self) -> None:
        self._auto_launch_attempted = True
        self._auto_launch_result = self.start_stack(self.auto_stack_mode)
        if self._auto_launch_result.ok:
            self.bridge.get_logger().info(
                f'Auto-started dashboard stack in "{self.auto_stack_mode}" mode.'
            )
            return

        self.bridge.get_logger().error(
            f'Automatic dashboard stack launch failed: {self._auto_launch_result.message}'
        )

    def _handler_factory(self):
        app = self

        class DashboardRequestHandler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                parsed = urlparse(self.path)
                if parsed.path == '/':
                    self._serve_file('index.html', 'text/html; charset=utf-8')
                    return
                if parsed.path == '/assets/style.css':
                    self._serve_file('style.css', 'text/css; charset=utf-8')
                    return
                if parsed.path == '/assets/app.js':
                    self._serve_file('app.js', 'application/javascript; charset=utf-8')
                    return
                if parsed.path == '/api/state':
                    self._send_json(app.state_snapshot(self.headers.get('Host', 'localhost')))
                    return
                if parsed.path == '/stream/camera.mjpg':
                    self._stream_camera()
                    return
                if parsed.path == '/stream/top_camera.mjpg':
                    self._stream_top_camera()
                    return
                self.send_error(HTTPStatus.NOT_FOUND, 'Not found')

            def do_POST(self) -> None:
                parsed = urlparse(self.path)
                body = self._read_json_body()

                if parsed.path == '/api/stack/start':
                    result = app.start_stack(str(body.get('mode', '')))
                    self._send_json({'ok': result.ok, 'message': result.message}, status=HTTPStatus.OK if result.ok else HTTPStatus.BAD_REQUEST)
                    return
                if parsed.path == '/api/stack/stop':
                    result = app.stop_stack()
                    self._send_json({'ok': result.ok, 'message': result.message}, status=HTTPStatus.OK if result.ok else HTTPStatus.BAD_REQUEST)
                    return
                if parsed.path == '/api/desktop/start':
                    result = app.desktop.start()
                    self._send_json({'ok': result.ok, 'message': result.message}, status=HTTPStatus.OK if result.ok else HTTPStatus.BAD_REQUEST)
                    return
                if parsed.path == '/api/desktop/stop':
                    result = app.desktop.stop()
                    self._send_json({'ok': result.ok, 'message': result.message}, status=HTTPStatus.OK if result.ok else HTTPStatus.BAD_REQUEST)
                    return
                if parsed.path == '/api/command':
                    result = app.dispatch_command(str(body.get('command', '')))
                    self._send_json({'ok': result.ok, 'message': result.message}, status=HTTPStatus.OK if result.ok else HTTPStatus.BAD_REQUEST)
                    return
                self.send_error(HTTPStatus.NOT_FOUND, 'Not found')

            def log_message(self, format: str, *args) -> None:
                return

            def _read_json_body(self) -> Dict[str, object]:
                length = int(self.headers.get('Content-Length', '0'))
                if length <= 0:
                    return {}
                raw = self.rfile.read(length)
                if not raw:
                    return {}
                try:
                    return json.loads(raw.decode('utf-8'))
                except json.JSONDecodeError:
                    return {}

            def _serve_file(self, filename: str, content_type: str) -> None:
                file_path = STATIC_DIR / filename
                if not file_path.exists():
                    self.send_error(HTTPStatus.NOT_FOUND, 'Missing asset')
                    return
                content = file_path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(content)))
                self.end_headers()
                self.wfile.write(content)

            def _send_json(self, payload: Dict[str, object], status: HTTPStatus = HTTPStatus.OK) -> None:
                encoded = json.dumps(payload).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def _stream_camera(self) -> None:
                self._stream_frames(app.bridge.camera_frame)

            def _stream_top_camera(self) -> None:
                self._stream_frames(app.bridge.top_camera_frame)

            def _stream_frames(self, frame_supplier) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header('Age', '0')
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()

                while True:
                    frame = frame_supplier()
                    if frame is None:
                        time.sleep(0.2)
                        continue

                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode('ascii'))
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                        time.sleep(0.12)
                    except (BrokenPipeError, ConnectionResetError):
                        break

        return DashboardRequestHandler


TOOL_FALLBACK_PATHS = {
    'novnc_proxy': [
        '/usr/share/novnc/utils/novnc_proxy',
        '/usr/lib/novnc/utils/novnc_proxy',
    ],
    'novnc_web_root': [
        '/usr/share/novnc',
    ],
}


def shutil_which(command: str) -> str:
    for path in os.environ.get('PATH', '').split(os.pathsep):
        candidate = Path(path) / command
        if candidate.exists() and os.access(candidate, os.X_OK):
            return str(candidate)
    return ''


def resolve_tool_path(command: str) -> str:
    resolved = shutil_which(command)
    if resolved:
        return resolved

    for raw_path in TOOL_FALLBACK_PATHS.get(command, []):
        candidate = Path(raw_path)
        if candidate.exists() and os.access(candidate, os.X_OK):
            return str(candidate)
    return ''


def resolve_directory_path(name: str) -> str:
    for raw_path in TOOL_FALLBACK_PATHS.get(name, []):
        candidate = Path(raw_path)
        if candidate.is_dir():
            return str(candidate)
    return ''


def novnc_available() -> bool:
    if resolve_tool_path('novnc_proxy'):
        return True
    return bool(resolve_tool_path('websockify') and resolve_directory_path('novnc_web_root'))


def build_novnc_command(novnc_port: int, vnc_port: int) -> List[str]:
    novnc_proxy_path = resolve_tool_path('novnc_proxy')
    if novnc_proxy_path:
        return [
            novnc_proxy_path,
            '--listen',
            str(novnc_port),
            '--vnc',
            f'localhost:{vnc_port}',
        ]

    websockify_path = resolve_tool_path('websockify')
    web_root = resolve_directory_path('novnc_web_root')
    if websockify_path and web_root:
        return [
            websockify_path,
            '--web',
            web_root,
            str(novnc_port),
            f'localhost:{vnc_port}',
        ]

    return []


def gui_environment() -> Dict[str, str]:
    env: Dict[str, str] = {}
    for key in GUI_ENV_KEYS:
        value = os.environ.get(key)
        if value:
            env[key] = value
    if env.get('DISPLAY'):
        env.setdefault('QT_QPA_PLATFORM', 'xcb')
    return env


def find_workspace_setup_bash() -> str:
    current = Path(__file__).resolve()
    for parent in [current] + list(current.parents):
        candidate = parent / 'setup.bash'
        if candidate.exists():
            return str(candidate)
        sibling = parent / 'install' / 'setup.bash'
        if sibling.exists():
            return str(sibling)
    return ''


def build_stack_command(launch_command: List[str]) -> List[str]:
    setup_bash = find_workspace_setup_bash()
    env_exports = []
    for key, value in gui_environment().items():
        env_exports.append(f'export {key}={shlex.quote(value)}')

    shell_parts = []
    if setup_bash:
        shell_parts.append(f'source {shlex.quote(setup_bash)}')
    shell_parts.extend(env_exports)
    shell_parts.append(' '.join(shlex.quote(part) for part in launch_command))
    shell_command = ' && '.join(shell_parts)
    return ['bash', '-lc', shell_command]


def parse_args(args: Optional[List[str]] = None) -> tuple[argparse.Namespace, List[str]]:
    parser = argparse.ArgumentParser(description='Start the my_robot web dashboard.')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--vnc-port', type=int, default=DEFAULT_VNC_PORT)
    parser.add_argument('--novnc-port', type=int, default=DEFAULT_NOVNC_PORT)
    parser.add_argument('--stack-mode', choices=sorted(STACK_COMMANDS.keys()), default=DEFAULT_STACK_MODE)
    parser.add_argument('--no-browser', action='store_true', help='Do not auto-open the dashboard in a browser.')
    return parser.parse_known_args(args=args)


def main(args: Optional[List[str]] = None) -> None:
    cli_args, ros_args = parse_args(args=args)
    loaded_env_file = load_local_env_file()
    rclpy.init(args=ros_args)
    app = DashboardApplication(
        host=cli_args.host,
        port=cli_args.port,
        vnc_port=cli_args.vnc_port,
        novnc_port=cli_args.novnc_port,
        open_browser=not cli_args.no_browser,
        auto_stack_mode=cli_args.stack_mode,
    )
    if loaded_env_file:
        app.bridge.get_logger().info(f'Loaded dashboard environment from {loaded_env_file}')
    app.serve_forever()


if __name__ == '__main__':
    main()
