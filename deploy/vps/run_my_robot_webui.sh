#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/deploy/ros2_workspace}"
WEBUI_HOST="${WEBUI_HOST:-127.0.0.1}"
WEBUI_PORT="${WEBUI_PORT:-8080}"
STACK_MODE="${STACK_MODE:-parking}"
XVFB_SCREEN="${XVFB_SCREEN:-1280x720x24}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi

if [[ ! -f "$WORKSPACE_ROOT/install/setup.bash" ]]; then
  echo "Workspace setup file not found: $WORKSPACE_ROOT/install/setup.bash" >&2
  exit 1
fi

cd "$WORKSPACE_ROOT"
source "$ROS_SETUP"
source "$WORKSPACE_ROOT/install/setup.bash"

export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

exec xvfb-run -a -s "-screen 0 ${XVFB_SCREEN}" \
  ros2 run my_robot_webui dashboard_server \
  --host "$WEBUI_HOST" \
  --port "$WEBUI_PORT" \
  --stack-mode "$STACK_MODE" \
  --no-browser
