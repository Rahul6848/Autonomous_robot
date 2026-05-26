# My Robot Web UI

This package adds a browser dashboard for your existing ROS 2 workspace without modifying the current controller or bringup packages.

## What it does

- Automatically starts the selected mission stack when the dashboard server comes up
- Publishes robot commands to `/robot_command`
- Supports `stop` plus `start/resume` so paused motion can continue from the stopped position
- Reads mission status from `/mission_status`
- Streams `/camera/image_raw` into the page
- Can embed the actual Gazebo and RViz desktop windows through noVNC when desktop streaming tools are installed

## Build

```bash
cd /home/rahul/ros2_workspace
colcon build --packages-select my_robot_webui
source install/setup.bash
```

## Run

```bash
ros2 run my_robot_webui dashboard_server
```

To enable natural-language commands from the dashboard command bar, either export an OpenAI API key before starting the server:

```bash
export OPENAI_API_KEY=your_api_key_here
ros2 run my_robot_webui dashboard_server
```

Or create this file in the workspace root so the backend loads it automatically on startup:

```text
/home/rahul/ros2_workspace/.my_robot_webui.env
```

File contents:

```bash
OPENAI_API_KEY=your_api_key_here
```

An example file is included at:

```text
/home/rahul/ros2_workspace/.my_robot_webui.env.example
```

Optional environment variables:

- `OPENAI_COMMAND_MODEL` to override the default command-routing model (`gpt-5.4-mini`)
- `OPENAI_RESPONSES_URL` to override the Responses API endpoint
- `OPENAI_COMMAND_TIMEOUT_SEC` to adjust request timeout

To auto-start the road mission instead of the default parking mission:

```bash
ros2 run my_robot_webui dashboard_server --stack-mode road
```

Default URL:

```text
http://localhost:8080
```

You can also run it with:

```bash
ros2 launch my_robot_webui web_dashboard.launch.py
```

## Desktop streaming requirements

To show the actual Gazebo and RViz desktop windows inside the browser, the machine running the dashboard needs:

- `x11vnc`
- `novnc_proxy`
- a valid `DISPLAY` environment

If those tools are not installed, the dashboard still works for command publishing and camera streaming. The browser desktop panel will show the missing requirement.

## Notes

- The dashboard uses your existing launch files:
  - `my_robot_mission.launch.xml`
  - `parking_mission.launch.xml`
- The launch selection buttons were removed from the page. The dashboard now auto-launches the configured stack on startup.
- Exact supported commands still work without the LLM. Natural-language interpretation only activates when `OPENAI_API_KEY` is set.
- The dashboard automatically loads `/home/rahul/ros2_workspace/.my_robot_webui.env` if that file exists.
- The dashboard does not publish `/cmd_vel` directly.
- Existing workspace files were not edited for this feature. Everything lives in the new `my_robot_webui` package.
