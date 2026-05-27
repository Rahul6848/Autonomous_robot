# VPS Deployment For `my_robot_webui`

This deployment path keeps the current application code unchanged.

It uses:

- `systemd` for auto-start and restart
- `xvfb-run` to provide a virtual `DISPLAY` on a headless VPS
- `nginx` as a reverse proxy
- HTTP basic auth in `nginx` so the dashboard is not exposed directly

## Important constraints

- Your app is a ROS 2 dashboard, not a static web app.
- The backend listens on `0.0.0.0:8080` by default and has no built-in authentication.
- The dashboard expects `DISPLAY` to exist so it can launch the mission stack.
- Without a domain, you will use the VPS public IP on plain HTTP unless you add your own TLS layer.

For long-term public use, buy a domain and add HTTPS. IP-only + HTTP basic auth is acceptable only as a temporary deployment.

## Assumed paths

These files assume:

- deploy user: `deploy`
- workspace path: `/home/deploy/ros2_workspace`
- ROS setup file: `/opt/ros/humble/setup.bash`

If your VPS uses different values, edit:

- `deploy/vps/my_robot_webui.env`
- `deploy/vps/my_robot_webui.service`

## 1. Prepare the VPS

Run as `root`:

```bash
apt update && apt upgrade -y
apt install -y git nginx apache2-utils ufw xvfb x11vnc novnc python3-colcon-common-extensions
ufw allow OpenSSH
ufw allow 'Nginx Full'
ufw enable
adduser deploy
usermod -aG sudo deploy
```

Optional hardening if you will access the dashboard only from one IP:

```bash
ufw delete allow 'Nginx Full'
ufw allow from YOUR_PUBLIC_IP to any port 80 proto tcp
```

## 2. Install ROS 2 on the VPS

Install the same ROS 2 distro that you used locally. After installation, this command must work:

```bash
source /opt/ros/humble/setup.bash
```

If your distro is not `humble`, change `ROS_SETUP` later in the env file.

## 3. Clone the repo

Switch to the deploy user:

```bash
su - deploy
git clone <YOUR_GITHUB_REPO_URL> /home/deploy/ros2_workspace
cd /home/deploy/ros2_workspace
```

If the repo is private, use GitHub SSH or a personal access token.

## 4. Build the workspace

```bash
source /opt/ros/humble/setup.bash
cd /home/deploy/ros2_workspace
colcon build --packages-select my_robot_webui my_robot_bringup my_robot_controller my_robot_description
```

If your robot stack needs extra apt packages, install them before this step until the build succeeds.

## 5. Create the runtime env file

```bash
cp /home/deploy/ros2_workspace/deploy/vps/my_robot_webui.env.example /home/deploy/ros2_workspace/deploy/vps/my_robot_webui.env
nano /home/deploy/ros2_workspace/deploy/vps/my_robot_webui.env
```

Set at minimum:

- `OPENAI_API_KEY`
- `ROS_SETUP`
- `WORKSPACE_ROOT`

## 6. Create the nginx password

```bash
sudo htpasswd -c /etc/nginx/.htpasswd_my_robot_webui admin
```

Use a strong password. This is the login prompt you will see before the dashboard.

## 7. Install the service and nginx config

As `root`:

```bash
cp /home/deploy/ros2_workspace/deploy/vps/my_robot_webui.service /etc/systemd/system/my_robot_webui.service
cp /home/deploy/ros2_workspace/deploy/vps/nginx-my_robot_webui.conf /etc/nginx/sites-available/my_robot_webui
ln -sf /etc/nginx/sites-available/my_robot_webui /etc/nginx/sites-enabled/my_robot_webui
rm -f /etc/nginx/sites-enabled/default
systemctl daemon-reload
nginx -t
systemctl restart nginx
systemctl enable my_robot_webui
systemctl start my_robot_webui
```

## 8. Check status

```bash
systemctl status my_robot_webui --no-pager
journalctl -u my_robot_webui -n 100 --no-pager
ss -ltnp | grep 8080
```

Expected result:

- the app listens only on `127.0.0.1:8080`
- nginx listens on port `80`

## 9. Open it in your browser

Visit:

```text
http://YOUR_VPS_PUBLIC_IP/
```

You should get the nginx basic-auth prompt first.

## Update workflow

When you push new code to GitHub:

```bash
su - deploy
cd /home/deploy/ros2_workspace
git pull
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_webui my_robot_bringup my_robot_controller my_robot_description
sudo systemctl restart my_robot_webui
```

## Troubleshooting

- `DISPLAY is not set`
  - `xvfb` is missing or the service is not using `run_my_robot_webui.sh`.
- `ros2: command not found`
  - `ROS_SETUP` points to the wrong ROS installation.
- `package not found`
  - the workspace was not built successfully or `install/setup.bash` is missing.
- page opens but desktop stream does not
  - `x11vnc` or `novnc_proxy` is missing, or Gazebo/RViz could not start in the virtual display.
- browser says connection refused
  - check `systemctl status my_robot_webui` and `nginx -t`.
