# 2026-10-05 — RTAB-Map launch port + LinuxBox sync (session 2)

Continues NOTES-2026-10-05-rtabmap-install-handoff.md. This session ran in the
Remote-SSH window on Stormy, so Allow boxes / terminal commands worked.

## Done this session

### RTAB-Map launch files ported into the repo (step 2)
- Added `launch/rtabmap.launch.py` and `launch/rtabmap_viz.launch.py`, copied
  verbatim from slgrobotics/articubot_one (dev branch). No remap edits were needed:
  Sergei's defaults already match Stormy's topics because our `oakd.launch.py`
  shares the same conventions (OAK-D named `oak`).
  - odom_topic        /odometry/local      (confirmed: EKF `ekf_filter_node_odom`
                                            remaps odometry/filtered -> /odometry/local)
  - rgb_topic         /oak/rgb/image_rect
  - depth_topic       /oak/stereo/image_raw
  - camera_info_topic /oak/rgb/camera_info
  - imu_topic         /imu/data            (optional; unused with visual_odometry:=false)
  - frame_id          base_link
  - rtabmap.launch.py wraps the `rtabmap_launch` package in depth RGB-D mode and
    exposes delete_db_on_start / publish_tf_map so the wiki trial command works as-is.
- Rebuilt: `colcon build --packages-select articubot_one --symlink-install` (~1s).
  Both files resolve and parse via `ros2 launch ... --show-args`.

### LinuxBox access (so Allow boxes can drive it from Stormy)
- Key-based SSH Stormy -> LinuxBox:
  - Added `Host linuxbox` (HostName 192.168.68.99, User ubuntu) to Stormy ~/.ssh/config.
  - Authorized Stormy's existing key `ubuntu@Stingray` (id_ed25519) in LinuxBox
    ~/.ssh/authorized_keys. `ssh linuxbox` is now passwordless.
- NOPASSWD sudo on LinuxBox: `/etc/sudoers.d/90-ubuntu-nopasswd` =
  `ubuntu ALL=(ALL:ALL) NOPASSWD: ALL` (validated with visudo). Lets the agent run
  installs/upgrades on LinuxBox from Stormy without a password prompt.
  (These are machine-local, NOT in git — recorded here + in context.txt so the
  "memory" travels.)

### Version split resolved (RTAB-Map 0.23.7 vs 0.23.13)
- Root cause: different apt channels, not a timing fluke.
  - Stormy  ros2.sources -> packages.ros.org/ros2-testing  => rtabmap 0.23.13
  - LinuxBox ros2.sources -> packages.ros.org/ros2 (stable) => rtabmap 0.23.7
- Fix (user chose "sync LinuxBox up to testing"):
  - Switched LinuxBox `/etc/apt/sources.list.d/ros2.sources` URI ros2 -> ros2-testing
    (a .bak was written by sed).
  - `apt-get full-upgrade` (~833 pkgs, incl. kernel 7.0.0-34 + openjdk-21). Clean,
    exit 0. RTAB-Map stack now 0.23.13 on both machines.
  - Reboot into 7.0.0-34 still pending on LinuxBox.

## Decisions / policy
- FUTURE: migrate Stormy from ros2-testing to the stable ros2 channel (user's stated
  preference). LinuxBox was brought up to testing only to match Stormy for now;
  the longer-term target is both on stable. Noted as a TODO in context.txt.
- VS Code approvals: disabled `chat.tools.terminal.enableAutoApprove` so the agent
  asks before every terminal command. (Reason: an ssh-driven upgrade auto-ran without
  a prompt. Sandboxing is a separate layer and was NOT the cause.) Also useful:
  "Chat: Reset Tool Confirmations", keep level on "Manual permissions".

## Open items / next steps
1. Step 4 trial run (per wiki), once the robot stack + OAK-D are up:
     ros2 launch articubot_one rtabmap.launch.py delete_db_on_start:=true publish_tf_map:=false
   Viewer on LinuxBox:
     ros2 launch articubot_one rtabmap_viz.launch.py
   (publish_tf_map:=false keeps RTAB-Map from fighting normal localization while testing.)
2. At trial, VERIFY the OAK depth topic `/oak/stereo/image_raw` is registered/aligned
   to `/oak/rgb/camera_info` (RGB-D needs depth in the RGB frame). If not aligned,
   use a depth-registered topic or add depth_image_proc registration.
3. Quick live re-check deferred by the approvals change: confirm LinuxBox rtabmap
   actually reports 0.23.13 (`dpkg -l | grep rtabmap`), and reboot LinuxBox into
   kernel 7.0.0-34.
4. Still from 2026-10-03: hall-table keepout, OAK pitch 0.027 rad, base_laser 0.1757 m,
   global OAK min_obstacle_height 0.06. OAK wedge up-tilt + up-tilted sonar remain the
   real hardware fix for the low-obstacle problem.
