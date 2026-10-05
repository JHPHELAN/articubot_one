# 2026-10-05 — RTAB-Map visual SLAM: install + handoff

## Goal
Try Visual SLAM with RTAB-Map (Sergei/slgrobotics wiki) as an interim approach to
the low-obstacle problem, while the OAK-D wedge (up-tilt) and ultrasonic sensors are
still pending hardware. Keep using the OAK-D as the camera for now; a fisheye USB
camera may be tried later for wider visual awareness.
Wiki: https://github.com/slgrobotics/articubot_one/wiki/Visual-SLAM-with-RTAB%E2%80%90Map

## Architecture (how RTAB-Map fits)
- RTAB-Map replaces slam_toolbox: it consumes camera RGB-D + odometry and publishes
  the map->odom TF plus an OccupancyGrid that Nav2 uses exactly like SLAM Toolbox's.
- robot_localization still publishes odom->base_link. Nav2 unchanged otherwise.
- Map/graph/images stored in SQLite .db (default $HOME/.ros/rtabmap.db).

## Done this session
- Installed on Stormy (Pi): ros-jazzy-rtabmap-ros 0.23.13 (full stack: rtabmap,
  -slam, -odom, -sync, -util, -conversions, -costmap-plugins, -msgs, -python,
  -launch, -examples, -demos, -viz, -rviz-plugins). Clean, no errors.

## Open items / next steps
1. Confirm the OAK-D topic names Stormy actually publishes. Sergei's OAK-D path needs:
   - /oak/rgb/image_rect        (sensor_msgs/Image, RGB)
   - /oak/stereo/image_raw      (sensor_msgs/Image, actually the DEPTH image)
   - /oak/rgb/camera_info       (sensor_msgs/CameraInfo)
   - /odometry/local            (nav_msgs/Odometry) -- Sergei's odom topic; VERIFY
     what Stormy's EKF publishes (likely /odometry/filtered or /odom) and remap.
2. Port Sergei's launch files into Stormy's repo (they do NOT exist here yet):
   - launch/rtabmap.launch.py
   - launch/rtabmap_viz.launch.py   (runs on Dev machine)
   Source: slgrobotics/articubot_one, dev branch. Adapt topic remaps + params.
   Stormy already has launch/oakd.launch.py, description/oakd.xacro,
   robots/stingray/config/oakd_nav_slim.yaml.
3. Install ros-jazzy-rtabmap-ros on LinuxBox (Dev machine) for rtabmap_viz / rviz.
4. Run RTAB-Map (per wiki) once wired:
   ros2 launch articubot_one rtabmap.launch.py delete_db_on_start:=true publish_tf_map:=false
   (publish_tf_map:=false keeps it from fighting normal localization while testing.)
   Viewer on LinuxBox: ros2 launch articubot_one rtabmap_viz.launch.py

## Deferred / reminders
- Fisheye USB camera: RTAB-Map standard mode needs depth (RGB-D or stereo). A single
  fisheye is RGB-only (no metric depth for the grid). Revisit after OAK-D path works.
  (User was thinking of Sergei's separate image_to_3d repo -- different project.)
- Still in place from 2026-10-03: keepout rectangle over the hall table; OAK pitch
  0.027 rad; base_laser 0.1757 m; global OAK min_obstacle_height 0.06. OAK wedge +
  up-tilted sonar remain the real hardware fix.

## Workflow note (important)
- This session ran in a LOCAL VS Code window on HankRearden, reaching Stormy only via
  ssh, so terminal Allow prompts did not behave. CONTINUE in the Remote-SSH window
  (workspace /home/ubuntu/robot_ws) so suggested commands run on Stormy with a real
  Allow button. Fresh Copilot chat there: run Startup = Read context.txt, Recall this
  NOTES file + skills, Pull.
