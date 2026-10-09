# 2026-10-09 — RTAB-Map in Gazebo sim on LinuxBox: Stingray bringup (session 3)

Continues NOTES-2026-10-05-rtabmap-launch-port-linuxbox-sync.md. New direction:
validate the whole RGB-D -> RTAB-Map pipeline in Gazebo sim on LinuxBox with a
simulated Stingray, before the real-robot trial.

## Branch sync
- LinuxBox was on branch `jp`; current work is on `exploration`.
- `jp` had 0 commits not already in `exploration`; `exploration` was 58 commits ahead,
  so jp was fully contained. Switched LinuxBox to `exploration` (no merge). jp retired.
- Repo renamed JHPHELAN/articubot_one -> JHPHELAN/articubot_new (old name redirects);
  local src dir still named articubot_one.
- Local joystick.yaml slow-teleop edit (0.1/0.5/0.5) was already identical to
  exploration -> discarded. A trivial launch/joystick.launch.py case-fix (True->true)
  remains uncommitted; decide whether to keep.

## Sim RGB-D: added a depth camera to the OAK-D
- gz OAK-D sensor was RGB-only. Added a co-located gz `depth` sensor in
  description/oakd.xacro (shared macro; only Stingray uses it): same optical frame
  (camera_rgb_optical_frame), same FOV (1.3962634) and resolution (800x800) as RGB, so
  depth is pixel-registered to RGB and /camera_info is valid for both.
- Publishes gz topic gz_depth_camera, which config/gz_ros_bridge.yaml already maps to
  ROS /depth_camera. No bridge edit. RGB path (/camera, /camera_info) untouched.
  depth format R_FLOAT32, clip 0.1..10.0 m.

## Gazebo mesh path fix
- Chassis mesh is package://articubot_one/assets/meshes/Stingray.stl. RViz resolved it
  (ament) but Gazebo did not — drive_sim's GZ_SIM_RESOURCE_PATH only had the assets
  folders. Added the package-share PARENT to GZ_SIM_RESOURCE_PATH in
  launch/drive_sim.launch.py (PathJoinSubstitution([FindPackageShare(package_name),
  '..'])) so package://articubot_one/... resolves. Chassis now renders in Gazebo.

## Stingray sim bringup (two sourced terminals)
- drive_sim.launch.py does NOT start robot_state_publisher; it only spawns from
  /robot_description and waits for it. Run rsp separately:
  - T1: ros2 launch articubot_one drive_sim.launch.py robot_model:=stingray
  - T2: ros2 launch articubot_one rsp.launch.py robot_model:=stingray use_sim_time:=true
  rsp publishes /robot_description -> gz `create` spawns Stingray -> controllers/odom
  come up.
- TODO: make a stingray sim bringup launch (rsp + drive_sim) like seggy.drive.launch.py
  so one command does it.

## Verified this session
- Robot renders in Gazebo + RViz; wheels/TF normal.
- Gazebo Image display shows /gz_camera and /gz_depth_camera.
- ROS topics via bridge: /camera, /camera_info, /depth_camera, /odometry/local,
  /odometry/global, /diff_cont/odom, /joint_states, /scan.
- NOT yet confirmed: camera/depth publish RATES (hz probe hit "rcl node's context is
  invalid" over slow XLaunch; recheck with a single clean probe) and /camera_info
  intrinsics content.

## Next steps
1. Confirm /camera + /depth_camera publish ~30 Hz and /camera_info is populated.
2. Launch RTAB-Map against SIM topics (ported rtabmap.launch.py defaults target the real
   OAK /oak/... topics, so remap):
   ros2 launch articubot_one rtabmap.launch.py \
     rgb_topic:=/camera depth_topic:=/depth_camera camera_info_topic:=/camera_info \
     odom_topic:=/odometry/local frame_id:=base_link \
     use_sim_time:=true approx_sync:=true \
     delete_db_on_start:=true publish_tf_map:=false
   (approx_sync because RGB and depth are two separate sim sensors.)
3. Viewer: ros2 launch articubot_one rtabmap_viz.launch.py
4. Drive (slow teleop) to build a map; verify map->odom TF and OccupancyGrid.
5. Performance: run Gazebo on LinuxBox's LOCAL desktop; XLaunch is very slow.

## LinuxBox housekeeping (2026-10-09, machine-local, not in git)
- apt upgrade + autoremove --purge done. Mesa stack + ros-jazzy-plotjuggler held back by
  Ubuntu phased updates (unrelated to RTAB-Map; left to land on their own).
- Disabled auto-updates: apt-daily.timer and apt-daily-upgrade.timer disabled;
  /etc/apt/apt.conf.d/20auto-upgrades already 0/0. Manual updates only now.
- Workspace built with --symlink-install, so xacro/launch/yaml edits need no rebuild.
