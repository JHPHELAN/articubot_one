#
# See  share/depthai_ros_driver_v3/launch/driver.launch.py
#      https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/OAK-D_Lite.md
#
#   meld ~/robot_ws/src/articubot_one/launch/oakd.launch.py /opt/ros/jazzy/share/depthai_ros_driver_v3/launch/driver.launch.py
#

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode


def is_launch_config_true(context, name):
    return LaunchConfiguration(name).perform(context) == "true"


def setup_launch_prefix(context, *args, **kwargs):
    use_gdb = LaunchConfiguration("use_gdb", default="false")
    use_valgrind = LaunchConfiguration("use_valgrind", default="false")
    use_perf = LaunchConfiguration("use_perf", default="false")

    launch_prefix = ""

    if use_gdb.perform(context) == "true":
        launch_prefix += "xterm -e gdb -ex run --args"
    if use_valgrind.perform(context) == "true":
        launch_prefix += "valgrind --tool=callgrind"
    if use_perf.perform(context) == "true":
        launch_prefix += (
            "perf record -g --call-graph dwarf --output=perf.out.node_name.data --"
        )

    return launch_prefix


def launch_setup(context, *args, **kwargs):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    urdf_launch_dir = PathJoinSubstitution([FindPackageShare('depthai_descriptions_v3'), 'launch'])

    parent_frame = LaunchConfiguration(
        "parent_frame", default="oak-d-base-frame"
    ).perform(context)
    cam_pos_x = LaunchConfiguration("cam_pos_x", default="0.0")
    cam_pos_y = LaunchConfiguration("cam_pos_y", default="0.0")
    cam_pos_z = LaunchConfiguration("cam_pos_z", default="0.0")
    cam_roll = LaunchConfiguration("cam_roll", default="0.0")
    cam_pitch = LaunchConfiguration("cam_pitch", default="0.0")
    cam_yaw = LaunchConfiguration("cam_yaw", default="0.0")
    use_composition = LaunchConfiguration("rsp_use_composition", default="true")
    imu_from_descr = LaunchConfiguration("imu_from_descr", default="false")
    publish_tf_from_calibration = LaunchConfiguration(
        "publish_tf_from_calibration", default="false"
    )
    override_cam_model = LaunchConfiguration("override_cam_model", default="false")
    params_file = LaunchConfiguration("params_file")
    camera_model = LaunchConfiguration("camera_model", default="OAK-D")
    rs_compat = LaunchConfiguration("rs_compat", default="false")
    pointcloud_enable = LaunchConfiguration("pointcloud.enable", default="false")
    rectify_rgb = LaunchConfiguration("rectify_rgb", default="true")
    namespace = LaunchConfiguration("namespace", default="").perform(context)
    name = LaunchConfiguration("name").perform(context)

    # If RealSense compatibility is enabled, we need to override some parameters, topics and node names
    infra1_enabled = is_launch_config_true(context, "enable_infra1")
    infra2_enabled = is_launch_config_true(context, "enable_infra2")
    parameter_overrides = {
        "left": {"i_publish_topic": infra1_enabled},
        "right": {"i_publish_topic": infra2_enabled},
        "stereo": {
            "i_left_rect_publish_topic": infra1_enabled,
            "i_right_rect_publish_topic": infra2_enabled,
        },
    }
    color_sens_name = "rgb"
    stereo_sens_name = "stereo"
    points_topic_name = f"{name}/points"
    # 2026-06-14: Do NOT force RGB/stereo sync when pointcloud is enabled.
    # PointCloudXyzNode only subscribes to depth+camera_info, so sync is
    # unneeded. Forcing sync with mismatched RGB/stereo configs was causing
    # the depthai v3 driver to SIGSEGV right after "Driver ready!".
    depth_topic_suffix = "image_raw"
    if rs_compat.perform(context) == "true":
        depth_topic_suffix = "image_rect_raw"
        depth_profile = LaunchConfiguration("depth_module.depth_profile").perform(
            context
        )
        color_profile = LaunchConfiguration("rgb_camera.color_profile").perform(context)
        infra_profile = LaunchConfiguration("depth_module.infra_profile").perform(
            context
        )
        # split profile string (0,0,0 or 0x0x0 or 0X0X0) into with (int) height(int) and fps(double)
        # find delimiter
        delimiter = ","
        if "x" in depth_profile:
            delimiter = "x"
        elif "X" in depth_profile:
            delimiter = "X"
        depth_profile = depth_profile.split(delimiter)
        color_profile = color_profile.split(delimiter)
        infra_profile = infra_profile.split(delimiter)

        color_sens_name = "color"
        stereo_sens_name = "depth"
        if name == "oak":
            name = "camera"
        points_topic_name = f"{name}/depth/color/points"
        if namespace == "":
            namespace = "camera"
        if parent_frame == "oak-d-base-frame":
            parent_frame = f"{name}_link"
        parameter_overrides = {
            "camera": {
                "i_rs_compat": True,
            },
            "pipeline_gen": {
                "i_enable_sync": True,
            },
            "color": {
                "i_publish_topic": is_launch_config_true(context, "enable_color"),
                "i_synced": True,
                "i_width": int(color_profile[0]),
                "i_height": int(color_profile[1]),
                "i_fps": float(color_profile[2]),
            },
            "depth": {
                "i_publish_topic": is_launch_config_true(context, "enable_depth"),
                "i_synced": True,
                "i_width": int(depth_profile[0]),
                "i_height": int(depth_profile[1]),
                "i_fps": float(depth_profile[2]),
            },
            "infra1": {
                "i_width": int(infra_profile[0]),
                "i_height": int(infra_profile[1]),
                "i_fps": float(infra_profile[2]),
            },
            "infra2": {
                "i_width": int(infra_profile[0]),
                "i_height": int(infra_profile[1]),
                "i_fps": float(infra_profile[2]),
            },
        }
        parameter_overrides["depth"] = {
            "i_left_rect_publish_topic": True,
            "i_right_rect_publish_topic": True,
        }

    tf_params = {}
    if publish_tf_from_calibration.perform(context) == "true":
        cam_model = ""
        if override_cam_model.perform(context) == "true":
            cam_model = camera_model.perform(context)
        # 2026-06-14: depthai_ros_driver_v3 namespaces TF params under 'driver.*'
        # (not 'camera.*' as in older v2 / driver.launch.py reference). Verified
        # via `ros2 param list /oak | grep tf` against the running driver.
        tf_params = {
            "driver": {
                "i_publish_tf_from_calibration": True,
                "i_tf_tf_prefix": name,
                "i_tf_camera_model": cam_model,
                "i_tf_base_frame": name,
                "i_tf_parent_frame": parent_frame,
                "i_tf_cam_pos_x": cam_pos_x.perform(context),
                "i_tf_cam_pos_y": cam_pos_y.perform(context),
                "i_tf_cam_pos_z": cam_pos_z.perform(context),
                "i_tf_cam_roll": cam_roll.perform(context),
                "i_tf_cam_pitch": cam_pitch.perform(context),
                "i_tf_cam_yaw": cam_yaw.perform(context),
                "i_tf_imu_from_descr": imu_from_descr.perform(context),
            }
        }
    else:
        # 2026-06-14: when we DON'T publish TF from calibration, we still must
        # disable it explicitly. The driver's built-in default is to auto-publish
        # its own URDF with parent_frame='oak_parent_frame' (we saw "[oak]: Published URDF"
        # in the log + a stale 'oak -> oak_parent_frame' branch in /tf_static that
        # competed with oak_state_publisher's correct 'oak -> oakd_front_panel' branch,
        # producing a split TF tree and Message Filter drops.
        # NB: parameter namespace is 'driver.*' for v3 (see comment above).
        tf_params = {"driver": {"i_publish_tf_from_calibration": False}}

    launch_prefix = setup_launch_prefix(context)

    return [
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz").perform(context)),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([urdf_launch_dir, 'urdf_launch.py'])
            ),
            launch_arguments={
                # 2026-06-14: resolve every value to a plain string here. When this
                # dict mixed Substitutions (LaunchConfiguration) and strings, urdf_launch
                # silently fell back to its defaults (use_base_descr=true equivalent
                # behavior, parent_frame=oak_parent_frame), which loaded base_descr.urdf.xacro
                # and orphaned oak_rgb_camera_optical_frame from the rest of the TF tree.
                "namespace": namespace,
                "tf_prefix": name,
                "camera_model": camera_model.perform(context),
                "base_frame": name,
                "parent_frame": parent_frame,
                "cam_pos_x": cam_pos_x.perform(context),
                "cam_pos_y": cam_pos_y.perform(context),
                "cam_pos_z": cam_pos_z.perform(context),
                "cam_roll": cam_roll.perform(context),
                "cam_pitch": cam_pitch.perform(context),
                "cam_yaw": cam_yaw.perform(context),
                "use_composition": use_composition.perform(context),
                "use_base_descr": publish_tf_from_calibration.perform(context),
                "rs_compat": rs_compat.perform(context),
            }.items(),
        ),
        ComposableNodeContainer(
            name=f"{name}_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver_v3",
                    plugin="depthai_ros_driver::Driver",
                    name=name,
                    namespace=namespace,
                    parameters=[
                        params_file,
                        tf_params,
                        parameter_overrides,
                    ],
                )
            ],
            arguments=["--ros-args", "--log-level", log_level],
            prefix=[launch_prefix],
            output="both",
        ),
        LoadComposableNodes(
            condition=IfCondition(rectify_rgb),
            target_container=f"{namespace}/{name}_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_color_node",
                    namespace=namespace,
                    remappings=[
                        ("image", f"{name}/{color_sens_name}/image_raw"),
                        ("camera_info", f"{name}/{color_sens_name}/camera_info"),
                        ("image_rect", f"{name}/{color_sens_name}/image_rect"),
                        (
                            "image_rect/compressed",
                            f"{name}/{color_sens_name}/image_rect/compressed",
                        ),
                        (
                            "image_rect/compressedDepth",
                            f"{name}/{color_sens_name}/image_rect/compressedDepth",
                        ),
                        (
                            "image_rect/theora",
                            f"{name}/{color_sens_name}/image_rect/theora",
                        ),
                    ],
                )
            ],
        ),
        LoadComposableNodes(
            condition=IfCondition(pointcloud_enable),
            target_container=f"{namespace}/{name}_container",
            composable_node_descriptions=[
                # 2026-06-14: switched from PointCloudXyzrgbNode to PointCloudXyzNode.
                # Nav2 obstacle costmaps don't use color, and depth-only avoids the
                # RGB/depth resolution-mismatch SIGSEGV in the depthai v3 driver.
                # RGB stream is unaffected; /oak/rgb/* still publishes.
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzNode",
                    name="point_cloud_xyz_node",
                    namespace=namespace,
                    remappings=[
                        (
                            "image_rect",
                            f"{name}/{stereo_sens_name}/{depth_topic_suffix}",
                        ),
                        (
                            "camera_info",
                            f"{name}/{stereo_sens_name}/camera_info",
                        ),
                        ("points", points_topic_name),
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    depthai_prefix = PathJoinSubstitution([FindPackageShare('depthai_ros_driver_v3')])

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D-LITE"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([FindPackageShare('depthai_ros_driver_v3'), 'config', 'driver.yaml']),
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([FindPackageShare('depthai_ros_driver_v3'), 'config', 'rviz', 'rgbd.rviz']),
        ),
        DeclareLaunchArgument("rsp_use_composition", default_value="true"),
        DeclareLaunchArgument(
            "publish_tf_from_calibration",
            default_value="false",
            description="Enables TF publishing from camera calibration file.",
        ),
        DeclareLaunchArgument(
            "imu_from_descr",
            default_value="false",
            description="Enables IMU publishing from URDF.",
        ),
        DeclareLaunchArgument(
            "override_cam_model",
            default_value="false",
            description="Overrides camera model from calibration file.",
        ),
        DeclareLaunchArgument("use_gdb", default_value="false"),
        DeclareLaunchArgument("use_valgrind", default_value="false"),
        DeclareLaunchArgument("use_perf", default_value="false"),
        DeclareLaunchArgument(
            "rs_compat",
            default_value="false",
            description="Enables compatibility with RealSense nodes.",
        ),
        DeclareLaunchArgument("rectify_rgb", default_value="true"),
        DeclareLaunchArgument("pointcloud.enable", default_value="false"),
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_infra1", default_value="true"),
        DeclareLaunchArgument("enable_infra2", default_value="true"),
        # DeclareLaunchArgument("depth_module.depth_profile", default_value="1280,720,30"),
        DeclareLaunchArgument("depth_module.depth_profile", default_value="640,480,30"),
        # DeclareLaunchArgument("rgb_camera.color_profile", default_value="1280,720,30"),
        DeclareLaunchArgument("rgb_camera.color_profile", default_value="640,480,30"),
        DeclareLaunchArgument("depth_module.infra_profile", default_value="1280,720,30"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
