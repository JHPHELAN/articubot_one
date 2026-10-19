from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from articubot_one.launch_utils.helpers import include_launch, namespace_wrap


#
# Stingray HARDWARE-ONLY bring-up: robot_state_publisher + drive + sensors.
#
# Unlike stingray.launch.py, this does NOT start the localizers (slam_toolbox)
# or Nav2. Use it when an external SLAM/localization source (e.g. RTAB-Map in
# MAPPING mode) will own the map->odom TF and a separate Nav2 is launched on top
# (the RTAB-Map / Nav2 / frontier-explorer recipe). Real robot only.
#
#   ros2 launch articubot_one stingray.hardware.launch.py
#

def generate_launch_description():

    package_name = 'articubot_one'
    robot_model = 'stingray'

    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_state_publisher = include_launch(
        package_name,
        ['launch', 'rsp.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
        },
    )

    drive_include = include_launch(
        package_name,
        ['robots', robot_model, 'launch', 'stingray.drive.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
        },
    )

    sensors_include = include_launch(
        package_name,
        ['robots', robot_model, 'launch', 'stingray.sensors.launch.py'],
        {
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
        },
        condition=UnlessCondition(use_sim_time),  # real robot only
    )

    namespaced_actions = namespace_wrap(namespace, [
        robot_state_publisher,
        drive_include,
        sensors_include,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace for multi-robot deployment',
        ),
        LogInfo(msg=[
            '============ starting STINGRAY HARDWARE-ONLY  namespace="', namespace,
            '"  use_sim_time=', use_sim_time,
            '  robot_model=', robot_model,
        ]),
        namespaced_actions,
    ])