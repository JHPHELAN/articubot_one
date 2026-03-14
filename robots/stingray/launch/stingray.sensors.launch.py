from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from articubot_one.launch_utils.helpers import include_launch

#
# Launch description for Stingray robot sensors.
# Sensors are robot-specific, so keep them in a separate launch file.
#

def generate_launch_description():

    package_name = 'articubot_one'

    robot_model = 'stingray'  # static per robot type

    # Allow the including launch file to set a namespace via a launch-argument
    namespace = LaunchConfiguration('namespace', default='')

    # Keep interface compatible with being included from stingray.launch.py
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # LiDAR node (LDROBOT). See: https://github.com/ldrobotSensorTeam/ldlidar_ros2
    # Adjust product_name/port_name for the installed model and udev rule.
    ldlidar_node = Node(
            package='ldlidar_ros2',
            executable='ldlidar_ros2_node',
            name='ldlidar_publisher',
            namespace=namespace,
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'product_name': 'LDLiDAR_LD19'},  # Change to LDLiDAR_LD06, LD14, LD14P, or LD19 as needed
                {'laser_scan_topic_name': 'scan'},
                {'point_cloud_2d_topic_name': 'pointcloud2d'},
                {'frame_id': 'base_laser'},  # Changed from 'laser_frame' to match URDF
                {'port_name': '/dev/ldlidar'},  # Persistent device name via udev rules
                {'serial_baudrate': 230400},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False},
                # {'angle_crop_min': 135.0},
                # {'angle_crop_max': 225.0},
                {'range_min': 0.02},
                {'range_max': 12.0}
            ]
    )

    # BNO085 IMU config is robot-specific.
    bno085_config = PathJoinSubstitution([
        FindPackageShare(package_name),
        'robots', robot_model, 'config', 'bno085_i2c.yaml'
    ])
    
    bno085_driver_node = Node(
        package='bno08x_driver',
        namespace=namespace,
        executable='bno08x_driver',
        name='bno08x_driver',
        output='screen',
        respawn=True,
        respawn_delay=4,
        parameters=[bno085_config],
        remappings=[("imu", "imu/data")]
    )

    # Run EKF first so odom->base_link is stable before SLAM/localization.
    ekf_imu_odom = include_launch(
        package_name,
        ['launch', 'ekf_imu_odom.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'namespace': namespace
        }
    )

    # OAK-D camera driver (DepthAI ROS driver) publishes image topics.
    oakd_launch = include_launch(
        package_name,
        ['launch', 'oakd.launch.py'],
        {
            'namespace': namespace,
            'parent_frame': 'oakd_front_panel'
        }
    )

    return LaunchDescription([
        ldlidar_node,
        bno085_driver_node,
        ekf_imu_odom,
        oakd_launch
    ])
