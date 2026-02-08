from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from articubot_one.launch_utils.helpers import include_launch
from ament_index_python.packages import get_package_share_directory
import os

#
# Generate launch description for Stingray robot sensors
#
# Sensors are almost always robot-specific, so we have this separate launch file.
#   

def generate_launch_description():

    package_name = 'articubot_one'

    robot_model = 'stingray'  # static per robot type

    # Allow the including launch file to set a namespace via a launch-argument
    namespace = LaunchConfiguration('namespace', default='')

    # Keep interface compatible with being included from stingray.launch.py
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # sensor nodes don't depend on robot_model and don't use package_name

    # Lidar node - LDROBOT LiDAR
    # See: https://github.com/ldrobotSensorTeam/ldlidar_ros2
    # Adjust 'product_name' and 'port_name' based on your specific ldlidar model
    ldlidar_node = Node(
            package='ldlidar_ros2',
            executable='ldlidar_ros2_node',
            name='ldlidar_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD19',  # Change to LDLiDAR_LD06, LD14, LD14P, or LD19 as needed
                'laser_scan_topic_name': 'scan',
                'point_cloud_2d_topic_name': 'pointcloud2d',
                'frame_id': 'base_laser',  # Changed from 'laser_frame' to match URDF
                'port_name': '/dev/ldlidar',  # Persistent device name via udev rules
                'serial_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0,
                'range_min': 0.02,
                'range_max': 12.0
            }]
    )

    # Load BNO085 config file
    bno085_config = os.path.join(
        get_package_share_directory(package_name),
        'robots', robot_model, 'config', 'bno085_i2c.yaml'
    )
    
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

    # We need to run an EKF filter here to ensure its output stabilizes before starting SLAM Toolbox or other Localizers.
    # Localizers/mappers only publish the map to odom transform. Robot needs EKF filter to publish odom to base_link transform.
    ekf_imu_odom = include_launch(
        package_name,
        ['launch', 'ekf_imu_odom.launch.py'],
        {
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'namespace': namespace
        }
    )

    return LaunchDescription([
        ldlidar_node,
        bno085_driver_node,
        ekf_imu_odom
    ])
