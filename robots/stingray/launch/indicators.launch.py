"""Launch the Stingray indicators node (headlights + safety beacon)."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('articubot_one')
    params = os.path.join(pkg_share, 'robots', 'stingray', 'config',
                          'indicators.yaml')

    return LaunchDescription([
        Node(
            package='articubot_one',
            executable='indicators_node.py',
            name='indicators_node',
            output='screen',
            parameters=[params],
        ),
    ])
