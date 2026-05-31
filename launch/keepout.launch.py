#!/usr/bin/env python3
# Launches the keepout-zone pipeline for Nav2 costmap filters:
#   1) nav2_map_server publishing the keepout mask PGM on /keepout_filter_mask
#   2) costmap_filter_info_server publishing /costmap_filter_info
#   3) lifecycle_manager to bring both up
# Run alongside the main stack:
#   ros2 launch articubot_one keepout.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('articubot_one')
    mask_yaml = os.path.join(pkg_share, 'assets', 'maps', 'keepout_mask.yaml')

    # Node 1: publish the keepout mask occupancy grid on /keepout_filter_mask
    mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': mask_yaml,
            'topic_name': '/keepout_filter_mask',
            'frame_id': 'map',
        }],
    )

    # Node 2: publish CostmapFilterInfo metadata so the KeepoutFilter plugin can interpret the mask
    filter_info = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
            'type': 0,                                # 0 = keepout filter
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )

    # Lifecycle manager for both
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['filter_mask_server', 'costmap_filter_info_server'],
        }],
    )

    return LaunchDescription([mask_server, filter_info, lifecycle])
