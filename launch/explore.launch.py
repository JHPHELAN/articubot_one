#!/usr/bin/env python3
# Standalone launcher for the frontier explorer (frontier_explorer_v2.py).
# Assumes the main robot stack (stingray.launch.py) is already running and Nav2
# is active. Parameters are loaded from
#   robots/stingray/config/explore.yaml
# Individual values can still be overridden on the command line, e.g.:
#   ros2 launch articubot_one explore.launch.py min_clearance_m:=0.20

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('articubot_one')
    default_params = os.path.join(
        pkg_share, 'robots', 'stingray', 'config', 'explore.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML parameter file for frontier_explorer_v2.',
    )

    # Convenience overrides for the values we flip most often. Each is optional;
    # if left empty, the value from params_file (or the script default) wins.
    dry_run_arg = DeclareLaunchArgument(
        'dry_run', default_value='',
        description='Override dry_run (true/false). Empty = use YAML.',
    )
    min_clearance_arg = DeclareLaunchArgument(
        'min_clearance_m', default_value='',
        description='Override min_clearance_m. Empty = use YAML.',
    )
    path_clearance_max_cost_arg = DeclareLaunchArgument(
        'path_clearance_max_cost', default_value='',
        description='Override path_clearance_max_cost. Empty = use YAML.',
    )
    goal_max_cost_arg = DeclareLaunchArgument(
        'goal_max_cost', default_value='',
        description='Override goal_max_cost. Empty = use YAML.',
    )
    failures_before_blacklist_arg = DeclareLaunchArgument(
        'failures_before_blacklist', default_value='',
        description='Override failures_before_blacklist. Empty = use YAML.',
    )
    blacklist_radius_arg = DeclareLaunchArgument(
        'blacklist_radius_m', default_value='',
        description='Override blacklist_radius_m. Empty = use YAML.',
    )

    # Build the parameters list: YAML first, then any non-empty CLI overrides.
    # An empty-string override is filtered out by Node when the value can't be
    # parsed; to be safe we use a small OpaqueFunction-free trick: pass the YAML,
    # then a dict whose values are LaunchConfigurations. ROS 2 will ignore params
    # whose substituted value is an empty string only if we guard with a wrapper.
    # Simpler approach: launch a tiny helper that builds the param list at
    # runtime via OpaqueFunction.
    from launch.actions import OpaqueFunction

    def _launch_setup(context, *args, **kwargs):
        overrides = {}
        for key in (
            'dry_run',
            'min_clearance_m',
            'path_clearance_max_cost',
            'goal_max_cost',
            'failures_before_blacklist',
            'blacklist_radius_m',
        ):
            raw = LaunchConfiguration(key).perform(context).strip()
            if raw == '':
                continue
            # Coerce to the right Python type so ROS sees the parameter as the
            # type the node declared (bool / int / float).
            if key == 'dry_run':
                overrides[key] = raw.lower() in ('1', 'true', 'yes', 'on')
            elif key in ('path_clearance_max_cost',
                         'goal_max_cost',
                         'failures_before_blacklist'):
                overrides[key] = int(raw)
            else:
                overrides[key] = float(raw)

        params = [LaunchConfiguration('params_file').perform(context)]
        if overrides:
            params.append(overrides)

        return [Node(
            package='articubot_one',
            executable='frontier_explorer_v2.py',
            name='frontier_explorer_v2',
            output='screen',
            emulate_tty=True,
            parameters=params,
        )]

    return LaunchDescription([
        params_file_arg,
        dry_run_arg,
        min_clearance_arg,
        path_clearance_max_cost_arg,
        goal_max_cost_arg,
        failures_before_blacklist_arg,
        blacklist_radius_arg,
        OpaqueFunction(function=_launch_setup),
    ])
