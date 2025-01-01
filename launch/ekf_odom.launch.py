# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():

    package_name='articubot_one' #<--- CHANGE ME

    package_path = get_package_share_directory(package_name)

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot specific files reside under "robots" directory - dragger, plucky, create1...
    robot_model = LaunchConfiguration('robot_model', default='')

    # define the launch argument that must be passed from the calling launch file or from the console:
    robot_model_arg= DeclareLaunchArgument('robot_model', default_value='')

    robot_model_path = PythonExpression(["'", package_path, "' + '/robots/", robot_model,"'"])
    
    rl_params_file = PythonExpression(["'", robot_model_path, "' + '/config/ekf_odom_params.yaml'"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time', default_value='false',
                description='Use simulation (Gazebo) clock if true'
            ),
            DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            DeclareLaunchArgument(
                "output_location", default_value="~/ekf_odom_example_debug.txt"
            ),

            LogInfo(msg='============ starting EKF ODOM  use_sim_time:'),
            LogInfo(msg=use_sim_time),
            LogInfo(msg=rl_params_file),

            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "odometry/local")],
            )
        ]
    )
