# Copyright 2019 Open Source Robotics Foundation, Inc.
#
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
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart', default=True)
    default_file = os.path.join(
        get_package_share_directory('nav2_single_node_navigator'),
        'param', 'novabot.yaml')
    params_file = LaunchConfiguration('params_file', default=default_file)
    map_yaml_file = LaunchConfiguration('map')
    map_dir = LaunchConfiguration(
        'map', default=os.path.join(
            get_package_share_directory('nav2_single_node_navigator'),
            'map', 'empty_map.yaml'))

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    lifecycle_nodes = ['nav2_single_node_navigator', 'map_server']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    start_nav2_single_node_navigator_cmd = Node(
        package='nav2_single_node_navigator',
        executable='nav2_single_node_navigator',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[configured_params],
        remappings=remappings,
        arguments=["-ros-args --disable-rosout-logs"])

    # start_map_saver_server_cmd = Node(
    #     package='nav2_map_server',
    #     executable='map_saver_server',
    #     name='map_saver_server',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time,
    #                  "free_thresh_default": 0.25,
    #                  "occupied_thresh_default": 0.65}],
    #     remappings=remappings)
    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{"yaml_filename": map_yaml_file,
                     'use_sim_time': use_sim_time}],
        remappings=remappings)
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart, 'bond_timeout': 120.0, 'bond_respawn_max_duration': 120.0},
                    {'node_names': lifecycle_nodes}])
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_file,
            description='Full path to param file to load'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        declare_namespace_cmd,
        start_map_server_cmd,
        start_nav2_single_node_navigator_cmd,
        start_lifecycle_manager_cmd,

    ])
