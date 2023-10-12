# Copyright 2022 Clearpath Robotics, Inc.
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
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)


from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    turtlebot4_python_tutorials = '/home/yuri/Documents/PhD/ROS_WS/humble/src/yd_turtlebot4_tutorials/turtlebot4_python_tutorials'

    nav2_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [turtlebot4_python_tutorials, 'config', 'nav2.yaml']),
        description='Nav2 parameters')

    namespace_arg = DeclareLaunchArgument(
                        'namespace',
                        default_value='',
                        description='Robot namespace')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value=True,)
    use_namespace = LaunchConfiguration('use_namespace')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2 = GroupAction([
        #PushRosNamespace(namespace),
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_composition': 'False',
                              'params_file': LaunchConfiguration('params_file')}.items()),
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(use_namespace_arg)
    ld.add_action(nav2_params_arg)
    ld.add_action(namespace_arg)
    ld.add_action(nav2)
    return ld