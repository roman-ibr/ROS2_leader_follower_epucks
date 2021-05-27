#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Launch Webots e-puck driver."""

import os
import launch
from webots_ros2_core.utils import ControllerLauncher
from webots_ros2_core.webots_launcher import WebotsLauncher
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    # Webots
    webots = WebotsLauncher(world=os.path.join(get_package_share_directory('webots_ros2_epuck'), 'worlds', 'epuck_world_2.wbt'))
    synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)

    e_1 = ControllerLauncher(
        package='webots_ros2_epuck',
        executable='driver',
        # executable='drive_calibrator',
        arguments=['--webots-robot-name=e_1', '--webots-node-name=e_1'],
        namespace='e_1',
        parameters=[{'synchronization': synchronization}],
        output='screen'
    )

    e_2 = ControllerLauncher(
        package='webots_ros2_epuck',
        executable='driver_2',
        arguments=['--webots-robot-name=e_2', '--webots-node-name=e_2'],
        namespace='e_2',
        parameters=[{'synchronization': synchronization}],
        output='screen'
    )

    # e_3 = ControllerLauncher(
    #     package='webots_ros2_epuck',
    #     executable='driver_3',
    #     arguments=['--webots-robot-name=e_3', '--webots-node-name=e_3'],
    #     namespace='e_3',
    #     parameters=[{'synchronization': synchronization}],
    #     output='screen'
    # )  
    # e_4 = ControllerLauncher(
    #     package='webots_ros2_epuck',
    #     executable='driver_4',
    #     arguments=['--webots-robot-name=e_4', '--webots-node-name=e_4'],
    #     namespace='e_4',
    #     parameters=[{'synchronization': synchronization}],
    #     output='screen'
    # )
    # e_5 = ControllerLauncher(
    #     package='webots_ros2_epuck',
    #     executable='driver_5',
    #     arguments=['--webots-robot-name=e_5', '--webots-node-name=e_5'],
    #     namespace='e_5',
    #     parameters=[{'synchronization': synchronization}],
    #     output='screen'
    # )
    # e_6 = ControllerLauncher(
    #     package='webots_ros2_epuck',
    #     executable='driver_6',
    #     arguments=['--webots-robot-name=e_6', '--webots-node-name=e_6'],
    #     namespace='e_6',
    #     parameters=[{'synchronization': synchronization}],
    #     output='screen'
    # )     
    # e_7 = ControllerLauncher(
    #     package='webots_ros2_epuck',
    #     executable='driver_7',
    #     arguments=['--webots-robot-name=e_7', '--webots-node-name=e_7'],
    #     namespace='e_7',
    #     parameters=[{'synchronization': synchronization}],
    #     output='screen'
    # )        

    # e_1_c = ControllerLauncher(
    #     package='webots_ros2_epuck',
    #     executable='driver',
    #     # arguments=['--webots-robot-name=e_3', '--webots-node-name=e_3'],
    #     # namespace='e_3_c',
    #     # parameters=[{'synchronization': synchronization}],
    #     output='screen'
    # ) 

    return launch.LaunchDescription([
        webots, e_1, e_2,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])    







# def generate_launch_description():
#     package_dir = get_package_share_directory('webots_ros2_epuck')
#     world = LaunchConfiguration('world')

#     webots = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
#         ),
#         launch_arguments=[
#             ('package', 'webots_ros2_epuck'),
#             ('executable', 'driver'),
#             ('world', PathJoinSubstitution([package_dir, 'worlds', world])),
#         ]
#     )


    # webots_2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
    #     ),
    #     launch_arguments=[
    #         ('package', 'webots_ros2_epuck'),
    #         ('executable', 'driver'),
    #         ('world', PathJoinSubstitution([package_dir, 'worlds', world])),
    #     ]
    # )

    # webots_3 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
    #     ),
    #     launch_arguments=[
    #         ('package', 'webots_ros2_epuck'),
    #         ('executable', 'driver'),
    #         ('world', PathJoinSubstitution([package_dir, 'worlds', world])),
    #     ]
    # )


    # return LaunchDescription([
    #     DeclareLaunchArgument(
    #         'world',
    #         default_value='epuck_world.wbt',
    #         description='Choose one of the world files from `/webots_ros2_epuck/world` directory'
    #     ),
    #     webots
    # ])


# def generate_launch_description():
#     # Webots
#     # webots = WebotsLauncher(
#     #     world=os.path.join(
#     #         get_package_share_directory('webots_ros2_epuck'),
#     #         'worlds',
#     #         'epuck_world.wbt'
#     #     )
#     # )
#     package_dir = get_package_share_directory('webots_ros2_epuck')
#     world = LaunchConfiguration('world')

#     webots = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
#         ),
#         launch_arguments=[
#             ('package', 'webots_ros2_epuck'),
#             ('executable', 'driver'),
#             ('world', PathJoinSubstitution([package_dir, 'worlds', world])),
#         ]
#     )

#     # Controller nodes
#     synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
#     ure3_controller = ControllerLauncher(
#         package='webots_ros2_core',
#         executable='webots_robotic_arm_node',
#         arguments=['--webots-robot-name=e_1'],
#         namespace='e_1',
#         parameters=[{'synchronization': synchronization}],
#         output='screen'
#     )
#     ure5_controller = ControllerLauncher(
#         package='webots_ros2_core',
#         executable='webots_robotic_arm_node',
#         arguments=['--webots-robot-name=e_2'],
#         namespace='e_2',
#         parameters=[{'synchronization': synchronization}],
#         output='screen'
#     )
#     return launch.LaunchDescription([
#         webots, ure3_controller, ure5_controller,
#         # Shutdown launch when Webots exits.
#         launch.actions.RegisterEventHandler(
#             event_handler=launch.event_handlers.OnProcessExit(
#                 target_action=webots,
#                 on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
#             )
#         ),
#     ])
