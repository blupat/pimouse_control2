#!/usr/bin/env python3
#
# =======================================================================
#   @file   pimouse_control2.launch.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    cv_params_file = launch.substitutions.LaunchConfiguration(
        'cv_params',
        default=[launch.substitutions.ThisLaunchFileDir(), '/cv_camera.yaml']
    )
    control_params_file = launch.substitutions.LaunchConfiguration(
        'control_params',
        default=[launch.substitutions.ThisLaunchFileDir(), '/pimouse_control2.yaml']
    )
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cv_camera', node_executable='cv_camera_node', output='screen',
            parameters=[cv_params_file]),
        launch_ros.actions.Node(
            package='web_video_server', node_executable='web_video_server', output='screen',
            parameters=[{'port': 10000}]),
        launch_ros.actions.Node(
            package='pimouse_ros2', node_executable='lightsensors', output='screen',
            parameters=[{'lightsensors_period': 0.05}]),
        launch_ros.actions.Node(
            package='pimouse_ros2', node_executable='motors', output='screen'),
        launch_ros.actions.Node(
            package='pimouse_control2', node_executable='pimouse_control_node', output='screen',
            parameters=[control_params_file]),
    ])
