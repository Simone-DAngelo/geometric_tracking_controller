#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lee_controller',
            executable='lee_controller',
            name='lee_controller',
            parameters=[os.path.join(
                get_package_share_directory('lee_controller'),
                'conf', 'iris_param.yaml')],
            output='screen'),
    ])

