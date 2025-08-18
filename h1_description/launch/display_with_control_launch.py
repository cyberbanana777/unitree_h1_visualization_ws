# Copyright (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
# All rights reserved.
# Modified by Alice Zenina and Alexander Grachev RTU MIREA (Russia)

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('h1_description')
    launch_description_file = os.path.join(
        pkg_path, 'launch', 'robot_description_and_tfs_launch.py'
    )
    launch_rviz_file = os.path.join(
        pkg_path, 'launch', 'rviz_with_config_launch.py'
    )

    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_description_file)
    )
    launch_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_rviz_file)
    )

    gui_node = (
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
    )

    return LaunchDescription(
        [
            launch_description,
            launch_rviz,
            gui_node,
        ]
    )
