# Copyright (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
# All rights reserved.
# Modified by Alice Zenina and Alexander Grachev RTU MIREA (Russia)

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():

    # Declare arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='with_hands',
        description='Robot configuration: with_hands or without_hands.',
        choices=['with_hands', 'without_hands']
    )

    rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='False',
        description='Enable or disable rviz',
        choices=['False', 'True']
    )

    control_by_gui_arg = DeclareLaunchArgument(
        'launch_control_by_gui',
        default_value='False',
        description='Enable or disable joint_state_publisher_gui (control_by_gui)',
        choices=['False', 'True']
    )

    # Declare conditions
    with_hands_condition = PythonExpression([
        '"', LaunchConfiguration('mode'), '" == "with_hands"'
    ])

    without_hands_condition = PythonExpression([
        '"', LaunchConfiguration('mode'), '" == "without_hands"'
    ])

    gui_condition = PythonExpression([
        '"', LaunchConfiguration('launch_control_by_gui'), '" == "True"'
    ])

    rviz_condition = PythonExpression([
        '"', LaunchConfiguration('launch_rviz'), '" == "True"'
    ])

    # Configure nodes
    pkg_path = get_package_share_directory('h1_description')
    rviz_config = os.path.join(pkg_path, 'rviz', 'check_joint.rviz')

    urdf_file_with_hands = os.path.join(pkg_path, 'urdf', 'h1_with_hand.urdf')
    urdf_file_without_hands = os.path.join(pkg_path, 'urdf', 'h1.urdf')

    # Declare nodes
    robot_state_publisher_with_hands_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_with_hands).read()}],
        condition=IfCondition(with_hands_condition),
    )

    robot_state_publisher_without_hands_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_without_hands).read()}],
        condition=IfCondition(without_hands_condition),
    )

    tf2_from_base_footprint_to_pelvis = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0',
            '0.0',
            '0.95',
            '0.0',
            '0.0',
            '0.0',
            'base_footprint',
            'pelvis',
        ],
    )

    gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui_condition),
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz_condition),
    )


    return LaunchDescription(
        [
            mode_arg,
            rviz_arg,
            control_by_gui_arg,

            robot_state_publisher_with_hands_node,
            robot_state_publisher_without_hands_node,
            tf2_from_base_footprint_to_pelvis,
            rviz_node,
            gui_node,
        ]
    )