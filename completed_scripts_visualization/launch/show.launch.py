# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='real',
        description='Choose robot: real (reading state from `/lowstate` or simulation (reading from `/positions_to_unitree`)',
        choices=['real', 'simulation', 'empty']
    )

    description_params = {
        'mode': LaunchConfiguration('mode'),
        'launch_rviz': LaunchConfiguration('launch_rviz'),
        'launch_control_by_gui': LaunchConfiguration('launch_control_by_gui'),
    }

    # Declare conditions
    real_robot_condition = PythonExpression([
        '"', LaunchConfiguration('robot'), '" == "real"'
    ])
    simulation_robot_condition = PythonExpression([
        '"', LaunchConfiguration('robot'), '" == "simulation"'
    ])

    # Get the paths to the launch files of other packages
    pkg1_launch_dir = os.path.join(
        get_package_share_directory('h1_description'), 'launch'
    )

    # Enabling the first launch file
    launch_file1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg1_launch_dir, 'description.launch.py')
        ),
        launch_arguments=description_params.items()
    )

    real_move_node = Node(
        package='h1_move_joint_rviz',
        executable='move_joint_rviz_with_real_robot_node',
        name='move_joint_rviz_with_real_robot_node',
        output='screen',
        condition=IfCondition(real_robot_condition),
    )

    simulation_move_node = Node(
        package='h1_move_joint_rviz',
        executable='move_joint_rviz_without_real_robot_node',
        name='move_joint_rviz_without_real_robot_node',
        output='screen',
        condition=IfCondition(simulation_robot_condition),
    )


    return LaunchDescription(
        [
            mode_arg,
            rviz_arg,
            control_by_gui_arg,
            robot_arg,

            launch_file1,
            
            real_move_node,
            simulation_move_node,
        ]
    )
