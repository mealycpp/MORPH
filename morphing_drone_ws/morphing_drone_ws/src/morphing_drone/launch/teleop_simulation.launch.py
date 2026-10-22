"""
Teleop Simulation Launch File
===============================
Identical to full_simulation.launch.py but WITHOUT the autonomous
morph sequence. The drone spawns and waits for commands from the
teleop_controller node.

Usage:
    Terminal 1: ros2 launch morphing_drone teleop_simulation.launch.py
    Terminal 2: ros2 run morphing_drone teleop_controller
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('morphing_drone')

    urdf_xacro = os.path.join(pkg_dir, 'urdf', 'morphing_drone.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'empty_flight.world')

    # Process xacro to URDF string
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro]),
        value_type=str
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation time'
        ),

        # --- Gazebo ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gazebo.launch.py'
                )
            ),
            launch_arguments={
                'world': world_file,
                'verbose': 'false',
            }.items(),
        ),

        # --- Robot State Publisher (processes URDF for TF + spawn) ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # --- Spawn drone from robot_description topic ---
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'morphing_drone',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.5',
            ],
            output='screen',
        ),

        # --- Gazebo Drone Bridge ---
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='morphing_drone',
                    executable='gazebo_drone_bridge',
                    name='gazebo_drone_bridge',
                    parameters=[{
                        'use_sim_time': True,
                        'model_name': 'morphing_drone',
                    }],
                    output='screen',
                ),
            ],
        ),

        # --- Flight Controller ---
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='morphing_drone',
                    executable='flight_controller',
                    name='flight_controller',
                    parameters=[{
                        'use_sim_time': True,
                        'target_altitude': 5.0,
                        'drone_mass': 2.5,
                        'max_thrust_per_edf': 17.86,
                    }],
                    output='screen',
                ),
            ],
        ),

        # --- Morph Controller ---
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='morphing_drone',
                    executable='morph_controller',
                    name='morph_controller',
                    parameters=[{
                        'use_sim_time': True,
                        'transition_time_s': 3.0,
                    }],
                    output='screen',
                ),
            ],
        ),

        # --- Data Logger ---
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='morphing_drone',
                    executable='data_logger',
                    name='data_logger',
                    parameters=[{
                        'use_sim_time': True,
                        'log_rate_hz': 50.0,
                    }],
                    output='screen',
                ),
            ],
        ),

        # --- NO morph_sequence or obstacle_course here ---
        # The teleop_controller runs in a separate terminal:
        #   ros2 run morphing_drone teleop_controller
    ])
