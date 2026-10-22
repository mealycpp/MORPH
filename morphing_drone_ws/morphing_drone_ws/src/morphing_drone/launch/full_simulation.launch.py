"""
Full Simulation Launch File
-----------------------------
Launches:
  1. Gazebo with empty flight world
  2. Robot state publisher (URDF → TF)
  3. Flight controller (PID hover)
  4. Morph controller (servo joints)
  5. Data logger (CSV recording)
  6. Morph sequence orchestrator (X → O → T → H)
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

    # Paths
    urdf_xacro = os.path.join(pkg_dir, 'urdf', 'morphing_drone.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'empty_flight.world')
    drone_params = os.path.join(pkg_dir, 'config', 'drone_params.yaml')
    pid_params = os.path.join(pkg_dir, 'config', 'pid_gains.yaml')

    # Process xacro to URDF string — wrap as ParameterValue(str) so
    # robot_state_publisher doesn't try to parse the XML as YAML
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro]),
        value_type=str
    )

    return LaunchDescription([

        # --- Launch Arguments ---
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation time'
        ),
        DeclareLaunchArgument(
            'target_altitude', default_value='5.0',
            description='Hover altitude in meters'
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

        # --- Spawn drone in Gazebo ---
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'morphing_drone',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.5',  # Start slightly above ground
            ],
            output='screen',
        ),

        # --- Robot State Publisher (TF tree) ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # --- Gazebo Drone Bridge (applies forces to links via service) ---
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
        # Delay 3s to let Gazebo finish loading
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

        # --- Morph Sequence Orchestrator ---
        # Start after 5s to let everything initialize
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='morphing_drone',
                    executable='run_morph_sequence',
                    name='morph_sequence',
                    parameters=[{
                        'use_sim_time': True,
                    }],
                    output='screen',
                ),
            ],
        ),
    ])
