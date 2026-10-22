"""
Obstacle Course Launch File
----------------------------
Launches the full autonomous obstacle course demonstration:
  1. Gazebo with morph_course world (pylons, tunnel, gates, wall, slot, landing pad)
  2. Robot state publisher (URDF → TF)
  3. Gazebo drone bridge (applies forces to base_link)
  4. Flight controller (PID with waypoint tracking)
  5. Morph controller (servo joints X/O/Y/T/H)
  6. Data logger (CSV recording)
  7. Obstacle course mission runner (autonomous waypoint follower)

Usage:
  ros2 launch morphing_drone obstacle_course.launch.py

Mission:
  Zone 1 — X: pylon weaving     (baseline stable flight)
  Zone 2 — O: narrow tunnel     (compact footprint)
  Zone 3 — Y: gate slalom       (forward cruise)
  Zone 4 — T: wall inspection   (asymmetric body offset)
  Zone 5 — H: slot + landing    (narrow profile + wide stance)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('morphing_drone')

    urdf_xacro = os.path.join(pkg_dir, 'urdf', 'morphing_drone.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'morph_course.world')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro]),
        value_type=str
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation time'
        ),

        # --- Gazebo with the course world ---
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

        # --- Spawn drone at the takeoff pad ---
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

        # --- Robot State Publisher ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
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
                        'target_altitude': 3.0,  # Initial hover altitude (overridden by waypoints)
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

        # --- Obstacle Course Mission Runner ---
        # Starts 6s in — gives the drone time to settle on the ground first
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='morphing_drone',
                    executable='run_obstacle_course',
                    name='obstacle_course_mission',
                    parameters=[{
                        'use_sim_time': True,
                    }],
                    output='screen',
                ),
            ],
        ),
    ])
