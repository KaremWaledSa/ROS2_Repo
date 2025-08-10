#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the package directories
    pkg_share = FindPackageShare('diffrobot_gazebo')
    pkg_description = FindPackageShare('diffrobot_description')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Robot pose arguments
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.0', description='Z position')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.0', description='Roll')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw')
    
    # World file path
    world_file = PathJoinSubstitution([
        pkg_share,
        'world',
        'home.world'
    ])
    
    # Include robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_description,
            '/launch',
            '/robot_description.launch.py'
        ])
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'),
            '/launch',
            '/gazebo.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': world_file,
            'gui': 'true',
            'debug': 'false'
        }.items()
    )
    
    # Spawn robot
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-entity', 'diffrobot',
            '-file', PathJoinSubstitution([
                pkg_description,
                'urdf',
                'diffrobot.xacro'
            ]),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw')
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        x_arg, y_arg, z_arg, roll_arg, pitch_arg, yaw_arg,
        
        # Launch nodes
        robot_description_launch,
        gazebo_launch,
        spawn_robot_node
    ]) 