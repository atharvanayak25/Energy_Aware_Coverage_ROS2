#!/usr/bin/env python3
"""
ROS2 Launch file for SAR Coverage System with Battery Management.
Launches coverage controller and visualization.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare launch arguments
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='3',
        description='Number of drones'
    )
    
    altitude_arg = DeclareLaunchArgument(
        'flight_altitude',
        default_value='5.0',
        description='Flight altitude in meters'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='10.0',
        description='Control loop frequency in Hz'
    )
    
    # Coverage control node with battery management
    coverage_node = Node(
        package='sar_coverage',
        executable='coverage_node',
        name='coverage_controller',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'num_drones': LaunchConfiguration('num_drones'),
            'flight_altitude': LaunchConfiguration('flight_altitude'),
            'control_rate': LaunchConfiguration('control_rate'),
        }]
    )
    
    # Delay coverage node start to allow simulation to initialize
    delayed_coverage = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="Starting coverage controller with battery management..."),
            coverage_node
        ]
    )
    
    # RViz node (optional - uncomment when ready)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        # arguments=['-d', '$(find sar_coverage)/config/coverage_viz.rviz'],
        condition=None
    )
    
    return LaunchDescription([
        # Arguments
        num_drones_arg,
        altitude_arg,
        control_rate_arg,
        
        # Info messages
        LogInfo(msg="=========================================="),
        LogInfo(msg="  SAR Coverage System with Battery Model"),
        LogInfo(msg="=========================================="),
        LogInfo(msg="Battery discharge: ~10-12 min flight time"),
        LogInfo(msg="Charging: ~3-4 min at 250W"),
        LogInfo(msg="Waiting 5 seconds for simulation..."),
        
        # Nodes (delayed start)
        delayed_coverage,
        
        # Uncomment when ready:
        # rviz_node,
    ])