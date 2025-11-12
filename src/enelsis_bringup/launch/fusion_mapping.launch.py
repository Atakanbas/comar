#!/usr/bin/env python3
"""
Sensor Fusion + High-Accuracy Mapping Launch File
Pixhawk 6X IMU + YDLiDAR + RF2O Odometry + EKF + SLAM
Author: AI Assistant
Date: 2025-10-21
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    enelsis_bringup_dir = get_package_share_directory('enelsis_bringup')
    config_dir = os.path.join(enelsis_bringup_dir, 'config')
    
    # Configuration files
    mavros_config = os.path.join(config_dir, 'mavros.yaml')
    ekf_config = os.path.join(config_dir, 'ekf.yaml')
    rf2o_config = os.path.join(config_dir, 'rf2o.yaml')
    slam_config = os.path.join(config_dir, 'slam_toolbox.yaml')
    lidar_config = os.path.join(config_dir, 'ydlidar_g2.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 1. YDLiDAR Driver Node
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_node',
        output='screen',
        parameters=[lidar_config],
        respawn=True,
        respawn_delay=2.0
    )
    
    # 2. Static TF: base_link -> laser_frame
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    # 3. RF2O Laser Odometry Node
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom_rf2o')
        ],
        respawn=True,
        respawn_delay=2.0
    )
    
    # 4. MAVROS Node - Pixhawk 6X IMU
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[mavros_config],
        remappings=[
            ('mavros/imu/data', '/imu/data_raw')
        ],
        respawn=True,
        respawn_delay=3.0
    )
    
    # 5. Robot Localization (EKF) - Sensor Fusion
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ],
        respawn=True,
        respawn_delay=2.0
    )
    
    # 6. SLAM Toolbox - Async Mapping
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry/filtered')
        ],
        respawn=True,
        respawn_delay=2.0
    )
    
    # 7. Static TF: odom -> base_footprint (if needed)
    tf_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        # Launch nodes
        ydlidar_node,
        tf_laser,
        rf2o_node,
        mavros_node,
        ekf_node,
        slam_toolbox_node,
        tf_base_footprint,
    ])







