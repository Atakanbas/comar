#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('enelsis_bringup')
    config_dir = os.path.join(pkg_share, 'config')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # MAVROS node for IMU data
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[os.path.join(config_dir, 'mavros.yaml')],
        remappings=[
            ('/mavros/imu/data', '/mavros/imu/data'),
            ('/mavros/imu/mag', '/mavros/imu/mag')
        ]
    )
    
    # RF2O Laser Odometry node
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'rf2o.yaml')],
        remappings=[
            ('/laser_scan', '/scan')
        ]
    )
    
    # EKF Sensor Fusion node (IMU + Lidar)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'ekf_enhanced.yaml')],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered')
        ]
    )
    
    # Cartographer node with IMU + Lidar fusion./
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_sera.lua'
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry/filtered'),  # Use EKF fused odometry instead of raw RF2O
            ('imu', '/mavros/imu/data')      # Add IMU data for Cartographer
        ]
    )
    
    # Occupancy grid publisher
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        mavros_node,           # IMU data from Pixhawk
        rf2o_node,            # Laser odometry
        ekf_node,             # Sensor fusion (IMU + Lidar)
        cartographer_node,    # SLAM with fused data
        occupancy_grid_node   # Map publisher
    ])
