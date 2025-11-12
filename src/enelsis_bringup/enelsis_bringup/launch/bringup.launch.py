from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    pkg = 'enelsis_bringup'
    cfg = os.path.join(get_package_share_directory(pkg), 'config')

    ydlidar = Node(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node', 
        output='screen',
        parameters=[os.path.join(cfg,'ydlidar_g2.yaml')])

    # scan_filter disabled to stabilize scan stream during bringup

    rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o', parameters=[os.path.join(cfg,'rf2o.yaml')],
        remappings=[('scan', '/scan')])

    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf', parameters=[os.path.join(cfg,'ekf.yaml')])

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', parameters=[os.path.join(cfg,'slam_toolbox.yaml')])

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0','0','0','0','0','0','base_link','laser_frame']
    )

    # Pixhawk 6X MAVROS - IMU verileri için
    mavros = Node(
        package='mavros', 
        executable='mavros_node',
        name='mavros', 
        output='screen',
        parameters=[{
            'fcu_url': 'serial:///dev/ttyACM0:921600',
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'system_id': 1,
            'component_id': 240,
            'plugin_allowlist': ['sys_*', 'imu'],
            'imu.frame_id': 'imu_link',
            'imu.linear_acceleration_stdev': 0.0003,
            'imu.angular_velocity_stdev': 0.0003490659,
            'imu.orientation_stdev': 0.0087,
            'imu.magnetic_stdev': 0.0
        }]
    )

    # IMU için TF - base_link ile imu_link arasındaki transform
    # Pixhawk'ın robota göre konumunu ayarlayın (örnek: 0.1m öne, 0m yan, 0.05m yukarı)
    imu_tf = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        name='imu_to_base_link',
        arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )

    return LaunchDescription([ydlidar, rf2o, ekf, slam, static_tf, mavros, imu_tf])
