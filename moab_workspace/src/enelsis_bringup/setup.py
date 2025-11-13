from setuptools import find_packages, setup

package_name = 'enelsis_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/enelsis_bringup']),
    ('share/enelsis_bringup', ['package.xml']),
    ('share/enelsis_bringup/launch', [
        'enelsis_bringup/launch/bringup.launch.py',
        'launch/cartographer_sera.launch.py'
    ]),
    ('share/enelsis_bringup/config', [
        'config/ydlidar_g2.yaml','config/laser_filters.yaml',
        'config/rf2o.yaml','config/ekf.yaml','config/slam_toolbox.yaml',
        'config/mavros.yaml','config/ekf_enhanced.yaml','config/ekf_imu_lidar.yaml',
        'config/cartographer_sera.lua','config/laser_filters_sera.yaml',
        'config/slam_toolbox_simple.yaml','config/visualization.rviz'
    ]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enelsis-pc',
    maintainer_email='enelsis-pc@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
