import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB1',
            description='Specifying usb port to connected lidar'
        ),
        
        # RPLIDAR Node (using sllidar_ros2's launch file)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'),
                    'launch',
                    'sllidar_a1_launch.py'
                ])
            ),
            launch_arguments={'serial_port': serial_port}.items(),
        ),
        
        # RANSAC Node
        Node(
            package='lidar_processing',
            executable='ransac_node',
            name='ransac_node',
            output='screen'
        ),
        

        # Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', PathJoinSubstitution([FindPackageShare('lidar_processing'), 'config']),
                '-configuration_basename', 'lidar_only_2d.lua'
            ]
        ),
        
        # Cartographer Occupancy Grid Node
        # 5秒遅延: 前回実行の DDS Transient Local キャッシュが期限切れになるのを待つ
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='cartographer_ros',
                    executable='cartographer_occupancy_grid_node',
                    name='cartographer_occupancy_grid_node',
                    output='screen',
                    arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
                )
            ]
        ),
        
        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', PathJoinSubstitution([FindPackageShare('lidar_processing'), 'config', 'slam_view.rviz'])
            ]
        )
    ])
