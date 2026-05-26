"""
robot_drive_launch.py

ロボット走行 + SLAM 一括起動ランチファイル

起動ノード:
  1. sllidar_ros2       – RPlidar A1 ドライバ
  2. static_tf          – base_link → laser_frame 変換
  3. cartographer_node  – SLAM (lidar_only_2d.lua)
  4. occupancy_grid     – /map 発行 (cartographer)
  5. mdd_can_node       – MDD CAN ドライバ + 差動二輪運動学
  6. websocket_bridge   – Android タブレット WebSocket サーバ (port 8765)
  7. http_server        – Android タブレット向け HTML 配信 (port 8080)

パラメータ:
  serial_port : RPlidar の USB ポート (デフォルト /dev/ttyUSB0)
  can_channel : CAN インターフェース名 (デフォルト can0)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    can_channel = LaunchConfiguration('can_channel', default='can0')

    html_dir = os.path.join(
        get_package_share_directory('altair_robot'), 'html')

    return LaunchDescription([

        # ── 引数宣言 ─────────────────────────────────────────
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='RPlidar USB シリアルポート',
        ),
        DeclareLaunchArgument(
            'can_channel',
            default_value='can0',
            description='socketcan チャンネル名 (例: can0, vcan0)',
        ),

        # ── 1. RPlidar ドライバ ──────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'),
                    'launch',
                    'sllidar_a1_launch.py',
                ])
            ),
            launch_arguments={'serial_port': serial_port}.items(),
        ),

        # ── 2. 静的 TF: base_link → laser_frame ─────────────
        #    ライダーはロボット中心の上部に搭載 (z=0.15m)
        #    実機に合わせて x, y, z, yaw を調整してください
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '--x',   '0.0',
                '--y',   '0.0',
                '--z',   '0.15',
                '--yaw', '0.0',
                '--pitch', '0.0',
                '--roll',  '0.0',
                '--frame-id',       'base_link',
                '--child-frame-id', 'laser_frame',
            ],
        ),

        # ── 3. Cartographer SLAM ─────────────────────────────
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory',
                PathJoinSubstitution([
                    FindPackageShare('lidar_processing'), 'config'
                ]),
                '-configuration_basename', 'lidar_only_2d.lua',
            ],
        ),

        # ── 4. Occupancy Grid (cartographer) ─────────────────
        #    5 秒遅延: DDS Transient Local キャッシュ期限切れ待ち
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='cartographer_ros',
                    executable='cartographer_occupancy_grid_node',
                    name='cartographer_occupancy_grid_node',
                    output='screen',
                    arguments=[
                        '-resolution',         '0.05',
                        '-publish_period_sec', '1.0',
                    ],
                )
            ],
        ),

        # ── 5. MDD CAN ドライバ ──────────────────────────────
        Node(
            package='altair_robot',
            executable='mdd_can_node',
            name='mdd_can_node',
            output='screen',
            parameters=[{
                'can_channel':    can_channel,
                'can_bitrate':    1000000,
                'wheel_diameter': 0.200,   # m (200 mm)
                'wheel_base':     0.500,   # m (500 mm)
                'pid_p':          100.0,
                'pid_i':          0.0,
                'pid_d':          0.0,
                'max_rps':        20.0,
            }],
        ),

        # ── 6. WebSocket ブリッジ ────────────────────────────
        Node(
            package='altair_robot',
            executable='websocket_bridge_node',
            name='websocket_bridge_node',
            output='screen',
            parameters=[{
                'port':              8765,
                'map_interval_sec':  1.0,
                'pose_interval_sec': 0.1,
                'max_linear':        2.0,   # m/s
                'max_angular':       10.0,  # rad/s
            }],
        ),

        # ── 7. HTML 静的ファイルサーバー (port 8080) ─────────
        #    Android タブレットから http://<ロボットIP>:8080/ でアクセス
        ExecuteProcess(
            cmd=[
                'python3', '-m', 'http.server', '8080',
                '--directory', html_dir,
            ],
            output='screen',
            name='html_http_server',
        ),

        # ── 8. RViz2 (PC で地図・自己位置確認) ──────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', os.path.join(
                    get_package_share_directory('altair_robot'),
                    'config', 'robot_drive.rviz')
            ],
        ),

    ])
