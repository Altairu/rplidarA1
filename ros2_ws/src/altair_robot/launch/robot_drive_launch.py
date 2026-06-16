"""
robot_drive_launch.py

SLAM + 自己位置可視化 一括起動ランチファイル

起動ノード:
  1. sllidar_ros2       – RPlidar A1 ドライバ
    2. cartographer_node  – SLAM (lidar_only_2d.lua)
    3. occupancy_grid     – /map 発行 (cartographer)
    4. websocket_bridge   – 自己位置配信 WebSocket サーバ (port 8876)
    5. http_server        – HTML 配信 (port 8091)
    6. mdd_can_node       – 差動二輪ノード (enable_drive=true のときのみ)

パラメータ:
  serial_port : RPlidar の USB ポート (デフォルト /dev/ttyUSB0)
  can_channel : CAN インターフェース名 (デフォルト can0)
    namespace   : トピック分離用ネームスペース (デフォルト rplidar_localization)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    can_channel = LaunchConfiguration('can_channel', default='can0')
    namespace = LaunchConfiguration('namespace', default='rplidar_localization')
    enable_drive = LaunchConfiguration('enable_drive', default='false')
    enable_realsense_imu = LaunchConfiguration('enable_realsense_imu', default='false')
    enable_spresense_imu = LaunchConfiguration('enable_spresense_imu', default='true')
    spresense_port = LaunchConfiguration('spresense_port', default='/dev/ttyUSB1')
    use_fixed_delay = LaunchConfiguration('use_fixed_delay', default='false')
    fixed_delay_sec = LaunchConfiguration('fixed_delay_sec', default='0.2')
    rs_serial_no = LaunchConfiguration('rs_serial_no', default='')
    laser_to_camera_x = LaunchConfiguration('laser_to_camera_x', default='0.0')
    laser_to_camera_y = LaunchConfiguration('laser_to_camera_y', default='0.0')
    laser_to_camera_z = LaunchConfiguration('laser_to_camera_z', default='0.0')
    laser_to_camera_roll = LaunchConfiguration('laser_to_camera_roll', default='0.0')
    laser_to_camera_pitch = LaunchConfiguration('laser_to_camera_pitch', default='0.0')
    laser_to_camera_yaw = LaunchConfiguration('laser_to_camera_yaw', default='0.0')

    html_dir = os.path.join(
        get_package_share_directory('altair_robot'), 'html')
    cartographer_cfg_dir = os.path.join(
        get_package_share_directory('lidar_processing'), 'config')
    cartographer_config_basename = PythonExpression([
        "'lidar_with_imu_2d.lua' if '",
        enable_realsense_imu,
        "' == 'true' else 'lidar_only_2d.lua'",
    ])

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
        DeclareLaunchArgument(
            'namespace',
            default_value='rplidar_localization',
            description='SLAM 系トピック分離用ネームスペース',
        ),
        DeclareLaunchArgument(
            'enable_drive',
            default_value='false',
            description='true のときのみ mdd_can_node を起動',
        ),
        DeclareLaunchArgument(
            'enable_realsense_imu',
            default_value='false',
            description='true のとき D435i IMU を自己位置推定に利用',
        ),
        DeclareLaunchArgument(
            'rs_serial_no',
            default_value='',
            description='RealSense シリアル番号 (複数台時のみ指定)',
        ),
        DeclareLaunchArgument(
            'enable_spresense_imu',
            default_value='true',
            description='true のとき Spresense IMU 自己位置推定ノードを起動',
        ),
        DeclareLaunchArgument(
            'spresense_port',
            default_value='/dev/ttyUSB1',
            description='Spresense IMU シリアルポート',
        ),
        DeclareLaunchArgument(
            'use_fixed_delay',
            default_value='false',
            description='true のとき、DBFにSLAMのタイムスタンプではなく固定遅延を使用する',
        ),
        DeclareLaunchArgument(
            'fixed_delay_sec',
            default_value='0.2',
            description='DBFに適用する固定遅延時間 [s]',
        ),
        DeclareLaunchArgument('laser_to_camera_x', default_value='0.0', description='laser->camera_link [m] x'),
        DeclareLaunchArgument('laser_to_camera_y', default_value='0.0', description='laser->camera_link [m] y'),
        DeclareLaunchArgument('laser_to_camera_z', default_value='0.0', description='laser->camera_link [m] z'),
        DeclareLaunchArgument('laser_to_camera_roll', default_value='0.0', description='laser->camera_link [rad] roll'),
        DeclareLaunchArgument('laser_to_camera_pitch', default_value='0.0', description='laser->camera_link [rad] pitch'),
        DeclareLaunchArgument('laser_to_camera_yaw', default_value='0.0', description='laser->camera_link [rad] yaw'),

        GroupAction(actions=[
            PushRosNamespace(namespace),

            # ── 1. RPlidar ドライバ (直接起動) ────────────────
            Node(
                package='sllidar_ros2',
                executable='sllidar_node',
                name='sllidar_node',
                respawn=True,
                respawn_delay=2.0,
                output='screen',
                parameters=[{
                    'channel_type': 'serial',
                    'serial_port': serial_port,
                    'serial_baudrate': 115200,
                    'frame_id': 'laser',
                    'inverted': False,
                    'angle_compensate': True,
                    'scan_mode': 'Standard',
                }],
            ),

            # ── 2. Cartographer SLAM ─────────────────────────
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                arguments=[
                    '-configuration_directory', cartographer_cfg_dir,
                    '-configuration_basename', cartographer_config_basename,
                ],
                remappings=[
                    ('imu', 'camera/camera/imu'),
                ],
            ),

            # ── 2.5 RealSense D435i (IMUのみ) ─────────────────
            Node(
                condition=IfCondition(enable_realsense_imu),
                package='realsense2_camera',
                executable='realsense2_camera_node',
                name='realsense2_camera_node',
                output='screen',
                parameters=[{
                    'camera_name': 'camera',
                    'camera_namespace': 'camera',
                    'serial_no': rs_serial_no,
                    'enable_gyro': True,
                    'enable_accel': True,
                    'unite_imu_method': 2,
                    'enable_color': False,
                    'enable_depth': False,
                    'enable_infra': False,
                    'enable_infra1': False,
                    'enable_infra2': False,
                    'publish_tf': True,
                }],
            ),

            # Cartographer が IMU を laser フレーム基準で利用できるよう固定TFを追加
            Node(
                condition=IfCondition(enable_realsense_imu),
                package='tf2_ros',
                executable='static_transform_publisher',
                name='laser_to_camera_tf',
                output='screen',
                arguments=[
                    '--x', laser_to_camera_x,
                    '--y', laser_to_camera_y,
                    '--z', laser_to_camera_z,
                    '--roll', laser_to_camera_roll,
                    '--pitch', laser_to_camera_pitch,
                    '--yaw', laser_to_camera_yaw,
                    '--frame-id', 'laser',
                    '--child-frame-id', 'camera_link',
                ],
            ),

            # ── 3. Occupancy Grid (cartographer) ─────────────
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package='cartographer_ros',
                        executable='cartographer_occupancy_grid_node',
                        name='cartographer_occupancy_grid_node',
                        namespace=namespace,
                        output='screen',
                        arguments=[
                            '-resolution', '0.05',
                            '-publish_period_sec', '1.0',
                        ],
                    )
                ],
            ),

            # ── 4. WebSocket ブリッジ (自己位置配信 & 地図配信) ─────
            Node(
                package='altair_robot',
                executable='websocket_bridge_node',
                name='websocket_bridge_node',
                output='screen',
                parameters=[{
                    'port': 8876,
                    'pose_topic': 'tracked_pose',
                    'map_topic': 'map',
                    'cartographer_config_dir': cartographer_cfg_dir,
                    'cartographer_config_basename': cartographer_config_basename,
                }],
            ),

            # ── 4.5 地図マーカー可視化 (RViz Map描画互換用) ───
            Node(
                package='altair_robot',
                executable='map_marker_node',
                name='map_marker_node',
                output='screen',
                parameters=[{
                    'map_topic': 'map',
                    'marker_topic': 'map_marker',
                    'occupied_threshold': 50,
                    'cell_skip': 2,
                }],
            ),

            # ── 5. 差動二輪ノード (既定は無効) ───────────────
            Node(
                condition=IfCondition(enable_drive),
                package='altair_robot',
                executable='mdd_can_node',
                name='mdd_can_node',
                output='screen',
                parameters=[{
                    'can_channel': can_channel,
                    'can_bitrate': 1000000,
                    'wheel_diameter': 0.200,
                    'wheel_base': 0.500,
                    'pid_p': 100.0,
                    'pid_i': 0.0,
                    'pid_d': 0.0,
                    'max_rps': 20.0,
                }],
            ),

            # ── 5.3 Spresense IMU 自己位置推定 ─────────────────
            Node(
                condition=IfCondition(enable_spresense_imu),
                package='altair_robot',
                executable='spresense_imu_node',
                name='spresense_imu_node',
                output='screen',
                parameters=[{
                    'serial_port': spresense_port,
                    'baudrate': 2000000,
                    'odom_frame': 'imu_odom',
                    'base_frame': 'imu_base_link',
                    'publish_tf': True,
                    'use_fixed_delay': use_fixed_delay,
                    'fixed_delay_sec': fixed_delay_sec,
                }],
            ),

            # ── 5.7 実験データロガー ──────────────────────────
            Node(
                condition=IfCondition(enable_spresense_imu),
                package='altair_robot',
                executable='experiment_logger_node',
                name='experiment_logger_node',
                output='screen',
                parameters=[{
                    'log_dir': '/home/altair/rplidarA1/experiment_logs',
                    'log_rate': 20.0,
                }],
            ),

            # ── 6. RViz2 (同一 namespace で購読) ──────────────
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                additional_env={
                    'LIBGL_ALWAYS_SOFTWARE': '1',
                    'MESA_GL_VERSION_OVERRIDE': '3.3',
                    'MESA_GLSL_VERSION_OVERRIDE': '330',
                    'QT_XCB_GL_INTEGRATION': 'none',
                    'OGRE_RTT_MODE': 'Copy',
                },
                output='screen',
                arguments=[
                    '-d', os.path.join(
                        get_package_share_directory('altair_robot'),
                        'config', 'robot_drive.rviz')
                ],
            ),
        ]),

        # ── 7. HTML 静的ファイルサーバー (port 8091) ─────────
        #    Android タブレットから http://<ロボットIP>:8091/ でアクセス
        ExecuteProcess(
            cmd=[
                'python3', '-m', 'http.server', '8091',
                '--directory', html_dir,
            ],
            output='screen',
            name='html_http_server',
        ),

    ])
