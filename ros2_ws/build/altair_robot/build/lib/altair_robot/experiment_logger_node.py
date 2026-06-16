#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
実験データロガーノード (ROS2)

SLAM位置、IMU生積分位置、カルマンフィルタ(KF)融合位置、遅延バイアスフィードバック(DBF)融合位置、
モータ指令値、生IMUデータをまとめて同期的にCSVに保存します。
"""

import csv
import os
import math
from datetime import datetime
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


class ExperimentLoggerNode(Node):

    def __init__(self):
        super().__init__('experiment_logger_node')

        # パラメータ宣言
        self.declare_parameter('log_dir', '/home/altair/rplidarA1/experiment_logs')
        self.declare_parameter('log_rate', 20.0)  # ロギング周波数 (Hz)

        self.log_dir = self.get_parameter('log_dir').value
        self.log_rate = self.get_parameter('log_rate').value

        # ディレクトリ作成
        os.makedirs(self.log_dir, exist_ok=True)

        # 状態管理変数
        self.lock = threading.Lock()
        self.is_logging = False
        self.csv_file = None
        self.csv_writer = None
        self.start_time = 0.0

        # 各トピックからの最新データキャッシュ
        self.slam_pose = [0.0, 0.0, 0.0]       # x, y, yaw
        self.imu_raw_pose = [0.0, 0.0, 0.0]   # x, y, yaw
        self.imu_kf_pose = [0.0, 0.0, 0.0]    # x, y, yaw
        self.imu_dbf_pose = [0.0, 0.0, 0.0]   # x, y, yaw
        self.cmd_vel = [0.0, 0.0]             # v, w
        self.imu_raw = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # ax, ay, az, gx, gy, gz

        # サブスクライバ
        self.slam_sub = self.create_subscription(
            PoseStamped, 'tracked_pose', self._slam_pose_cb, 10)
        self.imu_raw_pose_sub = self.create_subscription(
            PoseStamped, 'imu_pose_raw', self._imu_raw_pose_cb, 10)
        self.imu_kf_pose_sub = self.create_subscription(
            PoseStamped, 'imu_pose_kf', self._imu_kf_pose_cb, 10)
        self.imu_dbf_pose_sub = self.create_subscription(
            PoseStamped, 'imu_pose_dbf', self._imu_dbf_pose_cb, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.imu_raw_sub = self.create_subscription(
            Imu, 'imu_raw', self._imu_raw_cb, 10)

        # サービス
        self.start_srv = self.create_service(
            Trigger, '~/start', self._start_logging_cb)
        self.stop_srv = self.create_service(
            Trigger, '~/stop', self._stop_logging_cb)

        # 周期ロギングタイマー
        self.timer = self.create_timer(1.0 / self.log_rate, self._log_timer_cb)

        self.get_logger().info('実験データロガーノードが正常に初期化されました。')

    def _get_yaw(self, orientation):
        qz = orientation.z
        qw = orientation.w
        return 2.0 * math.atan2(qz, qw)

    def _slam_pose_cb(self, msg: PoseStamped):
        yaw = self._get_yaw(msg.pose.orientation)
        with self.lock:
            self.slam_pose = [msg.pose.position.x, msg.pose.position.y, yaw]

    def _imu_raw_pose_cb(self, msg: PoseStamped):
        yaw = self._get_yaw(msg.pose.orientation)
        with self.lock:
            self.imu_raw_pose = [msg.pose.position.x, msg.pose.position.y, yaw]

    def _imu_kf_pose_cb(self, msg: PoseStamped):
        yaw = self._get_yaw(msg.pose.orientation)
        with self.lock:
            self.imu_kf_pose = [msg.pose.position.x, msg.pose.position.y, yaw]

    def _imu_dbf_pose_cb(self, msg: PoseStamped):
        yaw = self._get_yaw(msg.pose.orientation)
        with self.lock:
            self.imu_dbf_pose = [msg.pose.position.x, msg.pose.position.y, yaw]

    def _cmd_vel_cb(self, msg: Twist):
        with self.lock:
            self.cmd_vel = [msg.linear.x, msg.angular.z]

    def _imu_raw_cb(self, msg: Imu):
        with self.lock:
            self.imu_raw = [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]

    def _start_logging_cb(self, request, response):
        """録画開始"""
        with self.lock:
            if self.is_logging:
                response.success = False
                response.message = 'ロギングは既に開始されています。'
                return response

            try:
                filename = datetime.now().strftime('%Y%m%d_%H%M%S_experiment.csv')
                filepath = os.path.join(self.log_dir, filename)
                self.csv_file = open(filepath, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_file)

                # ヘッダー行
                self.csv_writer.writerow([
                    'elapsed_sec',
                    'slam_x', 'slam_y', 'slam_yaw',
                    'imu_raw_x', 'imu_raw_y', 'imu_raw_yaw',
                    'imu_kf_x', 'imu_kf_y', 'imu_kf_yaw',
                    'imu_dbf_x', 'imu_dbf_y', 'imu_dbf_yaw',
                    'cmd_vel_v', 'cmd_vel_w',
                    'ax', 'ay', 'az',
                    'gx', 'gy', 'gz'
                ])

                self.is_logging = True
                self.start_time = self.get_clock().now().nanoseconds / 1e9

                response.success = True
                response.message = f'実験ログの記録を開始しました。保存先: {filepath}'
                self.get_logger().info(response.message)
            except Exception as e:
                response.success = False
                response.message = f'ログファイル作成失敗: {e}'
                self.get_logger().error(response.message)

        return response

    def _stop_logging_cb(self, request, response):
        """録画停止"""
        with self.lock:
            if not self.is_logging:
                response.success = False
                response.message = 'ロギングは開始されていません。'
                return response

            try:
                self.csv_file.flush()
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None
                self.is_logging = False

                response.success = True
                response.message = '実験ログの記録を停止し、ファイルを正常に保存しました。'
                self.get_logger().info(response.message)
            except Exception as e:
                response.success = False
                response.message = f'ログ保存時のクローズエラー: {e}'
                self.get_logger().error(response.message)

        return response

    def _log_timer_cb(self):
        """タイマー割り込みによるCSV出力 (20Hz)"""
        with self.lock:
            if not self.is_logging or self.csv_writer is None:
                return

            now = self.get_clock().now().nanoseconds / 1e9
            elapsed = now - self.start_time

            row = [
                f"{elapsed:.4f}",
                f"{self.slam_pose[0]:.4f}", f"{self.slam_pose[1]:.4f}", f"{self.slam_pose[2]:.4f}",
                f"{self.imu_raw_pose[0]:.4f}", f"{self.imu_raw_pose[1]:.4f}", f"{self.imu_raw_pose[2]:.4f}",
                f"{self.imu_kf_pose[0]:.4f}", f"{self.imu_kf_pose[1]:.4f}", f"{self.imu_kf_pose[2]:.4f}",
                f"{self.imu_dbf_pose[0]:.4f}", f"{self.imu_dbf_pose[1]:.4f}", f"{self.imu_dbf_pose[2]:.4f}",
                f"{self.cmd_vel[0]:.4f}", f"{self.cmd_vel[1]:.4f}",
                f"{self.imu_raw[0]:.6f}", f"{self.imu_raw[1]:.6f}", f"{self.imu_raw[2]:.6f}",
                f"{self.imu_raw[3]:.6f}", f"{self.imu_raw[4]:.6f}", f"{self.imu_raw[5]:.6f}"
            ]

            try:
                self.csv_writer.writerow(row)
            except Exception as e:
                self.get_logger().error(f'CSV書き出しエラー: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.is_logging:
            node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
