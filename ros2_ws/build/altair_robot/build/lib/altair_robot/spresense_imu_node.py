#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spresense IMU 自己位置推定・融合ノード (ROS2)

Spresense IMU を用いた自己位置推定において、
1. 角度ソース (ジャイロ積分 / SLAM / 指令角速度)
2. 位置ソース (加速度二重積分 / SLAM / 指令速度デッドレコニング)
を動的パラメータで切り替え可能にします。
また、ジャイロの度/秒(dps)からラジアン/秒への変換処理を実装し、ドリフトを抑止します。
"""

import math
import struct
import threading
import time
import collections
import glob
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


class PositionKalmanFilter2D:
    def __init__(self, q_pos=1e-3, q_vel=1e-2, q_bias=1e-5, r_pos=5.0):
        self.x = [0.0, 0.0, 0.0]  # pos, vel, bias
        self.P = [
            [0.1, 0.0, 0.0],
            [0.0, 0.1, 0.0],
            [0.0, 0.0, 0.1]
        ]
        self.Q = [
            [q_pos, 0.0, 0.0],
            [0.0, q_vel, 0.0],
            [0.0, 0.0, q_bias]
        ]
        self.R = r_pos

    def copy(self):
        new_kf = PositionKalmanFilter2D(self.Q[0][0], self.Q[1][1], self.Q[2][2], self.R)
        new_kf.x = list(self.x)
        new_kf.P = [list(row) for row in self.P]
        return new_kf

    def set_state(self, x, P):
        self.x = list(x)
        self.P = [list(row) for row in P]

    def predict(self, acc_input, dt):
        F = [
            [1.0, dt, -0.5 * dt**2],
            [0.0, 1.0, -dt],
            [0.0, 0.0, 1.0]
        ]
        B = [0.5 * dt**2, dt, 0.0]
        
        x_new = [
            F[0][0]*self.x[0] + F[0][1]*self.x[1] + F[0][2]*self.x[2] + B[0]*acc_input,
            F[1][0]*self.x[0] + F[1][1]*self.x[1] + F[1][2]*self.x[2] + B[1]*acc_input,
            F[2][0]*self.x[0] + F[2][1]*self.x[1] + F[2][2]*self.x[2] + B[2]*acc_input
        ]
        
        P_temp = [[0.0]*3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                P_temp[i][j] = F[i][0]*self.P[0][j] + F[i][1]*self.P[1][j] + F[i][2]*self.P[2][j]
                
        P_new = [[0.0]*3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                P_new[i][j] = P_temp[i][0]*F[j][0] + P_temp[i][1]*F[j][1] + P_temp[i][2]*F[j][2] + self.Q[i][j]
                
        self.x = x_new
        self.P = P_new

    def update(self, z):
        y = z - self.x[0]
        S = self.P[0][0] + self.R
        K = [self.P[0][0] / S, self.P[1][0] / S, self.P[2][0] / S]
        
        self.x = [
            self.x[0] + K[0] * y,
            self.x[1] + K[1] * y,
            self.x[2] + K[2] * y
        ]
        
        I_KH = [
            [1.0 - K[0], 0.0, 0.0],
            [-K[1], 1.0, 0.0],
            [-K[2], 0.0, 1.0]
        ]
        
        P_new = [[0.0]*3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                P_new[i][j] = I_KH[i][0]*self.P[0][j] + I_KH[i][1]*self.P[1][j] + I_KH[i][2]*self.P[2][j]
                
        self.P = P_new


class DelayedBiasFeedback2D:
    def __init__(self, delay_sec=0.5):
        self.pos = 0.0
        self.vel = 0.0
        self.history = collections.deque(maxlen=10000)
        self.last_update_time = None
        self.delay_sec = delay_sec

    def predict(self, acc_input, dt, timestamp):
        self.vel += acc_input * dt
        self.pos += self.vel * dt
        self.history.append((timestamp, self.pos, self.vel))

    def update(self, z, slam_time, current_time, use_fixed_delay=False, fixed_delay_sec=0.2):
        if len(self.history) < 2:
            return

        if use_fixed_delay:
            target_time = current_time - fixed_delay_sec
        else:
            target_time = slam_time
        
        best_idx = 0
        min_diff = float('inf')
        for idx, (t_hist, p_hist, v_hist) in enumerate(self.history):
            diff = abs(t_hist - target_time)
            if diff < min_diff:
                min_diff = diff
                best_idx = idx
                
        t_hist, p_hist, v_hist = self.history[best_idx]
        delta_p = p_hist - z
        
        if self.last_update_time is not None:
            delta_t = t_hist - self.last_update_time
        else:
            delta_t = 0.1
            
        if delta_t <= 0.0:
            delta_t = 0.001
            
        v_bias = delta_p / delta_t
        tau = current_time - t_hist
        if tau < 0.0:
            tau = 0.0
            
        self.vel -= v_bias
        self.pos -= (delta_p + v_bias * tau)
        self.last_update_time = t_hist
        
        new_history = collections.deque(maxlen=10000)
        for t_h, p_h, v_h in self.history:
            dt_h = t_h - t_hist
            p_corrected = p_h - delta_p - v_bias * dt_h
            v_corrected = v_h - v_bias
            new_history.append((t_h, p_corrected, v_corrected))
        self.history = new_history


class SpresenseImuNode(Node):

    def __init__(self):
        super().__init__('spresense_imu_node')

        # パラメータの宣言
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 2000000)
        self.declare_parameter('odom_frame', 'imu_odom')
        self.declare_parameter('base_frame', 'imu_base_link')
        self.declare_parameter('publish_tf', True)
        
        # 遅延補正用のパラメータ（デフォルトを1.0秒固定遅延に設定）
        self.declare_parameter('use_fixed_delay', True)
        self.declare_parameter('fixed_delay_sec', 1.0)
        
        # カルマンフィルター共分散用パラメータ
        self.declare_parameter('kf_q_pos', 0.1)
        self.declare_parameter('kf_q_vel', 0.1)
        self.declare_parameter('kf_q_bias', 1e-4)
        self.declare_parameter('kf_r_pos', 0.1)
        
        # ジャイロの単位設定（度/秒 dps の場合は True にして内部でラジアンに変換。Spresenseは既にrad/sのためFalseがデフォルト）
        self.declare_parameter('gyro_in_deg_sec', False)
        
        # ZUPT（静止時リセット）用のパラメータ
        self.declare_parameter('zupt_mode', 'auto')
        self.declare_parameter('gyro_still_thresh', 0.02)
        self.declare_parameter('acc_still_thresh', 0.05)
        self.declare_parameter('still_delay_samples', 200)
        
        # 推定アルゴリズム切り替えパラメータ
        # yaw_source: 'gyro' (ジャイロ積分), 'slam' (SLAMのYaw), 'cmd' (指令角速度積分)
        self.declare_parameter('yaw_source', 'gyro')
        # pos_source: 'acc' (加速度二重積分), 'slam' (SLAM位置直接), 'cmd' (指令速度デッドレコニング)
        self.declare_parameter('pos_source', 'acc')
        
        # キャリブレーション進捗用の動的パラメータ
        self.declare_parameter('calib_in_progress', False)
        self.declare_parameter('calib_seconds_left', 0.0)
        self.declare_parameter('exclude_port', '/dev/ttyUSB0')

        # センサ極性パラメータ
        self.declare_parameter('invert_ax', True)
        self.declare_parameter('invert_ay', False)
        self.declare_parameter('invert_az', False)
        self.declare_parameter('invert_gx', False)
        self.declare_parameter('invert_gy', False)
        self.declare_parameter('invert_gz', False)

        # パラメータの取得と初期化
        self.port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.use_fixed_delay = self.get_parameter('use_fixed_delay').value
        self.fixed_delay_sec = self.get_parameter('fixed_delay_sec').value
        
        self.kf_q_pos = self.get_parameter('kf_q_pos').value
        self.kf_q_vel = self.get_parameter('kf_q_vel').value
        self.kf_q_bias = self.get_parameter('kf_q_bias').value
        self.kf_r_pos = self.get_parameter('kf_r_pos').value
        
        self.zupt_mode = self.get_parameter('zupt_mode').value
        self.gyro_still_thresh = self.get_parameter('gyro_still_thresh').value
        self.acc_still_thresh = self.get_parameter('acc_still_thresh').value
        self.still_delay_samples = self.get_parameter('still_delay_samples').value
        self.still_counter = 0
        
        self.gyro_in_deg_sec = self.get_parameter('gyro_in_deg_sec').value
        self.yaw_source = self.get_parameter('yaw_source').value
        self.pos_source = self.get_parameter('pos_source').value
        self.exclude_port = self.get_parameter('exclude_port').value
        
        self.invert_ax = self.get_parameter('invert_ax').value
        self.invert_ay = self.get_parameter('invert_ay').value
        self.invert_az = self.get_parameter('invert_az').value
        self.invert_gx = self.get_parameter('invert_gx').value
        self.invert_gy = self.get_parameter('invert_gy').value
        self.invert_gz = self.get_parameter('invert_gz').value

        # パブリッシャの作成
        self.pose_raw_pub = self.create_publisher(PoseStamped, 'imu_pose_raw', 10)
        self.pose_kf_pub = self.create_publisher(PoseStamped, 'imu_pose_kf', 10)
        self.pose_dbf_pub = self.create_publisher(PoseStamped, 'imu_pose_dbf', 10)
        self.odom_pub = self.create_publisher(Odometry, 'imu_odom', 10)
        self.imu_raw_pub = self.create_publisher(Imu, 'imu_raw', 10)
        self.calib_status_pub = self.create_publisher(Float32, 'imu_calib_status', 10)
        
        self.tf_bc = TransformBroadcaster(self)

        # サブスクライバの作成
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.slam_pose_sub = self.create_subscription(
            PoseStamped, 'tracked_pose', self._slam_pose_cb, 10)

        # 動的パラメータ更新コールバックの追加
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # キャリブレーションサービス
        self.calib_srv = self.create_service(Trigger, 'spresense_imu_node/calibrate', self._trigger_calib_cb)

        # 内部状態変数
        self.lock = threading.Lock()
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = time.perf_counter()

        # 3次元姿勢（クオータニオン）と Mahony フィルタ
        self.q = [1.0, 0.0, 0.0, 0.0]     # [qw, qx, qy, qz]
        self.e_int = [0.0, 0.0, 0.0]      # Mahony誤差積分
        self.g_calib = 9.80665            # 重力加速度 [m/s^2] (キャリブレーションで上書き)

        # 10秒キャリブレーション変数
        self.calib_in_progress = False
        self.calib_required_count = 19200 # 10秒間(1920Hz)
        self.ax_bias = 0.0
        self.ay_bias = 0.0
        self.az_bias = 0.0
        self.gx_bias = 0.0
        self.gy_bias = 0.0
        self.gz_bias = 0.0
        
        self.slam_x = 0.0
        self.slam_y = 0.0
        self.slam_yaw = 0.0
        self.slam_received = False
        self.last_slam_time = 0.0

        # 1. 生積分による位置
        self.x_raw = 0.0
        self.y_raw = 0.0
        self.v_raw = 0.0
        self.theta = 0.0

        # 2. カルマンフィルタ (KF) - 進行方向1次元モデル
        self.kf = PositionKalmanFilter2D(
            q_pos=self.kf_q_pos, q_vel=self.kf_q_vel, q_bias=self.kf_q_bias, r_pos=self.kf_r_pos)
        self.x_kf = 0.0
        self.y_kf = 0.0
        self.imu_history = collections.deque(maxlen=10000)  # (ts, a_forward, dt)
        self.kf_history = collections.deque(maxlen=10000)   # (ts, state, P)

        # 3. 遅延バイアスフィードバック (DBF) - 進行方向1次元モデル
        self.dbf = DelayedBiasFeedback2D(delay_sec=self.fixed_delay_sec)
        self.x_dbf = 0.0
        self.y_dbf = 0.0

        # SLAM観測用の累積移動距離と座標キャッシュ
        self.slam_accum_dist = 0.0
        self.last_slam_raw_x = None
        self.last_slam_raw_y = None

        # 起動時は初期キャリブレーションが未完了とする
        self.is_calibrated = False
        self.last_imu_ts = None
        self.imu_raw_pub_counter = 0

        # 配信タイマー (50Hz)
        self.pub_timer = self.create_timer(0.02, self._publish_estimated_poses)

        self.ser = None
        self.running = False
        self.rx_thread = None

        self._connect_serial()

    def _trigger_calib_cb(self, request, response):
        """10秒間の静止キャリブレーション要求を受け付けるコールバック"""
        with self.lock:
            self.calib_in_progress = True
            self.calib_samples = []
            self.is_calibrated = False
            self.e_int = [0.0, 0.0, 0.0]
            
            # パブリッシャでステータス送信 (10.0秒)
            status_msg = Float32()
            status_msg.data = 10.0
            self.calib_status_pub.publish(status_msg)
            
            self.get_logger().info('IMU 10秒キャリブレーションを開始します。ロボットを完全に静止させてください。')
        
        response.success = True
        response.message = 'IMU 10秒キャリブレーションを開始しました。静止してください。'
        return response

    def _update_mahony_filter(self, ax, ay, az, gx, gy, gz, dt):
        """Mahony フィルタによるクオータニオン姿勢推定の更新"""
        qw, qx, qy, qz = self.q

        # 加速度ベクトルのノルムを計算
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-6:
            # 加速度が小さすぎる場合はジャイロのみで積分更新
            dq_w = 0.5 * (-qx * gx - qy * gy - qz * gz)
            dq_x = 0.5 * ( qw * gx + qy * gz - qz * gy)
            dq_y = 0.5 * ( qw * gy - qx * gz + qz * gx)
            dq_z = 0.5 * ( qw * gz + qx * gy - qy * gx)
            self.q = [qw + dq_w * dt, qx + dq_x * dt, qy + dq_y * dt, qz + dq_z * dt]
            q_norm = math.sqrt(sum(x * x for x in self.q))
            self.q = [x / q_norm for x in self.q]
            return

        # 加速度を正規化
        ax /= norm
        ay /= norm
        az /= norm

        # クオータニオンから予測される重力ベクトル (ロボット座標系)
        # R^T * [0, 0, 1]^T
        vx = 2.0 * (qx * qz - qw * qy)
        vy = 2.0 * (qy * qz + qw * qx)
        vz = qw * qw - qx * qx - qy * qy + qz * qz

        # 測定された重力方向と予測された重力方向の誤差（外積）
        ex = (ay * vz) - (az * vy)
        ey = (az * vx) - (ax * vz)
        ez = (ax * vy) - (ay * vx)

        # 積分項
        self.e_int[0] += ex * dt
        self.e_int[1] += ey * dt
        self.e_int[2] += ez * dt

        # ジャイロ入力に補正値を加算 (Kp = 2.0, Ki = 0.005)
        Kp = 2.0
        Ki = 0.005
        gx += Kp * ex + Ki * self.e_int[0]
        gy += Kp * ey + Ki * self.e_int[1]
        gz += Kp * ez + Ki * self.e_int[2]

        # クオータニオン微分
        dq_w = 0.5 * (-qx * gx - qy * gy - qz * gz)
        dq_x = 0.5 * ( qw * gx + qy * gz - qz * gy)
        dq_y = 0.5 * ( qw * gy - qx * gz + qz * gx)
        dq_z = 0.5 * ( qw * gz + qx * gy - qy * gx)

        # 積分と正規化
        self.q = [qw + dq_w * dt, qx + dq_x * dt, qy + dq_y * dt, qz + dq_z * dt]
        q_norm = math.sqrt(sum(x * x for x in self.q))
        self.q = [x / q_norm for x in self.q]

    def _rotate_vector_by_quaternion(self, v, q):
        """ベクトル v [x, y, z] をクオータニオン q で回転 (ロボット座標系 -> ワールド座標系)"""
        qw, qx, qy, qz = q
        vx, vy, vz = v
        
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)

        rx = vx + qw * tx + (qy * tz - qz * ty)
        ry = vy + qw * ty + (qz * tx - qx * tz)
        rz = vz + qw * tz + (qx * ty - qy * tx)
        return rx, ry, rz

    def _get_euler_angles(self):
        """現在のクオータニオン姿勢からロール、ピッチ、ヨーを算出"""
        qw, qx, qy, qz = self.q
        
        # ロール (x軸回転)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # ピッチ (y軸回転)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        # ヨー (z軸回転)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _on_set_parameters(self, params):
        """Web UI などから動的にパラメータが更新された際のコールバック"""
        for param in params:
            if param.name == 'yaw_source':
                self.yaw_source = param.value
                self.get_logger().info(f'パラメータ更新: yaw_source = {self.yaw_source}')
            elif param.name == 'pos_source':
                self.pos_source = param.value
                self.get_logger().info(f'パラメータ更新: pos_source = {self.pos_source}')
            elif param.name == 'use_fixed_delay':
                self.use_fixed_delay = param.value
                self.get_logger().info(f'パラメータ更新: use_fixed_delay = {self.use_fixed_delay}')
            elif param.name == 'fixed_delay_sec':
                self.fixed_delay_sec = param.value
                self.get_logger().info(f'パラメータ更新: fixed_delay_sec = {self.fixed_delay_sec}')
                # DBF の遅延設定も同期
                self.dbf.delay_sec = self.fixed_delay_sec
            elif param.name == 'exclude_port':
                self.exclude_port = param.value
                self.get_logger().info(f'パラメータ更新: exclude_port = {self.exclude_port}')
            # KF パラメータの動的更新
            elif param.name == 'kf_q_pos':
                self.kf_q_pos = param.value
                self.kf.Q[0][0] = self.kf_q_pos
                self.get_logger().info(f'パラメータ更新: kf_q_pos = {self.kf_q_pos}')
            elif param.name == 'kf_q_vel':
                self.kf_q_vel = param.value
                self.kf.Q[1][1] = self.kf_q_vel
                self.get_logger().info(f'パラメータ更新: kf_q_vel = {self.kf_q_vel}')
            elif param.name == 'kf_q_bias':
                self.kf_q_bias = param.value
                self.kf.Q[2][2] = self.kf_q_bias
                self.get_logger().info(f'パラメータ更新: kf_q_bias = {self.kf_q_bias}')
            elif param.name == 'kf_r_pos':
                self.kf_r_pos = param.value
                self.kf.R = self.kf_r_pos
                self.get_logger().info(f'パラメータ更新: kf_r_pos = {self.kf_r_pos}')
            elif param.name == 'zupt_mode':
                self.zupt_mode = param.value
                self.get_logger().info(f'パラメータ更新: zupt_mode = {self.zupt_mode}')
            elif param.name == 'gyro_still_thresh':
                self.gyro_still_thresh = param.value
                self.get_logger().info(f'パラメータ更新: gyro_still_thresh = {self.gyro_still_thresh}')
            elif param.name == 'acc_still_thresh':
                self.acc_still_thresh = param.value
                self.get_logger().info(f'パラメータ更新: acc_still_thresh = {self.acc_still_thresh}')
            elif param.name == 'still_delay_samples':
                self.still_delay_samples = param.value
                self.get_logger().info(f'パラメータ更新: still_delay_samples = {self.still_delay_samples}')
        return SetParametersResult(successful=True)

    def _connect_serial(self):
        self.running = True
        self.rx_thread = threading.Thread(target=self._connection_and_rx_loop, daemon=True)
        self.rx_thread.start()

    def _cmd_vel_cb(self, msg: Twist):
        with self.lock:
            self.cmd_v = msg.linear.x
            self.cmd_w = msg.angular.z
            self.last_cmd_time = time.perf_counter()

    def _slam_pose_cb(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self.lock:
            self.slam_x = x
            self.slam_y = y
            self.slam_yaw = yaw
            self.slam_received = True
            self.last_slam_time = stamp

            if not self.is_calibrated or self.pos_source != 'acc':
                return

            # A. SLAM位置増分の進行方向（ロボットの現在の推定 Yaw 角度）への射影と累積距離の計算
            if self.last_slam_raw_x is None:
                self.last_slam_raw_x = x
                self.last_slam_raw_y = y
                self.slam_accum_dist = 0.0
                # KF と DBF の初期位置を現在の SLAM 2次元座標に同期
                self.x_kf = x
                self.y_kf = y
                self.x_dbf = x
                self.y_dbf = y
                return

            dx = x - self.last_slam_raw_x
            dy = y - self.last_slam_raw_y
            self.last_slam_raw_x = x
            self.last_slam_raw_y = y

            # 進行方向（現在の推定角度 theta）への投影
            ds = dx * math.cos(self.theta) + dy * math.sin(self.theta)
            self.slam_accum_dist += ds

            # B. 遅延カルマンフィルタ (再伝搬)
            if len(self.kf_history) >= 2:
                current_time = self.get_clock().now().nanoseconds / 1e9
                target_time = current_time - self.fixed_delay_sec if self.use_fixed_delay else stamp
                
                best_idx = 0
                min_diff = float('inf')
                for idx, (t_hist, _, _) in enumerate(self.kf_history):
                    diff = abs(t_hist - target_time)
                    if diff < min_diff:
                        min_diff = diff
                        best_idx = idx
                
                t_match, state_match, P_match = self.kf_history[best_idx]
                
                temp_kf = PositionKalmanFilter2D()
                temp_kf.set_state(state_match, P_match)
                
                # 遅延観測更新 (1次元累積距離)
                temp_kf.update(self.slam_accum_dist)
                
                reprop_started = False
                for t_imu, a_f, dt_imu in self.imu_history:
                    if t_imu > t_match:
                        reprop_started = True
                        temp_kf.predict(a_f, dt_imu)
                
                if reprop_started:
                    # 1次元の補正量を計算し、現在の2次元位置を補正
                    ds_corr = temp_kf.x[0] - self.kf.x[0]
                    self.x_kf += ds_corr * math.cos(self.theta)
                    self.y_kf += ds_corr * math.sin(self.theta)
                    
                    self.kf.set_state(temp_kf.x, temp_kf.P)
                else:
                    ds_corr = temp_kf.x[0] - self.kf.x[0]
                    self.x_kf += ds_corr * math.cos(self.theta)
                    self.y_kf += ds_corr * math.sin(self.theta)
                    self.kf.update(self.slam_accum_dist)
            else:
                ds_corr = self.slam_accum_dist - self.kf.x[0]
                self.x_kf += ds_corr * math.cos(self.theta)
                self.y_kf += ds_corr * math.sin(self.theta)
                self.kf.update(self.slam_accum_dist)

            # C. 遅延バイアスフィードバック (DBF)
            current_time = self.get_clock().now().nanoseconds / 1e9
            pos_dbf_before = self.dbf.pos
            self.dbf.update(self.slam_accum_dist, stamp, current_time, self.use_fixed_delay, self.fixed_delay_sec)
            ds_dbf_corr = self.dbf.pos - pos_dbf_before
            self.x_dbf += ds_dbf_corr * math.cos(self.theta)
            self.y_dbf += ds_dbf_corr * math.sin(self.theta)

    def _find_and_connect_imu_sync(self):
        """システム内のUSBシリアルポートを自動探索して接続する"""
        ports = [self.port] if self.port else []
        usb_ports = sorted(glob.glob('/dev/ttyUSB*'))
        for p in usb_ports:
            if p not in ports:
                ports.append(p)
        acm_ports = sorted(glob.glob('/dev/ttyACM*'))
        for p in acm_ports:
            if p not in ports:
                ports.append(p)

        for p in ports:
            if self.exclude_port and p == self.exclude_port:
                continue
            try:
                import serial
                # 短いタイムアウトで接続テスト
                ser_test = serial.Serial(p, self.baudrate, timeout=0.1)
                time.sleep(0.05)
                read_data = ser_test.read(300)
                
                header_found = False
                for i in range(len(read_data) - 1):
                    if read_data[i] == 0xAA and read_data[i+1] == 0x55:
                        header_found = True
                        break
                
                if header_found:
                    ser_test.timeout = 1.0  # 切断判定用のタイムアウトを設定
                    self.ser = ser_test
                    self.port = p
                    self.get_logger().info(f'Spresense IMU 自動接続成功: {p} @ {self.baudrate} bps')
                    return
                else:
                    ser_test.close()
            except Exception:
                continue

    def _connection_and_rx_loop(self):
        """シリアル接続と受信処理を行うメインループスレッド"""
        buffer = bytearray()
        while self.running and rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                # 接続が切れている場合は自動探索を繰り返す
                self._find_and_connect_imu_sync()
                if self.ser is None or not self.ser.is_open:
                    time.sleep(1.0)
                    continue
                buffer = bytearray()  # バッファをリセット

            try:
                # データ読み込み
                data = self.ser.read(self.ser.in_waiting or 1)
                if not data:
                    time.sleep(0.001)
                    continue
                
                buffer.extend(data)
                # バッファ容量制限
                if len(buffer) > 2000:
                    del buffer[:-500]

                while len(buffer) >= 34:
                    if buffer[0] == 0xAA and buffer[1] == 0x55:
                        raw_packet = buffer[2:34]
                        del buffer[:34]
                        self._process_imu_packet(raw_packet)
                    else:
                        del buffer[0]
            except Exception as e:
                self.get_logger().warn(f'シリアル接続が切断されました: {e}')
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = None
                self.is_calibrated = False
                
                # トピックで停止通知
                try:
                    status_msg = Float32()
                    status_msg.data = -1.0
                    self.calib_status_pub.publish(status_msg)
                except Exception:
                    pass
                time.sleep(1.0)

    def _process_imu_packet(self, raw):
        try:
            ts_raw, temp, gx, gy, gz, ax, ay, az = struct.unpack('<If6f', raw)
        except Exception as e:
            return

        # 軸の極性反転を反映
        if self.invert_ax: ax = -ax
        if self.invert_ay: ay = -ay
        if self.invert_az: az = -az
        if self.invert_gx: gx = -gx
        if self.invert_gy: gy = -gy
        if self.invert_gz: gz = -gz

        # ジャイロ度/秒 (dps) からラジアン/秒への変換（重要：ジャイロが狂いすぎる問題の解決策）
        # キャリブレーション前に実行し、バイアス自体もラジアン/秒で統一する
        if self.gyro_in_deg_sec:
            gx = math.radians(gx)
            gy = math.radians(gy)
            gz = math.radians(gz)

        # タイムスタンプ差分 dt（キャリブレーション実行中も常に更新するため先頭へ移動）
        if self.last_imu_ts is None:
            self.last_imu_ts = ts_raw
            return

        if ts_raw >= self.last_imu_ts:
            dt = (ts_raw - self.last_imu_ts) / 19200000.0
        else:
            dt = ((0xFFFFFFFF - self.last_imu_ts) + ts_raw + 1) / 19200000.0
        self.last_imu_ts = ts_raw

        if dt <= 0.0 or dt > 0.05:
            return

        # キャリブレーション
        if not self.is_calibrated or self.calib_in_progress:
            if not self.calib_in_progress:
                self.calib_in_progress = True
                self.calib_samples = []
                # トピックで配信 (10.0秒)
                status_msg = Float32()
                status_msg.data = 10.0
                self.calib_status_pub.publish(status_msg)
                self.get_logger().info('IMU 10秒キャリブレーションを開始します。')

            self.calib_samples.append((ax, ay, az, gx, gy, gz))
            
            # 約0.1秒(200サンプル)ごとに残り時間をトピックで配信
            if len(self.calib_samples) % 200 == 0:
                seconds_left = max(0.0, 10.0 - len(self.calib_samples) / 1920.0)
                status_msg = Float32()
                status_msg.data = round(seconds_left, 1)
                self.calib_status_pub.publish(status_msg)

            if len(self.calib_samples) >= self.calib_required_count:
                ax_avg = sum(s[0] for s in self.calib_samples) / len(self.calib_samples)
                ay_avg = sum(s[1] for s in self.calib_samples) / len(self.calib_samples)
                az_avg = sum(s[2] for s in self.calib_samples) / len(self.calib_samples)
                self.gx_bias = sum(s[3] for s in self.calib_samples) / len(self.calib_samples)
                self.gy_bias = sum(s[4] for s in self.calib_samples) / len(self.calib_samples)
                self.gz_bias = sum(s[5] for s in self.calib_samples) / len(self.calib_samples)
                
                # 静止時の加速度ベクトルの長さから重力加速度定数を設定
                self.g_calib = math.sqrt(ax_avg**2 + ay_avg**2 + az_avg**2)
                
                # 平均加速度から初期傾斜姿勢(ロール・ピッチ)を求める
                roll_init = math.atan2(ay_avg, az_avg)
                pitch_init = math.atan2(-ax_avg, math.sqrt(ay_avg**2 + az_avg**2))
                yaw_init = self.theta
                
                # 初期クオータニオンの構築
                cy = math.cos(yaw_init * 0.5)
                sy = math.sin(yaw_init * 0.5)
                cp = math.cos(pitch_init * 0.5)
                sp = math.sin(pitch_init * 0.5)
                cr = math.cos(roll_init * 0.5)
                sr = math.sin(roll_init * 0.5)
                self.q = [
                    cr*cp*cy + sr*sp*sy,
                    sr*cp*cy - cr*sp*sy,
                    cr*sp*cy + sr*cp*sy,
                    cr*cp*sy - sr*sp*sy
                ]
                
                self.ax_bias = 0.0
                self.ay_bias = 0.0
                self.az_bias = 0.0
                
                self.is_calibrated = True
                self.calib_in_progress = False
                
                # トピックで完了通知 (-1.0)
                status_msg = Float32()
                status_msg.data = -1.0
                self.calib_status_pub.publish(status_msg)
                
                self.get_logger().info(
                    f'IMUキャリブレーション完了: g_calib={self.g_calib:.5f}, '
                    f'roll_init={math.degrees(roll_init):.2f}°, pitch_init={math.degrees(pitch_init):.2f}°, '
                    f'gx_bias={self.gx_bias:.5f}, gy_bias={self.gy_bias:.5f}, gz_bias={self.gz_bias:.5f}')
            return

        # バイアス補正
        gx_corr = gx - self.gx_bias
        gy_corr = gy - self.gy_bias
        gz_corr = gz - self.gz_bias
        ax_corr = ax
        ay_corr = ay
        az_corr = az

        # 3次元姿勢 Mahony フィルタの更新
        self._update_mahony_filter(ax, ay, az, gx_corr, gy_corr, gz_corr, dt)
        roll, pitch, yaw = self._get_euler_angles()

        # ----------------- 1. 角度更新 (yaw_source に基づく) -----------------
        if self.yaw_source == 'gyro':
            self.theta = yaw
        elif self.yaw_source == 'slam':
            with self.lock:
                slam_yaw = self.slam_yaw
            self.theta = slam_yaw
        elif self.yaw_source == 'cmd':
            with self.lock:
                cmd_w_val = self.cmd_w
            self.theta += cmd_w_val * dt
            
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # yaw_source が gyro 以外の場合はクオータニオン姿勢をその yaw で同期
        if self.yaw_source != 'gyro':
            cy = math.cos(self.theta * 0.5)
            sy = math.sin(self.theta * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            self.q = [
                cr*cp*cy + sr*sp*sy,
                sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*sy
            ]

        # ----------------- 2. 進行方向（ローカル x 軸）運動加速度の計算 -----------------
        # クオータニオン姿勢を用いて、重力方向（ワールド Z 軸）がロボットのローカル座標に投影されたベクトルを求める
        qw, qx, qy, qz = self.q
        vx = 2.0 * (qx * qz - qw * qy)
        vy = 2.0 * (qy * qz + qw * qx)
        vz = qw * qw - qx * qx - qy * qy + qz * qz

        # ローカル座標における重力加速度ベクトルは g_calib * v
        # ローカル運動加速度 = 測定値 - 重力成分
        ax_local_mot = ax - self.g_calib * vx
        ay_local_mot = ay - self.g_calib * vy
        az_local_mot = az - self.g_calib * vz

        # ロボットのローカル x 軸方向（進行方向）の純粋な運動加速度
        a_forward = ax_local_mot

        # ----------------- 3. 静止判定 (ZUPT判定) -----------------
        # A. センサーベースの静止検知 (一定時間変動が閾値未満であること)
        sensor_is_still = (
            abs(gx_corr) < self.gyro_still_thresh and
            abs(gy_corr) < self.gyro_still_thresh and
            abs(gz_corr) < self.gyro_still_thresh and
            abs(ax_local_mot) < self.acc_still_thresh and
            abs(ay_local_mot) < self.acc_still_thresh and
            abs(az_local_mot) < self.acc_still_thresh
        )
        
        if sensor_is_still:
            if self.still_counter < self.still_delay_samples:
                self.still_counter += 1
        else:
            self.still_counter = 0
            
        sensor_still_flag = (self.still_counter >= self.still_delay_samples)

        # B. 指令値ベースの静止検知
        with self.lock:
            cmd_v = self.cmd_v
            cmd_w = self.cmd_w
            slam_x = self.slam_x
            slam_y = self.slam_y
            time_since_cmd = time.perf_counter() - self.last_cmd_time
            
        cmd_still_flag = (abs(cmd_v) < 0.001 and abs(cmd_w) < 0.001) or (time_since_cmd > 1.0)

        # C. モード別の統合判定
        if self.zupt_mode == 'cmd_vel':
            is_still = cmd_still_flag
        elif self.zupt_mode == 'sensor':
            is_still = sensor_still_flag
        elif self.zupt_mode == 'none':
            is_still = False
        else: # 'auto'
            # 指令値が配信されていれば指令値ベース、配信が途絶えていればセンサーベース
            if time_since_cmd <= 1.0:
                is_still = cmd_still_flag
            else:
                is_still = sensor_still_flag

        # ----------------- 4. 位置更新 (pos_source に基づく) -----------------
        if self.pos_source == 'acc':
            # 加速度の二重積分 (ZUPTあり)
            if is_still:
                self.v_raw = 0.0
                # 静止時にジャイロバイアスを緩やかに微調整
                alpha = 0.9995
                self.gx_bias = alpha * self.gx_bias + (1.0 - alpha) * gx
                self.gy_bias = alpha * self.gy_bias + (1.0 - alpha) * gy
                self.gz_bias = alpha * self.gz_bias + (1.0 - alpha) * gz
                
                # ZUPT: 静止時は入力加速度を 0 にし、速度もリセットしてドリフトを防止
                a_input = 0.0
                self.kf.x[1] = 0.0
                self.dbf.vel = 0.0
            else:
                self.v_raw += a_forward * dt
                a_input = a_forward
                
            dist_raw = self.v_raw * dt
            self.x_raw += dist_raw * math.cos(self.theta)
            self.y_raw += dist_raw * math.sin(self.theta)
            
            # 加速度ベースのときのみ KF と DBF の予測を進める
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # KFの予測と2次元座標への復元積算
            pos_kf_before = self.kf.x[0]
            self.kf.predict(a_input, dt)
            ds_kf = self.kf.x[0] - pos_kf_before
            self.x_kf += ds_kf * math.cos(self.theta)
            self.y_kf += ds_kf * math.sin(self.theta)
            
            # 履歴の保存
            self.imu_history.append((current_time, a_input, dt))
            self.kf_history.append((
                current_time,
                list(self.kf.x), [list(row) for row in self.kf.P]
            ))
            
            # DBFの予測と2次元座標への復元積算
            pos_dbf_before = self.dbf.pos
            self.dbf.predict(a_input, dt, current_time)
            ds_dbf = self.dbf.pos - pos_dbf_before
            self.x_dbf += ds_dbf * math.cos(self.theta)
            self.y_dbf += ds_dbf * math.sin(self.theta)

        elif self.pos_source == 'slam':
            # SLAM位置をそのまま適用
            self.x_raw = slam_x
            self.y_raw = slam_y
            self.v_raw = 0.0
            
            # KF と DBF にも SLAM 位置を直接セット
            self.x_kf = slam_x
            self.y_kf = slam_y
            self.x_dbf = slam_x
            self.y_dbf = slam_y
            
            self.kf.x = [self.slam_accum_dist, 0.0, 0.0]
            self.dbf.pos = self.slam_accum_dist

        elif self.pos_source == 'cmd':
            # 指令速度からデッドレコニング (ダミーオドメトリ)
            dist_cmd = cmd_v * dt
            self.x_raw += dist_cmd * math.cos(self.theta)
            self.y_raw += dist_cmd * math.sin(self.theta)
            self.v_raw = cmd_v
            
            # KF と DBF にも指令値結果をセット
            self.x_kf = self.x_raw
            self.y_kf = self.y_raw
            self.x_dbf = self.x_raw
            self.y_dbf = self.y_raw
            
            self.kf.x = [self.slam_accum_dist, cmd_v, 0.0]
            self.dbf.pos = self.slam_accum_dist
            self.dbf.vel = cmd_v

        # ----------------- 4. 生IMUデータ配信 (間引き) -----------------
        self.imu_raw_pub_counter += 1
        if self.imu_raw_pub_counter >= 19:
            self.imu_raw_pub_counter = 0
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.base_frame
            imu_msg.linear_acceleration.x = float(ax)
            imu_msg.linear_acceleration.y = float(ay)
            imu_msg.linear_acceleration.z = float(az)
            imu_msg.angular_velocity.x = float(gx)
            imu_msg.angular_velocity.y = float(gy)
            imu_msg.angular_velocity.z = float(gz)
            try:
                self.imu_raw_pub.publish(imu_msg)
            except Exception:
                pass

    def _publish_estimated_poses(self):
        if not self.is_calibrated:
            return

        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # 1. 生積分/推定ソース Pose
        pose_raw = PoseStamped()
        pose_raw.header.stamp = stamp
        pose_raw.header.frame_id = self.odom_frame
        pose_raw.pose.position.x = self.x_raw
        pose_raw.pose.position.y = self.y_raw
        pose_raw.pose.orientation.z = qz
        pose_raw.pose.orientation.w = qw
        self.pose_raw_pub.publish(pose_raw)

        # 2. カルマンフィルタ (KF) Pose
        pose_kf = PoseStamped()
        pose_kf.header.stamp = stamp
        pose_kf.header.frame_id = self.odom_frame
        pose_kf.pose.position.x = self.x_kf
        pose_kf.pose.position.y = self.y_kf
        pose_kf.pose.orientation.z = qz
        pose_kf.pose.orientation.w = qw
        self.pose_kf_pub.publish(pose_kf)

        # 3. 遅延バイアスフィードバック (DBF) Pose
        pose_dbf = PoseStamped()
        pose_dbf.header.stamp = stamp
        pose_dbf.header.frame_id = self.odom_frame
        pose_dbf.pose.position.x = self.x_dbf
        pose_dbf.pose.position.y = self.y_dbf
        pose_dbf.pose.orientation.z = qz
        pose_dbf.pose.orientation.w = qw
        self.pose_dbf_pub.publish(pose_dbf)

        # 代表オドメトリおよび TF (KF を採用)
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x_kf
        odom_msg.pose.pose.position.y = self.y_kf
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = self.kf.x[1]
        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x_kf
            tf.transform.translation.y = self.y_kf
            tf.transform.translation.z = 0.0
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_bc.sendTransform(tf)

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpresenseImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
