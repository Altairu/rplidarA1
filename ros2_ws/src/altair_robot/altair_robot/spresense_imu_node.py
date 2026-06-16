#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spresense IMU 自己位置推定・融合 (SLAM Fusion - Delayed Measurement 対応) ノード (ROS2)

SLAM (Cartographer) の自己位置推定値が数秒レベルで遅れて届く（可変遅延）現象に対応するため、
過去約 5 秒分の IMU 積分データおよびカルマンフィルタ状態を履歴バッファに保持し、
SLAM が届いた際に過去の該当時刻に遡って観測更新を行い、そこから現在時刻まで
IMU 加速度を用いて再伝搬（再計算 / Re-propagation）を行う遅延カルマンフィルタを実装しています。
遅延バイアスフィードバック (DBF) もバッファを拡張して追従性を向上させています。
"""

import math
import struct
import threading
import time
import collections
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class PositionKalmanFilter2D:
    """
    1次元カルマンフィルタをX, Y独立に適用するためのクラス
    状態: x = [pos, vel, acc_bias]^T
    """
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
        """状態のディープコピーを作成して返す（再伝搬時のバックアップや履歴保存用）"""
        new_kf = PositionKalmanFilter2D(self.Q[0][0], self.Q[1][1], self.Q[2][2], self.R)
        new_kf.x = list(self.x)
        new_kf.P = [list(row) for row in self.P]
        return new_kf

    def set_state(self, x, P):
        self.x = list(x)
        self.P = [list(row) for row in P]

    def predict(self, acc_input, dt):
        # 状態遷移行列 F と入力行列 B
        F = [
            [1.0, dt, -0.5 * dt**2],
            [0.0, 1.0, -dt],
            [0.0, 0.0, 1.0]
        ]
        B = [0.5 * dt**2, dt, 0.0]
        
        # x_new = F * x + B * acc_input
        x_new = [
            F[0][0]*self.x[0] + F[0][1]*self.x[1] + F[0][2]*self.x[2] + B[0]*acc_input,
            F[1][0]*self.x[0] + F[1][1]*self.x[1] + F[1][2]*self.x[2] + B[1]*acc_input,
            F[2][0]*self.x[0] + F[2][1]*self.x[1] + F[2][2]*self.x[2] + B[2]*acc_input
        ]
        
        # P_new = F * P * F^T + Q
        # P_temp = F * P
        P_temp = [[0.0]*3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                P_temp[i][j] = F[i][0]*self.P[0][j] + F[i][1]*self.P[1][j] + F[i][2]*self.P[2][j]
                
        # P_new = P_temp * F^T + Q
        P_new = [[0.0]*3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                P_new[i][j] = P_temp[i][0]*F[j][0] + P_temp[i][1]*F[j][1] + P_temp[i][2]*F[j][2] + self.Q[i][j]
                
        self.x = x_new
        self.P = P_new

    def update(self, z):
        # 観測行列 H = [1, 0, 0]
        # イノベーション y = z - H * x = z - pos
        y = z - self.x[0]
        
        # S = H * P * H^T + R = P[0][0] + R
        S = self.P[0][0] + self.R
        
        # カルマンゲイン K = P * H^T / S
        K = [self.P[0][0] / S, self.P[1][0] / S, self.P[2][0] / S]
        
        # x_new = x + K * y
        self.x = [
            self.x[0] + K[0] * y,
            self.x[1] + K[1] * y,
            self.x[2] + K[2] * y
        ]
        
        # P_new = (I - K * H) * P
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
    """
    遅延バイアスフィードバック (DBF) を適用するためのクラス (X, Y 独立)
    """
    def __init__(self, delay_sec=0.5):
        self.pos = 0.0
        self.vel = 0.0
        self.history = collections.deque(maxlen=10000)  # 約5秒分の履歴に拡張 (1920Hz * 5s = 9600)
        self.last_update_time = None
        self.delay_sec = delay_sec

    def predict(self, acc_input, dt, timestamp):
        # 単純積分で予測
        self.vel += acc_input * dt
        self.pos += self.vel * dt
        self.history.append((timestamp, self.pos, self.vel))

    def update(self, z, slam_time, current_time, use_fixed_delay=False, fixed_delay_sec=0.2):
        if len(self.history) < 2:
            return

        # 照合先ターゲット時刻の決定
        if use_fixed_delay:
            target_time = current_time - fixed_delay_sec
        else:
            target_time = slam_time
        
        # 最も近い過去の履歴データを検索 (merge_asofに相当)
        best_idx = 0
        min_diff = float('inf')
        for idx, (t_hist, p_hist, v_hist) in enumerate(self.history):
            diff = abs(t_hist - target_time)
            if diff < min_diff:
                min_diff = diff
                best_idx = idx
                
        t_hist, p_hist, v_hist = self.history[best_idx]
        
        # 過去の位置誤差
        delta_p = p_hist - z
        
        # 前回の更新からの時間幅
        if self.last_update_time is not None:
            delta_t = t_hist - self.last_update_time
        else:
            delta_t = 0.1  # デフォルト値
            
        if delta_t <= 0.0:
            delta_t = 0.001
            
        # 速度バイアスを逆算
        v_bias = delta_p / delta_t
        
        # 現在時刻と過去の観測時刻の遅延幅
        tau = current_time - t_hist
        if tau < 0.0:
            tau = 0.0
            
        # 現在状態のフィードバック補正
        self.vel -= v_bias
        self.pos -= (delta_p + v_bias * tau)
        
        self.last_update_time = t_hist
        
        # 履歴バッファも補正を反映
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
        
        # 遅延補正用のパラメータ
        self.declare_parameter('use_fixed_delay', False)
        self.declare_parameter('fixed_delay_sec', 0.2)
        
        # センサ極性パラメータ
        self.declare_parameter('invert_ax', True)
        self.declare_parameter('invert_ay', False)
        self.declare_parameter('invert_az', False)
        self.declare_parameter('invert_gx', False)
        self.declare_parameter('invert_gy', False)
        self.declare_parameter('invert_gz', False)

        # パラメータの取得
        self.port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        self.use_fixed_delay = self.get_parameter('use_fixed_delay').value
        self.fixed_delay_sec = self.get_parameter('fixed_delay_sec').value
        
        self.invert_ax = self.get_parameter('invert_ax').value
        self.invert_ay = self.get_parameter('invert_ay').value
        self.invert_az = self.get_parameter('invert_az').value
        self.invert_gx = self.get_parameter('invert_gx').value
        self.invert_gy = self.get_parameter('invert_gy').value
        self.invert_gz = self.get_parameter('invert_gz').value

        # パブリッシャの作成 (生積分、KF、DBF それぞれのトピック)
        self.pose_raw_pub = self.create_publisher(PoseStamped, 'imu_pose_raw', 10)
        self.pose_kf_pub = self.create_publisher(PoseStamped, 'imu_pose_kf', 10)
        self.pose_dbf_pub = self.create_publisher(PoseStamped, 'imu_pose_dbf', 10)
        self.odom_pub = self.create_publisher(Odometry, 'imu_odom', 10)
        self.imu_raw_pub = self.create_publisher(Imu, 'imu_raw', 10)
        
        self.tf_bc = TransformBroadcaster(self)

        # サブスクライバの作成 (ZUPT用 cmd_vel, KF/DBF用 SLAM自己位置)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.slam_pose_sub = self.create_subscription(
            PoseStamped, 'tracked_pose', self._slam_pose_cb, 10)

        # 内部状態変数
        self.lock = threading.Lock()
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = time.perf_counter()
        
        self.slam_x = 0.0
        self.slam_y = 0.0
        self.slam_received = False
        self.last_slam_time = 0.0

        # 1. 生積分による位置 (x_raw, y_raw, v_raw)
        self.x_raw = 0.0
        self.y_raw = 0.0
        self.v_raw = 0.0
        
        # 角度 (共通のジャイロ積分角度)
        self.theta = 0.0

        # 2. カルマンフィルタ (KF) による位置
        # X軸, Y軸それぞれに独立した 1次元KF
        self.kf_x = PositionKalmanFilter2D(q_pos=1e-3, q_vel=1e-2, q_bias=1e-5, r_pos=5.0)
        self.kf_y = PositionKalmanFilter2D(q_pos=1e-3, q_vel=1e-2, q_bias=1e-5, r_pos=5.0)

        # カルマンフィルタ用履歴バッファ（再伝搬処理用）
        # imu_history: (timestamp, acc_world_x, acc_world_y, dt)
        self.imu_history = collections.deque(maxlen=10000)  # 約5秒分
        # kf_history: (timestamp, kf_x_state, kf_x_P, kf_y_state, kf_y_P)
        self.kf_history = collections.deque(maxlen=10000)

        # 3. 遅延バイアスフィードバック (DBF) による位置
        self.dbf_x = DelayedBiasFeedback2D(delay_sec=0.1)
        self.dbf_y = DelayedBiasFeedback2D(delay_sec=0.1)

        # キャリブレーション用変数
        self.is_calibrated = False
        self.calib_samples = []
        self.calib_required_count = 3840  # 1920Hz で約 2 秒間
        self.ax_bias = 0.0
        self.ay_bias = 0.0
        self.gz_bias = 0.0
        self.last_imu_ts = None
        self.imu_raw_pub_counter = 0

        # 配信タイマー (50Hz)
        self.pub_timer = self.create_timer(0.02, self._publish_estimated_poses)

        # シリアル接続と受信スレッドの起動
        self.ser = None
        self.running = False
        self.rx_thread = None

        self._connect_serial()

    def _connect_serial(self):
        try:
            import serial
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.get_logger().info(f'Spresense シリアル接続成功: {self.port} @ {self.baudrate} bps')
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
        except Exception as e:
            self.get_logger().error(f'Spresense シリアル接続失敗 ({self.port}): {e}')

    def _cmd_vel_cb(self, msg: Twist):
        with self.lock:
            self.cmd_v = msg.linear.x
            self.cmd_w = msg.angular.z
            self.last_cmd_time = time.perf_counter()

    def _slam_pose_cb(self, msg: PoseStamped):
        """SLAM(Cartographer) の自己位置を受け取って、KF / DBF の観測更新を行う"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self.lock:
            self.slam_x = x
            self.slam_y = y
            self.slam_received = True
            self.last_slam_time = stamp

            if not self.is_calibrated:
                return

            # ----------------- A. 遅延カルマンフィルタ (再伝搬 Re-propagation) -----------------
            if len(self.kf_history) >= 2:
                # 照合先ターゲット時刻の決定
                current_time = self.get_clock().now().nanoseconds / 1e9
                if self.use_fixed_delay:
                    target_time = current_time - self.fixed_delay_sec
                else:
                    target_time = stamp
                
                # SLAMデータ時刻に最も近い過去の KF 状態を履歴から探す
                best_idx = 0
                min_diff = float('inf')
                for idx, (t_hist, _, _, _, _) in enumerate(self.kf_history):
                    diff = abs(t_hist - target_time)
                    if diff < min_diff:
                        min_diff = diff
                        best_idx = idx
                
                t_match, x_state, x_P, y_state, y_P = self.kf_history[best_idx]
                
                # 1. 過去時点の状態を一時的な KF クラスにセット
                temp_kf_x = PositionKalmanFilter2D()
                temp_kf_y = PositionKalmanFilter2D()
                temp_kf_x.set_state(x_state, x_P)
                temp_kf_y.set_state(y_state, y_P)
                
                # 2. 過去時点の状態に対して SLAM 観測値 z = [x, y] を使って update を適用
                temp_kf_x.update(x)
                temp_kf_y.update(y)
                
                # 3. 過去時点から現在までの IMU 加速度入力を使い、順番に predict を再実行（再伝搬）
                # imu_history には (timestamp, ax_world, ay_world, dt) が格納されている
                # 過去マッチング時刻 t_match 以降の IMU 入力を探し出して再積分する
                reprop_started = False
                for t_imu, ax_w, ay_w, dt_imu in self.imu_history:
                    if t_imu > t_match:
                        reprop_started = True
                        temp_kf_x.predict(ax_w, dt_imu)
                        temp_kf_y.predict(ay_w, dt_imu)
                
                # 4. 再計算された最新状態を現在のメイン KF クラスへ適用
                if reprop_started:
                    self.kf_x.set_state(temp_kf_x.x, temp_kf_x.P)
                    self.kf_y.set_state(temp_kf_y.x, temp_kf_y.P)
                else:
                    # 履歴が現在に極めて近い、またはマッチングしなかった場合はそのまま直接アップデート
                    self.kf_x.update(x)
                    self.kf_y.update(y)

            else:
                # 履歴が不十分な場合は直接アップデート
                self.kf_x.update(x)
                self.kf_y.update(y)

            # ----------------- B. 遅延バイアスフィードバック (DBF) -----------------
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.dbf_x.update(x, stamp, current_time, self.use_fixed_delay, self.fixed_delay_sec)
            self.dbf_y.update(y, stamp, current_time, self.use_fixed_delay, self.fixed_delay_sec)

    def _rx_loop(self):
        buffer = bytearray()
        while self.running and rclpy.ok():
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if not data:
                    time.sleep(0.001)
                    continue
                buffer.extend(data)
                while len(buffer) >= 34:
                    if buffer[0] == 0xAA and buffer[1] == 0x55:
                        raw_packet = buffer[2:34]
                        del buffer[:34]
                        self._process_imu_packet(raw_packet)
                    else:
                        del buffer[0]
            except Exception as e:
                self.get_logger().warn(f'シリアル受信エラー: {e}')
                time.sleep(0.1)

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

        # キャリブレーション
        if not self.is_calibrated:
            self.calib_samples.append((ax, ay, gz))
            if len(self.calib_samples) >= self.calib_required_count:
                self.ax_bias = sum(s[0] for s in self.calib_samples) / len(self.calib_samples)
                self.ay_bias = sum(s[1] for s in self.calib_samples) / len(self.calib_samples)
                self.gz_bias = sum(s[2] for s in self.calib_samples) / len(self.calib_samples)
                self.is_calibrated = True
                self.get_logger().info(
                    f'キャリブレーション完了: ax_bias={self.ax_bias:.6f}, ay_bias={self.ay_bias:.6f}, gz_bias={self.gz_bias:.6f}')
            return

        # タイムスタンプ差分 dt
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

        # バイアス補正
        ax_corr = ax - self.ax_bias
        ay_corr = ay - self.ay_bias
        gz_corr = gz - self.gz_bias

        # 角度 (Yaw角) 積算 (共通のジャイロ積分角度)
        self.theta += gz_corr * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 加速度をロボット座標系からワールド座標系（map系）に回転変換
        ax_world = ax_corr * math.cos(self.theta) - ay_corr * math.sin(self.theta)
        ay_world = ax_corr * math.sin(self.theta) + ay_corr * math.cos(self.theta)

        # 静止判定 (ZUPT判定)
        with self.lock:
            cmd_v = self.cmd_v
            cmd_w = self.cmd_w
            time_since_cmd = time.perf_counter() - self.last_cmd_time
        is_still = (abs(cmd_v) < 0.001 and abs(cmd_w) < 0.001) or (time_since_cmd > 1.0)

        # ----------------- 1. 生積分オドメトリ (ZUPT付き) -----------------
        if is_still:
            self.v_raw = 0.0
            # 静止時にバイアスを動的に微更新 (長期安定化)
            alpha = 0.9995
            self.ax_bias = alpha * self.ax_bias + (1.0 - alpha) * ax
            self.ay_bias = alpha * self.ay_bias + (1.0 - alpha) * ay
            self.gz_bias = alpha * self.gz_bias + (1.0 - alpha) * gz
        else:
            # 運動時は進行方向(X軸)の加速度を積分してワールド位置に射影
            self.v_raw += ax_corr * dt
            
        dist_raw = self.v_raw * dt
        self.x_raw += dist_raw * math.cos(self.theta)
        self.y_raw += dist_raw * math.sin(self.theta)

        # ----------------- 2. カルマンフィルタ (KF - 再伝搬対応) -----------------
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # ワールド系の加速度を入力として予測ステップを実行
        self.kf_x.predict(ax_world, dt)
        self.kf_y.predict(ay_world, dt)

        # 予測ステップ実行後の KF の状態と IMU 入力を履歴バッファに積む
        self.imu_history.append((current_time, ax_world, ay_world, dt))
        self.kf_history.append((
            current_time,
            list(self.kf_x.x), [list(row) for row in self.kf_x.P],
            list(self.kf_y.x), [list(row) for row in self.kf_y.P]
        ))

        # ----------------- 3. 遅延バイアスフィードバック (DBF) -----------------
        self.dbf_x.predict(ax_world, dt, current_time)
        self.dbf_y.predict(ay_world, dt, current_time)

        # ----------------- 4. 生IMUデータの配信 (間引き) -----------------
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
        """推定された3種類（生積分、KF、DBF）の位置情報をトピック送信する (50Hz)"""
        if not self.is_calibrated:
            return

        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # 1. 生積分 Pose
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
        pose_kf.pose.position.x = self.kf_x.x[0]
        pose_kf.pose.position.y = self.kf_y.x[0]
        pose_kf.pose.orientation.z = qz
        pose_kf.pose.orientation.w = qw
        self.pose_kf_pub.publish(pose_kf)

        # 3. 遅延バイアスフィードバック (DBF) Pose
        pose_dbf = PoseStamped()
        pose_dbf.header.stamp = stamp
        pose_dbf.header.frame_id = self.odom_frame
        pose_dbf.pose.position.x = self.dbf_x.pos
        pose_dbf.pose.position.y = self.dbf_y.pos
        pose_dbf.pose.orientation.z = qz
        pose_dbf.pose.orientation.w = qw
        self.pose_dbf_pub.publish(pose_dbf)

        # 代表として KF の位置情報をオドメトリトピックおよび TF としてパブリッシュする
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.kf_x.x[0]
        odom_msg.pose.pose.position.y = self.kf_y.x[0]
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = self.kf_x.x[1]
        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.kf_x.x[0]
            tf.transform.translation.y = self.kf_y.x[0]
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
