#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MDD CAN ドライバーノード (ROS2)

【ステートマシン (mdd_gui_ubuntu.py 完全準拠)】

  起動
    │
    ▼
  FORCE_PARAM フェーズ (FORCE_PARAM_FRAMES 回 = 500ms)
    │  ── MCU の APP_MODE に関係なく、必ず 0x200-0x203 + 0x210 を送る
    │
    ▼  (完了後)
  通常ステートマシン
    ├─ APP_MODE == PARAMETER → param_send_req=True → 10ms ごとにパラメータ送信
    │    MCU が CONTROL に移行 → tx_enabled=True → 目標値送信
    └─ APP_MODE == CONTROL  → tx_enabled=True → 10ms ごとに目標値送信
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

CAN_ID_PARAM_BASE  = 0x200
CAN_ID_MODE        = 0x210
CAN_ID_TARGET      = 0x220
CAN_ID_STATUS      = 0x230

APP_MODE_PARAMETER = 0
APP_MODE_CONTROL   = 1
CONTROL_SPEED      = 0

TX_INTERVAL        = 0.010   # 10ms
RX_TIMEOUT         = 0.005   # 5ms
FORCE_PARAM_FRAMES = 50      # 起動時強制送信 500ms
LOG_PARAM_INTERVAL = 5.0
LOG_STATUS_INTERVAL = 1.0


class MddCanNode(Node):

    def __init__(self):
        super().__init__('mdd_can_node')

        self.declare_parameter('can_channel',    'can0')
        self.declare_parameter('can_bitrate',    1000000)
        self.declare_parameter('wheel_diameter', 0.200)
        self.declare_parameter('wheel_base',     0.500)
        self.declare_parameter('pid_p',          100.0)
        self.declare_parameter('pid_i',          0.0)
        self.declare_parameter('pid_d',          0.0)
        self.declare_parameter('max_rps',        20.0)
        self.declare_parameter('cmd_vel_topic',  'cmd_vel')
        self.declare_parameter('odom_topic',     'odom')
        self.declare_parameter('odom_frame',     'rplidar_odom')
        self.declare_parameter('base_frame',     'rplidar_base_link')

        channel           = self.get_parameter('can_channel').value
        bitrate           = self.get_parameter('can_bitrate').value
        self.wheel_diam   = self.get_parameter('wheel_diameter').value
        self.wheel_base_m = self.get_parameter('wheel_base').value
        self.pid_p        = self.get_parameter('pid_p').value
        self.pid_i        = self.get_parameter('pid_i').value
        self.pid_d        = self.get_parameter('pid_d').value
        self.max_rps      = self.get_parameter('max_rps').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic    = self.get_parameter('odom_topic').value
        self.odom_frame    = self.get_parameter('odom_frame').value
        self.base_frame    = self.get_parameter('base_frame').value
        self.wheel_circum = math.pi * self.wheel_diam

        self.bus  = None
        self.lock = threading.Lock()
        self._targets = [0.0, 0.0, 0.0, 0.0]

        # ステートマシン
        self._force_param_count = FORCE_PARAM_FRAMES
        self.remote_app_mode    = APP_MODE_PARAMETER
        self._last_remote_mode  = None   # GUI の last_remote_app_mode 相当
        self.param_send_req     = False
        self.tx_enabled         = False
        self._shutdown          = threading.Event()

        # ログ間引き
        self._last_param_log_t  = 0.0
        self._last_status_log_t = 0.0
        self._last_status_sig   = None

        # オドメトリ
        self._odom_x    = 0.0
        self._odom_y    = 0.0
        self._odom_th   = 0.0
        self._odom_time = time.perf_counter()
        self._cmd_v     = 0.0
        self._cmd_w     = 0.0

        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self._cmd_vel_cb, 10)
        self.odom_pub    = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_bc       = TransformBroadcaster(self)
        self.odom_timer  = self.create_timer(0.05, self._publish_odom)

        if not CAN_AVAILABLE:
            self.get_logger().error('python-can 未インストール: pip install python-can')
            return

        self._connect(channel, bitrate)
        if self.bus:
            self.get_logger().info(
                f'起動時強制パラメータ送信: {FORCE_PARAM_FRAMES}回({FORCE_PARAM_FRAMES*TX_INTERVAL*1000:.0f}ms)')
            threading.Thread(target=self._tx_loop, daemon=True, name='can_tx').start()
            threading.Thread(target=self._rx_loop, daemon=True, name='can_rx').start()
        else:
            self.get_logger().error('CAN 接続失敗')

    def _connect(self, channel, bitrate):
        try:
            self.bus = can.interface.Bus(
                interface='socketcan', channel=channel, bitrate=bitrate)
            self.get_logger().info(f'socketcan 接続: {channel} @ {bitrate} bps')
        except Exception as e1:
            self.get_logger().warn(f'socketcan 失敗: {e1}')
            try:
                self.bus = can.interface.Bus(channel=channel, bitrate=bitrate)
                self.get_logger().info(f'汎用 CAN 接続: {channel} @ {bitrate} bps')
            except Exception as e2:
                self.get_logger().error(f'CAN 接続失敗: {e2}')
                self.bus = None

    def _cmd_vel_cb(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        v_right = v + w * self.wheel_base_m / 2.0
        v_left  = v - w * self.wheel_base_m / 2.0
        m0 = float(max(-self.max_rps, min(self.max_rps, -v_left  / self.wheel_circum)))
        m1 = float(max(-self.max_rps, min(self.max_rps, -v_right / self.wheel_circum)))
        with self.lock:
            self._targets[0] = m0
            self._targets[1] = m1
            self._cmd_v = v
            self._cmd_w = w

    def _tx_loop(self):
        next_t = time.perf_counter()
        while not self._shutdown.is_set():
            now = time.perf_counter()
            if now >= next_t:
                self._tick()
                next_t += TX_INTERVAL
            time.sleep(0.001)

    def _tick(self):
        if not self.bus:
            return
        try:
            # ── 起動時強制パラメータフェーズ ──────────────────────────
            if self._force_param_count > 0:
                self._send_param_sequence()
                self._force_param_count -= 1
                if self._force_param_count == 0:
                    # 強制送信完了 → MCU の現在モードで通常フェーズ決定
                    if self.remote_app_mode == APP_MODE_PARAMETER:
                        self.param_send_req = True
                        self.get_logger().info(
                            '強制送信完了。MCU=PARAMETER → 通常パラメータ送信開始')
                    else:
                        self.tx_enabled = True
                        self.get_logger().info(
                            '強制送信完了。MCU=CONTROL → 目標値送信開始')
                return

            # ── 通常ステートマシン (GUI の _send_by_remote_state 準拠) ──
            if self.remote_app_mode == APP_MODE_PARAMETER and self.param_send_req:
                self._send_param_sequence()
            elif self.remote_app_mode == APP_MODE_CONTROL and self.tx_enabled:
                self._send_target()
        except Exception as e:
            self.get_logger().warn(f'CAN 送信エラー: {e}')

    def _send_param_sequence(self):
        wheel_mm = int(round(self.wheel_diam * 1000.0))
        p_raw    = int(round(self.pid_p * 100.0))
        i_raw    = int(round(self.pid_i * 100.0))
        d_raw    = int(round(self.pid_d * 100.0))
        motor_dirs = [-1, -1, 1, 1]  # M0=左dir=-1, M1=右dir=-1 (両輪とも負方向が前進)

        for idx in range(4):
            signed_wheel = wheel_mm * motor_dirs[idx]
            payload = bytearray(8)
            p_l, p_h = self._i16le(p_raw)
            i_l, i_h = self._i16le(i_raw)
            d_l, d_h = self._i16le(d_raw)
            w_l, w_h = self._i16le(signed_wheel)
            payload[:] = [p_l, p_h, i_l, i_h, d_l, d_h, w_l, w_h]
            self.bus.send(can.Message(
                is_extended_id=False,
                arbitration_id=CAN_ID_PARAM_BASE + idx,
                data=bytes(payload)))

        self.bus.send(can.Message(
            is_extended_id=False,
            arbitration_id=CAN_ID_MODE,
            data=bytes([CONTROL_SPEED] * 4)))

        now = time.perf_counter()
        if now - self._last_param_log_t >= LOG_PARAM_INTERVAL:
            self._last_param_log_t = now
            self.get_logger().info(
                f'[TX] param: P={self.pid_p}(raw={p_raw}) I={self.pid_i} D={self.pid_d}'
                f' wheel={wheel_mm}mm dirs={motor_dirs}'
                f' force_remain={self._force_param_count}')

    def _send_target(self):
        payload = bytearray(8)
        with self.lock:
            targets = list(self._targets)
        for i, t in enumerate(targets):
            scaled = max(-32768, min(32767, int(round(t * 10.0))))
            l, h = self._i16le(scaled)
            payload[i * 2]     = l
            payload[i * 2 + 1] = h
        self.bus.send(can.Message(
            is_extended_id=False,
            arbitration_id=CAN_ID_TARGET,
            data=bytes(payload)))

    def _rx_loop(self):
        no_warn = False
        start_t = time.perf_counter()
        while not self._shutdown.is_set():
            if not self.bus:
                time.sleep(0.01)
                continue
            try:
                msg = self.bus.recv(timeout=RX_TIMEOUT)
                while msg is not None:
                    if msg.arbitration_id == CAN_ID_STATUS:
                        self._parse_status(msg.data)
                        no_warn = False
                        start_t = time.perf_counter()
                    msg = self.bus.recv(timeout=0.0)
            except Exception as e:
                self.get_logger().warn(f'CAN 受信エラー: {e}')
            if not no_warn and self._last_remote_mode is None and \
                    time.perf_counter() - start_t > 10.0:
                no_warn = True
                self.get_logger().warn(
                    '⚠ MCU から 0x230 が 10秒以上届きません。'
                    'CAN 配線・MCU 電源・ビットレートを確認してください。')

    def _parse_status(self, data):
        if len(data) < 6:
            return
        limit_states = [int(bool(data[i])) for i in range(4)]
        error_code   = data[4]
        app_mode     = data[5] & 0x01

        # GUI の previous_mode ロジック: last_remote_app_mode が None なら previous_mode=None
        previous_mode = self.remote_app_mode if self._last_remote_mode is not None else None
        self._last_remote_mode = self.remote_app_mode
        self.remote_app_mode   = app_mode

        # 強制送信フェーズ中は遷移発火しない (app_mode 更新のみ)
        if self._force_param_count <= 0:
            if previous_mode == APP_MODE_PARAMETER and app_mode == APP_MODE_CONTROL:
                self.param_send_req = False
                self.tx_enabled     = True
                self.get_logger().info('✅ MDD PARAMETER→CONTROL 移行 → 目標値送信開始')
            if previous_mode == APP_MODE_CONTROL and app_mode == APP_MODE_PARAMETER:
                self.param_send_req = False
                self.tx_enabled     = False
                self.get_logger().warn('⛔ MDD CONTROL→PARAMETER 移行 → 送信停止')

        # ステータスログ間引き
        now = time.perf_counter()
        sig = (tuple(limit_states), error_code, app_mode)
        if sig != self._last_status_sig or now - self._last_status_log_t >= LOG_STATUS_INTERVAL:
            self._last_status_sig   = sig
            self._last_status_log_t = now
            sw  = ' '.join(f'SW{i+1}={"ON" if s else "off"}' for i, s in enumerate(limit_states))
            mode = 'CONTROL' if app_mode else 'PARAMETER'
            self.get_logger().info(
                f'[RX] 0x230: {sw} Mode={mode} Err=0x{error_code:02X}'
                f' force={self._force_param_count} tx={self.tx_enabled} prm={self.param_send_req}')

    def _publish_odom(self):
        now = time.perf_counter()
        dt  = now - self._odom_time
        self._odom_time = now
        with self.lock:
            v = self._cmd_v
            w = self._cmd_w
        self._odom_x  += v * math.cos(self._odom_th) * dt
        self._odom_y  += v * math.sin(self._odom_th) * dt
        self._odom_th += w * dt
        self._odom_th  = math.atan2(math.sin(self._odom_th), math.cos(self._odom_th))
        qz = math.sin(self._odom_th / 2.0)
        qw = math.cos(self._odom_th / 2.0)
        stamp = self.get_clock().now().to_msg()

        tf = TransformStamped()
        tf.header.stamp        = stamp
        tf.header.frame_id     = self.odom_frame
        tf.child_frame_id      = self.base_frame
        tf.transform.translation.x = self._odom_x
        tf.transform.translation.y = self._odom_y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self.tf_bc.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp            = stamp
        odom.header.frame_id         = self.odom_frame
        odom.child_frame_id          = self.base_frame
        odom.pose.pose.position.x    = self._odom_x
        odom.pose.pose.position.y    = self._odom_y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = v
        odom.twist.twist.angular.z   = w
        self.odom_pub.publish(odom)

    @staticmethod
    def _i16le(val: int):
        val = int(val) & 0xFFFF
        return val & 0xFF, (val >> 8) & 0xFF

    def destroy_node(self):
        self._shutdown.set()
        if self.bus:
            try:
                self.bus.send(can.Message(
                    is_extended_id=False,
                    arbitration_id=CAN_ID_TARGET,
                    data=bytes(8)))
                self.bus.shutdown()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MddCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
