#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket ブリッジノード (ROS2) - 自己位置可視化 & 地図配信

クライアント → ロボット (JSON):
    {"type": "reset_pose"}  # 表示上の自己位置原点を現在位置へリセット
    {"type": "full_reset"}  # Cartographer の軌跡を再作成し地図をリセット

ロボット → クライアント (JSON):
    {"type": "pose", "x": X, "y": Y, "theta": T}
    {"type": "status", "ok": true/false, "message": "..."}
    {"type": "map", "width": W, "height": H, "resolution": RES, "originX": OX, "originY": OY, "data": "base64_encoded_occupancy_grid"}
"""

import asyncio
import base64
import json
import math
import queue
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

try:
        from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
        CARTOGRAPHER_SRVS_AVAILABLE = True
except ImportError:
        CARTOGRAPHER_SRVS_AVAILABLE = False

try:
    import websockets
    WS_AVAILABLE = True
except ImportError:
    WS_AVAILABLE = False


class WebSocketBridgeNode(Node):

    def __init__(self):
        super().__init__('websocket_bridge_node')

        self.declare_parameter('port', 8876)
        self.declare_parameter('pose_topic', 'tracked_pose')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cartographer_config_dir', '')
        self.declare_parameter('cartographer_config_basename', 'lidar_only_2d.lua')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tracking_frame', 'laser')
        self.declare_parameter('pose_publish_period_sec', 0.1)
        self.declare_parameter('map_publish_period_sec', 0.5)
        self.declare_parameter('scan_publish_period_sec', 0.1)

        self._port = int(self.get_parameter('port').value)
        self._pose_topic = str(self.get_parameter('pose_topic').value)
        self._map_topic = str(self.get_parameter('map_topic').value)
        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._cfg_dir = str(self.get_parameter('cartographer_config_dir').value)
        self._cfg_base = str(self.get_parameter('cartographer_config_basename').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._tracking_frame = str(self.get_parameter('tracking_frame').value)
        self._pose_publish_period_sec = float(self.get_parameter('pose_publish_period_sec').value)
        self._map_publish_period_sec = float(self.get_parameter('map_publish_period_sec').value)
        self._scan_publish_period_sec = float(self.get_parameter('scan_publish_period_sec').value)

        self._pose_sub = self.create_subscription(
            PoseStamped, self._pose_topic, self._on_pose, 10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, self._map_topic, self._on_map, map_qos)
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._scan_sub = self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, scan_qos)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._tf_pose_timer = self.create_timer(self._pose_publish_period_sec, self._update_pose_from_tf)
        self._map_publish_timer = self.create_timer(self._map_publish_period_sec, self._publish_map)
        self._scan_publish_timer = self.create_timer(self._scan_publish_period_sec, self._publish_scan)

        self._raw_x = 0.0
        self._raw_y = 0.0
        self._raw_th = 0.0
        self._has_pose = False
        self._last_pose_source = 'none'
        self._last_tf_warn_ns = 0

        # Map data cache
        self._map_data = None
        self._map_lock = threading.Lock()
        self._map_rx_logged = False
        self._map_tx_logged = False
        self._scan_data = None
        self._scan_lock = threading.Lock()
        self._scan_rx_logged = False
        self._scan_tx_logged = False

        self._origin_x = 0.0
        self._origin_y = 0.0
        self._origin_th = 0.0
        self._pose_seq = 0

        self._active_trajectory_id = 0
        self._full_reset_in_progress = False

        # asyncio スレッド → ROS2 main スレッドへ制御を渡すキュー
        self._control_q: queue.Queue = queue.Queue(maxsize=10)
        self._control_timer = self.create_timer(0.05, self._process_control_queue)

        self._send_q: queue.Queue = queue.Queue(maxsize=30)

        self._finish_client = None
        self._start_client = None
        if CARTOGRAPHER_SRVS_AVAILABLE:
            self._finish_client = self.create_client(FinishTrajectory, 'finish_trajectory')
            self._start_client = self.create_client(StartTrajectory, 'start_trajectory')
        else:
            self.get_logger().warn('cartographer_ros_msgs が未検出のため full_reset は無効です')

        if not WS_AVAILABLE:
            self.get_logger().error(
                'websockets 未インストール: pip install websockets')
            return

        self._loop       = asyncio.new_event_loop()
        self._ws_clients: set = set()  # rclpy の self._clients と衝突しないよう _ws_clients
        threading.Thread(target=self._run_loop, daemon=True,
                         name='ws_loop').start()
        self.get_logger().info(
            f'WebSocket サーバー起動: ws://0.0.0.0:{self._port} '
            f'(pose_topic={self._pose_topic}, tf={self._map_frame}->{self._tracking_frame})')

    # ── asyncio スレッド ─────────────────────────────────
    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self):
        async with websockets.serve(self._handle_client, '0.0.0.0', self._port):
            while True:
                await self._drain_queue()
                await asyncio.sleep(0.05)   # 20Hz

    async def _drain_queue(self):
        while True:
            try:
                msg = self._send_q.get_nowait()
            except queue.Empty:
                break
            await self._broadcast(msg)

    async def _handle_client(self, websocket, path=None):
        self._ws_clients.add(websocket)
        addr = getattr(websocket, 'remote_address', '?')
        self.get_logger().info(f'クライアント接続: {addr}')
        try:
            async for raw in websocket:
                self._handle_ws_msg(raw)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._ws_clients.discard(websocket)
            self.get_logger().info(f'クライアント切断: {addr}')

    async def _broadcast(self, message: str):
        dead = set()
        for ws in self._ws_clients.copy():
            try:
                await ws.send(message)
            except Exception:
                dead.add(ws)
        self._ws_clients -= dead

    # ── ROS2 タイマー: 制御キューを main スレッドで処理 ─────────
    def _process_control_queue(self):
        while True:
            try:
                cmd = self._control_q.get_nowait()
            except queue.Empty:
                break
            if cmd == 'reset_pose':
                self._reset_pose_origin()
            elif cmd == 'full_reset':
                self._full_reset_map_and_pose()

    # ── WebSocket 受信 ───────────────────────────────────
    def _handle_ws_msg(self, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return
        t = data.get('type', '')
        if t in ('reset_pose', 'full_reset'):
            try:
                self._control_q.put_nowait(t)
            except queue.Full:
                pass
    # ── OccupancyGrid 受信 ────────────────────────────────
    def _on_map(self, msg: OccupancyGrid):
        """OccupancyGrid メッセージを受信して キャッシュ"""
        with self._map_lock:
            self._map_data = msg
        if not self._map_rx_logged:
            self._map_rx_logged = True
            self.get_logger().info(
                f'地図受信開始: {msg.info.width}×{msg.info.height} @ {msg.info.resolution:.3f}m/px')

    def _publish_map(self):
        """定期的に地図データを圧縮してクライアントに送信"""
        try:
            with self._map_lock:
                if self._map_data is None:
                    return
                msg = self._map_data

            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y

            # OccupancyGrid data は int8 (-1..100) なので、
            # Base64 送信用に 0..255 の byte 列へ正規化する。
            data_bytes = bytes((v & 0xFF) for v in msg.data)
            data_b64 = base64.b64encode(data_bytes).decode('utf-8')

            payload = json.dumps({
                'type': 'map',
                'width': width,
                'height': height,
                'resolution': resolution,
                'originX': origin_x,
                'originY': origin_y,
                'data': data_b64,
            })
            self._send_q.put_nowait(payload)
            if not self._map_tx_logged:
                self._map_tx_logged = True
                self.get_logger().info(
                    f'地図配信開始: {width}×{height} @ {resolution:.3f}m/px')
        except queue.Full:
            pass
        except Exception as exc:
            self.get_logger().error(f'地図配信エラー: {exc}')

    # ── LaserScan 受信/配信 ────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        with self._scan_lock:
            self._scan_data = msg
        if not self._scan_rx_logged:
            self._scan_rx_logged = True
            self.get_logger().info(
                f'LiDAR受信開始: points={len(msg.ranges)}, range=[{msg.range_min:.2f}, {msg.range_max:.2f}]m')

    def _publish_scan(self):
        try:
            with self._scan_lock:
                if self._scan_data is None:
                    return
                msg = self._scan_data

            angle = float(msg.angle_min)
            inc = float(msg.angle_increment)
            points = []
            valid_count = 0
            for r in msg.ranges:
                rv = float(r)
                if math.isfinite(rv) and msg.range_min <= rv <= msg.range_max:
                    x = rv * math.cos(angle)
                    y = rv * math.sin(angle)
                    points.append([round(x, 3), round(y, 3)])
                    valid_count += 1
                angle += inc

            payload = json.dumps({
                'type': 'scan',
                'frame': msg.header.frame_id,
                'rangeMin': round(float(msg.range_min), 3),
                'rangeMax': round(float(msg.range_max), 3),
                'pointCount': valid_count,
                'points': points,
            })
            self._send_q.put_nowait(payload)
            if not self._scan_tx_logged:
                self._scan_tx_logged = True
                self.get_logger().info(
                    f'LiDAR配信開始: points={valid_count}, frame={msg.header.frame_id}')
        except queue.Full:
            pass
        except Exception as exc:
            self.get_logger().error(f'LiDAR配信エラー: {exc}')
    # ── tracked_pose 受信 ─────────────────────────────────
    def _on_pose(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        theta = 2.0 * math.atan2(qz, qw)

        self._last_pose_source = 'topic'

        self._update_pose_state(x, y, theta)

    def _update_pose_from_tf(self):
        # Prefer topic data if available, but fall back to TF when no topic publisher exists.
        if self._last_pose_source == 'topic':
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._tracking_frame,
                Time())
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns >= 5_000_000_000:
                self._last_tf_warn_ns = now_ns
                self.get_logger().warn(
                    f'TF lookup待ち: {self._map_frame} -> {self._tracking_frame} ({exc})')
            return

        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)
        qz = float(transform.transform.rotation.z)
        qw = float(transform.transform.rotation.w)
        theta = 2.0 * math.atan2(qz, qw)
        self._last_pose_source = 'tf'
        self._update_pose_state(x, y, theta)

    def _update_pose_state(self, x: float, y: float, theta: float):

        self._raw_x = x
        self._raw_y = y
        self._raw_th = theta
        self._has_pose = True

        px, py, pth = self._pose_in_reset_frame()

        payload = json.dumps({
            'type': 'pose',
            'x': round(px, 3),
            'y': round(py, 3),
            'theta': round(pth, 4),
            'seq': self._pose_seq,
            'source': self._last_pose_source,
        })
        try:
            self._send_q.put_nowait(payload)
        except queue.Full:
            pass

    def _pose_in_reset_frame(self):
        dx = self._raw_x - self._origin_x
        dy = self._raw_y - self._origin_y
        c = math.cos(self._origin_th)
        s = math.sin(self._origin_th)
        x = c * dx + s * dy
        y = -s * dx + c * dy
        th = self._normalize_angle(self._raw_th - self._origin_th)
        return x, y, th

    def _reset_pose_origin(self):
        if not self._has_pose:
            self._enqueue_status(False, '自己位置が未受信のためリセットできません')
            return
        self._origin_x = self._raw_x
        self._origin_y = self._raw_y
        self._origin_th = self._raw_th
        self._pose_seq += 1
        self._enqueue_status(True, '自己位置原点をリセットしました')

    def _full_reset_map_and_pose(self):
        if self._full_reset_in_progress:
            self._enqueue_status(False, '完全リセット処理が実行中です')
            return
        if not CARTOGRAPHER_SRVS_AVAILABLE:
            self._enqueue_status(False, 'cartographer_ros_msgs 未導入のため full_reset 非対応')
            return
        if not self._cfg_dir:
            self._enqueue_status(False, 'cartographer_config_dir が未設定です')
            return
        if self._finish_client is None or self._start_client is None:
            self._enqueue_status(False, 'Cartographer サービスクライアント初期化失敗')
            return

        if not self._finish_client.wait_for_service(timeout_sec=1.0):
            self._enqueue_status(False, 'finish_trajectory サービスが見つかりません')
            return
        if not self._start_client.wait_for_service(timeout_sec=1.0):
            self._enqueue_status(False, 'start_trajectory サービスが見つかりません')
            return

        try:
            self._full_reset_in_progress = True
            finish_req = FinishTrajectory.Request()
            finish_req.trajectory_id = int(self._active_trajectory_id)
            finish_fut = self._finish_client.call_async(finish_req)
            finish_fut.add_done_callback(self._on_finish_trajectory_done)
        except Exception as e:
            self._full_reset_in_progress = False
            self._enqueue_status(False, f'完全リセット失敗: {e}')

    def _on_finish_trajectory_done(self, future):
        try:
            _ = future.result()
        except Exception as e:
            self._full_reset_in_progress = False
            self._enqueue_status(False, f'finish_trajectory 失敗: {e}')
            return

        try:
            start_req = StartTrajectory.Request()
            start_req.configuration_directory = self._cfg_dir
            start_req.configuration_basename = self._cfg_base
            start_req.use_initial_pose = False
            start_req.relative_to_trajectory_id = 0
            start_fut = self._start_client.call_async(start_req)
            start_fut.add_done_callback(self._on_start_trajectory_done)
        except Exception as e:
            self._full_reset_in_progress = False
            self._enqueue_status(False, f'start_trajectory 要求失敗: {e}')

    def _on_start_trajectory_done(self, future):
        try:
            result = future.result()
            if result is None:
                self._enqueue_status(False, 'start_trajectory の応答を取得できませんでした')
                return
            self._active_trajectory_id = int(result.trajectory_id)
            self._reset_pose_origin()
            self._enqueue_status(True, f'完全リセット完了 (trajectory_id={self._active_trajectory_id})')
        except Exception as e:
            self._enqueue_status(False, f'start_trajectory 失敗: {e}')
        finally:
            self._full_reset_in_progress = False

    def _enqueue_status(self, ok: bool, message: str):
        payload = json.dumps({
            'type': 'status',
            'ok': bool(ok),
            'message': str(message),
        })
        try:
            self._send_q.put_nowait(payload)
        except queue.Full:
            pass

    @staticmethod
    def _normalize_angle(rad: float) -> float:
        return math.atan2(math.sin(rad), math.cos(rad))

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
