#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket ブリッジノード (ROS2) - シンプル版 (地図なし)

Android タブレット ↔ ロボット間の WebSocket 通信。

クライアント → ロボット (JSON):
  {"type": "cmd_vel", "linear_x": 0.5, "angular_z": 0.3}
  {"type": "stop"}

ロボット → クライアント (JSON, 10Hz):
  {"type": "pose", "x": X, "y": Y, "theta": T, "v": V, "w": W}
"""

import asyncio
import json
import math
import queue
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    import websockets
    WS_AVAILABLE = True
except ImportError:
    WS_AVAILABLE = False


class WebSocketBridgeNode(Node):

    def __init__(self):
        super().__init__('websocket_bridge_node')

        self.declare_parameter('port',           8765)
        self.declare_parameter('max_linear',     2.0)
        self.declare_parameter('max_angular',    10.0)

        self._port    = self.get_parameter('port').value
        self._max_lin = self.get_parameter('max_linear').value
        self._max_ang = self.get_parameter('max_angular').value

        self._cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

        # asyncio スレッド → ROS2 main スレッドへコマンドを渡すキュー
        # (asyncio スレッドから直接 rclpy publish すると DDS スレッド競合でクラッシュ)
        self._cmd_q: queue.Queue = queue.Queue(maxsize=5)
        self._cmd_timer = self.create_timer(0.02, self._process_cmd_queue)

        self._send_q: queue.Queue = queue.Queue(maxsize=30)

        if not WS_AVAILABLE:
            self.get_logger().error(
                'websockets 未インストール: pip install websockets')
            return

        self._loop       = asyncio.new_event_loop()
        self._ws_clients: set = set()  # rclpy の self._clients と衝突しないよう _ws_clients
        threading.Thread(target=self._run_loop, daemon=True,
                         name='ws_loop').start()
        self.get_logger().info(
            f'WebSocket サーバー起動: ws://0.0.0.0:{self._port}')

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
            if not self._ws_clients:
                self._pub_stop()

    async def _broadcast(self, message: str):
        dead = set()
        for ws in self._ws_clients.copy():
            try:
                await ws.send(message)
            except Exception:
                dead.add(ws)
        self._ws_clients -= dead

    # ── ROS2 タイマー: コマンドキューを main スレッドで処理 ──────
    # asyncio スレッドから直接 rclpy.publish() するとクラッシュするため
    # キュー経由で main スレッド（spin）に委ねる
    def _process_cmd_queue(self):
        while True:
            try:
                twist = self._cmd_q.get_nowait()
                self._cmd_pub.publish(twist)
            except queue.Empty:
                break

    # ── WebSocket 受信 ───────────────────────────────────
    def _handle_ws_msg(self, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return
        t = data.get('type', '')
        if t == 'cmd_vel':
            lin = max(-self._max_lin, min(self._max_lin, float(data.get('linear_x',  0.0))))
            ang = max(-self._max_ang, min(self._max_ang, float(data.get('angular_z', 0.0))))
            twist = Twist()
            twist.linear.x  = lin
            twist.angular.z = ang
            try:
                self._cmd_q.put_nowait(twist)
            except queue.Full:
                pass  # キュー満杯時は最新コマンドを優先して古いものを破棄
        elif t == 'stop':
            try:
                # キューをクリアして停止コマンドのみにする
                while not self._cmd_q.empty():
                    self._cmd_q.get_nowait()
                self._cmd_q.put_nowait(Twist())
            except (queue.Empty, queue.Full):
                pass

    def _pub_stop(self):
        try:
            while not self._cmd_q.empty():
                self._cmd_q.get_nowait()
            self._cmd_q.put_nowait(Twist())
        except (queue.Empty, queue.Full):
            pass

    # ── /odom 受信 ───────────────────────────────────────
    def _on_odom(self, msg: Odometry):
        x  = msg.pose.pose.position.x
        y  = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        payload = json.dumps({
            'type':  'pose',
            'x':     round(x, 3),
            'y':     round(y, 3),
            'theta': round(theta, 4),
            'v':     round(v, 3),
            'w':     round(w, 3),
        })
        try:
            self._send_q.put_nowait(payload)
        except queue.Full:
            pass

    def destroy_node(self):
        self._pub_stop()
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
