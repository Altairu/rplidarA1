#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from visualization_msgs.msg import Marker


class MapMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('map_marker_node')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('marker_topic', 'map_marker')
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('cell_skip', 2)

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value

        self.occupied_threshold = int(
            self.get_parameter('occupied_threshold').get_parameter_value().integer_value
        )
        self.cell_skip = max(
            1,
            int(self.get_parameter('cell_skip').get_parameter_value().integer_value),
        )

        self.pub = self.create_publisher(Marker, marker_topic, 10)
        self.sub = self.create_subscription(OccupancyGrid, map_topic, self._on_map, 10)
        self.get_logger().info(
            f'Map marker node started: map_topic={map_topic}, marker_topic={marker_topic}, '
            f'occupied_threshold={self.occupied_threshold}, cell_skip={self.cell_skip}'
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        width = info.width
        height = info.height
        if width == 0 or height == 0:
            return

        marker = Marker()
        marker.header = msg.header
        marker.ns = 'map_cells'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.01, info.resolution * 0.8)
        marker.scale.y = max(0.01, info.resolution * 0.8)
        marker.color.r = 0.1
        marker.color.g = 0.95
        marker.color.b = 0.1
        marker.color.a = 0.9

        data = msg.data
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        step = self.cell_skip

        pts = []
        for y in range(0, height, step):
            base = y * width
            for x in range(0, width, step):
                v = data[base + x]
                if v >= self.occupied_threshold:
                    p = Point()
                    p.x = origin_x + (x + 0.5) * info.resolution
                    p.y = origin_y + (y + 0.5) * info.resolution
                    p.z = 0.0
                    pts.append(p)

        marker.points = pts
        self.pub.publish(marker)


def main() -> None:
    rclpy.init()
    node = MapMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
