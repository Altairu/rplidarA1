import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from sklearn.linear_model import RANSACRegressor

class RansacNode(Node):
    def __init__(self):
        super().__init__('ransac_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.marker_pub = self.create_publisher(MarkerArray, '/ransac_lines', 10)
        self.get_logger().info("RANSAC Node has been started.")

    def scan_callback(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        
        # Filter valid ranges
        valid_idx = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        angles = angles[valid_idx]
        ranges = ranges[valid_idx]
        
        if len(ranges) < 10:
            return

        # Polar to Cartesian
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        
        # We try to extract the most prominent line.
        X_data = x.reshape(-1, 1)
        y_data = y
        
        try:
            # Using scikit-learn RANSAC
            ransac = RANSACRegressor(min_samples=10, residual_threshold=0.05, max_trials=100)
            ransac.fit(X_data, y_data)
            
            inlier_mask = ransac.inlier_mask_
            
            if np.sum(inlier_mask) < 10:
                return

            line_X = np.array([X_data[inlier_mask].min(), X_data[inlier_mask].max()])
            line_y_ransac = ransac.predict(line_X.reshape(-1, 1))
            
            self.publish_line(line_X, line_y_ransac, msg.header)
        except ValueError:
            pass

    def publish_line(self, line_X, line_y, header):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header = header
        marker.ns = "ransac_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        p1 = Point()
        p1.x = float(line_X[0])
        p1.y = float(line_y[0])
        p1.z = 0.0
        
        p2 = Point()
        p2.x = float(line_X[1])
        p2.y = float(line_y[1])
        p2.z = 0.0
        
        marker.points.append(p1)
        marker.points.append(p2)
        
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RansacNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
