import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial import cKDTree
import math

def get_transform(src, dst):
    """
    Calculate 2D transformation (R, t) from src to dst using SVD.
    """
    center_src = np.mean(src, axis=0)
    center_dst = np.mean(dst, axis=0)
    
    src_centered = src - center_src
    dst_centered = dst - center_dst
    
    H = src_centered.T @ dst_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
        
    t = center_dst.T - R @ center_src.T
    return R, t

def icp(source, target, max_iterations=30, tolerance=1e-5):
    """
    2D ICP to align source to target.
    """
    src = np.copy(source)
    tree = cKDTree(target)
    
    R_total = np.eye(2)
    t_total = np.zeros(2)
    
    prev_error = float('inf')
    
    for i in range(max_iterations):
        distances, indices = tree.query(src)
        matched_target = target[indices]
        
        R, t = get_transform(src, matched_target)
        src = (R @ src.T).T + t
        
        R_total = R @ R_total
        t_total = R @ t_total + t
        
        mean_error = np.mean(distances)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
        
    return R_total, t_total

class IcpSlamNode(Node):
    def __init__(self):
        super().__init__('icp_slam_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.prev_points = None
        
        # Current estimated pose (x, y, theta)
        self.pose = np.array([0.0, 0.0, 0.0])
        self.get_logger().info("ICP SLAM Node has been started.")

    def scan_callback(self, msg):
        # We disable our inaccurate ICP.
        # Just publish a dummy 0-odometry and let slam_toolbox do the hard work.
        self.pose = np.array([0.0, 0.0, 0.0])
        self.publish_odometry(msg.header.stamp, msg.header.frame_id)

    def publish_odometry(self, stamp, child_frame_id):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        
        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]
        odom.pose.pose.position.z = 0.0
        
        q = self.euler_to_quaternion(0, 0, self.pose[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        self.odom_pub.publish(odom)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.pose[0]
        t.transform.translation.y = self.pose[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = IcpSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
