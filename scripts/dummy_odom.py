#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class DummyOdomPublisher(Node):
    def __init__(self):
        super().__init__('dummy_odom_publisher')
        self.get_logger().info("Publishing dummy odometry for 3 drones...")
        
        # Use different name to avoid conflict
        self.odom_pubs = []
        for i in range(3):
            pub = self.create_publisher(Odometry, f'/drone_{i}/odom', 10)
            self.odom_pubs.append(pub)
        
        self.timer = self.create_timer(0.1, self.publish_odom)
        self.drone_positions = [
            np.array([-5.0, 0.0]),
            np.array([0.0, 5.0]),
            np.array([5.0, 0.0])
        ]
        
    def publish_odom(self):
        for i, pub in enumerate(self.odom_pubs):
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            
            # Simple circular motion
            t = self.get_clock().now().nanoseconds / 1e9
            msg.pose.pose.position.x = self.drone_positions[i][0] + 2*np.sin(0.1*t)
            msg.pose.pose.position.y = self.drone_positions[i][1] + 2*np.cos(0.1*t)
            msg.pose.pose.position.z = 5.0
            
            msg.twist.twist.linear.x = 0.2*np.cos(0.1*t)
            msg.twist.twist.linear.y = -0.2*np.sin(0.1*t)
            
            pub.publish(msg)

def main():
    rclpy.init()
    node = DummyOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
