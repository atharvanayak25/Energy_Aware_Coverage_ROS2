#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np

class SmartOdomPublisher(Node):
    def __init__(self):
        super().__init__('smart_odom_publisher')
        self.get_logger().info("Publishing smart odometry with dock positions...")
        
        self.odom_pubs = []
        for i in range(3):
            pub = self.create_publisher(Odometry, f'/drone_{i}/odom', 10)
            self.odom_pubs.append(pub)
        
        # Subscribe to metrics to know drone states
        self.metrics_sub = self.create_subscription(
            Float32MultiArray,
            '/coverage/metrics',
            self.metrics_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.publish_odom)
        
        # Spawn positions
        self.patrol_positions = [
            np.array([-5.0, 0.0]),
            np.array([0.0, 5.0]),
            np.array([5.0, 0.0])
        ]
        
        # Dock positions (corners)
        self.dock_positions = [
            np.array([-8.0, -8.0]),
            np.array([0.0, -8.0]),
            np.array([8.0, -8.0])
        ]
        
        # Drone states: 1.0 = active/patrol, 0.0 = charging
        self.drone_states = [1.0, 1.0, 1.0]
        
    def metrics_callback(self, msg):
        # Extract active states from metrics
        # Format: [coverage, D0_v, D0_soc, D0_active, D1_v, D1_soc, D1_active, ...]
        if len(msg.data) >= 10:
            self.drone_states[0] = msg.data[3]  # D0 active
            self.drone_states[1] = msg.data[6]  # D1 active
            self.drone_states[2] = msg.data[9]  # D2 active
        
    def publish_odom(self):
        for i, pub in enumerate(self.odom_pubs):
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            
            t = self.get_clock().now().nanoseconds / 1e9
            
            # If charging (state = 0), stay at dock. If patrol (state = 1), move
            if self.drone_states[i] < 0.5:  # Charging
                msg.pose.pose.position.x = self.dock_positions[i][0]
                msg.pose.pose.position.y = self.dock_positions[i][1]
                msg.twist.twist.linear.x = 0.0
                msg.twist.twist.linear.y = 0.0
            else:  # Patrolling
                msg.pose.pose.position.x = self.patrol_positions[i][0] + 2*np.sin(0.1*t)
                msg.pose.pose.position.y = self.patrol_positions[i][1] + 2*np.cos(0.1*t)
                msg.twist.twist.linear.x = 0.2*np.cos(0.1*t)
                msg.twist.twist.linear.y = -0.2*np.sin(0.1*t)
            
            msg.pose.pose.position.z = 5.0
            pub.publish(msg)

def main():
    rclpy.init()
    node = SmartOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
