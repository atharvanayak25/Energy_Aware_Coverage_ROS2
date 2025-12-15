#!/usr/bin/env python3
"""
ROS2 node for energy-aware coverage control with realistic battery discharge.
Integrates with PX4 SITL and Gazebo simulation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, String

# Import our integrated battery model
from sar_coverage.energy_aware_coverage import (
    EnergyAwareCoverageController,
    DroneState,
    BatteryModel,
    BatteryParams
)


class CoverageControlNode(Node):
    """
    ROS2 node for multi-drone coverage with integrated battery management.
    """
    
    def __init__(self):
        super().__init__('coverage_control_node')
        
        # Parameters
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('flight_altitude', 5.0)
        self.declare_parameter('control_rate', 10.0)  # Hz
        
        self.num_drones = self.get_parameter('num_drones').value
        self.flight_altitude = self.get_parameter('flight_altitude').value
        self.control_rate = self.get_parameter('control_rate').value
        
        # Coverage controller
        self.controller = EnergyAwareCoverageController(
            region_bounds=(-10, -10, 10, 10),
            gamma=1.5,
            gamma_alpha=10.0
        )
        
        # Initialize drone states
        self.drones = self._initialize_drones()
        
        # QoS profile for subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Subscribers (per drone) - using standard Odometry
        self.odom_subs = []
        for i in range(self.num_drones):
            odom_sub = self.create_subscription(
                Odometry,
                f'/drone_{i}/odom',
                lambda msg, idx=i: self.odom_callback(msg, idx),
                qos_profile
            )
            self.odom_subs.append(odom_sub)
        
        # Publishers (per drone)
        self.setpoint_pubs = []
        for i in range(self.num_drones):
            pub = self.create_publisher(
                PoseStamped,
                f'/drone_{i}/setpoint',
                10
            )
            self.setpoint_pubs.append(pub)
        
        # Visualization publishers
        self.drone_marker_pub = self.create_publisher(
            MarkerArray, '/coverage/drone_markers', 10
        )
        self.voronoi_pub = self.create_publisher(
            MarkerArray, '/coverage/voronoi_cells', 10
        )
        self.metrics_pub = self.create_publisher(
            Float32MultiArray, '/coverage/metrics', 10
        )
        self.battery_pub = self.create_publisher(
            String, '/coverage/battery_events', 10
        )
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, 
            self.control_loop
        )
        
        # Simulation time tracking
        self.start_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        
        self.get_logger().info(
            f'Coverage control node initialized with {self.num_drones} drones'
        )
        self.get_logger().info(
            f'Battery params: v_safe={self.controller.battery_params.v_safe}V, '
            f'v_crit={self.controller.battery_params.v_crit}V'
        )
    
    def _initialize_drones(self) -> list:
        """Initialize drone states with positions and base stations"""
        drones = []
        
        # Spawn positions (spread across region)
        spawn_positions = [
            np.array([-5.0, 0.0]),
            np.array([0.0, 5.0]),
            np.array([5.0, 0.0])
        ]
        
        # Base station positions (along bottom edge)
        base_stations = [
            np.array([-8.0, -8.0]),
            np.array([0.0, -8.0]),
            np.array([8.0, -8.0])
        ]
        
        for i in range(self.num_drones):
            pos_2d = spawn_positions[i % len(spawn_positions)]
            base = base_stations[i % len(base_stations)]
            
            drone = DroneState(
                id=i,
                position=pos_2d.copy(),
                base_station=base.copy(),
                battery=BatteryModel()  # Fresh battery
            )
            drone.state = "PATROL"
            
            drones.append(drone)
        
        return drones
    
    def odom_callback(self, msg: Odometry, drone_id: int):
        """Update drone position and velocity from odometry"""
        if drone_id >= len(self.drones):
            return
        
        drone = self.drones[drone_id]
        
        # Extract 2D position
        drone.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        
        # Extract 2D velocity
        drone.velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        ])
    
    def control_loop(self):
        """Main control loop - runs at control_rate Hz"""
        current_time_ros = self.get_clock().now()
        current_time = (current_time_ros - self.start_time).nanoseconds / 1e9
        
        # Compute time step
        dt = (current_time_ros - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time_ros
        
        # Get active (patrolling) drones
        active_drones = [d for d in self.drones if d.state == "PATROL"]
        
        # Compute Voronoi tessellation for active drones
        if len(active_drones) > 0:
            positions = [d.position for d in active_drones]
            cells = self.controller.compute_bounded_voronoi(positions)
            
            # Update each active drone
            for drone, cell in zip(active_drones, cells):
                self._update_drone(drone, cell, current_time, dt)
        
        # Update drones in other states (RTD, CHARGING, etc.)
        for drone in self.drones:
            if drone.state != "PATROL":
                # Use region as dummy cell for non-patrolling drones
                self._update_drone(drone, self.controller.region, current_time, dt)
        
        # Publish commands
        self._publish_setpoints()
        
        # Publish visualization
        self._publish_visualization(active_drones, cells if len(active_drones) > 0 else [])
        
        # Publish metrics
        self._publish_metrics()
    
    def _update_drone(self, drone: DroneState, cell, current_time: float, dt: float):
        """
        Update single drone state including battery and control.
        Logs battery events to ROS.
        """
        old_state = drone.state
        old_voltage = drone.voltage
        
        # Update using controller (handles battery discharge + state machine)
        self.controller.update_drone_state(drone, current_time, cell, dt)
        
        # FAKE DOCKING: Instantly teleport to base when entering RTD
        if drone.state == "RTD" and old_state == "PATROL":
            if drone.base_station is not None:
                self.get_logger().info(f"D{drone.id}: Teleporting to dock (fake docking)")
                drone.position = drone.base_station.copy()
                drone.velocity = np.array([0.0, 0.0])
        
        # Log state transitions
        if drone.state != old_state:
            event_msg = String()
            event_msg.data = f"D{drone.id}: {old_state} -> {drone.state}, V={drone.voltage:.2f}V"
            self.battery_pub.publish(event_msg)
            self.get_logger().info(event_msg.data)
        
        # Log battery warnings
        if drone.battery.should_request_handoff() and old_voltage >= self.controller.battery_params.v_safe:
            self.get_logger().warn(
                f'D{drone.id}: Battery LOW! V={drone.voltage:.2f}V'
            )
        if drone.battery.must_dock_immediately() and old_voltage >= self.controller.battery_params.v_crit:
            self.get_logger().error(
                f'D{drone.id}: Battery CRITICAL! V={drone.voltage:.2f}V'
            )
    
    def _publish_setpoints(self):
        """Publish position setpoints to PX4"""
        for i, drone in enumerate(self.drones):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            
            # Convert 2D position to 3D with altitude
            msg.pose.position.x = float(drone.position[0])
            msg.pose.position.y = float(drone.position[1])
            
            # Altitude depends on state
            if drone.state == "CHARGING" or drone.is_docked:
                msg.pose.position.z = 0.5  # Low altitude at dock
            else:
                msg.pose.position.z = self.flight_altitude
            
            # Orientation (yaw toward velocity direction)
            if np.linalg.norm(drone.velocity) > 0.1:
                yaw = np.arctan2(drone.velocity[1], drone.velocity[0])
            else:
                yaw = 0.0
            
            msg.pose.orientation.z = np.sin(yaw / 2.0)
            msg.pose.orientation.w = np.cos(yaw / 2.0)
            
            self.setpoint_pubs[i].publish(msg)
    
    def _publish_visualization(self, active_drones, cells):
        """Publish RViz visualization markers"""
        # Drone markers
        drone_markers = MarkerArray()
        for drone in self.drones:
            # Drone sphere
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "drones"
            m.id = drone.id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(drone.position[0])
            m.pose.position.y = float(drone.position[1])
            m.pose.position.z = self.flight_altitude
            m.scale.x = m.scale.y = m.scale.z = 0.5
            
            # Color based on battery status
            if drone.voltage >= self.controller.battery_params.v_safe:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0  # Green
            elif drone.voltage >= self.controller.battery_params.v_crit:
                m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0  # Orange
            else:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0  # Red
            
            m.color.a = 1.0 if not drone.is_docked else 0.3
            drone_markers.markers.append(m)
            
            # Text label
            t = Marker()
            t.header = m.header
            t.ns = "labels"
            t.id = drone.id + 100
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(drone.position[0])
            t.pose.position.y = float(drone.position[1])
            t.pose.position.z = self.flight_altitude + 1.0
            t.scale.z = 0.5
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f"D{drone.id}: {drone.voltage:.1f}V\n{drone.state}"
            drone_markers.markers.append(t)
        
        self.drone_marker_pub.publish(drone_markers)
        
        # Voronoi cells
        if len(active_drones) > 0 and len(cells) > 0:
            cell_markers = MarkerArray()
            colors = [(1, 0, 0), (0, 0, 1), (0, 1, 0)]
            
            for drone, cell in zip(active_drones, cells):
                if hasattr(cell, 'exterior'):
                    m = Marker()
                    m.header.frame_id = "map"
                    m.header.stamp = self.get_clock().now().to_msg()
                    m.ns = "voronoi"
                    m.id = drone.id
                    m.type = Marker.LINE_STRIP
                    m.action = Marker.ADD
                    m.scale.x = 0.1
                    
                    c = colors[drone.id % len(colors)]
                    m.color.r, m.color.g, m.color.b = float(c[0]), float(c[1]), float(c[2])
                    m.color.a = 0.8
                    
                    coords = list(cell.exterior.coords)
                    for x, y in coords:
                        p = Point()
                        p.x, p.y, p.z = float(x), float(y), self.flight_altitude
                        m.points.append(p)
                    
                    cell_markers.markers.append(m)
            
            self.voronoi_pub.publish(cell_markers)
    
    def _publish_metrics(self):
        """Publish coverage metrics and battery status"""
        coverage = self.controller.compute_coverage_metric(self.drones)
        
        metrics = Float32MultiArray()
        metrics.data = [float(coverage)]
        
        for d in self.drones:
            metrics.data.extend([
                float(d.voltage),
                float(d.battery.soc),
                1.0 if d.state == "PATROL" else 0.0
            ])
        
        self.metrics_pub.publish(metrics)


def main(args=None):
    rclpy.init(args=args)
    
    node = CoverageControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down coverage control...')
    finally:
        # Print final battery status
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("  FINAL BATTERY STATUS")
        node.get_logger().info("="*60)
        for d in node.drones:
            node.get_logger().info(
                f"  Drone {d.id}: {d.voltage:.2f}V ({d.battery.soc*100:.1f}% SoC), "
                f"State={d.state}, Docked={d.is_docked}"
            )
        node.get_logger().info("="*60)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()