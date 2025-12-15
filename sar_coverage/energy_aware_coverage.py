#!/usr/bin/env python3
"""
Energy-aware coverage control with realistic battery discharge model.
Integrates physics-based battery consumption for persistent multi-drone coverage.
"""

import numpy as np
from scipy.spatial import Voronoi
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Callable
from shapely.geometry import Polygon, Point, box
from shapely.ops import unary_union


@dataclass
class BatteryParams:
    """Battery and power consumption parameters - tuned for realistic flight times"""
    # Battery specs (tuned for 10-15 min flight time)
    nominal_voltage: float = 12.6      # V (3S LiPo fully charged)
    min_voltage: float = 9.0           # V (3S LiPo cutoff)
    capacity_wh: float = 50.0          # Wh (balanced for realistic discharge)
    
    # Voltage thresholds
    v_safe: float = 11.0               # Request handoff
    v_crit: float = 10.0               # Emergency dock
    
    # Power consumption (realistic for small quadcopter)
    power_hover: float = 165.0         # W (baseline hovering)
    power_per_vel: float = 18.0        # W/(m/s) (drag)
    power_per_accel: float = 25.0      # W/(m/s²) (thrust)
    efficiency: float = 0.72           # Motor/ESC efficiency
    
    # Charging (ultra-fast charging for persistent coverage)
    charge_rate: float = 250.0         # W (C/0.2 fast charging, ~3 min to 95%)
    charge_voltage: float = 12.6       # V (fully charged)


class BatteryModel:
    """Physics-based battery discharge model for quadcopter drones"""
    
    def __init__(self, params: BatteryParams = None):
        self.params = params or BatteryParams()
        self.voltage = self.params.nominal_voltage
        self.soc = 1.0  # State of charge (0-1)
        
        # For computing acceleration
        self.last_velocity = np.array([0.0, 0.0])
        self.last_time = None
        
    def reset(self):
        """Reset battery to fully charged"""
        self.voltage = self.params.nominal_voltage
        self.soc = 1.0
        self.last_velocity = np.array([0.0, 0.0])
        self.last_time = None
        
    def compute_power_consumption(self, velocity: np.ndarray, 
                                   acceleration: np.ndarray = None) -> float:
        """
        Compute instantaneous power in Watts.
        
        Args:
            velocity: 2D velocity [vx, vy] in m/s
            acceleration: 2D acceleration [ax, ay] in m/s² (optional)
        """
        # Base hovering power
        power = self.params.power_hover
        
        # Power for movement (drag ~ v²)
        vel_mag = np.linalg.norm(velocity)
        power += self.params.power_per_vel * vel_mag**2 / 5.0
        
        # Power for acceleration
        if acceleration is not None:
            accel_mag = np.linalg.norm(acceleration)
            power += self.params.power_per_accel * accel_mag
        
        # Account for inefficiency
        power /= self.params.efficiency
        
        return power
    
    def discharge(self, current_time: float, 
                  velocity: np.ndarray) -> Tuple[float, float, str]:
        """
        Update battery state.
        
        Returns:
            voltage: Current battery voltage
            voltage_dot: Rate of voltage change (V/s)
            status: 'ACTIVE', 'LOW', or 'CRITICAL'
        """
        if self.last_time is None:
            self.last_time = current_time
            self.last_velocity = velocity
            return self.voltage, 0.0, self.get_status()
        
        dt = current_time - self.last_time
        if dt <= 0:
            return self.voltage, 0.0, self.get_status()
        
        # Compute acceleration
        acceleration = (velocity - self.last_velocity) / dt
        
        # Compute power consumption
        power = self.compute_power_consumption(velocity, acceleration)
        
        # Energy consumed (Wh)
        energy_consumed = (power * dt) / 3600.0
        
        # Update SoC
        old_soc = self.soc
        self.soc -= energy_consumed / self.params.capacity_wh
        self.soc = max(0.0, self.soc)
        
        # Update voltage (LiPo discharge curve)
        old_voltage = self.voltage
        v_range = self.params.nominal_voltage - self.params.min_voltage
        self.voltage = self.params.min_voltage + (v_range * self.soc)
        
        # Compute voltage rate
        voltage_dot = (self.voltage - old_voltage) / dt if dt > 0 else 0.0
        
        # Update stored state
        self.last_time = current_time
        self.last_velocity = velocity.copy()
        
        return self.voltage, voltage_dot, self.get_status()
    
    def charge(self, dt: float) -> Tuple[float, bool]:
        """
        Charge battery while docked.
        
        Returns:
            voltage: Current voltage
            fully_charged: True if SoC >= 95% (resume patrol threshold)
        """
        energy_added = (self.params.charge_rate * dt) / 3600.0
        self.soc += energy_added / self.params.capacity_wh
        self.soc = min(1.0, self.soc)
        
        v_range = self.params.nominal_voltage - self.params.min_voltage
        self.voltage = self.params.min_voltage + (v_range * self.soc)
        
        return self.voltage, self.soc >= 0.95  # Resume at 95% instead of 99%
    
    def get_status(self) -> str:
        """Get battery status string"""
        if self.voltage >= self.params.v_safe:
            return "ACTIVE"
        elif self.voltage >= self.params.v_crit:
            return "LOW"
        else:
            return "CRITICAL"
    
    def should_request_handoff(self) -> bool:
        """Check if drone should request coverage handoff"""
        return self.voltage < self.params.v_safe
    
    def must_dock_immediately(self) -> bool:
        """Check if drone must return to dock"""
        return self.voltage < self.params.v_crit
    
    def get_energy_weight(self) -> float:
        """
        Energy-based weight for coverage (sigmoid function).
        Returns value in [0, 1]: 1 = full coverage, 0 = must dock
        """
        v_range = self.params.nominal_voltage - self.params.min_voltage
        v_norm = (self.voltage - self.params.min_voltage) / v_range
        
        k = 15.0  # Steepness
        threshold = 0.3  # Center (30% energy)
        
        weight = 1.0 / (1.0 + np.exp(-k * (v_norm - threshold)))
        return weight
    
    def time_to_critical(self, avg_power: float = None) -> float:
        """Estimate time until critical battery (seconds)"""
        if avg_power is None:
            avg_power = self.params.power_hover
        
        v_range = self.params.nominal_voltage - self.params.min_voltage
        soc_crit = (self.params.v_crit - self.params.min_voltage) / v_range
        energy_remaining = (self.soc - soc_crit) * self.params.capacity_wh
        
        if energy_remaining <= 0:
            return 0.0
        
        return (energy_remaining / avg_power) * 3600.0


@dataclass
class DroneState:
    """State of a single drone with integrated battery model"""
    id: int
    position: np.ndarray              # [x, y] in coverage plane
    velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    base_station: np.ndarray = None   # [x, y] docking position
    entry_point: np.ndarray = None    # [x, y] entry/exit point
    
    # Battery model
    battery: BatteryModel = field(default_factory=BatteryModel)
    
    # Cached battery properties (updated by battery.discharge)
    voltage: float = 12.6
    voltage_dot: float = 0.0
    
    # State machine
    is_docked: bool = False
    is_charging: bool = False
    state: str = "PATROL"  # PATROL, RTD, DOCKING, CHARGING
    
    def __post_init__(self):
        """Initialize battery model"""
        if self.battery is None:
            self.battery = BatteryModel()


class EnergyAwareCoverageController:
    """
    Distributed energy-aware coverage controller using CVT.
    Integrates realistic battery discharge and charging dynamics.
    """
    
    def __init__(
        self,
        region_bounds: Tuple[float, float, float, float],
        gamma: float = 1.0,
        kappa: float = 1.0,
        gamma_alpha: float = 10.0,
        density_func: Optional[Callable] = None
    ):
        """
        Initialize controller.
        
        Args:
            region_bounds: (xmin, ymin, xmax, ymax) of coverage region Q
            gamma: Control gain
            kappa: Convergence rate  
            gamma_alpha: Sigmoid steepness (for alpha function)
            density_func: Optional importance density φ(q)
        """
        self.xmin, self.ymin, self.xmax, self.ymax = region_bounds
        self.region = box(self.xmin, self.ymin, self.xmax, self.ymax)
        
        self.gamma = gamma
        self.kappa = kappa
        self.gamma_alpha = gamma_alpha
        
        self.phi = density_func if density_func else lambda q: 1.0
        
        # Battery parameters (accessible for UI display)
        self.battery_params = BatteryParams()
            
    def alpha(self, voltage: float) -> float:
        """
        Energy weighting function (Eq. 7 from paper).
        
        Returns:
            Value in [0, 1]: 1 = focus on coverage, 0 = focus on docking
        """
        v_safe = self.battery_params.v_safe
        return 1.0 / (1.0 + np.exp(-self.gamma_alpha * (voltage - v_safe)))
    
    def compute_bounded_voronoi(self, positions: List[np.ndarray]) -> List[Polygon]:
        """
        Compute Voronoi tessellation bounded by region Q.
        Uses mirror point technique.
        """
        if len(positions) == 0:
            return []
        if len(positions) == 1:
            return [self.region]
        
        positions = np.array(positions)
        
        # Add mirror points
        margin = 2.0 * max(self.xmax - self.xmin, self.ymax - self.ymin)
        mirrors = []
        for pos in positions:
            mirrors.extend([
                pos + np.array([margin, 0]),
                pos + np.array([-margin, 0]),
                pos + np.array([0, margin]),
                pos + np.array([0, -margin]),
            ])
        all_points = np.vstack([positions, mirrors])
        
        # Compute Voronoi
        vor = Voronoi(all_points)
        
        # Extract bounded cells
        cells = []
        for i in range(len(positions)):
            region_idx = vor.point_region[i]
            vertex_indices = vor.regions[region_idx]
            
            if -1 in vertex_indices or len(vertex_indices) == 0:
                cells.append(self.region)
                continue
                
            vertices = vor.vertices[vertex_indices]
            cell_poly = Polygon(vertices)
            bounded_cell = cell_poly.intersection(self.region)
            cells.append(bounded_cell if not bounded_cell.is_empty else self.region)
        
        return cells
    
    def compute_centroid(self, cell: Polygon) -> np.ndarray:
        """Compute mass centroid of cell with importance density"""
        if not hasattr(cell, 'exterior'):
            return np.array([0.0, 0.0])
        
        # Simplified: use geometric centroid (assuming uniform density)
        centroid = cell.centroid
        return np.array([centroid.x, centroid.y])
    
    def compute_control(self, drone: DroneState, cell: Polygon) -> np.ndarray:
        """
        Compute control input for single drone (Eq. 6 from paper).
        
        Smoothly blends between:
        - Coverage objective: Move toward Voronoi centroid
        - Docking objective: Move toward base station
        
        Based on energy level via sigmoid alpha(V).
        """
        # Get energy weighting
        alpha_val = self.alpha(drone.voltage)
        
        # Coverage term: toward centroid
        c_v = self.compute_centroid(cell)
        u_coverage = -self.gamma * (drone.position - c_v)
        
        # Docking term: toward base station
        if drone.base_station is not None:
            u_dock = -self.gamma * (drone.position - drone.base_station)
        else:
            u_dock = np.array([0.0, 0.0])
        
        # Blended control
        u = alpha_val * u_coverage + (1 - alpha_val) * u_dock
        
        return u
    
    def compute_coverage_metric(self, drones: List[DroneState]) -> float:
        """
        Compute coverage quality metric H(P, Q) from Eq. 4.
        
        Returns value in [0, 1] where 1 = perfect coverage.
        """
        if len(drones) == 0:
            return 0.0
        
        active = [d for d in drones if d.state == "PATROL"]
        if len(active) == 0:
            return 0.0
        
        positions = [d.position for d in active]
        cells = self.compute_bounded_voronoi(positions)
        
        # Compute weighted coverage metric
        total_error = 0.0
        total_mass = 0.0
        
        for drone, cell in zip(active, cells):
            if not hasattr(cell, 'exterior'):
                continue
            
            c_v = self.compute_centroid(cell)
            dist_sq = np.sum((drone.position - c_v)**2)
            
            # Mass of cell (area if uniform density)
            mass_v = cell.area
            
            total_error += mass_v * dist_sq
            total_mass += mass_v
        
        if total_mass == 0:
            return 0.0
        
        # Normalize to [0, 1]
        region_diagonal = np.sqrt((self.xmax - self.xmin)**2 + 
                                 (self.ymax - self.ymin)**2)
        max_error = total_mass * (region_diagonal / 2)**2
        
        coverage_quality = 1.0 - (total_error / max_error) if max_error > 0 else 1.0
        return max(0.0, min(1.0, coverage_quality))
    
    def update_drone_state(self, drone: DroneState, current_time: float, 
                          cell: Polygon, dt: float):
        """
        Update single drone state including battery discharge and state machine.
        
        Args:
            drone: Drone to update
            current_time: Current simulation time
            cell: Drone's Voronoi cell
            dt: Time step
        """
        # Update battery based on state
        if drone.state == "CHARGING":
            # Charging at dock
            voltage, fully_charged = drone.battery.charge(dt)
            drone.voltage = voltage
            drone.voltage_dot = 0.0  # Simplified for charging
            
            if fully_charged:
                print(f"  D{drone.id}: Fully charged! Resuming patrol. V={voltage:.2f}V")
                drone.state = "PATROL"
                drone.is_charging = False
                drone.is_docked = False
                # CRITICAL: Reset battery timer to prevent instant drain
                drone.battery.last_time = None
                drone.battery.last_velocity = np.array([0.0, 0.0])
                
        else:
            # Discharging (PATROL, RTD, DOCKING)
            voltage, voltage_dot, status = drone.battery.discharge(
                current_time, drone.velocity
            )
            drone.voltage = voltage
            drone.voltage_dot = voltage_dot
            
            # State machine transitions
            if drone.state == "PATROL":
                if drone.battery.must_dock_immediately():
                    print(f"  D{drone.id}: CRITICAL BATTERY! Emergency RTD. V={voltage:.2f}V")
                    drone.state = "RTD"
                elif drone.battery.should_request_handoff():
                    print(f"  D{drone.id}: Low battery, requesting handoff. V={voltage:.2f}V")
                    # In a full implementation, trigger handoff coordination here
                    drone.state = "RTD"  # Simplified: go straight to RTD
            
            elif drone.state == "RTD":
                # Check if at dock
                if drone.base_station is not None:
                    dist_to_dock = np.linalg.norm(drone.position - drone.base_station)
                    if dist_to_dock < 0.5:  # Within 50cm
                        print(f"  D{drone.id}: Arrived at dock. Starting charge. V={voltage:.2f}V")
                        drone.state = "CHARGING"
                        drone.is_docked = True
                        drone.is_charging = True
                        drone.velocity = np.array([0.0, 0.0])
                        # Reset battery timer when starting to charge
                        drone.battery.last_time = None
                        drone.battery.last_velocity = np.array([0.0, 0.0])
        
        # Compute control input based on state
        if drone.state == "PATROL":
            # Normal coverage control
            u = self.compute_control(drone, cell)
        elif drone.state == "RTD" or drone.state == "DOCKING":
            # Navigate to dock
            if drone.base_station is not None:
                u = -self.gamma * (drone.position - drone.base_station)
            else:
                u = np.array([0.0, 0.0])
        else:  # CHARGING
            u = np.array([0.0, 0.0])
        
        # Update velocity and position (simple integrator dynamics)
        drone.velocity = u
        drone.position = drone.position + drone.velocity * dt
        
        # Clamp to region bounds
        drone.position[0] = np.clip(drone.position[0], self.xmin, self.xmax)
        drone.position[1] = np.clip(drone.position[1], self.ymin, self.ymax)


class CoverageSimulator:
    """
    Simulator wrapper for testing coverage algorithm without ROS2.
    Provides the same interface as the test file expects.
    """
    
    def __init__(
        self,
        num_drones: int = 3,
        region_bounds: Tuple[float, float, float, float] = (-10, -10, 10, 10),
        gamma: float = 1.5,
        dt: float = 0.1
    ):
        self.num_drones = num_drones
        self.dt = dt
        self.time = 0.0
        
        # Controller
        self.controller = EnergyAwareCoverageController(
            region_bounds=region_bounds,
            gamma=gamma,
            gamma_alpha=10.0
        )
        
        # Initialize drones
        self.drones = self._initialize_drones()
        
        # Tracking for metrics
        self.coverage_history = []
        self.voltage_history = [[] for _ in range(num_drones)]
        self.time_history = []
        
    def _initialize_drones(self) -> List[DroneState]:
        """Initialize drone states"""
        drones = []
        
        spawn_positions = [
            np.array([-5.0, 0.0]),
            np.array([0.0, 5.0]),
            np.array([5.0, 0.0])
        ]
        
        base_stations = [
            np.array([-8.0, -8.0]),
            np.array([0.0, -8.0]),
            np.array([8.0, -8.0])
        ]
        
        # Stagger initial battery levels to prevent simultaneous docking
        initial_soc = [1.0, 0.85, 0.70]  # 100%, 85%, 70%
        
        for i in range(self.num_drones):
            pos_2d = spawn_positions[i % len(spawn_positions)]
            base = base_stations[i % len(base_stations)]
            
            drone = DroneState(
                id=i,
                position=pos_2d.copy(),
                base_station=base.copy(),
                battery=BatteryModel()
            )
            
            # Set staggered initial battery level
            drone.battery.soc = initial_soc[i % len(initial_soc)]
            v_range = drone.battery.params.nominal_voltage - drone.battery.params.min_voltage
            drone.battery.voltage = drone.battery.params.min_voltage + (v_range * drone.battery.soc)
            drone.voltage = drone.battery.voltage
            
            drone.state = "PATROL"
            drones.append(drone)
        
        return drones
    
    def step(self):
        """Execute one simulation step"""
        # Get active drones
        active = [d for d in self.drones if d.state == "PATROL"]
        
        # Compute Voronoi for active drones
        if len(active) > 0:
            positions = [d.position for d in active]
            cells = self.controller.compute_bounded_voronoi(positions)
            
            # Update each active drone
            for drone, cell in zip(active, cells):
                self.controller.update_drone_state(drone, self.time, cell, self.dt)
        
        # Update non-active drones
        for drone in self.drones:
            if drone.state != "PATROL":
                self.controller.update_drone_state(
                    drone, self.time, self.controller.region, self.dt
                )
        
        # Record metrics
        self.time_history.append(self.time)
        coverage = self.controller.compute_coverage_metric(self.drones)
        self.coverage_history.append(coverage)
        
        for i, drone in enumerate(self.drones):
            self.voltage_history[i].append(drone.voltage)
        
        self.time += self.dt
    
    def run(self, duration: float, print_interval: float = 60.0):
        """Run simulation for specified duration"""
        print("="*60)
        print("  ENERGY-AWARE COVERAGE CONTROL - BATTERY TEST")
        print("="*60)
        print(f"\nRunning simulation for {duration/60:.0f} minutes...")
        print("Expected: Drones should handoff around 10-15 minutes\n")
        
        last_print = 0.0
        
        while self.time < duration:
            self.step()
            
            # Print status
            if self.time - last_print >= print_interval:
                self._print_status()
                last_print = self.time
        
        # Final report
        self._print_final_report(duration)
    
    def _print_status(self):
        """Print current status"""
        print(f"\nt = {self.time/60:.1f} min:")
        for d in self.drones:
            ttc = d.battery.time_to_critical() / 60.0
            print(f"  D{d.id}: {d.voltage:.2f}V ({d.battery.soc*100:.1f}% SoC), "
                  f"State={d.state}, TTC={ttc:.1f}min")
        
        coverage = self.controller.compute_coverage_metric(self.drones)
        print(f"  Coverage quality: {coverage*100:.1f}%")
    
    def _print_final_report(self, duration: float):
        """Print final simulation report"""
        print("\n" + "="*60)
        print("  SIMULATION COMPLETE")
        print("="*60)
        print(f"  Duration: {duration/60:.1f} minutes")
        
        avg_coverage = np.mean(self.coverage_history) if self.coverage_history else 0.0
        min_coverage = np.min(self.coverage_history) if self.coverage_history else 0.0
        
        print(f"  Mean coverage: {avg_coverage*100:.1f}%")
        print(f"  Min coverage: {min_coverage*100:.1f}%")
        
        for d in self.drones:
            print(f"  Drone {d.id}: {d.voltage:.2f}V, State={d.state}, "
                  f"Docked={d.is_docked}")
        print("="*60)
    
    def get_metrics(self):
        """Get simulation metrics for plotting"""
        return {
            'time': np.array(self.time_history),
            'coverage': np.array(self.coverage_history),
            'voltages': [np.array(v) for v in self.voltage_history],
            'drones': self.drones
        }


# Test/example usage
if __name__ == "__main__":
    """Standalone test of battery-integrated coverage control"""
    
    # Create simulator
    sim = CoverageSimulator(
        num_drones=3,
        region_bounds=(-10, -10, 10, 10),
        gamma=1.5,
        dt=0.1
    )
    
    # Run simulation for 30 minutes
    sim.run(duration=1800.0, print_interval=60.0)