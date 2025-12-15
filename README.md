# Energy-Aware Multi-Drone Coverage with Battery Management

A ROS2 implementation of persistent multi-robot coverage control with realistic battery discharge and automated charging cycles.

Based on the energy-aware coverage algorithm from Derenick, Michael, and Kumar (IROS 2011).

## Features

🔋 **Realistic Battery Physics**
- LiPo discharge model: 10-12 min flight time per charge
- Fast charging: 3-5 min recharge at 250W
- Automatic state transitions: PATROL → RTD → CHARGING

🤖 **Multi-Drone Coverage**
- Voronoi tessellation for optimal space partitioning
- Energy-aware control with sigmoid weighting
- Smooth handoffs when drones need to charge
- 90%+ coverage quality maintained continuously

⚡ **Key Results**
- Infinite mission duration with rotating charges
- Staggered batteries prevent simultaneous docking
- Distributed control - each drone autonomous

## Quick Start

### Prerequisites
```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Python dependencies
pip3 install numpy scipy shapely matplotlib
```

### Build and Run
```bash
cd ~/ws_sar
colcon build --packages-select coverage_system --symlink-install
source install/setup.bash

# Terminal 1: Launch system
ros2 launch sar_coverage coverage_system.launch.py

# Terminal 2: Test odometry
python3 scripts/dummy_odom_with_docking.py

# Terminal 3: Visualize
rviz2  # Add /coverage/drone_markers and /coverage/voronoi_cells
```

### Standalone Test
```bash
python3 scripts/test_battery_simple.py
```

Runs 30-minute simulation with battery discharge curves and coverage plots.

## Configuration

Tune battery parameters in `energy_aware_coverage.py`:
```python
nominal_voltage: 12.6V    # Fully charged
v_safe: 11.0V             # Request handoff
v_crit: 10.0V             # Emergency dock
capacity_wh: 50.0         # Battery capacity
power_hover: 165.0W       # Baseline power
charge_rate: 250.0W       # Charging power
```

## Results

- **Flight time**: ~10-12 minutes per charge
- **Charge time**: ~3-5 minutes to 95% SoC
- **Coverage quality**: 90.3% mean with 3 drones
- **Continuous operation**: Indefinite with rotating charges

## Tech Stack

ROS2 Humble • Python 3.10 • NumPy • SciPy • Shapely • RViz2

## Reference

Derenick, J., Michael, N., & Kumar, V. (2011). "Energy-aware coverage control with docking for robot teams." *IROS 2011*.

---

**Author**: Atharva Nayak | MS Robotics @ Northeastern University  
**Date**: December 2024  
