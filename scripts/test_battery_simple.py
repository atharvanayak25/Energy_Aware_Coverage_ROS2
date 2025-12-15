#!/usr/bin/env python3
"""
Simple test script for battery-integrated coverage system.
Run without ROS2 to verify battery discharge works correctly.
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sar_coverage'))

from energy_aware_coverage import (
    EnergyAwareCoverageController,
    DroneState,
    BatteryModel,
    CoverageSimulator
)

import numpy as np
import matplotlib.pyplot as plt


def test_battery_discharge():
    """Test 1: Battery discharge in isolation"""
    print("="*60)
    print("  TEST 1: Battery Discharge Model")
    print("="*60)
    
    battery = BatteryModel()
    
    print(f"Initial: {battery.voltage:.2f}V, {battery.soc*100:.1f}% SoC")
    print(f"Expected flight time: ~10-15 minutes\n")
    
    # Simulate flight
    t = 0.0
    dt = 1.0
    velocity = np.array([5.0, 0.0])  # 5 m/s horizontal flight
    
    voltages = []
    times = []
    
    while battery.voltage > 10.0 and t < 1200:  # Max 20 min
        voltage, voltage_dot, status = battery.discharge(t, velocity)
        voltages.append(voltage)
        times.append(t / 60.0)  # Convert to minutes
        
        if t % 60 == 0:  # Print every minute
            print(f"  t={t/60:.0f}min: {voltage:.2f}V, {battery.soc*100:.1f}% SoC, {status}")
        
        t += dt
    
    flight_time = t / 60.0
    print(f"\n✓ Flight time until 10.0V: {flight_time:.1f} minutes")
    
    if flight_time < 8 or flight_time > 20:
        print("⚠ Warning: Flight time outside expected range (10-15 min)")
    
    # Plot
    plt.figure(figsize=(10, 4))
    plt.plot(times, voltages, 'b-', linewidth=2)
    plt.axhline(11.0, color='orange', linestyle='--', label='v_safe (handoff)')
    plt.axhline(10.0, color='red', linestyle='--', label='v_crit (emergency)')
    plt.xlabel('Time (minutes)')
    plt.ylabel('Battery Voltage (V)')
    plt.title('Battery Discharge Curve')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('battery_discharge.png', dpi=150)
    print("✓ Saved plot: battery_discharge.png\n")


def test_alpha_function():
    """Test 2: Energy weighting function"""
    print("="*60)
    print("  TEST 2: Energy Weighting Function α(v)")
    print("="*60)
    
    controller = EnergyAwareCoverageController(
        region_bounds=(-10, -10, 10, 10),
        gamma_alpha=10.0
    )
    
    # Test range of voltages
    voltages = np.linspace(9.0, 12.6, 100)
    alphas = [controller.alpha(v) for v in voltages]
    
    print("Sample values:")
    for v in [9.0, 10.0, 11.0, 12.0, 12.6]:
        alpha = controller.alpha(v)
        print(f"  V={v:.1f}V → α={alpha:.3f} ({'coverage' if alpha > 0.5 else 'docking'})")
    
    # Plot
    plt.figure(figsize=(10, 4))
    plt.plot(voltages, alphas, 'g-', linewidth=2)
    plt.axvline(11.0, color='orange', linestyle='--', label='v_safe')
    plt.axvline(10.0, color='red', linestyle='--', label='v_crit')
    plt.axhline(0.5, color='gray', linestyle=':', alpha=0.5)
    plt.xlabel('Battery Voltage (V)')
    plt.ylabel('α(v) - Coverage Weight')
    plt.title('Sigmoid Energy Weighting Function')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.ylim([-0.05, 1.05])
    plt.tight_layout()
    plt.savefig('alpha_function.png', dpi=150)
    print("\n✓ Saved plot: alpha_function.png\n")


def test_full_simulation():
    """Test 3: Full multi-drone coverage with battery"""
    print("="*60)
    print("  TEST 3: Full Coverage Simulation")
    print("="*60)
    
    sim = CoverageSimulator(
        num_drones=3,
        region_bounds=(-10, -10, 10, 10),
        gamma=1.5,
        dt=0.1
    )
    
    # Run for 30 minutes
    sim.run(duration=1800.0, print_interval=120.0)
    
    # Get metrics
    metrics = sim.get_metrics()
    
    # Plot results
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # Battery voltages
    ax = axes[0]
    colors = ['red', 'blue', 'green']
    for i, voltages in enumerate(metrics['voltages']):
        ax.plot(metrics['time']/60, voltages, 
                color=colors[i], linewidth=2, label=f'Drone {i}')
    
    ax.axhline(11.0, color='orange', linestyle='--', 
               linewidth=1, alpha=0.7, label='v_safe')
    ax.axhline(10.0, color='red', linestyle='--', 
               linewidth=1, alpha=0.7, label='v_crit')
    ax.set_ylabel('Battery Voltage (V)')
    ax.set_title('Battery Status Over Time')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    
    # Coverage quality
    ax = axes[1]
    ax.plot(metrics['time']/60, metrics['coverage']*100, 
            'purple', linewidth=2)
    ax.axhline(95, color='green', linestyle='--', 
               linewidth=1, alpha=0.5, label='95% target')
    ax.set_xlabel('Time (minutes)')
    ax.set_ylabel('Coverage Quality (%)')
    ax.set_title('Coverage Quality Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim([0, 105])
    
    plt.tight_layout()
    plt.savefig('simulation_results.png', dpi=150)
    print("\n✓ Saved plot: simulation_results.png\n")
    
    # Statistics
    print("="*60)
    print("  FINAL STATISTICS")
    print("="*60)
    print(f"  Mean coverage: {np.mean(metrics['coverage'])*100:.1f}%")
    print(f"  Min coverage: {np.min(metrics['coverage'])*100:.1f}%")
    print(f"  Max coverage: {np.max(metrics['coverage'])*100:.1f}%")
    
    # Count state changes
    for i, drone in enumerate(metrics['drones']):
        print(f"  Drone {i} final state: {drone.state}, V={drone.voltage:.2f}V")
    
    print("="*60)


def main():
    """Run all tests"""
    print("\n" + "="*60)
    print("  ENERGY-AWARE COVERAGE - BATTERY TESTS")
    print("  Testing integrated battery discharge model")
    print("="*60 + "\n")
    
    try:
        # Test 1: Battery discharge
        test_battery_discharge()
        input("Press Enter to continue to Test 2...")
        
        # Test 2: Alpha function
        test_alpha_function()
        input("Press Enter to continue to Test 3...")
        
        # Test 3: Full simulation
        test_full_simulation()
        
        print("\n" + "="*60)
        print("  ✓ ALL TESTS COMPLETE!")
        print("  Check the generated PNG files for plots.")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\nTests interrupted by user.")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()