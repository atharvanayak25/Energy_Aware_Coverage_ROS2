"""
SAR Coverage Package

Energy-Aware Persistent Aerial Search and Rescue
Based on: Derenick, Michael, Kumar (IROS 2011)
"""

from .energy_aware_coverage import (
    DroneState,
    EnergyAwareCoverageController,
    BatteryModel,
    CoverageSimulator,
)

__version__ = "1.0.0"
__author__ = "Atharva"

__all__ = [
    "DroneState",
    "EnergyAwareCoverageController", 
    "BatteryModel",
    "CoverageSimulator",
]