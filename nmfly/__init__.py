"""flygame: High-performance flygym/NeuroMechFly reimplementation on MuJoCo."""

__version__ = "0.1.0"

from .types import MotorCommand, SensoryReading
from .gait import (
    GaitState, FlightState,
    compute_joint_targets, compute_wing_angles,
    N_LEGS, N_JOINTS_PER_LEG, N_JOINTS_TOTAL,
    REST_POSE, SWING_AMPLITUDE, TRIPOD_PHASE,
    LEG_NAMES,
)
from .proprioception import encode_proprioception, ProprioceptionConfig

__all__ = [
    "MotorCommand", "SensoryReading",
    "GaitState", "FlightState",
    "compute_joint_targets", "compute_wing_angles",
    "encode_proprioception", "ProprioceptionConfig",
    "N_LEGS", "N_JOINTS_PER_LEG", "N_JOINTS_TOTAL",
    "REST_POSE", "SWING_AMPLITUDE", "TRIPOD_PHASE", "LEG_NAMES",
]
