"""flygame: High-performance flygym/NeuroMechFly reimplementation on MuJoCo.

The primary interface is FlygameSim, which wraps the C++ flygame executable
and communicates via TCP::

    from nmfly import FlygameSim, MotorCommand

    with FlygameSim() as sim:
        while sim.running:
            state = sim.step(MotorCommand(forward_velocity=15.0))
"""

__version__ = "0.1.0"

from .types import MotorCommand, SensoryReading
from .flygame import FlygameSim, FlygameConfig, find_flygame_exe, launch
from .gait import (
    GaitState, FlightState,
    compute_joint_targets, compute_wing_angles,
    N_LEGS, N_JOINTS_PER_LEG, N_JOINTS_TOTAL,
    REST_POSE, SWING_AMPLITUDE, TRIPOD_PHASE,
    LEG_NAMES,
)
from .proprioception import encode_proprioception, ProprioceptionConfig

__all__ = [
    "FlygameSim", "FlygameConfig", "find_flygame_exe", "launch",
    "MotorCommand", "SensoryReading",
    "GaitState", "FlightState",
    "compute_joint_targets", "compute_wing_angles",
    "encode_proprioception", "ProprioceptionConfig",
    "N_LEGS", "N_JOINTS_PER_LEG", "N_JOINTS_TOTAL",
    "REST_POSE", "SWING_AMPLITUDE", "TRIPOD_PHASE", "LEG_NAMES",
]
