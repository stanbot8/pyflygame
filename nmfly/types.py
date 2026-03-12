"""Core data types for nmfly.

These are framework-agnostic: no dependency on any particular spiking
simulator, network protocol, or physics engine.
"""

from dataclasses import dataclass


@dataclass
class MotorCommand:
    """High-level locomotion command.

    Any controller (spiking network, RL policy, keyboard input, etc.)
    produces these. The gait CPG translates them to joint targets.
    """
    forward_velocity: float = 0.0   # mm/s (positive = forward)
    angular_velocity: float = 0.0   # rad/s (positive = left turn)
    approach_drive: float = 0.0     # positive = approach stimulus
    freeze: float = 0.0             # [0, 1], 1 = stop all motion


@dataclass
class SensoryReading:
    """One sensory neuron's output.

    Generic representation of a proprioceptive or exteroceptive signal.
    Adapters convert these to protocol-specific formats (e.g. FWMC BioReading).
    """
    channel: int = 0           # sensor channel index
    activation: float = 0.0    # [0, 1] normalized activation
    raw_value: float = 0.0     # original physical value (radians, N, etc.)
