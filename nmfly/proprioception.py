"""Encode body state as sensory readings.

Maps proprioceptive signals (joint angles, contact forces, body velocity)
to normalized activations suitable for driving sensory neuron populations.

Channel mapping (configurable, defaults match Drosophila VNC layout):
    0..41    joint angle sensors (6 legs x 7 joints)
    42..83   joint velocity sensors (6 legs x 7 joints)
    84..89   leg contact sensors (6 legs, binary ground contact)
    90..92   body velocity (forward, lateral, angular)
    93..94   wing angle sensors (left, right)
"""

import math
from dataclasses import dataclass

import numpy as np

from .types import SensoryReading

# Default channel ranges
JOINT_ANGLE_START = 0
JOINT_VEL_START = 42
CONTACT_START = 84
BODY_VEL_START = 90
WING_ANGLE_START = 93
TOTAL_SENSORY_CHANNELS = 95


@dataclass
class ProprioceptionConfig:
    """Configure how body state maps to activations."""
    joint_angle_start: int = JOINT_ANGLE_START
    joint_vel_start: int = JOINT_VEL_START
    contact_start: int = CONTACT_START
    body_vel_start: int = BODY_VEL_START
    wing_angle_start: int = WING_ANGLE_START

    # Gain: how strongly physical values drive activation
    angle_gain: float = 0.3      # radians -> activation
    velocity_gain: float = 0.1   # rad/s -> activation
    contact_prob: float = 0.8    # activation when leg is on ground
    body_vel_gain: float = 0.05  # mm/s -> activation


def encode_proprioception(
    joint_angles: np.ndarray,
    joint_velocities: np.ndarray | None = None,
    contact_forces: np.ndarray | None = None,
    body_velocity: np.ndarray | None = None,
    wing_angles: tuple[float, float] | None = None,
    config: ProprioceptionConfig | None = None,
) -> list[SensoryReading]:
    """Convert body state to a list of SensoryReadings.

    Args:
        joint_angles: (42,) array of joint angles in radians.
        joint_velocities: (42,) array of joint angular velocities (rad/s).
        contact_forces: (6,) array of ground contact force per leg (N).
        body_velocity: (3,) array [forward_mm_s, lateral_mm_s, angular_rad_s].
        wing_angles: (left, right) stroke angles in radians.
        config: channel mapping and gain parameters.

    Returns:
        List of SensoryReading structs.
    """
    cfg = config or ProprioceptionConfig()
    readings: list[SensoryReading] = []

    # Joint angle sensors
    flat_angles = joint_angles.flatten()
    for i, angle in enumerate(flat_angles[:42]):
        val = float(angle)
        act = _sigmoid(abs(val) * cfg.angle_gain)
        readings.append(SensoryReading(
            channel=cfg.joint_angle_start + i,
            activation=act,
            raw_value=val,
        ))

    # Joint velocity sensors
    if joint_velocities is not None:
        flat_vel = joint_velocities.flatten()
        for i, vel in enumerate(flat_vel[:42]):
            val = float(vel)
            act = _sigmoid(abs(val) * cfg.velocity_gain)
            readings.append(SensoryReading(
                channel=cfg.joint_vel_start + i,
                activation=act,
                raw_value=val,
            ))

    # Contact sensors (binary threshold on force)
    if contact_forces is not None:
        for i, force in enumerate(contact_forces[:6]):
            val = float(force)
            on_ground = val > 0.01
            act = cfg.contact_prob if on_ground else 0.02
            readings.append(SensoryReading(
                channel=cfg.contact_start + i,
                activation=act,
                raw_value=val,
            ))

    # Body velocity sensors
    if body_velocity is not None:
        for i, vel in enumerate(body_velocity[:3]):
            val = float(vel)
            act = _sigmoid(abs(val) * cfg.body_vel_gain)
            readings.append(SensoryReading(
                channel=cfg.body_vel_start + i,
                activation=act,
                raw_value=val,
            ))

    # Wing angle sensors
    if wing_angles is not None:
        for i, angle in enumerate(wing_angles):
            val = float(angle)
            act = _sigmoid(abs(val) * 0.2)
            readings.append(SensoryReading(
                channel=cfg.wing_angle_start + i,
                activation=act,
                raw_value=val,
            ))

    return readings


def _sigmoid(x: float) -> float:
    """Squash to [0, 1] with smooth saturation."""
    return 1.0 / (1.0 + math.exp(-x + 2.0))
