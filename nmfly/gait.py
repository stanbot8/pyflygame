"""Tripod gait CPG and flight wing kinematics.

Converts MotorCommand into per-leg joint angle targets for a 6-legged
insect body (Drosophila-like). Framework-agnostic: works with any physics
engine or even without one.

Joint order per leg (matches NeuroMechFly v2 naming):
    0: ThC_yaw     (coxa yaw)
    1: ThC_pitch   (coxa pitch, main swing)
    2: ThC_roll    (coxa roll)
    3: CTr_pitch   (trochanter/femur pitch)
    4: CTr_roll    (trochanter roll)
    5: FTi_pitch   (tibia pitch)
    6: TiTa_pitch  (tarsus pitch)
"""

import math
from dataclasses import dataclass

import numpy as np

from .types import MotorCommand

# 6 legs: L1 R1 L2 R2 L3 R3 (front to back, alternating sides)
LEG_NAMES = ["LF", "RF", "LM", "RM", "LH", "RH"]
N_LEGS = 6
N_JOINTS_PER_LEG = 7
N_JOINTS_TOTAL = N_LEGS * N_JOINTS_PER_LEG  # 42

# Tripod gait phase offsets (radians): two alternating groups
# Group A (phase 0): L1, R2, L3
# Group B (phase pi): R1, L2, R3
TRIPOD_PHASE = np.array([0.0, math.pi, math.pi, 0.0, 0.0, math.pi])

# Rest pose (radians) for each joint, used as baseline
REST_POSE = np.array([
    0.0,    # ThC_yaw
    0.3,    # ThC_pitch (slightly forward)
    0.0,    # ThC_roll
    -0.8,   # CTr_pitch (femur down)
    0.0,    # CTr_roll
    1.2,    # FTi_pitch (tibia bent)
    0.5,    # TiTa_pitch (tarsus on ground)
])

# Swing amplitude per joint during walking
SWING_AMPLITUDE = np.array([
    0.1,    # ThC_yaw   (slight lateral)
    0.4,    # ThC_pitch (main swing, big)
    0.05,   # ThC_roll  (minimal)
    0.2,    # CTr_pitch (femur lift during swing)
    0.0,    # CTr_roll
    0.15,   # FTi_pitch (tibia extends during swing)
    0.1,    # TiTa_pitch
])


@dataclass
class GaitState:
    """CPG state for locomotion."""
    phase: float = 0.0        # gait cycle phase [0, 2*pi]
    frequency: float = 8.0    # Hz, natural walking frequency for Drosophila

    # Smoothed motor command
    fwd_vel: float = 0.0
    ang_vel: float = 0.0
    frozen: bool = False

    # EMA smoothing
    tau: float = 0.05  # seconds

    def update(self, cmd: MotorCommand, dt: float) -> None:
        """Advance gait phase and smooth motor command."""
        alpha = 1.0 - math.exp(-dt / self.tau) if self.tau > 0 else 1.0

        self.fwd_vel += alpha * (cmd.forward_velocity - self.fwd_vel)
        self.ang_vel += alpha * (cmd.angular_velocity - self.ang_vel)
        self.frozen = cmd.freeze > 0.5

        if self.frozen:
            return  # freeze: don't advance phase

        # Phase rate proportional to forward speed
        # At 30 mm/s (Drosophila max), frequency is ~12 Hz
        speed = abs(self.fwd_vel)
        self.frequency = max(2.0, min(15.0, speed * 0.4))
        direction = 1.0 if self.fwd_vel >= 0 else -1.0

        self.phase += direction * 2.0 * math.pi * self.frequency * dt
        self.phase %= (2.0 * math.pi)


def compute_joint_targets(gait: GaitState) -> np.ndarray:
    """Compute 42 joint angle targets from current gait state.

    Returns array of shape (6, 7) = (legs, joints_per_leg).
    """
    targets = np.tile(REST_POSE, (N_LEGS, 1))  # (6, 7)

    if gait.frozen:
        return targets  # freeze pose

    speed_scale = min(1.0, abs(gait.fwd_vel) / 30.0)

    for leg_idx in range(N_LEGS):
        leg_phase = gait.phase + TRIPOD_PHASE[leg_idx]
        swing = math.sin(leg_phase)  # [-1, 1]
        lift = max(0.0, math.sin(leg_phase))  # [0, 1] only during swing

        # Side bias for turning: reduce stride on inside, increase outside
        is_left = (leg_idx % 2 == 0)
        turn_bias = 1.0
        if gait.ang_vel > 0.1:  # turning left
            turn_bias = 0.5 if is_left else 1.5
        elif gait.ang_vel < -0.1:  # turning right
            turn_bias = 1.5 if is_left else 0.5

        for joint_idx in range(N_JOINTS_PER_LEG):
            amp = SWING_AMPLITUDE[joint_idx] * speed_scale * turn_bias

            if joint_idx == 1:  # ThC_pitch: main swing joint
                targets[leg_idx, joint_idx] += amp * swing
            elif joint_idx == 3:  # CTr_pitch: lift during swing phase
                targets[leg_idx, joint_idx] += amp * lift
            elif joint_idx == 5:  # FTi_pitch: extend during swing
                targets[leg_idx, joint_idx] -= amp * lift * 0.5
            else:
                targets[leg_idx, joint_idx] += amp * swing * 0.3

    return targets


@dataclass
class FlightState:
    """Wing state for flight mode."""
    phase: float = 0.0
    frequency: float = 200.0  # Hz, Drosophila wing beat frequency
    amplitude: float = 2.5    # radians, full stroke amplitude

    # Smoothed steering
    roll_bias: float = 0.0  # asymmetric wing beat for turning

    def update(self, cmd: MotorCommand, dt: float) -> None:
        self.phase += 2.0 * math.pi * self.frequency * dt
        self.phase %= (2.0 * math.pi)

        alpha = 1.0 - math.exp(-dt / 0.02)
        self.roll_bias += alpha * (cmd.angular_velocity * 0.1 - self.roll_bias)


def compute_wing_angles(flight: FlightState) -> tuple[float, float]:
    """Compute left and right wing stroke angles.

    Returns (left_angle, right_angle) in radians.
    """
    base = math.sin(flight.phase) * flight.amplitude
    left = base + flight.roll_bias
    right = base - flight.roll_bias
    return left, right
