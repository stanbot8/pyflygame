"""Tests for nmfly core library."""

import math

import numpy as np
import pytest

from nmfly.types import MotorCommand, SensoryReading
from nmfly.gait import (
    GaitState, compute_joint_targets, N_LEGS, N_JOINTS_PER_LEG,
    FlightState, compute_wing_angles, REST_POSE, TRIPOD_PHASE,
)
from nmfly.proprioception import encode_proprioception


class TestMotorCommand:
    def test_defaults(self):
        cmd = MotorCommand()
        assert cmd.forward_velocity == 0.0
        assert cmd.freeze == 0.0

    def test_custom(self):
        cmd = MotorCommand(forward_velocity=20.0, angular_velocity=1.5)
        assert cmd.forward_velocity == 20.0
        assert cmd.angular_velocity == 1.5


class TestSensoryReading:
    def test_defaults(self):
        r = SensoryReading()
        assert r.channel == 0
        assert r.activation == 0.0

    def test_custom(self):
        r = SensoryReading(channel=42, activation=0.75, raw_value=1.2)
        assert r.channel == 42
        assert r.activation == 0.75
        assert r.raw_value == 1.2


class TestGaitState:
    def test_initial_state(self):
        g = GaitState()
        assert g.phase == 0.0
        assert g.fwd_vel == 0.0
        assert not g.frozen

    def test_phase_advances_with_forward(self):
        g = GaitState()
        cmd = MotorCommand(forward_velocity=20.0)
        g.update(cmd, 0.01)
        assert g.phase > 0.0

    def test_freeze_stops_phase(self):
        g = GaitState()
        g.phase = 1.0
        cmd = MotorCommand(freeze=1.0)
        g.update(cmd, 0.01)
        for _ in range(100):
            g.update(cmd, 0.01)
        phase_before = g.phase
        g.update(cmd, 0.01)
        assert g.phase == phase_before

    def test_reverse_reverses_phase(self):
        g = GaitState()
        cmd = MotorCommand(forward_velocity=-10.0)
        for _ in range(50):
            g.update(cmd, 0.01)
        assert g.fwd_vel < 0


class TestJointTargets:
    def test_shape(self):
        g = GaitState()
        targets = compute_joint_targets(g)
        assert targets.shape == (N_LEGS, N_JOINTS_PER_LEG)

    def test_frozen_returns_rest(self):
        g = GaitState()
        g.frozen = True
        targets = compute_joint_targets(g)
        expected = np.tile(REST_POSE, (N_LEGS, 1))
        np.testing.assert_array_almost_equal(targets, expected)

    def test_tripod_alternation(self):
        """Legs in opposite tripod groups should have different targets."""
        g = GaitState()
        g.fwd_vel = 20.0
        g.phase = 0.5
        targets = compute_joint_targets(g)

        l1_swing = targets[0, 1]
        r1_swing = targets[1, 1]
        assert l1_swing != r1_swing

    def test_turning_creates_asymmetry(self):
        g = GaitState()
        g.fwd_vel = 20.0
        g.ang_vel = 2.0
        g.phase = 0.5
        targets = compute_joint_targets(g)

        l1_amp = abs(targets[0, 1] - REST_POSE[1])
        r1_amp = abs(targets[1, 1] - REST_POSE[1])
        assert r1_amp > l1_amp


class TestFlight:
    def test_wing_angles(self):
        f = FlightState()
        cmd = MotorCommand()
        f.update(cmd, 0.001)
        left, right = compute_wing_angles(f)
        assert abs(left - right) < 0.01

    def test_turning_asymmetry(self):
        f = FlightState()
        cmd = MotorCommand(angular_velocity=3.0)
        for _ in range(100):
            f.update(cmd, 0.001)
        left, right = compute_wing_angles(f)
        assert left != right


class TestProprioception:
    def test_encoding_count(self):
        angles = np.zeros(42)
        readings = encode_proprioception(joint_angles=angles)
        assert len(readings) == 42

    def test_full_encoding(self):
        readings = encode_proprioception(
            joint_angles=np.zeros(42),
            joint_velocities=np.zeros(42),
            contact_forces=np.ones(6),
            body_velocity=np.array([10.0, 0.0, 0.5]),
            wing_angles=(1.0, -1.0),
        )
        assert len(readings) == 95

    def test_activation_range(self):
        readings = encode_proprioception(
            joint_angles=np.random.randn(42) * 2,
            joint_velocities=np.random.randn(42) * 5,
        )
        for r in readings:
            assert 0.0 <= r.activation <= 1.0

    def test_returns_sensory_readings(self):
        readings = encode_proprioception(joint_angles=np.zeros(42))
        assert all(isinstance(r, SensoryReading) for r in readings)


class TestFwmcAdapter:
    def test_bio_reading_roundtrip(self):
        from nmfly.adapters.fwmc import BioReading
        orig = BioReading(neuron_idx=42, spike_prob=0.75,
                          calcium_raw=0.3, voltage_mv=-65.0)
        packed = orig.pack()
        restored = BioReading.unpack(packed)
        assert restored.neuron_idx == 42
        assert abs(restored.spike_prob - 0.75) < 1e-5
        assert abs(restored.voltage_mv - (-65.0)) < 1e-3

    def test_from_sensory(self):
        from nmfly.adapters.fwmc import BioReading
        sr = SensoryReading(channel=10, activation=0.6, raw_value=1.5)
        bio = BioReading.from_sensory(sr)
        assert bio.neuron_idx == 10
        assert abs(bio.spike_prob - 0.6) < 1e-5


class TestFlyModel:
    def test_stick_fly_xml(self):
        from nmfly.fly_model import stick_fly_mjcf
        xml = stick_fly_mjcf()
        assert "nmfly_stick_fly" in xml
        assert "LF_ThC_yaw" in xml
        assert "wing_L_stroke" in xml
        assert xml.count("</body>") > 10
