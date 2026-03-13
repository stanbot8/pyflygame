"""MuJoCo bridge loop for insect body simulation.

Generic, callback-based design: any controller can drive the fly.
No dependency on any specific spiking simulator or protocol.

Usage with a custom controller::

    from nmfly import MotorCommand
    from nmfly.bridge import Bridge, BridgeConfig

    def my_controller(sensory, info):
        # Your logic here (spiking network, RL policy, PID, etc.)
        return MotorCommand(forward_velocity=15.0)

    bridge = Bridge(BridgeConfig(render=True))
    bridge.run(my_controller)
"""

import logging
import time
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .types import MotorCommand, SensoryReading
from .gait import (
    GaitState, FlightState,
    compute_joint_targets, compute_wing_angles,
    N_JOINTS_TOTAL, N_LEGS,
)
from .proprioception import (
    ProprioceptionConfig, encode_proprioception,
)
from .fly_model import find_nmf_model, stick_fly_mjcf

log = logging.getLogger("nmfly")


class Controller(Protocol):
    """Interface for anything that produces MotorCommands from sensory input."""
    def __call__(
        self,
        sensory: list[SensoryReading],
        info: dict,
    ) -> MotorCommand: ...


def default_controller(sensory: list[SensoryReading], info: dict) -> MotorCommand:
    """Walk forward at moderate speed. Useful for testing."""
    return MotorCommand(forward_velocity=15.0)


@dataclass
class BridgeConfig:
    """Bridge configuration."""
    dt: float = 0.002          # MuJoCo timestep (500 Hz)
    render: bool = False       # open MuJoCo viewer
    model_path: str = ""       # path to MJCF model (auto-detect if empty)
    max_steps: int = 0         # 0 = run forever


class Bridge:
    """Closed-loop MuJoCo bridge for insect body simulation.

    Decoupled from any network protocol or spiking simulator.
    Feed it a controller callback and it runs the loop.
    """

    def __init__(self, config: BridgeConfig | None = None):
        self.config = config or BridgeConfig()
        self.gait = GaitState()
        self.flight = FlightState()
        self.proprio_config = ProprioceptionConfig()
        self.mj_model = None
        self.mj_data = None
        self.viewer = None
        self._step_count = 0

    def setup(self) -> None:
        """Load the MuJoCo model."""
        import mujoco

        model_path = self.config.model_path

        if not model_path:
            model_path = find_nmf_model()

        if not model_path:
            log.warning("NeuroMechFly not found. Using built-in stick fly.")
            xml = stick_fly_mjcf()
            self.mj_model = mujoco.MjModel.from_xml_string(xml)
        else:
            log.info("Loading model: %s", model_path)
            self.mj_model = mujoco.MjModel.from_xml_path(model_path)

        self.mj_data = mujoco.MjData(self.mj_model)
        self.mj_model.opt.timestep = self.config.dt

        if self.config.render:
            try:
                from mujoco.viewer import launch_passive
                self.viewer = launch_passive(
                    self.mj_model, self.mj_data)
            except Exception as e:
                log.warning("Could not open viewer: %s", e)
                self.viewer = None

    def step(self, cmd: MotorCommand) -> dict:
        """Run one simulation step.

        Args:
            cmd: Motor command from controller.

        Returns:
            Dict with step info (joint_angles, contacts, body_vel, etc.)
        """
        import mujoco

        # Update gait CPG
        self.gait.update(cmd, self.config.dt)

        # Compute joint targets
        targets = compute_joint_targets(self.gait)  # (6, 7)

        # Apply joint targets to MuJoCo actuators
        flat_targets = targets.flatten()
        n_ctrl = min(len(flat_targets), self.mj_model.nu)
        self.mj_data.ctrl[:n_ctrl] = flat_targets[:n_ctrl]

        # Step physics
        mujoco.mj_step(self.mj_model, self.mj_data)

        # Read proprioception
        joint_angles = np.zeros(N_JOINTS_TOTAL)
        joint_velocities = np.zeros(N_JOINTS_TOTAL)
        n_qpos = min(N_JOINTS_TOTAL, self.mj_model.nq)
        joint_angles[:n_qpos] = self.mj_data.qpos[:n_qpos]
        n_qvel = min(N_JOINTS_TOTAL, self.mj_model.nv)
        joint_velocities[:n_qvel] = self.mj_data.qvel[:n_qvel]

        # Contact forces
        contacts = np.zeros(N_LEGS)
        for i in range(self.mj_data.ncon):
            contact = self.mj_data.contact[i]
            for leg_idx in range(N_LEGS):
                geom_start = leg_idx * 4
                geom_end = geom_start + 4
                if (geom_start <= contact.geom1 < geom_end or
                    geom_start <= contact.geom2 < geom_end):
                    contacts[leg_idx] = 1.0

        # Body velocity
        body_vel = np.zeros(3)
        if self.mj_model.nv >= 6:
            body_vel[0] = self.mj_data.qvel[0] * 1000  # m/s -> mm/s
            body_vel[1] = self.mj_data.qvel[1] * 1000
            body_vel[2] = self.mj_data.qvel[5]  # yaw rate

        # Update viewer
        if self.viewer is not None:
            self.viewer.sync()

        self._step_count += 1

        return {
            "joint_angles": joint_angles,
            "joint_velocities": joint_velocities,
            "contacts": contacts,
            "body_velocity": body_vel,
            "targets": targets,
            "gait_phase": self.gait.phase,
            "frozen": self.gait.frozen,
            "step": self._step_count,
        }

    def run(self, controller: Controller | None = None) -> None:
        """Main bridge loop.

        Args:
            controller: Callable(sensory, info) -> MotorCommand.
                        If None, uses default_controller (walk forward).
        """
        if controller is None:
            controller = default_controller

        self.setup()

        log.info("Bridge running (dt=%.3fs, render=%s)",
                 self.config.dt, self.config.render)

        try:
            sensory: list[SensoryReading] = []
            info: dict = {}

            while True:
                if self.config.max_steps > 0 and \
                   self._step_count >= self.config.max_steps:
                    break

                # Get motor command from controller
                cmd = controller(sensory, info)

                # Step physics
                info = self.step(cmd)

                # Encode proprioception for next cycle
                sensory = encode_proprioception(
                    joint_angles=info["joint_angles"],
                    joint_velocities=info["joint_velocities"],
                    contact_forces=info["contacts"],
                    body_velocity=info["body_velocity"],
                    config=self.proprio_config,
                )

                # Rate limit if no viewer
                if self.viewer is None:
                    time.sleep(self.config.dt)

                # Check viewer closed
                if self.viewer is not None and not self.viewer.is_running():
                    break

        except KeyboardInterrupt:
            log.info("Bridge stopped by user")
        finally:
            if self.viewer:
                self.viewer.close()
            log.info("Bridge exited after %d steps", self._step_count)
