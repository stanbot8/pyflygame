"""Bridge between FWMC spiking brain and flygym body.

Pipeline:
    WASD (FWMC viewer) -> spiking brain -> TCP MotorCommand
        -> HybridTurningController -> MuJoCo physics

All keyboard input is in the FWMC viewer. This module just receives
motor commands over TCP and makes the fly walk.
"""

import logging
import math

import numpy as np
from flygym import Fly
from flygym.examples.locomotion import HybridTurningController
from flygym.preprogrammed import default_leg_sensor_placements

from .types import MotorCommand, SensoryReading
from .proprioception import encode_proprioception, ProprioceptionConfig

log = logging.getLogger("nmfly.bridge")


def motor_to_dn_drive(cmd: MotorCommand) -> np.ndarray:
    """Convert MotorCommand to descending neuron drive [left, right].

    HybridTurningController expects action = np.array([left, right])
    where each value is the amplitude of leg drive on that side.
    Positive = forward, negative = backward.
    """
    # Brain outputs 0-30 mm/s forward, -6..+6 rad/s angular.
    # Map to [0,1.4] drive range for HybridTurningController.
    fwd = np.clip(cmd.forward_velocity / 8.0, -1.0, 1.4)
    turn = np.clip(cmd.angular_velocity / 2.0, -1.0, 1.0)

    # Differential drive: turn by slowing one side
    left = fwd + turn * 0.5
    right = fwd - turn * 0.5

    # Clamp to HybridTurningController's stable range
    return np.clip(np.array([left, right]), -0.4, 1.4)


def run_bridge(host: str = "127.0.0.1", port: int = 9100,
               timestep: float = 0.0002, standalone: bool = False):
    """Run the flygym body, driven by FWMC brain over TCP.

    Args:
        host: FWMC TCP host.
        port: FWMC TCP port.
        timestep: flygym physics timestep.
        standalone: If True, walk forward without FWMC connection.
    """
    import mujoco
    from mujoco.viewer import launch_passive

    log.info("Setting up flygym body...")

    fly = Fly(
        init_pose="stretch",
        control="position",
        enable_adhesion=True,
        contact_sensor_placements=default_leg_sensor_placements,
    )
    sim = HybridTurningController(fly=fly, timestep=timestep)
    obs, info = sim.reset()

    log.info("flygym ready (HybridTurningController). Tripod gait active.")

    proprio_config = ProprioceptionConfig()

    # Connect to FWMC brain
    client = None
    if not standalone:
        try:
            from .adapters.fwmc import FwmcClient
            client = FwmcClient(host, port)
            client.connect()
            log.info("Connected to FWMC brain at %s:%d", host, port)
        except Exception as e:
            log.warning("Cannot connect to FWMC: %s. Running standalone.", e)
            client = None

    # Launch viewer (no UI panels, just the fly)
    mj_model = sim.physics.model.ptr
    mj_data = sim.physics.data.ptr
    # Disable keyboard input on the flygym viewer so it doesn't steal
    # controls from the FWMC brain viewer (MuJoCo's A/S/etc. conflict).
    def _noop_key(*_args):
        pass

    viewer = launch_passive(
        mj_model, mj_data,
        show_left_ui=False, show_right_ui=False,
        key_callback=_noop_key,
    )

    # Camera follows the fly's thorax (flygym namespaces bodies as "0/...")
    thorax_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "0/Thorax")
    if thorax_id < 0:
        thorax_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "Thorax")

    # Third-person camera: behind and slightly above the fly
    if thorax_id >= 0:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = thorax_id
        viewer.cam.azimuth = 0.0      # behind the fly
        viewer.cam.elevation = -25.0  # slightly above
        viewer.cam.distance = 15.0    # close third-person

    log.info("Viewer open. Controlled by FWMC brain over TCP.")
    if standalone:
        log.info("Standalone mode: walking forward")

    step_count = 0
    # Full controller step every N physics steps; fast mj_step in between
    exchange_every = 2   # full controller steps between TCP exchanges
    sync_every = 10      # full controller steps between viewer syncs

    # Default: stand still until brain sends commands
    dn_drive = np.array([0.0, 0.0])
    ctrl_count = 0

    # Get raw MuJoCo handles for fast sub-stepping
    mj_model = sim.physics.model.ptr
    mj_data = sim.physics.data.ptr

    # Motor command batch queue for replaying between exchanges
    drive_queue: list[np.ndarray] = []
    drive_idx = 0

    try:
        while viewer.is_running():
            # TCP exchange at reduced rate
            if ctrl_count % exchange_every == 0:
                if client:
                    try:
                        joint_angles = obs["joints"][0]
                        sensory = encode_proprioception(
                            joint_angles=joint_angles,
                            config=proprio_config,
                        )
                        stims, motor = client.exchange(sensory)
                        if motor:
                            motor.freeze = 0.0
                            # Convert batch to drive commands for replay
                            batch = getattr(client, "last_motor_batch", [])
                            if len(batch) > 1:
                                drive_queue = [motor_to_dn_drive(m)
                                               for m in batch]
                                drive_idx = 0
                            else:
                                dn_drive = motor_to_dn_drive(motor)
                                drive_queue = []
                    except (ConnectionError, TimeoutError) as e:
                        log.warning("Lost FWMC connection: %s", e)
                        client = None

            # Replay batch: step through queued drive commands
            if drive_queue and drive_idx < len(drive_queue):
                dn_drive = drive_queue[drive_idx]
                drive_idx += 1

            # One full controller step (CPG + stumble detection + obs)
            try:
                obs, reward, terminated, truncated, info = sim.step(dn_drive)
            except Exception:
                log.warning("Physics unstable, resetting")
                obs, info = sim.reset()
                dn_drive = np.array([0.0, 0.0])
                drive_queue = []
                ctrl_count = 0
                continue

            step_count += 1
            ctrl_count += 1

            if ctrl_count % sync_every == 0:
                # Re-apply camera tracking each sync (viewer resets it)
                if thorax_id >= 0:
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                    viewer.cam.trackbodyid = thorax_id
                viewer.sync()

            if terminated or truncated:
                obs, info = sim.reset()

    except KeyboardInterrupt:
        log.info("Stopped by user")
    finally:
        viewer.close()
        if client:
            client.close()
        log.info("Exited after %d steps (%.1fs sim time)",
                 step_count, step_count * timestep)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    run_bridge(standalone=True)
