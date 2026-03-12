"""WASD-controlled NeuroMechFly via flygym.

Launches a MuJoCo viewer with a full biomechanical Drosophila model
and lets you walk it around with keyboard controls:

    W/S  = forward/backward
    A/D  = turn left/right
    Space = speed boost

Uses flygym's preprogrammed tripod gait CPG with velocity modulation.
"""

import logging
import math
import time
import threading

import numpy as np
from flygym import Fly, SingleFlySimulation
from flygym.preprogrammed import get_cpg_biases

log = logging.getLogger("nmfly.flygym")

# CPG parameters
COUPLING_STRENGTH = 10.0
BASE_AMP = 1.0
FREQ_HZ = 12.0  # base stepping frequency


class KeyState:
    """Keyboard state tracked via key_callback (press only in MuJoCo 3.2).

    Since launch_passive's key_callback only fires on press (not release),
    we use a toggle/timer approach: each press sets a flag that decays
    after a short window unless re-pressed.
    """
    def __init__(self):
        self.forward = 0.0
        self.turn = 0.0
        self.boost = False
        self._lock = threading.Lock()
        self._pressed: dict[int, float] = {}  # key -> timestamp

    def on_key(self, keycode: int):
        """Called by MuJoCo viewer on key press."""
        import time
        with self._lock:
            self._pressed[keycode] = time.monotonic()

    def poll(self):
        """Read current key state. Keys expire after 0.15s without re-press."""
        import glfw
        import time
        now = time.monotonic()
        hold_window = 0.15  # seconds

        with self._lock:
            self.forward = 0.0
            self.turn = 0.0
            self.boost = False

            def held(key):
                return key in self._pressed and (now - self._pressed[key]) < hold_window

            if held(glfw.KEY_W):
                self.forward += 1.0
            if held(glfw.KEY_S):
                self.forward -= 0.5
            if held(glfw.KEY_A):
                self.turn += 1.0
            if held(glfw.KEY_D):
                self.turn -= 1.0
            if held(glfw.KEY_SPACE):
                self.boost = True


class WasdController:
    """Tripod CPG controller driven by keyboard state.

    Uses Kuramoto-style coupled oscillators with flygym's coupling matrix.
    """

    def __init__(self, n_joints: int, timestep: float):
        self.timestep = timestep
        self.n_joints = n_joints

        # CPG state: one oscillator per leg (6 legs)
        # Initialize with tripod phase pattern: legs 0,2,4 at 0, legs 1,3,5 at pi
        self.phases = np.array([0.0, math.pi, 0.0, math.pi, 0.0, math.pi])

        # (6,6) coupling matrix: biases[i][j] = desired phase diff between i and j
        self.coupling = get_cpg_biases("tripod")

        # Keyboard state
        self.forward = 0.0
        self.turn = 0.0
        self.boost = False

    def update_keys(self, forward: float, turn: float, boost: bool):
        self.forward = forward
        self.turn = turn
        self.boost = boost

    def get_action(self) -> np.ndarray:
        """Advance CPG and compute joint action array."""
        speed = abs(self.forward)
        if speed < 0.01:
            return np.zeros(self.n_joints)

        if self.boost:
            speed *= 2.0

        freq = FREQ_HZ * max(0.3, speed)
        amp = BASE_AMP * max(0.2, speed)

        # Kuramoto oscillator update with coupling matrix
        dphase = np.full(6, 2.0 * math.pi * freq)
        for i in range(6):
            for j in range(6):
                if i != j:
                    dphase[i] += COUPLING_STRENGTH * np.sin(
                        self.phases[j] - self.phases[i] - self.coupling[i, j]
                    )
        self.phases += dphase * self.timestep
        self.phases %= (2.0 * math.pi)

        action = np.zeros(self.n_joints)
        joints_per_leg = self.n_joints // 6

        for leg_idx in range(6):
            phase = self.phases[leg_idx]
            swing = math.sin(phase)
            lift = max(0.0, swing)

            is_left = leg_idx < 3
            turn_bias = 1.0
            if self.turn > 0.1:
                turn_bias = 0.5 if is_left else 1.5
            elif self.turn < -0.1:
                turn_bias = 1.5 if is_left else 0.5

            direction = 1.0 if self.forward >= 0 else -1.0
            a = amp * turn_bias * direction

            base = leg_idx * joints_per_leg
            # Coxa (main swing)
            action[base + 0] = a * swing * 0.5
            # Coxa roll
            if joints_per_leg > 1:
                action[base + 1] = a * swing * 0.1
            # Coxa yaw
            if joints_per_leg > 2:
                action[base + 2] = a * swing * 0.15
            # Femur pitch (lift)
            if joints_per_leg > 3:
                action[base + 3] = a * lift * 0.4
            # Femur roll
            if joints_per_leg > 4:
                action[base + 4] = 0.0
            # Tibia
            if joints_per_leg > 5:
                action[base + 5] = -a * lift * 0.2
            # Tarsus
            if joints_per_leg > 6:
                action[base + 6] = a * lift * 0.1

        return action


def run_wasd(timestep: float = 0.0001, run_time: float = 0.0):
    """Launch flygym with WASD keyboard control.

    Args:
        timestep: Physics timestep in seconds.
        run_time: Max run time in seconds (0 = forever).
    """
    from mujoco.viewer import launch_passive

    log.info("Setting up NeuroMechFly...")

    fly = Fly(
        init_pose="stretch",
        control="position",
        enable_adhesion=True,
    )

    sim = SingleFlySimulation(
        fly=fly,
        timestep=timestep,
    )

    obs, info = sim.reset()
    log.info("Simulation reset. nq=%d nv=%d nu=%d",
             sim.physics.model.ptr.nq,
             sim.physics.model.ptr.nv,
             sim.physics.model.ptr.nu)

    n_joints = len(fly.actuated_joints)
    controller = WasdController(n_joints, timestep)

    # Get raw mujoco model/data for viewer
    mj_model = sim.physics.model.ptr
    mj_data = sim.physics.data.ptr

    # Keyboard state with viewer callback
    keys = KeyState()

    viewer = launch_passive(mj_model, mj_data, key_callback=keys.on_key)

    log.info("NeuroMechFly ready. WASD to walk, Space for speed boost.")
    log.info("Close the viewer window to exit.")

    step_count = 0
    sim_steps_per_viewer_sync = 10  # sync viewer every N steps for performance

    try:
        while viewer.is_running():
            keys.poll()
            controller.update_keys(keys.forward, keys.turn, keys.boost)

            joint_action = controller.get_action()
            action = {
                "joints": joint_action,
                "adhesion": np.ones(6),  # keep tarsi sticky
            }

            obs, reward, terminated, truncated, info = sim.step(action)

            step_count += 1

            if step_count % sim_steps_per_viewer_sync == 0:
                viewer.sync()

            if terminated or truncated:
                obs, info = sim.reset()

            if run_time > 0 and step_count * timestep >= run_time:
                break

    except KeyboardInterrupt:
        log.info("Stopped by user")
    finally:
        viewer.close()
        log.info("Exited after %d steps (%.1fs sim time)",
                 step_count, step_count * timestep)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    run_wasd()
