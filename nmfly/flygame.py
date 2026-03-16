"""Wrapper for the flygame C++ simulator (nmfly-sim).

Launches the C++ executable as a subprocess and communicates via the
FWMC TCP protocol. pyflygame acts as the brain (TCP server), flygame
connects as the body (TCP client).

Usage::

    from nmfly.flygame import FlygameSim

    with FlygameSim() as sim:
        while sim.running:
            state = sim.step()
            # state.joint_angles, state.contacts, state.body_velocity, ...
            sim.send_motor(forward_velocity=15.0, angular_velocity=0.0)

Or with a controller callback::

    from nmfly.flygame import FlygameSim
    from nmfly.types import MotorCommand

    def my_controller(state):
        return MotorCommand(forward_velocity=15.0)

    FlygameSim.run(my_controller)
"""

import logging
import socket
import struct
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .types import MotorCommand
from .adapters.fwmc import (
    HEADER_FMT, HEADER_SIZE,
    MSG_HELLO_CLIENT, MSG_HELLO_SERVER,
    MSG_BIO_READINGS, MSG_BODY_STATE, MSG_MOTOR,
    MOTOR_CMD_FMT,
    BIO_READING_SIZE, BODY_STATE_SIZE,
    PROTOCOL_VERSION,
    BioReading, BodyState,
)

log = logging.getLogger("nmfly.flygame")

# Search paths for nmfly-sim executable (relative to pyflygame repo root).
_EXE_SEARCH = [
    "../flygame/build/Release/nmfly-sim.exe",
    "../flygame/build/Debug/nmfly-sim.exe",
    "../flygame/build/RelWithDebInfo/nmfly-sim.exe",
    "../flygame/build/nmfly-sim.exe",
    "../flygame/build/Release/nmfly-sim",
    "../flygame/build/nmfly-sim",
]


def find_flygame_exe() -> str | None:
    """Locate the nmfly-sim executable relative to this package."""
    pkg_dir = Path(__file__).resolve().parent.parent  # pyflygame repo root
    for rel in _EXE_SEARCH:
        p = pkg_dir / rel
        if p.is_file():
            return str(p.resolve())

    # Also check PATH.
    import shutil as _shutil
    return _shutil.which("nmfly-sim") or _shutil.which("nmfly-sim.exe")


def _find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@dataclass
class FlygameConfig:
    """Configuration for launching flygame."""
    exe_path: str = ""          # path to nmfly-sim; auto-detect if empty
    headless: bool = False      # --headless flag
    model: str = ""             # --model path (default: bundled NMF)
    duration: float = 0.0       # --duration seconds (0 = unlimited)
    extra_args: list = None     # additional CLI args  # type: ignore[assignment]

    def __post_init__(self):
        if self.extra_args is None:
            self.extra_args = []


class FlygameSim:
    """Wraps the flygame C++ simulator.

    Acts as a TCP server (brain side). Launches nmfly-sim which connects
    back as the body client. Provides a Python API to receive body state
    and send motor commands each frame.
    """

    def __init__(self, config: FlygameConfig | None = None):
        self.config = config or FlygameConfig()
        self._server_sock: socket.socket | None = None
        self._client_sock: socket.socket | None = None
        self._proc: subprocess.Popen | None = None
        self._port: int = 0
        self._running = False
        self._frame_count: int = 0

    @property
    def running(self) -> bool:
        if self._proc and self._proc.poll() is not None:
            self._running = False
        return self._running

    def start(self) -> None:
        """Start the TCP server and launch nmfly-sim."""
        exe = self.config.exe_path or find_flygame_exe()
        if not exe:
            raise FileNotFoundError(
                "Cannot find nmfly-sim executable. Build flygame first, "
                "or set FlygameConfig.exe_path."
            )

        # Start TCP server.
        self._port = _find_free_port()
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind(("127.0.0.1", self._port))
        self._server_sock.listen(1)
        self._server_sock.settimeout(15.0)
        log.info("Listening on 127.0.0.1:%d", self._port)

        # Build command line.
        cmd = [exe, "--brain", f"127.0.0.1:{self._port}"]
        if self.config.headless:
            cmd.append("--headless")
        if self.config.model:
            cmd.extend(["--model", self.config.model])
        if self.config.duration > 0:
            cmd.extend(["--duration", str(self.config.duration)])
        cmd.extend(self.config.extra_args)

        log.info("Launching: %s", " ".join(cmd))
        self._proc = subprocess.Popen(cmd)

        # Wait for flygame to connect.
        try:
            self._client_sock, addr = self._server_sock.accept()
            self._client_sock.setsockopt(
                socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            log.info("flygame connected from %s:%d", *addr)
        except socket.timeout:
            self.stop()
            raise TimeoutError("flygame did not connect within 15 seconds")

        # HELLO handshake (flygame sends HELLO as client, we reply as server).
        msg_type, payload = self._recv_msg()
        if msg_type == MSG_HELLO_CLIENT and len(payload) >= 4:
            client_ver = struct.unpack("<I", payload[:4])[0]
            log.info("flygame protocol v%d", client_ver)
        # Reply with server HELLO.
        self._send_msg(MSG_HELLO_SERVER, struct.pack("<I", PROTOCOL_VERSION))

        self._running = True

    def stop(self) -> None:
        """Shut down flygame and close sockets."""
        self._running = False
        if self._client_sock:
            try:
                self._client_sock.close()
            except OSError:
                pass
            self._client_sock = None
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None
        if self._proc:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()

    # -- I/O ----------------------------------------------------------------

    def _send_msg(self, msg_type: int, payload: bytes) -> None:
        assert self._client_sock is not None
        header = struct.pack(HEADER_FMT, msg_type, len(payload))
        self._client_sock.sendall(header + payload)

    def _recv_exact(self, n: int) -> bytes:
        assert self._client_sock is not None
        buf = bytearray()
        while len(buf) < n:
            chunk = self._client_sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("flygame disconnected")
            buf.extend(chunk)
        return bytes(buf)

    def _recv_msg(self) -> tuple[int, bytes]:
        header = self._recv_exact(HEADER_SIZE)
        msg_type, payload_size = struct.unpack(HEADER_FMT, header)
        if payload_size > 1 << 20:
            raise ValueError(f"Payload too large: {payload_size}")
        payload = self._recv_exact(payload_size) if payload_size > 0 else b""
        return msg_type, payload

    # -- public API ---------------------------------------------------------

    def recv_state(self) -> BodyState:
        """Receive the current body state from flygame.

        Handles both BODY_STATE (full state) and BIO_READINGS (sensory-only)
        messages. When BIO_READINGS are received, converts them into a
        partial BodyState with available sensor data.
        """
        msg_type, payload = self._recv_msg()

        if msg_type == MSG_BODY_STATE and len(payload) >= BODY_STATE_SIZE:
            return BodyState.unpack(payload)

        if msg_type == MSG_BIO_READINGS:
            # flygame sends BioReadings in --brain TCP mode.
            # Convert to BodyState with sensor activations in joint_angles.
            n = len(payload) // BIO_READING_SIZE
            readings = [BioReading.unpack(payload, i * BIO_READING_SIZE)
                        for i in range(n)]
            state = BodyState()
            state.step = self._frame_count
            self._frame_count += 1
            # Store readings as joint angles (channels 0-41 map to joints).
            for r in readings:
                if r.neuron_idx < 42:
                    state.joint_angles[r.neuron_idx] = r.spike_prob
                elif r.neuron_idx < 84:
                    state.joint_velocities[r.neuron_idx - 42] = r.spike_prob
                elif r.neuron_idx < 90:
                    state.contacts[r.neuron_idx - 84] = r.spike_prob
            return state

        raise ValueError(f"Expected BODY_STATE or BIO_READINGS, got 0x{msg_type:02x}")

    def send_motor(self, forward_velocity: float = 0.0,
                   angular_velocity: float = 0.0,
                   approach_drive: float = 0.0,
                   freeze: float = 0.0) -> None:
        """Send a motor command to flygame."""
        payload = struct.pack(MOTOR_CMD_FMT,
                              forward_velocity, angular_velocity,
                              approach_drive, freeze)
        self._send_msg(MSG_MOTOR, payload)

    def send_motor_command(self, cmd: MotorCommand) -> None:
        """Send a MotorCommand dataclass to flygame."""
        self.send_motor(
            cmd.forward_velocity, cmd.angular_velocity,
            cmd.approach_drive, cmd.freeze,
        )

    def step(self, cmd: MotorCommand | None = None) -> BodyState:
        """One exchange cycle: receive state, send motor command.

        If cmd is None, sends a default walk-forward command.
        Returns the body state received from flygame.
        """
        state = self.recv_state()
        if cmd is None:
            cmd = MotorCommand(forward_velocity=15.0)
        self.send_motor_command(cmd)
        return state

    # -- convenience --------------------------------------------------------

    @staticmethod
    def run(controller: Callable[[BodyState], MotorCommand] | None = None,
            config: FlygameConfig | None = None) -> None:
        """Launch flygame and run a controller loop until exit.

        Args:
            controller: Callable that takes BodyState, returns MotorCommand.
                        If None, walks forward at 15 mm/s.
            config: Optional FlygameConfig.
        """
        if controller is None:
            controller = lambda state: MotorCommand(forward_velocity=15.0)

        with FlygameSim(config) as sim:
            try:
                while sim.running:
                    state = sim.recv_state()
                    cmd = controller(state)
                    sim.send_motor_command(cmd)
            except (ConnectionError, KeyboardInterrupt):
                pass
            log.info("Controller loop ended")


def launch(config: FlygameConfig | None = None) -> None:
    """Launch flygame standalone with its embedded brain.

    Runs nmfly-sim directly (no TCP bridge). The embedded mechabrain
    auto-enables when a .brain spec file is found. WASD injects currents
    into descending neurons. Close the viewer window to exit.
    """
    config = config or FlygameConfig()
    exe = config.exe_path or find_flygame_exe()
    if not exe:
        raise FileNotFoundError(
            "Cannot find nmfly-sim executable. Build flygame first, "
            "or set FlygameConfig.exe_path."
        )

    cmd = [exe]
    if config.headless:
        cmd.append("--headless")
    if config.model:
        cmd.extend(["--model", config.model])
    if config.duration > 0:
        cmd.extend(["--duration", str(config.duration)])
    cmd.extend(config.extra_args)

    log.info("Launching standalone: %s", " ".join(cmd))
    proc = subprocess.Popen(cmd)

    try:
        proc.wait()
        log.info("flygame exited (code %d)", proc.returncode)
    except KeyboardInterrupt:
        log.info("Interrupted")
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
