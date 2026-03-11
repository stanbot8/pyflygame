"""FWMC TCP bridge adapter.

Connects nmfly to the FWMC spiking connectome simulator via its binary
TCP protocol (port 9100).

Wire format: [uint32_le type] [uint32_le payload_size] [payload...]

All integers are little-endian. All floats are IEEE 754 single precision.

Connection flow:
  1. Body sim connects to brain sim on TCP port (default 9100).
  2. Body sim sends HELLO with protocol version (uint32).
  3. Brain sim replies HELLO with its protocol version (uint32).
  4. Main loop: body sends BIO_READINGS or BODY_STATE,
     brain replies with MOTOR (binary) or STIM_COMMANDS.

Protocol v1 adds binary MotorCommand (MSG_MOTOR 0x84) and
BodyState (MSG_BODY_STATE 0x04). Legacy JSON motor via MSG_STATUS
(0x82) is still accepted from older servers.
"""

import json
import logging
import socket
import struct
from dataclasses import dataclass, field

from ..types import MotorCommand, SensoryReading

log = logging.getLogger("nmfly.fwmc")

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

PROTOCOL_VERSION = 1
DEFAULT_PORT = 9100

# Message types (body -> brain)
MSG_HELLO_CLIENT  = 0x00
MSG_BIO_READINGS  = 0x01
MSG_CONFIG        = 0x02
MSG_PING          = 0x03
MSG_BODY_STATE    = 0x04

# Message types (brain -> body)
MSG_HELLO_SERVER  = 0x80
MSG_STIM_COMMANDS = 0x81
MSG_STATUS        = 0x82  # JSON motor command (legacy, v0)
MSG_PONG          = 0x83
MSG_MOTOR         = 0x84  # Binary motor command (v1+)
MSG_MOTOR_BATCH   = 0x85  # Binary motor command batch (v1+)

# Wire formats (little-endian, packed)
HEADER_FMT       = "<II"
HEADER_SIZE      = struct.calcsize(HEADER_FMT)

BIO_READING_FMT  = "<Ifff"     # neuron_idx, spike_prob, calcium_raw, voltage_mv
BIO_READING_SIZE = struct.calcsize(BIO_READING_FMT)

STIM_COMMAND_FMT  = "<IfBf"    # neuron_idx, intensity, excitatory, duration_ms
STIM_COMMAND_SIZE = struct.calcsize(STIM_COMMAND_FMT)

MOTOR_CMD_FMT  = "<ffff"       # forward_vel, angular_vel, approach, freeze
MOTOR_CMD_SIZE = struct.calcsize(MOTOR_CMD_FMT)

# BodyState: 42f joint_angles + 42f joint_velocities + 6f contacts
#          + 3f body_velocity + 3f position + 1f heading + 1f sim_time + 1I step
BODY_STATE_FMT  = "<" + "f" * (42 + 42 + 6 + 3 + 3 + 1 + 1) + "I"
BODY_STATE_SIZE = struct.calcsize(BODY_STATE_FMT)

# Sanity: BodyState should be 396 bytes (matches C++ protocol::BodyState).
assert BODY_STATE_SIZE == 396, f"BodyState size mismatch: {BODY_STATE_SIZE}"


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class BioReading:
    """FWMC binary neural reading (matches C++ protocol::BioReading, 16 bytes)."""
    neuron_idx: int = 0
    spike_prob: float = 0.0
    calcium_raw: float = 0.0
    voltage_mv: float = float("nan")

    def pack(self) -> bytes:
        return struct.pack(BIO_READING_FMT,
                           self.neuron_idx, self.spike_prob,
                           self.calcium_raw, self.voltage_mv)

    @classmethod
    def unpack(cls, data: bytes, offset: int = 0) -> "BioReading":
        vals = struct.unpack_from(BIO_READING_FMT, data, offset)
        return cls(*vals)

    @classmethod
    def from_sensory(cls, reading: SensoryReading) -> "BioReading":
        """Convert generic SensoryReading to FWMC BioReading."""
        return cls(
            neuron_idx=reading.channel,
            spike_prob=reading.activation,
            calcium_raw=reading.activation * 0.8,
        )


@dataclass
class StimCommand:
    """FWMC stimulation command (matches C++ protocol::StimCommand, 13 bytes)."""
    neuron_idx: int = 0
    intensity: float = 0.0
    excitatory: bool = True
    duration_ms: float = 1.0

    @classmethod
    def unpack(cls, data: bytes, offset: int = 0) -> "StimCommand":
        idx, intensity, exc, dur = struct.unpack_from(
            STIM_COMMAND_FMT, data, offset)
        return cls(idx, intensity, bool(exc), dur)


@dataclass
class BodyState:
    """Full body state sent to the brain each frame.

    Matches C++ protocol::BodyState (396 bytes packed).
    """
    joint_angles: list[float] = field(default_factory=lambda: [0.0] * 42)
    joint_velocities: list[float] = field(default_factory=lambda: [0.0] * 42)
    contacts: list[float] = field(default_factory=lambda: [0.0] * 6)
    body_velocity: list[float] = field(default_factory=lambda: [0.0] * 3)
    position: list[float] = field(default_factory=lambda: [0.0] * 3)
    heading: float = 0.0
    sim_time: float = 0.0
    step: int = 0

    def pack(self) -> bytes:
        floats = (
            list(self.joint_angles)
            + list(self.joint_velocities)
            + list(self.contacts)
            + list(self.body_velocity)
            + list(self.position)
            + [self.heading, self.sim_time]
        )
        return struct.pack(BODY_STATE_FMT, *floats, self.step)

    @classmethod
    def unpack(cls, data: bytes, offset: int = 0) -> "BodyState":
        vals = struct.unpack_from(BODY_STATE_FMT, data, offset)
        return cls(
            joint_angles=list(vals[0:42]),
            joint_velocities=list(vals[42:84]),
            contacts=list(vals[84:90]),
            body_velocity=list(vals[90:93]),
            position=list(vals[93:96]),
            heading=vals[96],
            sim_time=vals[97],
            step=vals[98],
        )


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------

class FwmcClient:
    """TCP client that speaks FWMC's binary bridge protocol.

    After ``connect()``, the client has completed the v1 HELLO handshake
    (or detected a legacy v0 server). Use ``exchange()`` for the standard
    send-readings-receive-motor loop, or the lower-level ``send_*`` /
    ``recv_*`` methods for custom flows.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_PORT,
                 timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: socket.socket | None = None
        self.server_version: int = 0  # 0 until HELLO handshake completes

    # -- connection ---------------------------------------------------------

    def connect(self) -> None:
        """Connect and perform the protocol v1 HELLO handshake."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.connect((self.host, self.port))

        # Send HELLO with our protocol version.
        self._send_msg(MSG_HELLO_CLIENT,
                       struct.pack("<I", PROTOCOL_VERSION))

        # Wait for server HELLO.
        msg_type, payload = self._recv_msg()
        if msg_type == MSG_HELLO_SERVER and len(payload) >= 4:
            self.server_version = struct.unpack("<I", payload[:4])[0]
            log.info("FWMC handshake OK, server v%d", self.server_version)
        else:
            # Server didn't reply with HELLO; treat as legacy v0.
            self.server_version = 0
            log.warning("FWMC server did not send HELLO, assuming v0")

    def close(self) -> None:
        if self.sock:
            self.sock.close()
            self.sock = None
        self.server_version = 0

    @property
    def is_v1(self) -> bool:
        """True if server supports protocol v1 (binary motor, body state)."""
        return self.server_version >= 1

    # -- low-level I/O ------------------------------------------------------

    def _send_msg(self, msg_type: int, payload: bytes) -> None:
        assert self.sock is not None
        header = struct.pack(HEADER_FMT, msg_type, len(payload))
        self.sock.sendall(header + payload)

    def _recv_exact(self, n: int) -> bytes:
        assert self.sock is not None
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("FWMC connection closed")
            buf.extend(chunk)
        return bytes(buf)

    def _recv_msg(self) -> tuple[int, bytes]:
        header = self._recv_exact(HEADER_SIZE)
        msg_type, payload_size = struct.unpack(HEADER_FMT, header)
        if payload_size > 1 << 20:
            raise ValueError(f"Payload too large: {payload_size}")
        payload = self._recv_exact(payload_size) if payload_size > 0 else b""
        return msg_type, payload

    # -- sending ------------------------------------------------------------

    def send_readings(self, readings: list[SensoryReading]) -> None:
        """Send sensory readings as FWMC BioReadings."""
        bio = [BioReading.from_sensory(r) for r in readings]
        payload = b"".join(r.pack() for r in bio)
        self._send_msg(MSG_BIO_READINGS, payload)

    def send_bio_readings(self, readings: list[BioReading]) -> None:
        """Send raw BioReadings (low-level)."""
        payload = b"".join(r.pack() for r in readings)
        self._send_msg(MSG_BIO_READINGS, payload)

    def send_body_state(self, state: BodyState) -> None:
        """Send full body state (v1+). Falls back to no-op on v0 servers."""
        if not self.is_v1:
            return
        self._send_msg(MSG_BODY_STATE, state.pack())

    def send_config(self, config: dict) -> None:
        payload = json.dumps(config).encode("utf-8")
        self._send_msg(MSG_CONFIG, payload)

    def send_ping(self) -> None:
        self._send_msg(MSG_PING, b"")

    # -- receiving ----------------------------------------------------------

    def recv_stim_commands(self) -> list[StimCommand]:
        msg_type, payload = self._recv_msg()
        if msg_type != MSG_STIM_COMMANDS:
            return []
        n = len(payload) // STIM_COMMAND_SIZE
        return [StimCommand.unpack(payload, i * STIM_COMMAND_SIZE)
                for i in range(n)]

    def _parse_motor(self, msg_type: int, payload: bytes) -> MotorCommand | None:
        """Parse a motor command from binary, batch, or JSON format.

        For batch messages (kMotorBatch), returns the last command in the
        batch (most recent brain state). The full batch is stored in
        self.last_motor_batch for interpolation if needed.
        """
        if msg_type == MSG_MOTOR_BATCH and len(payload) >= MOTOR_CMD_SIZE:
            n = len(payload) // MOTOR_CMD_SIZE
            self.last_motor_batch = []
            for i in range(n):
                fwd, ang, app, frz = struct.unpack_from(
                    MOTOR_CMD_FMT, payload, i * MOTOR_CMD_SIZE)
                self.last_motor_batch.append(MotorCommand(
                    forward_velocity=fwd, angular_velocity=ang,
                    approach_drive=app, freeze=frz,
                ))
            # Use last command (most current brain state)
            return self.last_motor_batch[-1] if self.last_motor_batch else None

        if msg_type == MSG_MOTOR and len(payload) >= MOTOR_CMD_SIZE:
            fwd, ang, app, frz = struct.unpack_from(MOTOR_CMD_FMT, payload)
            cmd = MotorCommand(
                forward_velocity=fwd, angular_velocity=ang,
                approach_drive=app, freeze=frz,
            )
            self.last_motor_batch = [cmd]
            return cmd

        if msg_type == MSG_STATUS and payload:
            data = json.loads(payload.decode("utf-8"))
            if "motor" in data:
                m = data["motor"]
                cmd = MotorCommand(
                    forward_velocity=m.get("forward_velocity", 0),
                    angular_velocity=m.get("angular_velocity", 0),
                    approach_drive=m.get("approach_drive", 0),
                    freeze=m.get("freeze", 0),
                )
                self.last_motor_batch = [cmd]
                return cmd
        return None

    # -- high-level exchange ------------------------------------------------

    def exchange(self, readings: list[SensoryReading]
                 ) -> tuple[list[StimCommand], MotorCommand | None]:
        """Send sensory readings, receive stim commands and/or motor command.

        Returns (stim_commands, motor_command_or_none).
        Handles both v0 JSON and v1 binary motor responses.
        """
        self.send_readings(readings)

        msg_type, payload = self._recv_msg()

        stims: list[StimCommand] = []
        motor: MotorCommand | None = None

        if msg_type == MSG_STIM_COMMANDS:
            n = len(payload) // STIM_COMMAND_SIZE
            stims = [StimCommand.unpack(payload, i * STIM_COMMAND_SIZE)
                     for i in range(n)]
        else:
            motor = self._parse_motor(msg_type, payload)

        return stims, motor

    def exchange_body_state(self, state: BodyState
                            ) -> tuple[list[StimCommand], MotorCommand | None]:
        """Send body state, receive stim commands and/or motor command.

        Preferred over ``exchange()`` for v1+ servers that want full
        proprioceptive feedback instead of just neural readings.
        """
        self.send_body_state(state)

        msg_type, payload = self._recv_msg()

        stims: list[StimCommand] = []
        motor: MotorCommand | None = None

        if msg_type == MSG_STIM_COMMANDS:
            n = len(payload) // STIM_COMMAND_SIZE
            stims = [StimCommand.unpack(payload, i * STIM_COMMAND_SIZE)
                     for i in range(n)]
        else:
            motor = self._parse_motor(msg_type, payload)

        return stims, motor


# ---------------------------------------------------------------------------
# Convenience
# ---------------------------------------------------------------------------

def make_fwmc_controller(host: str = "127.0.0.1", port: int = DEFAULT_PORT):
    """Create a controller callback that bridges to a running FWMC instance.

    Usage::

        from nmfly.bridge import Bridge, BridgeConfig
        from nmfly.adapters.fwmc import make_fwmc_controller

        bridge = Bridge(BridgeConfig(render=True))
        bridge.run(make_fwmc_controller("127.0.0.1", 9100))
    """
    client = FwmcClient(host, port)
    client.connect()
    log.info("Connected to FWMC at %s:%d (server v%d)",
             host, port, client.server_version)

    def controller(sensory, info):
        stims, motor = client.exchange(sensory)
        return motor or MotorCommand()

    return controller
