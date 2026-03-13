# pyflygame

**Python wrapper for [flygame](https://github.com/stanbot8/flygame): high-performance [flygym](https://github.com/NeLy-EPFL/flygym)/NeuroMechFly on MuJoCo, with spiking connectome integration.**

pyflygame wraps the flygame C++ simulator (`nmfly-sim`) and exposes it as a Python API. The C++ sim runs at 7x realtime; pyflygame launches it, connects via TCP, and lets you write controllers in Python.

Also includes a pure-Python fallback (slower, no C++ build needed) for prototyping.

## Install

```bash
pip install -e .

# With MuJoCo Python fallback
pip install -e ".[mujoco]"

# With flygym high-fidelity model
pip install -e ".[flygym]"
```

## Quick start (C++ backend)

Build [flygame](https://github.com/stanbot8/flygame) first, then:

```python
from nmfly import FlygameSim, MotorCommand

# Launch nmfly-sim and control it from Python
with FlygameSim() as sim:
    while sim.running:
        state = sim.step(MotorCommand(forward_velocity=15.0))
        print(f"step {state.step}, pos {state.position}")
```

Or with a controller callback:

```python
from nmfly import FlygameSim, MotorCommand
from nmfly.flygame import FlygameConfig
from nmfly.adapters.fwmc import BodyState

def my_controller(state: BodyState) -> MotorCommand:
    # state has: joint_angles, contacts, body_velocity, position, heading, ...
    return MotorCommand(forward_velocity=15.0)

FlygameSim.run(my_controller)
```

## With FWMC spiking connectome

```python
from nmfly.flygame import FlygameSim, FlygameConfig

# flygame has an embedded brain — just launch it
sim = FlygameSim(FlygameConfig(headless=False))
sim.start()
```

## Pure-Python fallback

No C++ build needed — uses MuJoCo Python bindings directly (slower):

```python
from nmfly import MotorCommand
from nmfly.bridge import Bridge, BridgeConfig

def my_controller(sensory, info):
    return MotorCommand(forward_velocity=15.0)

bridge = Bridge(BridgeConfig(render=True))
bridge.run(my_controller)
```

## CLI

```bash
# Launch C++ flygame (default)
nmfly

# Launch with options
nmfly --exe /path/to/nmfly-sim --headless

# Pure-Python fallback
nmfly --backend builtin --standalone

# flygym backend
nmfly --backend flygym --host 127.0.0.1 --port 9100
```

## Package structure

```
nmfly/                    Python package
  flygame.py              C++ wrapper (FlygameSim — launches and controls nmfly-sim)
  types.py                MotorCommand, SensoryReading (framework-agnostic)
  gait.py                 Tripod gait CPG, flight wing kinematics
  proprioception.py       95-channel sensory encoder
  fly_model.py            Stick-figure Drosophila MJCF generator (fallback)
  bridge.py               Pure-Python MuJoCo simulation loop (fallback)
  cli.py                  CLI entry point (nmfly command)
  launcher.py             Launch nmfly-sim or brain viewer
  adapters/
    fwmc.py               FWMC TCP protocol (binary wire format)
  flygym_bridge.py        flygym HybridTurningController bridge
  flygym_wasd.py          WASD keyboard control via flygym
  dockable.py             Window docking manager (Windows)

tests/
  test_nmfly.py           Unit tests
```

## Requirements

- Python 3.10+
- numpy
- [flygame](https://github.com/stanbot8/flygame) C++ build (for FlygameSim)
- Optional: mujoco (pip, for pure-Python fallback), flygym

## License

MIT License. See [LICENSE](LICENSE).

## Related

- [flygame](https://github.com/stanbot8/flygame): C++ body sim (7x realtime)
- [flygym](https://github.com/NeLy-EPFL/flygym): Original Python NeuroMechFly framework
- [FWMC](https://github.com/stanbot8/fwmc): Spiking connectome simulator
