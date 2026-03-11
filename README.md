# pyflygame

**Python bridge for [flygame](https://github.com/stanbot8/flygame): high-performance [flygym](https://github.com/NeLy-EPFL/flygym)/NeuroMechFly on MuJoCo, with spiking connectome integration.**

pyflygame provides the Python interface for driving a MuJoCo insect body simulation. Use it standalone with MuJoCo's Python bindings, or connect to the **flygame** C++ sim for 7x realtime performance.

## Install

```bash
pip install -e .

# With MuJoCo viewer support
pip install -e ".[mujoco]"

# With flygym high-fidelity model
pip install -e ".[flygym]"
```

## Quick start

```python
from nmfly import MotorCommand
from nmfly.bridge import Bridge, BridgeConfig

# Custom controller: just walk forward
def my_controller(sensory, info):
    return MotorCommand(forward_velocity=15.0)

bridge = Bridge(BridgeConfig(render=True))
bridge.run(my_controller)
```

## With FWMC spiking connectome

```python
from nmfly.bridge import Bridge, BridgeConfig
from nmfly.adapters.fwmc import make_fwmc_controller

bridge = Bridge(BridgeConfig(render=True))
bridge.run(make_fwmc_controller("127.0.0.1", 9100))
```

## With flygym (high-fidelity model)

```bash
pip install flygym
nmfly --backend flygym --host 127.0.0.1 --port 9100
```

## Writing your own adapter

Implement a controller callback: `(sensory, info) -> MotorCommand`

```python
from nmfly import MotorCommand, SensoryReading

def my_spiking_controller(sensory: list[SensoryReading], info: dict) -> MotorCommand:
    activations = [s.activation for s in sensory]
    # ... run your network ...
    return MotorCommand(forward_velocity=output_speed, angular_velocity=output_turn)
```

## Package structure

```
nmfly/                    Python package
  types.py                MotorCommand, SensoryReading (framework-agnostic)
  gait.py                 Tripod gait CPG, flight wing kinematics
  proprioception.py       95-channel sensory encoder
  fly_model.py            Stick-figure Drosophila MJCF generator
  bridge.py               MuJoCo simulation loop (callback-based)
  cli.py                  CLI entry point (nmfly command)
  adapters/
    fwmc.py               Optional FWMC brain connector
  flygym_bridge.py        flygym HybridTurningController bridge
  flygym_wasd.py          WASD keyboard control via flygym
  launcher.py             Launch brain viewer + body sim together
  dockable.py             Window docking manager (Windows)

tests/
  test_nmfly.py           Unit tests
```

## Requirements

- Python 3.10+
- numpy
- Optional: mujoco (pip), flygym

## License

Apache License 2.0. See [LICENSE](LICENSE).

## Related

- [flygame](https://github.com/stanbot8/flygame): C++ body sim (7x realtime, header-only)
- [flygym](https://github.com/NeLy-EPFL/flygym): Original Python NeuroMechFly framework
- [FWMC](https://github.com/stanbot8/fwmc): Spiking connectome simulator
