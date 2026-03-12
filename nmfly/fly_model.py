"""Generate MuJoCo MJCF models for insect bodies.

Provides a minimal stick-figure Drosophila model for testing and
prototyping without requiring NeuroMechFly.
"""


def find_nmf_model() -> str | None:
    """Try to locate NeuroMechFly's MJCF model on disk."""
    import importlib.util
    import pathlib

    spec = importlib.util.find_spec("neuromechfly")
    if spec is None:
        return None

    pkg_path = pathlib.Path(spec.origin).parent if spec.origin else None
    if pkg_path is None:
        return None

    candidates = [
        pkg_path / "data" / "mjcf" / "neuromechfly.xml",
        pkg_path / "data" / "design" / "neuromechfly.xml",
        pkg_path / "data" / "neuromechfly_v2.xml",
    ]

    for p in candidates:
        if p.exists():
            return str(p)

    return None


def stick_fly_mjcf() -> str:
    """Generate a minimal 6-legged stick fly MJCF for testing.

    Returns a complete MuJoCo XML string with:
    - Thorax with freejoint
    - Head and abdomen
    - 6 articulated legs (4 DOF each: coxa yaw/pitch, femur, tibia)
    - 2 wings with stroke joints
    - Position actuators for all joints
    - Floor plane
    """
    legs_xml = ""
    for name, side, y_off in [
        ("LF", -1, 0.02), ("RF", 1, 0.02),
        ("LM", -1, 0.0),  ("RM", 1, 0.0),
        ("LH", -1, -0.02), ("RH", 1, -0.02),
    ]:
        cx, cy = side * 0.008, y_off
        legs_xml += f"""
        <body name="{name}_coxa" pos="{cx} {cy} 0">
          <joint name="{name}_ThC_yaw" type="hinge" axis="0 0 1" range="-0.5 0.5"/>
          <joint name="{name}_ThC_pitch" type="hinge" axis="1 0 0" range="-1 1"/>
          <geom type="capsule" size="0.001" fromto="0 0 0 {side*0.01} 0 -0.005" rgba="0.7 0.5 0.3 1"/>
          <body name="{name}_femur" pos="{side*0.01} 0 -0.005">
            <joint name="{name}_CTr_pitch" type="hinge" axis="1 0 0" range="-1.5 0.5"/>
            <geom type="capsule" size="0.0008" fromto="0 0 0 0 0 -0.012" rgba="0.7 0.5 0.3 1"/>
            <body name="{name}_tibia" pos="0 0 -0.012">
              <joint name="{name}_FTi_pitch" type="hinge" axis="1 0 0" range="-0.5 2.0"/>
              <geom type="capsule" size="0.0006" fromto="0 0 0 0 0 -0.01" rgba="0.6 0.4 0.2 1"/>
              <body name="{name}_tarsus" pos="0 0 -0.01">
                <joint name="{name}_TiTa_pitch" type="hinge" axis="1 0 0" range="-0.5 1.0"/>
                <geom type="sphere" size="0.0005" rgba="0.5 0.3 0.1 1"/>
              </body>
            </body>
          </body>
        </body>"""

    leg_actuators = "".join(
        f'\n    <position name="{name}_{jnt}" joint="{name}_{jnt}" kp="0.001" ctrlrange="-2 2"/>'
        for name in ["LF", "RF", "LM", "RM", "LH", "RH"]
        for jnt in ["ThC_yaw", "ThC_pitch", "CTr_pitch", "FTi_pitch"]
    )

    return f"""<?xml version="1.0"?>
<mujoco model="nmfly_stick_fly">
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <default>
    <joint damping="0.0001" armature="0.00001"/>
    <geom contype="1" conaffinity="1" friction="1.0 0.005 0.001"/>
  </default>

  <worldbody>
    <light pos="0 0 0.5" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="0.5 0.5 0.01" rgba="0.2 0.3 0.2 1"/>

    <body name="thorax" pos="0 0 0.025">
      <freejoint name="root"/>
      <geom type="ellipsoid" size="0.004 0.012 0.003" rgba="0.8 0.6 0.3 1"/>

      <!-- Head -->
      <body name="head" pos="0 0.014 0.001">
        <geom type="sphere" size="0.004" rgba="0.7 0.5 0.2 1"/>
      </body>

      <!-- Abdomen -->
      <body name="abdomen" pos="0 -0.015 -0.001">
        <geom type="ellipsoid" size="0.003 0.01 0.0025" rgba="0.6 0.4 0.2 1"/>
      </body>

      <!-- Wings -->
      <body name="wing_L" pos="-0.004 0.005 0.003">
        <joint name="wing_L_stroke" type="hinge" axis="0 1 0" range="-2.5 2.5"/>
        <geom type="ellipsoid" size="0.015 0.004 0.0003" rgba="0.9 0.9 1.0 0.3"/>
      </body>
      <body name="wing_R" pos="0.004 0.005 0.003">
        <joint name="wing_R_stroke" type="hinge" axis="0 1 0" range="-2.5 2.5"/>
        <geom type="ellipsoid" size="0.015 0.004 0.0003" rgba="0.9 0.9 1.0 0.3"/>
      </body>

      <!-- Legs -->
      {legs_xml}
    </body>
  </worldbody>

  <actuator>
    <!-- Leg actuators (position-controlled) -->
    {leg_actuators}

    <!-- Wing actuators -->
    <position name="wing_L_act" joint="wing_L_stroke" kp="0.01" ctrlrange="-2.5 2.5"/>
    <position name="wing_R_act" joint="wing_R_stroke" kp="0.01" ctrlrange="-2.5 2.5"/>
  </actuator>
</mujoco>"""
