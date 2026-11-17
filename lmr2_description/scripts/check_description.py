#!/usr/bin/env python3
"""Static contract checks on expanded Xacro (requires Jazzy xacro on PATH)."""
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

package = Path(__file__).resolve().parents[1]
xacro = package / 'urdf' / 'robot.urdf.xacro'
result = subprocess.run(['xacro', str(xacro)], text=True, capture_output=True)

if result.returncode:
    sys.exit('XACRO FAILED:\n' + result.stderr)
root = ET.fromstring(result.stdout)

assert root.attrib['name'] == 'lmr2'

links = {item.get('name') for item in root.findall('link')}
joints = {item.get('name'): item.get('type') for item in root.findall('joint')}

assert {'base_link', 'left_wheel_link', 'right_wheel_link', 'caster_wheel_link', 'left_camera_link', 'right_camera_link', 'lidar_link', 'imu_link'} <= links
assert joints['left_wheel_joint'] == joints['right_wheel_joint'] == 'continuous'

plugins = {x.get('name') for x in root.findall('.//plugin')}

assert 'gz::sim::systems::DiffDrive' in plugins
assert 'gz::sim::systems::JointStatePublisher' in plugins
assert 'gz::sim::systems::Sensors' not in plugins  # world-level plugin, not model

for filename in ('visuals/wheel.dae', 'visuals/lmr2_base.dae', 'collisions/lmr2_base.stl'):
    assert (package / 'meshes' / filename).is_file()

print('PASS: generated URDF, frame/joint contract, Gazebo plugins and meshes')
