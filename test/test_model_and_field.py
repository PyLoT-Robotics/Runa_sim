from pathlib import Path
import os
import xml.etree.ElementTree as ET
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory
from runa_sim.world import prepare_world
from runa_sim.kinematics import tick_to_angle, angle_to_tick


def test_head_conversion():
    for servo_id, ticks in [(1,800),(1,1200),(2,1900),(2,2100)]:
        assert abs(angle_to_tick(servo_id,tick_to_angle(servo_id,ticks))-ticks) <= 1
    assert tick_to_angle(1,1200) == tick_to_angle(2,1900) == 0
    assert tick_to_angle(1,-100) == tick_to_angle(1,0)
    assert tick_to_angle(1,0) <= 1.0471975512
    with pytest.raises(ValueError):
        tick_to_angle(3,0)


def test_simulation_model_has_matching_controllers():
    import yaml
    desc = Path(get_package_share_directory('runa_description'))
    sim = Path(get_package_share_directory('runa_sim'))
    model = ET.fromstring(xacro.process_file(str(desc/'urdf/runa.urdf.xacro'), mappings={
        'sim':'true','controllers_file':str(sim/'config/controllers.yaml')}).toxml())
    joints = {j.get('name') for j in model.findall('joint')}
    configured = yaml.safe_load((sim/'config/controllers.yaml').read_text())
    for name, data in configured.items():
        if name != 'controller_manager':
            assert set(data['ros__parameters']['joints']) <= joints
    hardware = model.find('ros2_control/hardware/plugin')
    assert hardware.text == 'gz_ros2_control/GazeboSimSystem'
    assert '/dev/' not in ET.tostring(model,encoding='unicode')


def test_fork_field_and_asset_validation(tmp_path):
    root = Path(get_package_share_directory('sobits_gazebo_worlds'))
    name, omitted = prepare_world(root/'worlds/rcjo2025_arena.world.xacro', [root/'models'], tmp_path/'world.sdf')
    assert name == 'rcjo2025_arena'
    text = (tmp_path/'world.sdf').read_text()
    assert 'model://rcjo2025_arena/floor' in text
    assert 'model://rcjo2025_arena/frame' in text
    for node in ET.fromstring(text).findall('world/include'):
        assert (root/'models'/node.findtext('uri').removeprefix('model://')).is_dir()
    assert omitted  # External WRS/AWS furniture is explicit, never silently substituted.
    with pytest.raises(FileNotFoundError,match='Full-field assets missing'):
        prepare_world(root/'worlds/rcjo2025_arena.world.xacro', [root/'models'], tmp_path/'full.sdf', True)


def test_lidar_excludes_only_robot_visuals(tmp_path):
    from runa_sim.model import prepare_model
    desc = Path(get_package_share_directory('runa_description'))
    sim = Path(get_package_share_directory('runa_sim'))
    urdf = tmp_path/'robot.urdf'
    urdf.write_text(xacro.process_file(str(desc/'urdf/runa.urdf.xacro'), mappings={
        'sim':'true','controllers_file':str(sim/'config/controllers.yaml')}).toxml())
    prepare_model(urdf, tmp_path/'robot.sdf')
    root = ET.parse(tmp_path/'robot.sdf').getroot()
    assert root.findall('model/link/visual')
    assert all(v.findtext('visibility_flags')=='2' for v in root.findall('model/link/visual'))
    assert root.findtext('.//sensor[@name="hokuyo"]/lidar/visibility_mask')=='1'
    assert root.find('.//sensor[@name="d455"]') is not None
