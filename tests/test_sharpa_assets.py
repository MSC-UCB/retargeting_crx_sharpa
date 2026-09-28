"""Local assets, model configuration and five-finger kinematics contracts."""

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from retargeting.config import load_retargeting_profile_config, load_robot_config
from retargeting.core.kinematics import RobotAdaptor, RobotPinocchio
from teleoperation.backends.sharpa_contract import sharpa_names


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('combined', [False, True])
def test_local_model_and_profile(side, combined):
    name = f'crx5ia_sharpa_wave_{side}' if combined else f'sharpa_wave_{side}'
    robot = load_robot_config(f'configs/robots/{name}.yaml')
    profile = load_retargeting_profile_config(f'configs/retargeting_profiles/vector_wrist_joint_{name}.yaml')
    profile.validate(robot)
    model = RobotPinocchio(robot.robot_file_path, 'urdf')
    adaptor = RobotAdaptor(model, list(robot.actuated_joints))
    size, arm = (28, 6) if combined else (22, 0)
    assert model.dof == size and profile.retargeting.arm_dof == arm
    assert robot.human_hand_scale == 1.2
    assert tuple(robot.actuated_joints[arm:]) == sharpa_names(side)
    assert len(profile.target.link_pairs) == 15
    assert len(robot.benchmark.fingertips) == 5
    assert all(tip.robot_direction_axis == 'z' for tip in robot.benchmark.fingertips)
    qpos = np.asarray(robot.initial_qpos)
    bounds = adaptor.backward_qpos(model.joint_limits)
    assert np.all(qpos >= bounds[:, 0]) and np.all(qpos <= bounds[:, 1])
    xml = ET.parse(robot.robot_file_path)
    for mesh in xml.iter('mesh'):
        filename = mesh.get('filename')
        assert not Path(filename).is_absolute() and '://' not in filename
        path = (Path(robot.robot_file_path).parent / filename).resolve()
        assert path.is_relative_to(ROOT / 'assets') and path.is_file()
    model_q = adaptor.forward_qpos(qpos)
    for finger in ('thumb', 'index', 'middle', 'ring', 'pinky'):
        tip = model.get_frame_pose(f'{side}_{finger}_fingertip', qpos=model_q)
        base = model.get_frame_pose(f'{side}_{finger}_DP', qpos=model_q)
        direction = tip[:3, 3] - base[:3, 3]
        direction /= np.linalg.norm(direction)
        # The native fingertip local +Z follows the distal segment.
        np.testing.assert_allclose(tip[:3, 2], direction, atol=3e-5)
    if combined:
        expected_arm_degrees = ([0, 30, -30, 0, 60, 0] if side == 'left'
                                else [-90, -30, 210, 0, -60, 0])
        np.testing.assert_allclose(qpos[:6], np.radians(expected_arm_degrees), atol=1e-12)
        np.testing.assert_array_equal(qpos[6:], np.zeros(22))
        mounts = yaml.safe_load((ROOT / 'configs/sharpa_mounts.yaml').read_text())
        mount = xml.find(f"joint[@name='{side}_sharpa_mount']")
        assert mount.find('parent').get('link') == mounts[side]['parent']
        assert not any('leap' in j.get('name') for j in xml.findall('joint'))
        np.testing.assert_allclose(np.fromstring(mount.find('origin').get('xyz'), sep=' '), mounts[side]['xyz'])
        np.testing.assert_allclose(np.fromstring(mount.find('origin').get('rpy'), sep=' '), mounts[side]['rpy'])
        flange = model.get_frame_pose('fanuc_flange', qpos=model_q)
        hand_flange = model.get_frame_pose(f'{side}_hand_flange', qpos=model_q)
        relative_rotation = flange[:3, :3].T @ hand_flange[:3, :3]
        expected_rotation = np.diag([-1., -1., 1.]) if side == 'left' else np.eye(3)
        np.testing.assert_allclose(relative_rotation, expected_rotation, atol=1e-12)
    else:
        assert profile.objective.weights.world_thumb == profile.objective.weights.wrist_rotation == 0


@pytest.mark.parametrize('side', ['left', 'right'])
def test_original_bundle_checksums_and_no_external_symlinks(side):
    bundle = ROOT / f'assets/robots/sharpa_wave_{side}'
    for line in (bundle / 'SHA256SUMS.txt').read_text().splitlines():
        digest, path = line.split(maxsplit=1)
        assert hashlib.sha256((bundle / path).read_bytes()).hexdigest() == digest
    assert not any(path.is_symlink() for path in bundle.rglob('*'))


@pytest.mark.parametrize('combined', [False, True])
def test_all_visual_meshes_load_without_ros_packages(combined):
    yourdfpy = pytest.importorskip('yourdfpy')
    for side in ('left', 'right'):
        name = f'crx5ia_sharpa_wave_{side}' if combined else f'sharpa_wave_{side}'
        robot = load_robot_config(f'configs/robots/{name}.yaml')
        model = yourdfpy.URDF.load(robot.robot_file_path, load_meshes=True)
        model.update_cfg(dict(zip(robot.actuated_joints, robot.initial_qpos)))
        assert len(model.scene.geometry) >= 25
