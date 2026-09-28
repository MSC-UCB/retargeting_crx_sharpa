"""Exercise the shared lifecycle with actual Sharpa solvers, without sensors."""

from dataclasses import replace
from types import SimpleNamespace as NS
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from retargeting_apps.sharpa_teleop import build_flow
from teleoperation.inputs.quest3 import (
    HandFrame, HandSample, JointPose, JOINT_NAMES, decode_quest3_sample,
)
from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput
from teleoperation.types import BimanualSensorHandSample


def arguments(**overrides):
    values = dict(config=None, backend='preview', duration=0., command_hz=20.,
                  adb=None, serial=None, viewer=False, viewer_port=9219,
                  publish_hz=100., interpolation_horizon_ms=None,
                  startup_timeout=3., crx_namespace='crx5ia', sharpa_namespace='sharpa')
    values.update(overrides)
    return NS(**values)


def webxr_hand_sample(side, wrist_rotation, curl=0.):
    """Independent WebXR skeleton: fingers along -Z, flexion toward palm -Y."""
    rotation = Rotation.from_euler('xyz', wrist_rotation, degrees=True)
    origin = np.array([.3, 1.2, -.4])
    positions = {'wrist': np.zeros(3)}
    segments = ('metacarpal', 'proximal', 'intermediate', 'distal', 'tip')
    for finger, lateral in zip(('thumb', 'index', 'middle', 'ring', 'little'),
                               (.05, .03, .01, -.01, -.03)):
        names = tuple(s for s in segments if finger != 'thumb' or s != 'intermediate')
        for i, segment in enumerate(names):
            distance = .025 * (i + 1)
            positions[f'{finger}_{segment}'] = np.array([
                -lateral * (1 if side == 'right' else -1),
                -distance * np.sin(curl), -.035 - distance * np.cos(curl),
            ])
    joints = {
        name: JointPose(tuple(origin + rotation.apply(positions[name])),
                        tuple(rotation.as_quat()), .008)
        for name in JOINT_NAMES
    }
    frame = HandFrame(sequence=0, source_sequence=0, sender_timestamp_ns=0,
                      received_monotonic_ns=time.monotonic_ns(), reference_space='local',
                      hands={side: HandSample(side=side, tracked=True, joints=joints)})
    return decode_quest3_sample(frame, hand_side=side)


@pytest.mark.parametrize('with_arms', [False, True])
@pytest.mark.parametrize('wrist_rotation', [(0., 0., 0.), (20., -35., 80.)])
def test_quest_decoded_finger_axes_match_sharpa_fk(with_arms, wrist_rotation):
    flow, _ = build_flow(arguments(), with_arms=with_arms, source=SyntheticBimanualInput())
    for side, mapper, joints in zip(('left', 'right'),
                                    (flow.pipeline.left_mapper, flow.pipeline.right_mapper),
                                    flow.robot_slices):
        qpos = flow.initial_qpos[joints].copy()
        assert mapper.initialize(webxr_hand_sample(side, wrist_rotation), qpos)
        # Exercise both longitudinal direction and the palm/flexion direction.
        for curl in (0., .25):
            observation = mapper.map(webxr_hand_sample(side, wrist_rotation, curl))
            pose = observation.wrist_pose_world
            human = observation.keypoints_wrist @ pose[:3, :3].T + pose[:3, 3]
            robot_qpos = qpos.copy()
            # The profile owns actuator order; do not assume Pinocchio's order.
            retargeter = getattr(flow.pipeline, f'{side}_retargeter')
            for finger in ('index', 'middle', 'ring', 'pinky'):
                index = list(retargeter.robot_config.actuated_joints).index(f'{side}_{finger}_MCP_FE')
                robot_qpos[index] = curl
            model_qpos = mapper.robot_adaptor.forward_qpos(robot_qpos)
            def position(frame):
                return mapper.robot_model.get_frame_pose(frame, qpos=model_qpos)[:3, 3]
            def unit(vector):
                return vector / np.linalg.norm(vector)
            for finger, tip in zip(('index', 'middle', 'ring', 'pinky'), (8, 12, 16, 20)):
                robot_direction = position(f'{side}_{finger}_fingertip') - position(f'{side}_{finger}_DP')
                np.testing.assert_allclose(unit(human[tip] - human[tip - 1]),
                                           unit(robot_direction), atol=1e-4)
            # Check handedness across the palm as well as finger direction.
            robot_lateral = position(f'{side}_index_MCP_VL') - position(f'{side}_pinky_MCP_VL')
            assert unit(human[5] - human[17]) @ unit(robot_lateral) > .98


@pytest.mark.parametrize('with_arms', [False, True])
def test_real_solver_smoothing_without_speed_cap_and_finite_input(with_arms, monkeypatch, request):
    source = SyntheticBimanualInput(frames=4, max_age_s=10.)
    flow, _ = build_flow(arguments(), with_arms=with_arms, source=source)
    request.addfinalizer(flow.close)
    flow.start_solver()
    assert len(flow.pair_solver.ready) == 2
    assert all(worker['maxtime'] == pytest.approx(.030) for worker in flow.pair_solver.ready)
    assert all(worker['torch_threads'] == 1 for worker in flow.pair_solver.ready)
    def no_parent_solve(*args, **kwargs):
        pytest.fail('Sharpa must solve in the worker processes')
    for retargeter in (flow.pipeline.left_retargeter, flow.pipeline.right_retargeter):
        monkeypatch.setattr(retargeter, 'solve', no_parent_solve)
    source.open()
    assert flow.command_limiters is None
    assert flow.arm_output_filters is not None and flow.hand_output_filters is not None
    previous = flow.initial_qpos.copy()
    raw_results = []
    original_step = flow.pipeline.step

    def record_raw(sample):
        result = original_step(sample)
        raw_results.append(result.qpos.copy())
        return result

    monkeypatch.setattr(flow.pipeline, 'step', record_raw)
    assert flow.robot_dofs == ((28, 28) if with_arms else (22, 22))
    for _ in range(4):
        result = flow.step(source.read())
        assert result is not None and np.isfinite(result.qpos).all()
        for i, r in enumerate((flow.pipeline.left_retargeter, flow.pipeline.right_retargeter)):
            assert r.optimizer.opt._opt.get_maxtime() == pytest.approx(.030)
            assert r.human_fingertip_indices.tolist() == [4, 8, 12, 16, 20]
            joints = flow.robot_slices[i]
            qpos = result.qpos[joints]
            alpha = np.full(flow.robot_dofs[i], .3)
            alpha[:flow.arm_dofs[i]] = .5
            expected = alpha * raw_results[-1][joints] + (1. - alpha) * previous[joints]
            np.testing.assert_allclose(qpos, expected, atol=1e-12)
            assert np.all(qpos >= r.optimizer.joint_limits[:, 0] - 1e-7)
            assert np.all(qpos <= r.optimizer.joint_limits[:, 1] + 1e-7)
            np.testing.assert_allclose(r.previous_qpos, qpos)
        previous = result.qpos.copy()
    assert flow.command_count == 4
    with pytest.raises(StopIteration):
        source.read()


def test_hand_only_mapper_ignores_global_wrist_motion():
    source = SyntheticBimanualInput()
    flow, _ = build_flow(arguments(), with_arms=False, source=source)
    sample = source.read()
    pipeline = flow.pipeline
    assert pipeline.initialize(sample, flow.initial_qpos[:22], flow.initial_qpos[22:])
    for mapper, hand in ((pipeline.left_mapper, sample.left), (pipeline.right_mapper, sample.right)):
        before = mapper.map(hand)
        pose = np.eye(4)
        pose[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        pose[:3, 3] = [10, -3, 8]
        after = mapper.map(replace(hand, wrist_pose_sensor=pose))
        np.testing.assert_array_equal(after.wrist_pose_world, before.wrist_pose_world)
        np.testing.assert_array_equal(after.keypoints_wrist, before.keypoints_wrist)


def test_combined_initial_wrist_alignment_preserves_measured_orientation():
    source = SyntheticBimanualInput()
    flow, _ = build_flow(arguments(), with_arms=True, source=source)
    sample = source.read()
    assert flow.pipeline.initialize(sample, flow.initial_qpos[:28], flow.initial_qpos[28:])
    for mapper, hand in ((flow.pipeline.left_mapper, sample.left), (flow.pipeline.right_mapper, sample.right)):
        first = mapper.map(hand)
        np.testing.assert_allclose(first.wrist_pose_world, mapper._robot_initial_wrist_pose, atol=1e-12)
        pose = hand.wrist_pose_sensor.copy()
        pose[0, 3] += .01
        moved = mapper.map(replace(hand, wrist_pose_sensor=pose))
        assert np.linalg.norm(moved.wrist_pose_world[:3, 3] - first.wrist_pose_world[:3, 3]) == pytest.approx(.01)


def test_measured_56_dim_seed_pause_and_recovery(monkeypatch, request):
    source = SyntheticBimanualInput(max_age_s=10.)
    flow, _ = build_flow(arguments(), with_arms=True, source=source)
    request.addfinalizer(flow.close)
    flow.start_solver()
    measured = flow.initial_qpos.copy()
    measured[[0, 28]] += .01
    calls = []
    seeds = []
    original_step = flow.pipeline.step

    def record_seed(sample):
        seeds.append(np.concatenate((flow.pipeline.left_retargeter.previous_qpos,
                                     flow.pipeline.right_retargeter.previous_qpos)))
        return original_step(sample)

    monkeypatch.setattr(flow.pipeline, 'step', record_seed)
    backend = NS(get_joint_pos=lambda: measured.copy(), execute=lambda q: calls.append(q.copy()),
                 pause_tracking=lambda: None, resume_tracking=lambda: False, close=lambda: None)
    flow.backend_factory = lambda: backend
    assert flow.step(source.read()) is None
    result = flow.step(source.read())
    assert len(calls) == 1
    np.testing.assert_array_equal(calls[-1], result.qpos)
    np.testing.assert_array_equal(seeds[0], measured)
    backend.assert_tracking = lambda: False
    assert flow.step(source.read()) is None
    assert flow.tracking_paused and len(calls) == 1
    bad = source.read()
    flow.step(BimanualSensorHandSample(replace(bad.left, keypoints_wrist=None), bad.right))
    assert flow.tracking_paused
    flow._recovery_started = time.monotonic() - 1
    assert flow.step(source.read()) is None
    assert flow.tracking_paused and len(calls) == 1
    backend.resume_tracking = lambda: True
    backend.assert_tracking = lambda: True
    flow._recovery_started = time.monotonic() - 1
    flow.step(source.read())
    assert not flow.tracking_paused and not flow.pipeline.initialized
    result = flow.step(source.read())
    assert len(calls) == 2
    np.testing.assert_array_equal(seeds[-1], measured)
    np.testing.assert_array_equal(calls[-1], result.qpos)


def test_cli_help_never_opens_devices():
    import subprocess
    import sys
    for script in ('run_sharpa_joint_teleop.py', 'run_crx_sharpa_joint_teleop.py'):
        result = subprocess.run([sys.executable, f'scripts/{script}', '--help'], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        assert '--backend' in result.stdout and '--synthetic-frames' in result.stdout
        assert '--publish-hz' in result.stdout and '--interpolation-horizon-ms' in result.stdout


@pytest.mark.parametrize('with_arms', [False, True])
def test_ros_composition_uses_100hz_independently_of_solver_rate(with_arms, monkeypatch):
    from retargeting_ros import sharpa_joint

    monkeypatch.setattr(sharpa_joint, 'SharpaJointBackend', lambda **kwargs: NS(**kwargs))
    flow, _ = build_flow(arguments(backend='ros'), with_arms=with_arms,
                         source=SyntheticBimanualInput())
    backend = flow.backend_factory()
    assert backend.publish_hz == 100.
    assert backend.control_period == backend.interpolation_horizon == .05
    assert len(backend.initial_qpos) == (56 if with_arms else 44)


@pytest.mark.parametrize('overrides', [dict(publish_hz=0.), dict(publish_hz=float('nan')),
                                      dict(interpolation_horizon_ms=250.),
                                      dict(interpolation_horizon_ms=float('inf'))])
def test_ros_invalid_publication_settings_rejected_before_open(overrides):
    with pytest.raises(ValueError):
        build_flow(arguments(backend='ros', **overrides), with_arms=True,
                   source=SyntheticBimanualInput())
