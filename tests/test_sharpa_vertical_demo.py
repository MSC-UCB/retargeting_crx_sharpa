"""Finite vertical input, calibrated world-up mapping and the actual solver path."""

import subprocess
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from retargeting_apps.sharpa_vertical_demo import build_flow, main, parse_args
from teleoperation.inputs.vertical_hand import VerticalBimanualInput


def test_stroke_is_twenty_cm_above_start_with_smooth_turnarounds():
    source = VerticalBimanualInput()
    times = np.linspace(0., 24., 24001)
    z = np.array([source.displacement(t) for t in times])
    assert z.min() == 0. and z.max() == pytest.approx(.20)
    for t in (0., 8., 16., 24., 30.):
        assert source.displacement(t) == 0.
    for t in (4., 12., 20.):
        assert source.displacement(t) == pytest.approx(.20)
    for t in (4., 8., 12., 16., 20.):
        h = 1e-3
        assert abs((source.displacement(t+h)-source.displacement(t-h))/(2*h)) < 1e-6
        assert abs((source.displacement(t+h)-2*source.displacement(t)
                    + source.displacement(t-h))/h**2) < 1e-4


def test_motion_waits_for_calibration_and_requires_acknowledged_endpoint_hold():
    source = VerticalBimanualInput(cycles=1, hold_time=.2)
    clock = [None]
    source.elapsed = lambda: clock[0]
    source.open()
    first = source.read()
    assert first.source_index == first.right.source_index == 0
    np.testing.assert_array_equal(first.left.wrist_pose_sensor, np.eye(4))
    clock[0] = 4.
    top = source.read()
    assert top.left.wrist_pose_sensor[2, 3] == pytest.approx(.2)
    np.testing.assert_array_equal(top.left.keypoints_wrist, first.left.keypoints_wrist)
    assert top.left.raw.received_monotonic_ns == top.right.raw.received_monotonic_ns
    clock[0] = 8.
    source.read()
    source.acknowledge(reached=False)
    assert not source.done
    for t in (8.1, 8.2, 8.31):
        clock[0] = t
        source.read()
        source.acknowledge(reached=True)
    assert source.done
    with pytest.raises(StopIteration):
        source.read()


def test_missing_final_results_or_feedback_timeout_cannot_report_success():
    source = VerticalBimanualInput(cycles=1)
    source.elapsed = lambda: 18.01
    with pytest.raises(RuntimeError, match='settle'):
        source.read()
    assert not source.done


@pytest.mark.parametrize('options', [
    ['--stroke-m', '0'], ['--stroke-m', 'nan'], ['--period', '-1'], ['--cycles', '0'],
    ['--hold-time', '10'], ['--publish-hz', 'inf'], ['--crx-namespace', '/'],
])
def test_invalid_arguments_fail_before_opening_anything(options):
    with pytest.raises(SystemExit):
        parse_args(options)


def test_cli_help_is_device_free():
    result = subprocess.run([sys.executable, 'scripts/run_crx_sharpa_vertical_demo.py', '--help'],
                            capture_output=True, text=True, check=True)
    assert '--stroke-m' in result.stdout and '--backend' in result.stdout


def test_mapping_keeps_orientation_and_moves_both_sides_in_world_z():
    flow, config = build_flow(parse_args(['--no-viewer']))
    source = flow.source
    clock = [None]
    source.elapsed = lambda: clock[0]
    sample = source.read()
    # Calibration may start from measured joints different from the configured pose.
    seed = flow.initial_qpos.copy()
    seed[[5, 33]] = [.2, -.3]
    try:
        assert flow.pipeline.initialize(sample, seed[:28], seed[28:])
        mappers = (flow.pipeline.left_mapper, flow.pipeline.right_mapper)
        first = [mapper.map(hand).wrist_pose_world.copy()
                 for mapper, hand in zip(mappers, (sample.left, sample.right))]
        clock[0] = 4.
        top = source.read()
        for side, mapper, hand, origin in zip(('left', 'right'), mappers,
                                               (top.left, top.right), first):
            pose = mapper.map(hand).wrist_pose_world
            placement = Rotation.from_euler('xyz', config['bimanual'][side]['placement']['rpy']).as_matrix()
            np.testing.assert_allclose(placement @ (pose[:3, 3]-origin[:3, 3]), [0, 0, .2], atol=1e-12)
            np.testing.assert_allclose(pose[:3, :3], origin[:3, :3], atol=1e-12)
    finally:
        flow.close()


def test_tracking_interruption_aborts_instead_of_restarting_motion():
    flow, _ = build_flow(parse_args(['--no-viewer']))
    try:
        flow.tracking_paused = True
        with pytest.raises(RuntimeError, match='interrupted'):
            flow.source.read()
    finally:
        flow.close()


def test_complete_twenty_cm_cycle_with_real_process_solvers():
    flow, _ = build_flow(parse_args(['--no-viewer', '--cycles', '1']))
    source, clock = flow.source, [0.]
    source.elapsed = lambda: clock[0]
    positions, errors = [], []
    try:
        flow.start_solver()
        source.open()
        for t in np.arange(0., 9.1, .05):
            if source.done:
                break
            clock[0] = float(t)
            result = flow.step(source.read())
            assert result is not None
            poses, deviation = [], []
            for r, obs, q in zip((flow.pipeline.left_retargeter, flow.pipeline.right_retargeter),
                                (result.left_observation, result.right_observation),
                                (result.left_qpos, result.right_qpos)):
                pose = r.robot_adaptor.robot_model.get_frame_pose(
                    r.robot_config.wrist_frame_name, qpos=r.robot_adaptor.forward_qpos(q)).copy()
                poses.append(pose[:3, 3])
                deviation.append(np.linalg.norm(pose[:3, 3]-obs.wrist_pose_world[:3, 3]))
                assert np.all(q >= r.optimizer.joint_limits[:, 0]-1e-7)
                assert np.all(q <= r.optimizer.joint_limits[:, 1]+1e-7)
            positions.append(poses)
            errors.append(deviation)
        assert source.done and flow.stale_count == 0 and flow.command_count >= 180
        assert all(w['maxtime'] == .030 for w in flow.pair_solver.ready)
        # The existing smoothing/soft objectives make actual FK approximate.
        assert np.max(errors) < .015
        np.testing.assert_allclose(np.ptp(np.asarray(positions)[:, :, 2], axis=0), [.20, .20], atol=.01)
        print(f'vertical FK max error m={np.max(errors, axis=0)}, '
              f'z travel m={np.ptp(np.asarray(positions)[:, :, 2], axis=0)}')
    finally:
        flow.close()
    assert len(flow.pair_solver.cleanup) == 2
    assert all(not w['alive'] and w['exitcode'] == 0 for w in flow.pair_solver.cleanup)


def test_main_preserves_demo_observer_when_attaching_viewer(monkeypatch):
    from retargeting_apps import sharpa_vertical_demo as app
    from retargeting_apps.visualization.execution import manager

    events = []
    flow = NS(source=NS(done=False), observer=lambda r: events.append('monitor'),
              close=lambda: events.append('flow close'))
    def run():
        flow.observer(None)
        flow.source.done = True
    flow.run = run
    monkeypatch.setattr(app, 'build_flow', lambda args: (flow, {}))
    def create(config, actual):
        actual.observer = lambda r: events.append('viewer')
        return NS(close=lambda: events.append('viewer close'))
    monkeypatch.setattr(manager, 'create_optional_execution_visualizer', create)
    assert main([]) == 0
    assert events == ['monitor', 'viewer', 'flow close', 'viewer close']
