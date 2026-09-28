"""Process protocol and lifecycle failures without ROS or hardware."""

from dataclasses import replace
import multiprocessing as mp
import os
from pathlib import Path
import signal
import sys
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from retargeting.core.types import RetargetingHandObservation, RetargetingResult
from teleoperation import parallel_solver
from teleoperation.parallel_solver import BimanualProcessSolver
from teleoperation.bimanual_execution import BimanualExecutionFlow


def fake_retargeters():
    return tuple(NS(robot_config=NS(actuated_joints=[f'{side}_1', f'{side}_2']),
                    profile_config=None, method_config=None, solver_config=NS(params={'maxtime': .030}),
                    optimizer=NS(joint_limits=np.array([[-1., 1.], [-1., 1.]])))
                 for side in ('left', 'right'))


def controlled_worker(connection, config, parent_pid):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    parallel_solver._exit_with_parent(parent_pid)
    behavior = config.get('behavior', 'normal')
    try:
        if behavior == 'startup_error':
            connection.send(('error', 'injected startup failure'))
            return
        connection.send(('ready', dict(joints=list(config['robot_config'].actuated_joints),
                                       thread_env=os.environ.get('OPENBLAS_NUM_THREADS'))))
        while True:
            request = connection.recv()
            if request is None:
                return
            tag, observation, seed = request
            assert observation.raw is None
            if behavior == 'hang':
                while True:
                    time.sleep(.1)
            if behavior == 'crash':
                os._exit(7)
            if behavior == 'error':
                connection.send(('error', 'injected solve failure'))
                return
            if behavior == 'old_tag':
                tag = (tag[0] - 1, tag[1])
            q = seed.copy()
            if behavior == 'nan':
                q[0] = np.nan
            elif behavior == 'bounds':
                q[0] = 2.
            connection.send(('result', (tag, RetargetingResult(q, {}))))
    except (EOFError, BrokenPipeError):
        pass
    finally:
        connection.close()


@pytest.fixture
def make_pair(monkeypatch):
    monkeypatch.setattr(parallel_solver, '_solver_worker', controlled_worker)
    pairs = []

    def create(behavior='normal'):
        pair = BimanualProcessSolver(fake_retargeters(), timeout=.15, close_timeout=.1)
        pair.configs[0]['behavior'] = behavior
        pairs.append(pair)
        return pair

    yield create
    for pair in pairs:
        pair.close()
        assert len(pair.cleanup) == 2
        assert all(not p['alive'] and not Path(f'/proc/{p["pid"]}').exists() for p in pair.cleanup)


def observation():
    return RetargetingHandObservation(np.zeros((21, 3)), np.eye(4), raw=threading.Lock())


def test_explicit_seed_replaces_worker_history_and_raw_stays_in_parent(make_pair, monkeypatch):
    monkeypatch.setenv('OPENBLAS_NUM_THREADS', '4')
    pair = make_pair()
    pair.start()
    assert os.environ['OPENBLAS_NUM_THREADS'] == '4'
    assert all(item['thread_env'] == '1' for item in pair.ready)
    for frame, seeds in ((1, ([.1, .2], [-.1, -.2])), (2, ([.7, -.4], [-.8, .4]))):
        result = pair.solve((observation(), observation()), seeds, frame_id=frame)
        for side in range(2):
            np.testing.assert_array_equal(result[side].qpos, seeds[side])


@pytest.mark.parametrize('behavior, message', [
    ('hang', 'timed out'), ('crash', 'exited without'), ('error', 'injected solve failure'),
    ('old_tag', 'obsolete frame'), ('nan', 'invalid joint'), ('bounds', 'invalid joint'),
])
def test_worker_failure_is_latched_and_never_returns_a_partial_pair(make_pair, behavior, message):
    pair = make_pair(behavior)
    pair.start()
    with pytest.raises(RuntimeError, match=message):
        pair.solve((observation(), observation()), (np.zeros(2), np.zeros(2)), frame_id=1)
    assert pair.failed
    with pytest.raises(RuntimeError, match='not running'):
        pair.solve((observation(), observation()), (np.zeros(2), np.zeros(2)), frame_id=2)


def test_startup_failure_cleans_both_workers(make_pair):
    pair = make_pair('startup_error')
    with pytest.raises(RuntimeError, match='injected startup failure'):
        pair.start()
    assert pair.closed


def test_flow_stops_backend_before_reaping_failed_workers(make_pair):
    pair = make_pair('hang')
    events = []
    pipeline = NS(initialized=True)
    pipeline.step = lambda sample: pair.solve((observation(), observation()),
                                              (np.zeros(2), np.zeros(2)), frame_id=sample.source_index)
    sample = NS(complete=True, source_index=1, left=NS(source_index=1), right=NS(source_index=1))
    source = NS(open=lambda: None, read=lambda: sample, close=lambda: events.append('source'))
    flow = BimanualExecutionFlow(source=source, pipeline=pipeline, initial_qpos=np.zeros(4),
                                robot_dofs=(2, 2), arm_dofs=(0, 0))
    flow.pair_solver = pair
    flow.backend = NS(request_stop=lambda reason: events.append('stop'), close=lambda: events.append('backend'))
    original_close = pair.close

    def close_solver():
        events.append('solver')
        original_close()

    pair.close = close_solver
    with pytest.raises(RuntimeError, match='timed out'):
        flow.run()
    assert events[:4] == ['stop', 'backend', 'solver', 'source']
    assert pair.cleanup[0]['method'] == 'terminate'


def test_duration_cancels_pending_pair_and_never_publishes(make_pair):
    pair = make_pair('hang')
    pair.start()
    events = []
    pipeline = NS(initialized=True)
    pipeline.step = lambda sample: pair.solve((observation(), observation()),
                                              (np.zeros(2), np.zeros(2)), frame_id=sample.source_index)
    flow = BimanualExecutionFlow(source=NS(close=lambda: None), pipeline=pipeline,
                                initial_qpos=np.zeros(4), robot_dofs=(2, 2), arm_dofs=(0, 0))
    flow.pair_solver = pair
    flow.backend = NS(request_stop=lambda reason: events.append('stop'), close=lambda: None,
                      execute=lambda q: pytest.fail('Cancelled result must never be published'))
    timer = threading.Timer(.03, flow._expire_duration)
    timer.start()
    try:
        sample = NS(complete=True, source_index=1, left=NS(source_index=1), right=NS(source_index=1))
        assert flow.step(sample) is None
        assert flow.command_count == 0 and events == ['stop']
    finally:
        timer.join()
        flow.close()


def test_source_open_failure_still_reaps_workers(make_pair):
    def fail():
        raise RuntimeError('input failed')

    pair = make_pair()
    flow = BimanualExecutionFlow(source=NS(open=fail, close=lambda: None), pipeline=NS(),
                                initial_qpos=np.zeros(4), robot_dofs=(2, 2), arm_dofs=(0, 0))
    flow.pair_solver = pair
    with pytest.raises(RuntimeError, match='input failed'):
        flow.run()
    assert pair.closed and len(pair.cleanup) == 2


def test_lazy_start_discards_sample_before_calibration(make_pair):
    pair = make_pair()
    pipeline = NS(initialize=lambda *args: pytest.fail('Pre-startup frame must be discarded'))
    flow = BimanualExecutionFlow(source=NS(close=lambda: None), pipeline=pipeline,
                                initial_qpos=np.zeros(4), robot_dofs=(2, 2), arm_dofs=(0, 0))
    flow.pair_solver = pair
    try:
        assert flow.step(NS()) is None
        assert pair.started and flow.command_count == 0 and flow.started_at is None
    finally:
        flow.close()


def test_actual_sharpa_run_starts_before_input_and_reaps_workers(monkeypatch):
    from retargeting_apps.sharpa_teleop import build_flow
    from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput

    args = NS(config=None, backend='preview', duration=0., command_hz=20.,
              adb=None, serial=None, viewer=False, viewer_port=9219)
    source = SyntheticBimanualInput(frames=4)
    flow, _ = build_flow(args, with_arms=True, source=source)
    original_open = source.open

    def open_after_ready():
        assert flow.pair_solver.started
        original_open()

    monkeypatch.setattr(source, 'open', open_after_ready)
    flow.run()
    assert flow.command_count == 4 and flow.stale_count == 0
    assert flow._closed and len(flow.pair_solver.cleanup) == 2
    assert all(p['exitcode'] == 0 and not p['alive'] for p in flow.pair_solver.cleanup)


def test_stale_result_restores_parent_seed_before_next_process_request(make_pair):
    from teleoperation.bimanual import BimanualRetargetedFrame

    pair = make_pair()
    pair.start()
    pipeline = NS(initialized=True,
                  left_retargeter=NS(previous_qpos=np.array([.1, .2])),
                  right_retargeter=NS(previous_qpos=np.array([-.1, -.2])))
    observations = (observation(), observation())
    seeds_seen = []

    def step(sample):
        seeds = (pipeline.left_retargeter.previous_qpos, pipeline.right_retargeter.previous_qpos)
        seeds_seen.append(np.concatenate(seeds))
        left, right = pair.solve(observations, seeds, frame_id=sample.source_index)
        pipeline.left_retargeter.previous_qpos = left.qpos + .1
        pipeline.right_retargeter.previous_qpos = right.qpos + .1
        return BimanualRetargetedFrame(left.qpos, right.qpos, *observations)

    pipeline.step = step
    measured = np.array([.1, .2, -.1, -.2])
    flow = BimanualExecutionFlow(source=NS(close=lambda: None, max_age_s=.15), pipeline=pipeline,
                                initial_qpos=measured, robot_dofs=(2, 2), arm_dofs=(0, 0))
    flow.pair_solver = pair
    try:
        old = NS(received_monotonic_ns=time.monotonic_ns() - 1_000_000_000)
        sample = NS(complete=True, source_index=1, left=NS(source_index=1, raw=old), right=NS(source_index=1))
        assert flow.step(sample) is None
        assert flow.stale_count == 1 and flow.command_count == 0
        sample = NS(complete=True, source_index=2, left=NS(source_index=2), right=NS(source_index=2))
        assert flow.step(sample) is not None
        np.testing.assert_array_equal(seeds_seen[1], measured)
    finally:
        flow.close()


def orphan_parent(connection):
    parallel_solver._solver_worker = controlled_worker
    pair = BimanualProcessSolver(fake_retargeters())
    pair.start()
    connection.send([item.pid for item in pair._processes])
    # Deliberately skip cleanup when this parent is killed by the test.
    connection.recv()


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux parent-death signal and child subreaper')
def test_parent_sigkill_terminates_workers_and_test_reaps_them():
    import ctypes

    libc = ctypes.CDLL(None, use_errno=True)
    previous = ctypes.c_int()
    assert libc.prctl(37, ctypes.byref(previous), 0, 0, 0) == 0  # PR_GET_CHILD_SUBREAPER
    assert libc.prctl(36, 1, 0, 0, 0) == 0  # Adopt test grandchildren so no zombie is left to init.
    context = mp.get_context('spawn')
    parent_pipe, child_pipe = context.Pipe()
    parent = context.Process(target=orphan_parent, args=(child_pipe,))
    children = []
    try:
        parent.start()
        child_pipe.close()
        assert parent_pipe.poll(30.)
        children = parent_pipe.recv()
        parent.kill()
        parent.join(5.)
        assert not parent.is_alive()
        for pid in children[:]:
            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline:
                waited, status = os.waitpid(pid, os.WNOHANG)
                if waited:
                    assert os.waitstatus_to_exitcode(status) == -signal.SIGKILL
                    children.remove(pid)
                    break
                time.sleep(.01)
            else:
                pytest.fail('Solver survived parent SIGKILL')
    finally:
        parent_pipe.close()
        child_pipe.close()
        if parent.pid is not None:
            if parent.is_alive():
                parent.kill()
            parent.join(5.)
            parent.close()
        for pid in children:
            try:
                os.kill(pid, signal.SIGKILL)
                os.waitpid(pid, 0)
            except ProcessLookupError:
                pass
        assert libc.prctl(36, previous.value, 0, 0, 0) == 0
