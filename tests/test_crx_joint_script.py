"""Headless script checks; explicitly opt in to isolated ROS mock integration."""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/run_crx_joint_teleop.py'
spec = importlib.util.spec_from_file_location('crx_joint_script', SCRIPT)
script = importlib.util.module_from_spec(spec)
spec.loader.exec_module(script)


@pytest.fixture
def connection():
    """Exercise message handling without importing any ROS packages."""
    link = script.JointConnection.__new__(script.JointConnection)
    link._actual = np.full(44, 0.123)
    link._target = link._actual.copy()
    link._lock = threading.RLock()
    link._stopped = link._paused = link._closed = False
    link._feedback_error = 'waiting for complete arm feedback'
    link._stamp_ns = 0
    link._received_at = link._advanced_at = 0.0
    link._spin_error = None
    link._feedback_timeout = 0.5
    link._publish_hz = None
    link._output_method = 'cubic'
    link._horizon = .05
    link._target_timeout = .25
    link._output_timer = link._interpolator = link._pending_target = None
    link._last_target_at = None
    link._last_published = link._actual[script.ARM_INDICES].copy()
    link._output_generation = 0
    link._target_count = link._publish_count = link._rejected_segments = 0
    link._context = NS(ok=lambda: True)
    link._thread = NS(is_alive=lambda: True)
    link.logs = []
    link._node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)),
                    get_logger=lambda: NS(warning=link.logs.append, error=link.logs.append))
    link.commands = []
    link.names = []
    def capture(msg):
        link.commands.append(list(msg.position))
        link.names.append(list(msg.name))
    link._publisher = NS(publish=capture)
    link._command_type = lambda **kwargs: NS(**kwargs)
    return link


def state(names=None, positions=None, sec=100, nanosec=0):
    return NS(name=list(script.ARM_NAMES) if names is None else names,
              position=list(range(12)) if positions is None else positions,
              header=NS(stamp=NS(sec=sec, nanosec=nanosec)))


def test_named_feedback_and_44_to_12_mapping(connection):
    link = connection
    with pytest.raises(RuntimeError, match='waiting'):
        link.execute(np.arange(44))
    link._receive_state(state(list(reversed(script.ARM_NAMES)), list(reversed(range(12)))))
    measured = link.get_joint_pos()
    np.testing.assert_equal(measured[script.ARM_INDICES], np.arange(12))
    np.testing.assert_equal(measured[6:22], 0.123)
    np.testing.assert_equal(measured[28:], 0.123)
    measured[:] = 999  # Returned arrays must not mutate the feedback cache.
    result = link.execute(np.arange(44))
    assert link.commands == [[*range(6), *range(22, 28)]]
    np.testing.assert_equal(result.actual_qpos[script.ARM_INDICES], np.arange(12))
    assert not result.diagnostics['right_hand_output_enabled']
    assert not result.diagnostics['left_hand_feedback_fresh']


@pytest.mark.parametrize('values', [np.zeros(12), np.zeros(43), np.zeros(45),
                                  np.full(44, np.nan), np.full(44, np.inf),
                                  np.full(44, -np.inf)])
def test_bad_commands_rejected(connection, values):
    connection._receive_state(state())
    with pytest.raises(ValueError, match='44 finite'):
        connection.execute(values)
    assert not connection.commands


@pytest.mark.parametrize('message', [
    state(names=list(script.ARM_NAMES)[:-1], positions=[0.] * 11),
    state(names=['left_J1'] * 12), state(positions=[0.] * 11),
    state(positions=[np.nan] * 12), state(sec=0), state(sec=99), state(sec=101),
])
def test_bad_feedback_blocks_output(connection, message):
    connection._receive_state(state())
    connection._receive_state(message)
    with pytest.raises(RuntimeError):
        connection.execute(np.zeros(44))
    assert not connection.commands


def test_cached_feedback_does_not_refresh_source_freshness(connection):
    link = connection
    link._receive_state(state())
    # ROS time is frozen but callbacks still arrive, just like cached bridge feedback.
    link._advanced_at = time.monotonic() - 1.0
    link._receive_state(state())
    with pytest.raises(RuntimeError, match='frozen'):
        link.execute(np.zeros(44))
    assert not link.commands


def test_feedback_loss_and_clock_regression(connection):
    link = connection
    link._receive_state(state())
    link._received_at -= 1.0
    with pytest.raises(RuntimeError, match='stale'):
        link.get_joint_pos()
    link._receive_state(state(sec=99, nanosec=990_000_000))
    with pytest.raises(RuntimeError, match='backwards'):
        link.get_joint_pos()


def test_reset_pause_resume_and_stop_never_command_home(connection):
    link = connection
    link._receive_state(state())
    link.reset(np.full(44, 2.0))
    np.testing.assert_equal(link.get_target_joint_pos()[script.ARM_INDICES], np.arange(12))
    link.pause_tracking()
    link.assert_tracking()  # Healthy feedback while paused is normal.
    with pytest.raises(RuntimeError, match='paused'):
        link.execute(np.zeros(44))
    link._receive_state(state(positions=[0.25] * 12))
    link.resume_tracking()
    np.testing.assert_equal(link.get_target_joint_pos()[script.ARM_INDICES], 0.25)
    assert not link.commands
    link.execute(np.zeros(44))
    link.request_stop('session expired')
    with pytest.raises(RuntimeError, match='stopped'):
        link.resume_tracking()
    with pytest.raises(RuntimeError, match='stopped'):
        link.execute(np.ones(44))
    assert len(link.commands) == 1


def test_cli_help_and_invalid_timing_do_not_import_ros_or_open_devices():
    command = (
        'import importlib.util, sys; '
        f'spec=importlib.util.spec_from_file_location("joint_script", {str(SCRIPT)!r}); '
        'module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); '
        'assert "rclpy" not in sys.modules; '
        'assert "teleoperation.inputs.quest3.online" not in sys.modules'
    )
    subprocess.run([sys.executable, '-c', command], check=True, cwd=ROOT)
    result = subprocess.run([sys.executable, str(SCRIPT), '--help'], capture_output=True, text=True)
    assert result.returncode == 0 and '--command-hz' in result.stdout and '--publish-hz' in result.stdout
    for args in (['--command-hz', 'nan'], ['--command-hz', '0'], ['--duration', '-1'],
                 ['--publish-hz', 'nan'], ['--publish-hz', '0'], ['--publish-hz', '501'],
                 ['--publish-hz', '100', '--interpolation-horizon-ms', '0']):
        result = subprocess.run([sys.executable, str(SCRIPT), *args], capture_output=True, text=True)
        assert result.returncode == 2
        assert 'must be finite' in result.stderr


def test_flow_composition_keeps_measured_seed_and_arm_filtering(monkeypatch):
    from teleoperation.bimanual import BimanualRetargetedFrame

    args = NS(command_hz=20.0, duration=1.0, serial='test-headset', viewer=True, viewer_port=9219,
              publish_hz=100., output_interpolation='cubic', interpolation_horizon_ms=50.)
    flow, config = script.build_flow(args)  # Actual configs/models, but no source.open() or ROS.
    expected_arms = np.radians([0, 30, -30, 0, 60, 0, -90, -30, 210, 0, -60, 0])
    np.testing.assert_allclose(flow.initial_qpos[script.ARM_INDICES], expected_arms)
    from retargeting.config import load_robot_config
    for side, retargeter in zip(('left', 'right'), (
            flow.pipeline.left_retargeter, flow.pipeline.right_retargeter)):
        sharpa = load_robot_config(f'configs/robots/crx5ia_sharpa_wave_{side}.yaml')
        np.testing.assert_allclose(retargeter.qpos_init[:6], sharpa.initial_qpos[:6])
        limits = retargeter.optimizer.joint_limits
        assert np.all(retargeter.qpos_init >= limits[:, 0])
        assert np.all(retargeter.qpos_init <= limits[:, 1])
    assert config['viewer']['enabled'] and config['viewer']['port'] == 9219
    assert not config['viewer']['wait_for_client']
    assert flow.backend is None
    assert flow.period == 0.05 and flow.duration == 1.0
    assert flow.hand_output_filters is None
    assert len(flow.arm_output_filters) == 2
    assert all(f.mode_config.output.smooth_output_qpos for f in flow.arm_output_filters)
    assert all(f.mode_config.output.smoothing_alpha == 0.3 for f in flow.arm_output_filters)
    measured = np.full(44, 0.12)
    commands, seeds = [], []
    link = NS(get_joint_pos=lambda: measured.copy(), execute=lambda q: commands.append(q.copy()))
    def make_connection(initial, period, **kwargs):
        assert period == .05
        assert kwargs == dict(publish_hz=100., output_interpolation='cubic',
                              interpolation_horizon=.05, target_timeout=flow.timeout,
                                  namespace='crx5ia')
        return link

    monkeypatch.setattr(script, 'JointConnection', make_connection)
    pipeline = NS(initialized=False)
    pipeline.left_retargeter = NS(reset=lambda q: seeds.append(q.copy()), previous_qpos=None)
    pipeline.right_retargeter = NS(reset=lambda q: seeds.append(q.copy()), previous_qpos=None)

    def initialize(sample, left, right):
        pipeline.initialized = True
        return True

    pipeline.initialize = initialize
    pipeline.step = lambda sample: BimanualRetargetedFrame(
        np.ones(22), np.full(22, -1.), NS(), NS())
    flow.pipeline = pipeline
    flow.source = NS(max_age_s=10.0)
    flow.duration = 0.0  # This test does not start a duration timer.
    sample = lambda index: NS(complete=True, source_index=index,
                             left=NS(source_index=index), right=NS(source_index=index))
    assert flow.step(sample(1)) is None  # Backend startup discards the pre-startup sample.
    assert not commands
    flow.step(sample(2))
    np.testing.assert_allclose(seeds, np.full((2, 22), 0.12))
    np.testing.assert_allclose(commands[0][:6], 0.3 * 1. + 0.7 * 0.12)
    np.testing.assert_allclose(commands[0][22:28], 0.3 * -1. + 0.7 * 0.12)
    flow.step(sample(2))
    assert len(commands) == 1


def test_preview_solves_from_configured_pose_without_ros(monkeypatch):
    from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput

    monkeypatch.setattr(script, 'JointConnection',
                        lambda *a, **kw: pytest.fail('Preview must not connect to ROS'))
    args = NS(backend='preview', command_hz=20., duration=0., serial=None,
              viewer=False, viewer_port=9219, publish_hz=100.,
              output_interpolation='cubic', interpolation_horizon_ms=50.)
    flow, _ = script.build_flow(args)
    assert flow.backend_factory is None and flow.backend is None
    flow.source = SyntheticBimanualInput(frames=2, max_age_s=10.)
    try:
        flow.source.open()
        for _ in range(2):
            result = flow.step(flow.source.read())
            assert result is not None and np.isfinite(result.qpos).all()
        assert flow.command_count == 2 and flow.backend is None
    finally:
        flow.close()


@pytest.mark.parametrize('failure', [None, KeyboardInterrupt(), RuntimeError('source failed')])
def test_script_starts_viewer_and_closes_it_on_exit(monkeypatch, failure):
    from retargeting_apps.visualization.execution import manager

    events = []

    def run():
        events.append('run')
        if failure is not None:
            raise failure

    flow = NS(run=run)

    def build(args):
        assert args.viewer is True
        assert args.backend == 'ros'
        return flow, {'viewer': {'enabled': args.viewer, 'port': args.viewer_port}}

    def create(config, actual_flow):
        assert actual_flow is flow and config['viewer']['port'] == 9220
        events.append('viewer')
        return NS(close=lambda: events.append('close'))

    monkeypatch.setattr(script, 'build_flow', build)
    monkeypatch.setattr(manager, 'create_optional_execution_visualizer', create)
    assert script.main(['--viewer-port', '9220']) == (1 if isinstance(failure, RuntimeError) else 0)
    assert events == ['viewer', 'run', 'close']


@pytest.mark.parametrize('backend', ['preview', 'ros'])
def test_script_no_viewer_never_constructs_visualizer(monkeypatch, backend):
    from retargeting_apps.visualization.execution import manager

    ran = []

    def build(args):
        assert args.viewer is False
        assert args.backend == backend
        return NS(run=lambda: ran.append(True)), {}

    monkeypatch.setattr(script, 'build_flow', build)
    monkeypatch.setattr(manager, 'create_optional_execution_visualizer',
                        lambda *args: pytest.fail('headless script must not create a viewer'))
    assert script.main(['--no-viewer', '--backend', backend]) == 0
    assert ran == [True]


@pytest.mark.skipif(os.environ.get('CRX_JOINT_ROS_TEST') != '1',
                    reason='Set CRX_JOINT_ROS_TEST=1 after sourcing ROS to run isolated mock')
@pytest.mark.parametrize('publish_hz,solver_load', [(None, False), (100.0, False), (100.0, True)])
def test_ros_mock_round_trip_and_cleanup(monkeypatch, tmp_path, publish_hz, solver_load):
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    monkeypatch.setenv('ROS_DOMAIN_ID', '185')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_STATIC_PEERS', '')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    # Failure to obtain feedback must clean up its own ROS executor and context.
    with pytest.raises(RuntimeError, match='startup timeout'):
        script.JointConnection(np.zeros(44), 0.05, startup_timeout=0.2)
    assert not any(t.name == 'crx_joint_feedback' for t in threading.enumerate())
    context = Context()
    rclpy.init(args=[], context=context)
    peer = Node('joint_script_test_monitor', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    commands, receipt_times, interpolated_times = [], [], []
    command_names = []
    from sensor_msgs.msg import JointState

    def receive(msg):
        commands.append(list(msg.position))
        command_names.append(list(msg.name))
        receipt_times.append(time.monotonic())

    peer.create_subscription(JointState, '/crx5ia/joint_targets',
                             receive, 100)
    peer.create_subscription(JointState, '/crx5ia/interpolated_joint_commands',
                             lambda msg: interpolated_times.append(time.monotonic()), 100)

    def wait_for(predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), (tmp_path / 'launch.log').read_text()

    process = link = observer_thread = None
    observer_stop = threading.Event()
    try:
        with (tmp_path / 'launch.log').open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'dual_crx_control', 'dual_arm.launch.py',
                 'namespace:=crx5ia', 'mock:=true', 'rviz:=false',
                 f'input_rate_hz:={publish_hz or 20.0}', 'method:=linear'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            for side in ('left', 'right'):
                client = peer.create_client(ListControllers, f'/crx5ia/{side}/controller_manager/list_controllers')
                wait_for(client.service_is_ready)
                deadline = time.monotonic() + 10
                while True:
                    future = client.call_async(ListControllers.Request())
                    wait_for(future.done)
                    states = {c.name: c.state for c in future.result().controller}
                    if states == {'joint_state_broadcaster': 'active', 'forward_position_controller': 'active'}:
                        break
                    assert time.monotonic() < deadline, states
                    time.sleep(0.05)  # Do not flood the service while spawners activate.
            initial = np.full(44, 0.123)
            link = script.JointConnection(initial, 0.05, publish_hz=publish_hz)
            wait_for(lambda: link._publisher.get_subscription_count() >= 2)
            assert not commands
            mock_pose = [0., 0., 0., 0., -np.pi / 2, 0.,
                         -np.pi / 2, 0., np.pi, 0., np.pi / 2, 0.]
            np.testing.assert_allclose(link.get_joint_pos()[script.ARM_INDICES], mock_pose)
            np.testing.assert_allclose(link.get_target_joint_pos()[script.ARM_INDICES], mock_pose)
            q = link.get_joint_pos()
            q[0] += 0.00872665
            q[22] -= 0.00872665
            link.execute(q)
            wait_for(lambda: bool(commands))
            wait_for(lambda: np.allclose(
                link.get_joint_pos()[script.ARM_INDICES], q[script.ARM_INDICES]), timeout=5.0)
            if publish_hz is None:
                assert len(commands) == 1
                assert command_names[0] == list(script.ARM_NAMES)
                np.testing.assert_allclose(commands[0], q[script.ARM_INDICES])
            else:
                # First publication starts at the measured pose, not the new target.
                np.testing.assert_allclose(commands[0], mock_pose, atol=1e-8)
                solve_times = []
                if solver_load:
                    from dataclasses import replace
                    from test_bimanual_quest import _frame
                    from teleoperation.inputs.quest3.common import decode_quest3_sample
                    from teleoperation.types import BimanualSensorHandSample

                    flow, _ = script.build_flow(NS(
                        command_hz=20., duration=0., serial=None, viewer=False, viewer_port=9219,
                        publish_hz=100., output_interpolation='cubic', interpolation_horizon_ms=50.))

                    def sensor_sample(index):
                        frame = replace(_frame(), sequence=index, source_sequence=index,
                                        received_monotonic_ns=time.monotonic_ns())
                        return BimanualSensorHandSample(
                            left=decode_quest3_sample(frame, hand_side='left'),
                            right=decode_quest3_sample(frame, hand_side='right'))

                    assert flow.pipeline.initialize(sensor_sample(1), q[:22], q[22:])
                    flow._reset_output_filters(q)

                def observe():
                    while not observer_stop.is_set():
                        executor.spin_once(timeout_sec=.01)

                # Timestamp arrivals independently of the IK loop; otherwise solver
                # time is misreported as transport jitter and queued callbacks burst.
                observer_thread = threading.Thread(target=observe, name='crx_test_observer')
                observer_thread.start()
                start = time.monotonic()
                next_target = start
                target_count = 0
                while time.monotonic() - start < 10.5:
                    now = time.monotonic()
                    if now >= next_target:
                        target = q.copy()
                        target_count += 1
                        if solver_load:
                            solve_start = time.monotonic()
                            result = flow.pipeline.step(sensor_sample(target_count+1))
                            assert result is not None
                            target = flow._filter_commands(result).qpos
                            solve_times.append(time.monotonic()-solve_start)
                        else:
                            target[0] += .001*np.sin(2*np.pi*.5*(now-start))
                            target[22] -= .001*np.sin(2*np.pi*.5*(now-start))
                        link.execute(target)
                        next_target = max(now+.05, time.monotonic())
                    time.sleep(.001)
                observer_stop.set()
                observer_thread.join()
                target_times = np.array([t for t in receipt_times if start+.25 <= t <= start+10.25])
                assert len(target_times) > 500, len(target_times)
                target_hz = (len(target_times)-1)/(target_times[-1]-target_times[0])
                target_intervals = np.diff(target_times)*1000
                assert 75 <= target_hz <= 125, target_hz
                assert np.percentile(target_intervals, 99) <= 30
                assert target_intervals.max() <= 100
                print(f'target (solver_load={solver_load}): {target_hz:.2f} Hz, p99 {np.percentile(target_intervals,99):.2f} ms, max {target_intervals.max():.2f} ms')
                core_times = np.array([t for t in interpolated_times if start+.25 <= t <= start+10.25])
                assert len(core_times) > 1000, len(core_times)
                core_hz = (len(core_times)-1)/(core_times[-1]-core_times[0])
                core_intervals = np.diff(core_times)*1000
                assert 250 <= core_hz <= 650, core_hz
                assert np.percentile(core_intervals, 99) <= 20
                assert core_intervals.max() <= 100
                print(f'core (solver_load={solver_load}): {core_hz:.2f} Hz, p99 {np.percentile(core_intervals,99):.2f} ms, max {core_intervals.max():.2f} ms')
                if solve_times:
                    print(f'Real dual-arm IK: {len(solve_times)} solves, p99 {np.percentile(solve_times,99)*1000:.2f} ms, max {max(solve_times)*1000:.2f} ms')
                assert link._target_count < link._publish_count / 3
            link.pause_tracking()
            paused_count = link._publish_count
            time.sleep(0.3)
            assert link._publish_count == paused_count
            link.assert_tracking()
            link.resume_tracking()
            np.testing.assert_allclose(link.get_joint_pos()[6:22], initial[6:22])
            link.request_stop('test complete')
            stopped_count = link._publish_count
            with pytest.raises(RuntimeError, match='stopped'):
                link.execute(q)
            link.close()
            link.close()
            assert context.ok()  # Closing the script must not shut down the monitor.
            assert not link._thread.is_alive()
            executor.spin_once(timeout_sec=0.1)
            assert link._publish_count == stopped_count
            if publish_hz is None:
                assert len(commands) == 1
                assert command_names[0] == list(script.ARM_NAMES)
    finally:
        observer_stop.set()
        if observer_thread is not None:
            observer_thread.join()
        if link is not None:
            link.close()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        executor.shutdown()
        peer.destroy_node()
        context.try_shutdown()
    assert process.returncode == 0, (tmp_path / 'launch.log').read_text()
