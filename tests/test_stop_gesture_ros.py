"""Opt-in isolated ROS mocks; never connects to the lab's default ROS domain."""
import os
import signal
import subprocess
import threading
import time

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('STOP_GESTURE_ROS_TEST') != '1',
                                reason='Set STOP_GESTURE_ROS_TEST=1 in a sourced ROS environment')


@pytest.mark.parametrize('mode,release', [('waypoint', False), ('waypoint', True), ('stream', False)])
def test_mock_gesture_stop_and_preflight(monkeypatch, tmp_path, mode, release):
    monkeypatch.setenv('ROS_DOMAIN_ID', '194')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_STATIC_PEERS', '')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from retargeting_apps.sharpa_teleop import build_flow
    from test_sharpa_execution import arguments
    from test_stop_gesture import PinchAfterCommands

    flow, _ = build_flow(arguments(backend='ros', startup_timeout=20.,
                                   stop_gesture='dual-thumb-ring-pinch',
                                   stop_gesture_hold_s=2.), with_arms=True)
    flow.source = PinchAfterCommands(flow, release=release)
    topics = ('/crx5ia/joint_targets', '/sharpa/left_hand/joint_command',
              '/sharpa/right_hand/joint_command')
    messages = {topic: [] for topic in topics}
    context = Context()
    rclpy.init(args=[], context=context)
    peer = Node('gesture_stop_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    for topic in topics:
        peer.create_subscription(JointState, topic, messages[topic].append, 100)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    processes, logs, boundary = [], [], {}

    def observe(message):
        if not boundary:
            b = flow.backend
            boundary.update(stamp=b._node.get_clock().now().nanoseconds,
                            publications=b._publish_count, target_count=b._target_count,
                            actual=b._actual.copy(), at=time.monotonic())
    flow.stop_observer = observe
    launches = [
        ['ros2', 'launch', 'dual_crx_control', 'dual_arm.launch.py', 'mock:=true',
         'rviz:=false', 'method:=ruckig', f'ruckig_target_mode:={mode}', 'input_rate_hz:=100.0'],
        ['ros2', 'launch', 'dual_sharpa_wave', 'dual_sharpa.launch.py',
         'backend:=mock', 'use_rviz:=false', 'publish_rate_hz:=100.0'],
    ]
    try:
        for i, command in enumerate(launches):
            log = (tmp_path / f'driver_{i}.log').open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        if mode == 'stream':
            with pytest.raises(RuntimeError, match='requires CRX'):
                flow.run()
            time.sleep(.1)
            assert all(not rows for rows in messages.values())
        else:
            flow.run()
            time.sleep(.1)  # Drain the probe's DDS queue after backend shutdown.
            assert flow.stop_gesture.state == ('EXIT_UNCONFIRMED' if release else 'EXIT_CONFIRMED')
            assert flow.command_count == boundary['target_count'] == 5
            b = flow.backend
            assert b._publish_count == boundary['publications']
            assert b._pending_target is None and b._stopped
            for topic, key, size in zip(topics, ('arms', 'left_hand', 'right_hand'), (12, 22, 22)):
                rows = messages[topic]
                assert rows and all(len(m.position) == size for m in rows)
                stamps = [m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec for m in rows]
                assert max(stamps) <= boundary['stamp']
                # Exactly one hold in addition to interpolation publications.
                assert len(rows) == boundary['publications'] + 1
                np.testing.assert_allclose(rows[-1].position, boundary['actual'][b.channels[key][1]])
            print(f'{mode}, release={release}: {flow.stop_gesture.state}; '
                  f'{boundary["publications"]} publications + one hold; '
                  f'exit {time.monotonic()-boundary["at"]:.3f}s after latch')
    finally:
        try:
            flow.close()
        finally:
            executor.shutdown()
            thread.join()
            peer.destroy_node()
            context.try_shutdown()
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=5)
            for log in logs:
                log.close()
    assert flow.source.closed
    assert len(flow.pair_solver.cleanup) == 2
    assert all(not p['alive'] and p['exitcode'] == 0 for p in flow.pair_solver.cleanup)
    assert not any(t.name == 'sharpa_feedback' for t in threading.enumerate())
