"""Opt-in tests against installed Sharpa/CRX ROS drivers in mock mode only."""

import os
import signal
import subprocess
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest


pytestmark = pytest.mark.skipif(os.environ.get('SHARPA_ROS_TEST') != '1',
                                reason='Set SHARPA_ROS_TEST=1 in a sourced ROS environment')


@pytest.mark.parametrize('with_arms, arm_method', [(False, None), (True, 'linear'), (True, 'ruckig')])
def test_mock_drivers_receive_solver_commands_and_cleanup(with_arms, arm_method, monkeypatch, tmp_path):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from retargeting_apps.sharpa_teleop import build_flow
    from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput

    monkeypatch.setenv('ROS_DOMAIN_ID', '189')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_STATIC_PEERS', '')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    args = NS(config=None, backend='ros', duration=0., command_hz=20., adb=None, serial=None,
              publish_hz=100., interpolation_horizon_ms=50.,
              viewer=False, viewer_port=9219, startup_timeout=30.,
              crx_namespace='crx5ia', sharpa_namespace='sharpa')
    source = SyntheticBimanualInput(100)
    flow, _ = build_flow(args, with_arms=with_arms, source=source)
    launches = [['ros2', 'launch', 'dual_sharpa_wave', 'dual_sharpa.launch.py',
                 'backend:=mock', 'use_rviz:=false', 'publish_rate_hz:=100.0']]
    if with_arms:
        launches.append(['ros2', 'launch', 'dual_crx_control', 'dual_arm.launch.py',
                         'mock:=true', 'rviz:=false', f'method:={arm_method}', 'input_rate_hz:=100.0'])
    processes, logs = [], []
    context = Context()
    rclpy.init(args=[], context=context)
    peer = Node('sharpa_test_monitor', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    received = {}
    topics = ['/sharpa/left_hand/joint_command', '/sharpa/right_hand/joint_command']
    if with_arms:
        topics.append('/crx5ia/joint_targets')
    for topic in topics:
        received[topic] = []
        peer.create_subscription(JointState, topic, received[topic].append, 10)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        for i, command in enumerate(launches):
            log = (tmp_path / f'launch_{i}.log').open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        flow.start_solver()
        assert len(flow.pair_solver.ready) == 2
        assert all(worker['maxtime'] == .030 for worker in flow.pair_solver.ready)
        source.open()
        assert flow.step(source.read()) is None  # connect and seed; no target publication
        assert flow.backend is not None
        assert all(not values for values in received.values())
        for _ in range(35):
            start = time.monotonic()
            assert all(p.poll() is None for p in processes)
            flow.step(source.read())
            time.sleep(max(0, flow.period - (time.monotonic() - start)))
        assert flow.command_count >= 5, (flow.command_count, flow.stale_count, flow.last_solve_ms)
        assert all(len(values) >= 5 for values in received.values())
        measured = flow.backend.get_joint_pos()
        target = flow.backend.get_target_joint_pos()
        assert measured.shape == target.shape == flow.initial_qpos.shape
        assert np.isfinite(measured).all()
        assert np.max(np.abs(target - measured)) < .2
        for topic, messages in received.items():
            assert all(len(msg.name) == (12 if 'crx5ia' in topic else 22) for msg in messages)
            stamps = np.asarray([msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                                 for msg in messages])
            rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
            print(f'{topic}: {rate:.1f} Hz, {len(messages)} publications, '
                  f'{flow.command_count} solver targets', flush=True)
            assert 80. <= rate <= 120., (topic, rate)
            assert len(messages) > 2 * flow.command_count
        # Every publication uses one common timestamp for all required channels.
        stamp_sets = [{(msg.header.stamp.sec, msg.header.stamp.nanosec) for msg in values}
                      for values in received.values()]
        assert len(set.intersection(*stamp_sets)) >= .9 * min(map(len, stamp_sets))
        flow.backend.pause_tracking()
        time.sleep(.1)
        counts = {topic: len(values) for topic, values in received.items()}
        time.sleep(.3)
        assert counts == {topic: len(values) for topic, values in received.items()}
        assert flow.backend.resume_tracking()
    finally:
        flow.close()
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
                    process.wait(timeout=5)
        for log in logs:
            log.close()
        assert not any(t.name == 'sharpa_feedback' for t in threading.enumerate())
        assert len(flow.pair_solver.cleanup) == 2
        assert all(not p['alive'] and p['exitcode'] == 0 for p in flow.pair_solver.cleanup)
