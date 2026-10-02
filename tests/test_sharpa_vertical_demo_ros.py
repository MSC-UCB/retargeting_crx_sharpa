"""Opt-in, isolated mock validation of the finite 20 cm vertical demo."""

import os
import signal
import subprocess
import threading

import numpy as np
import pytest


pytestmark = pytest.mark.skipif(os.environ.get('SHARPA_VERTICAL_ROS_TEST') != '1',
                                reason='Set SHARPA_VERTICAL_ROS_TEST=1 in a sourced ROS environment')


def test_vertical_demo_publishes_three_topics_and_finishes_with_mock_feedback(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_DOMAIN_ID', '192')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_STATIC_PEERS', '')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from retargeting_apps.sharpa_vertical_demo import build_flow, parse_args

    flow, _ = build_flow(parse_args(['--backend', 'ros', '--no-viewer', '--cycles', '1',
                                     '--startup-timeout', '30']))
    commands = {topic: [] for topic in ('/crx5ia/joint_targets',
                '/sharpa/left_hand/joint_command', '/sharpa/right_hand/joint_command')}
    context = Context()
    rclpy.init(args=[], context=context)
    peer = Node('vertical_demo_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    for topic, messages in commands.items():
        peer.create_subscription(JointState, topic, messages.append, 10)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    processes, logs, actual_positions, endpoint_errors = [], [], [], []
    monitor = flow.observer

    def observe(result):
        monitor(result)
        actual = flow.backend.get_joint_pos()
        poses = []
        for r, joints in zip((flow.pipeline.left_retargeter, flow.pipeline.right_retargeter),
                             flow.robot_slices):
            poses.append(r.robot_adaptor.robot_model.get_frame_pose(
                r.robot_config.wrist_frame_name,
                qpos=r.robot_adaptor.forward_qpos(actual[joints]))[:3, 3].copy())
        actual_positions.append(poses)
        if flow.source.last_elapsed >= flow.source.motion_duration:
            endpoint_errors.append(float(np.max(np.abs(actual-result.qpos))))

    flow.observer = observe
    launches = [
        ['ros2', 'launch', 'dual_crx_control', 'dual_arm.launch.py',
         'mock:=true', 'rviz:=false', 'method:=linear', 'input_rate_hz:=100.0'],
        ['ros2', 'launch', 'dual_sharpa_wave', 'dual_sharpa.launch.py',
         'backend:=mock', 'use_rviz:=false', 'publish_rate_hz:=100.0'],
    ]
    try:
        for i, command in enumerate(launches):
            log = (tmp_path / f'driver_{i}.log').open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        flow.run()
        assert flow.source.done and flow.command_count >= 150
        assert all(p.poll() is None for p in processes)
        assert endpoint_errors[-1] <= np.radians(2.)
        travel = np.ptp(np.asarray(actual_positions)[:, :, 2], axis=0)
        np.testing.assert_allclose(travel, [.20, .20], atol=.02)
        stamps = []
        for topic, messages in commands.items():
            assert len(messages) > 600
            assert all(len(m.position) == (12 if 'crx5ia' in topic else 22) for m in messages)
            values = [m.header.stamp.sec + m.header.stamp.nanosec / 1e9 for m in messages]
            rate = (len(values)-1)/(values[-1]-values[0])
            assert 80 < rate < 120
            stamps.append({(m.header.stamp.sec, m.header.stamp.nanosec) for m in messages})
            print(f'{topic}: {rate:.1f} Hz, {len(messages)} messages')
        assert len(set.intersection(*stamps)) > .9 * min(map(len, stamps))
        print(f'mock wrist z travel m={travel}; {flow.command_count} solver targets; '
              f'stale={flow.stale_count}; final joint error rad={endpoint_errors[-1]:.6f}')
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
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
        for log in logs:
            log.close()
    assert len(flow.pair_solver.cleanup) == 2
    assert all(not p['alive'] and p['exitcode'] == 0 for p in flow.pair_solver.cleanup)
    assert not any(t.name == 'sharpa_feedback' for t in threading.enumerate())
