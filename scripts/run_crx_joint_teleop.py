#!/usr/bin/env python3
"""Run CRX Quest retargeting in local preview or through ROS JointState targets."""

import argparse
from dataclasses import replace
import math
import sys
import threading
import time

import numpy as np


ARM_NAMES = tuple(f'{side}_J{i}' for side in ('left', 'right') for i in range(1, 7))
ARM_INDICES = np.array([*range(6), *range(22, 28)])
PROFILE_NAMES = tuple([f'J{i}' for i in range(1, 7)] + [f'joint_{i}' for i in range(16)])


def checked_qpos(qpos):
    values = np.asarray(qpos, dtype=float)
    if values.shape != (44,) or not np.isfinite(values).all():
        raise ValueError('Expected 44 finite joint positions: left arm/hand, right arm/hand')
    return values.copy()


class JointConnection:
    """Script-local transport implementing the existing bimanual flow callbacks."""

    def __init__(self, initial_qpos, control_period, *, startup_timeout=5.0,
                 feedback_timeout=0.5, publish_hz=None, output_interpolation='cubic',
                 interpolation_horizon=None, target_timeout=0.25, namespace='crx5ia'):
        self._actual = checked_qpos(initial_qpos)
        horizon = control_period if interpolation_horizon is None else interpolation_horizon
        for value in (control_period, startup_timeout, feedback_timeout, horizon, target_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Periods and timeouts must be finite and positive')
        if publish_hz is not None and (not math.isfinite(publish_hz) or not 0 < publish_hz <= 500):
            raise ValueError('publish_hz must be finite and in (0, 500]')
        if output_interpolation not in ('linear', 'cubic'):
            raise ValueError('output_interpolation must be linear or cubic')
        if publish_hz is not None and horizon >= target_timeout:
            raise ValueError('Interpolation horizon must be shorter than the target timeout')
        self._period = control_period
        self._feedback_timeout = feedback_timeout
        self._publish_hz = publish_hz
        self._output_method = output_interpolation
        self._horizon = horizon
        self._target_timeout = target_timeout
        self._output_timer = self._interpolator = self._pending_target = None
        self._last_target_at = None
        self._last_published = self._actual[ARM_INDICES].copy()
        self._output_generation = 0
        self._target_count = self._publish_count = self._rejected_segments = 0
        self._target = self._actual.copy()
        self._lock = threading.RLock()
        self._stopped = self._paused = self._closed = False
        self._feedback_error = 'waiting for complete arm feedback'
        self._stamp_ns = 0
        self._received_at = self._advanced_at = 0.0
        self._spin_error = None
        self._node = self._executor = self._thread = self._context = None
        try:
            import rclpy
            from rclpy.clock import Clock, ClockType
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except ImportError as exc:
            raise RuntimeError('Source ROS Jazzy and use a ROS-compatible Python 3.12 venv') from exc
        self._command_type = JointState
        self._namespace = namespace.strip('/') or 'crx5ia'
        try:
            # Own only this context; do not shut down another application's ROS nodes.
            self._context = Context()
            rclpy.init(args=[], context=self._context)
            self._node = Node('retargeting_crx_joint', namespace=self._namespace, context=self._context)
            self._publisher = self._node.create_publisher(JointState, 'joint_targets', 1)
            self._node.create_subscription(
                JointState, 'joint_states', self._receive_state, 1)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._spin, name='crx_joint_feedback', daemon=True)
            self._thread.start()
            deadline = time.monotonic() + startup_timeout
            while True:
                try:
                    with self._lock:
                        self._check_feedback()
                        if self._publisher.get_subscription_count() == 0:
                            raise RuntimeError('no joint-target subscriber; start dual_arm.launch.py')
                        self._target = self._actual.copy()
                        self._reset_output_locked(self._actual[ARM_INDICES])
                    break
                except RuntimeError as exc:
                    if time.monotonic() >= deadline:
                        raise RuntimeError(f'Joint bridge startup timeout: {exc}') from exc
                    time.sleep(0.01)
            if self._publish_hz is not None:
                self._output_timer = self._node.create_timer(
                    1.0 / self._publish_hz, self._publish_tick,
                    clock=Clock(clock_type=ClockType.STEADY_TIME), autostart=False)
            self._node.get_logger().info(
                f'Measured arm seed ready; output: /{self._namespace}/joint_targets. LEAP output disabled. '
                + ('Direct output.' if publish_hz is None else
                   f'{publish_hz:g} Hz {output_interpolation}; horizon {horizon*1000:g} ms.'))
        except BaseException:
            self.close()
            raise

    @property
    def control_period(self):
        return self._period

    def _spin(self):
        try:
            self._executor.spin()
        except Exception as exc:
            with self._lock:
                self._spin_error = str(exc)
                self._stopped = True
                if self._output_timer is not None:
                    self._output_timer.cancel()

    def _reset_output_locked(self, seed):
        """Invalidate in-flight interpolation and stop its timer. Caller holds lock."""
        self._output_generation += 1
        self._pending_target = self._last_target_at = None
        self._last_published = np.asarray(seed, dtype=float).copy()
        if self._output_timer is not None:
            self._output_timer.cancel()
        if self._publish_hz is not None:
            from retargeting_ros.joint_interpolation import JointCommandInterpolator

            self._interpolator = JointCommandInterpolator(seed, self._output_method)

    def _joint_state_message(self, values):
        values = np.asarray(values, dtype=float)
        if values.shape != (12,) or not np.isfinite(values).all():
            raise ValueError('CRX target must contain 12 finite joint positions')
        message = self._command_type()
        now = self._node.get_clock().now()
        if hasattr(now, 'to_msg'):
            message.header.stamp = now.to_msg()
        message.name = list(ARM_NAMES)
        message.position = values.tolist()
        return message

    def _publish_tick(self):
        """Sample once at actual time; never burst-replay missed timer slots."""
        with self._lock:
            if self._stopped or self._paused or self._last_target_at is None:
                return
            try:
                self._check_feedback()
            except RuntimeError as exc:
                self._spin_error = str(exc)
                self._stopped = True
                self._reset_output_locked(self._last_published)
                self._node.get_logger().error(f'Joint publisher stopped: {exc}')
                return
            now = time.monotonic()
            if now - self._last_target_at > self._target_timeout:
                self._reset_output_locked(self._last_published)
                self._node.get_logger().warning('Joint publisher suspended: no fresh IK target.')
                return
            pending, self._pending_target = self._pending_target, None
            interpolator, generation = self._interpolator, self._output_generation

        # Fitting happens outside the feedback/stop lock. Resets replace the helper,
        # and the generation check prevents an obsolete fit from publishing.
        try:
            if pending is not None:
                q, received_at = pending
                interpolator.set_target(q, received_at, self._horizon, now)
            values = interpolator.sample(now)
        except (ValueError, FloatingPointError) as exc:
            with self._lock:
                if generation != self._output_generation:
                    return
                self._rejected_segments += 1
                self._reset_output_locked(self._last_published)
            self._node.get_logger().warning(f'Joint interpolation rejected: {exc}')
            return

        with self._lock:
            if generation != self._output_generation or self._stopped or self._paused:
                return
            try:
                self._check_feedback()
            except RuntimeError as exc:
                self._spin_error = str(exc)
                self._stopped = True
                self._reset_output_locked(self._last_published)
                return
            published_at = time.monotonic()
            if (published_at - self._last_target_at > self._target_timeout
                    or published_at - now >= 1.0 / self._publish_hz):
                # A delayed computation cannot refresh old input into a new command.
                self._rejected_segments += 1
                self._reset_output_locked(self._last_published)
                self._node.get_logger().warning('Joint publisher suspended: sample calculation missed its deadline.')
                return
            self._publisher.publish(self._joint_state_message(values))
            interpolator.record_published(published_at, values)
            self._last_published = values.copy()
            self._publish_count += 1

    def _receive_state(self, message):
        with self._lock:
            names = list(message.name)
            positions = np.asarray(message.position, dtype=float)
            if (len(names) != 12 or len(set(names)) != 12 or set(names) != set(ARM_NAMES)
                    or positions.shape != (12,) or not np.isfinite(positions).all()):
                self._feedback_error = 'malformed 12-joint feedback'
                return
            stamp = message.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            age = (self._node.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if stamp_ns <= 0 or not -0.02 <= age <= self._feedback_timeout:
                self._feedback_error = 'stale or invalid feedback timestamp'
                return
            if stamp_ns < self._stamp_ns:
                self._feedback_error = 'feedback timestamp moved backwards'
                return
            now = time.monotonic()
            if stamp_ns > self._stamp_ns:
                self._advanced_at = now
            self._stamp_ns = stamp_ns
            self._received_at = now
            self._actual[ARM_INDICES] = [positions[names.index(name)] for name in ARM_NAMES]
            self._feedback_error = None

    def _check_feedback(self):
        # Caller holds _lock; repeat checks just before publishing, after any slow solve.
        if self._stopped:
            raise RuntimeError(f'Joint output stopped{": " + self._spin_error if self._spin_error else ""}')
        if not self._context.ok() or not self._thread.is_alive():
            raise RuntimeError('ROS feedback executor is unavailable')
        if self._feedback_error:
            raise RuntimeError(self._feedback_error)
        now = time.monotonic()
        age = (self._node.get_clock().now().nanoseconds - self._stamp_ns) / 1e9
        if (not -0.02 <= age <= self._feedback_timeout
                or now - self._received_at > self._feedback_timeout
                or now - self._advanced_at > self._feedback_timeout):
            raise RuntimeError('Joint feedback is stale or its source timestamp is frozen')

    def get_joint_pos(self):
        with self._lock:
            self._check_feedback()
            return self._actual.copy()

    def get_target_joint_pos(self):
        with self._lock:
            return self._target.copy()

    def reset(self, qpos=None):
        if qpos is not None:
            checked_qpos(qpos)
        with self._lock:
            self._check_feedback()
            self._target = self._actual.copy()  # Resynchronize; never command a home pose.
            self._reset_output_locked(self._actual[ARM_INDICES])

    def execute(self, qpos):
        from teleoperation.backends.base import BackendStepResult

        values = checked_qpos(qpos)
        with self._lock:
            self._check_feedback()
            if self._paused:
                raise RuntimeError('Joint output paused for tracking recovery')
            if self._publish_hz is None:
                self._publisher.publish(self._joint_state_message(values[ARM_INDICES]))
                self._last_published = values[ARM_INDICES].copy()
                self._publish_count += 1
            else:
                self._last_target_at = time.monotonic()
                self._pending_target = (values[ARM_INDICES].copy(), self._last_target_at)
                if self._output_timer.is_canceled():
                    self._output_timer.reset()
            self._target = values
            self._target_count += 1
            return BackendStepResult(
                command_qpos=values, actual_qpos=self._actual,
                diagnostics={'left_hand_output_enabled': 0.0, 'right_hand_output_enabled': 0.0,
                             'left_hand_feedback_fresh': 0.0, 'right_hand_feedback_fresh': 0.0})

    def pause_tracking(self):
        with self._lock:
            self._paused = True
            self._reset_output_locked(self._last_published)

    def resume_tracking(self):
        with self._lock:
            self._check_feedback()
            self._target = self._actual.copy()
            self._reset_output_locked(self._actual[ARM_INDICES])
            self._paused = False

    def assert_tracking(self):
        with self._lock:
            self._check_feedback()

    def request_stop(self, reason):
        with self._lock:
            self._stopped = True
            self._reset_output_locked(self._last_published)

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = self._stopped = True
            self._reset_output_locked(self._last_published)
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join()
        if self._node is not None:
            self._node.destroy_node()
        if self._context is not None:
            self._context.try_shutdown()


def build_flow(args):
    """Reuse the existing solver and lifecycle without registering a new backend."""
    from retargeting_apps.composition import build_bimanual_execution_flow
    from retargeting_apps.main import compose_hydra_base_config
    from teleoperation.config import load_teleoperation_mode_config
    from teleoperation.output import QposOutputFilter

    backend = getattr(args, 'backend', 'ros')
    if backend not in ('preview', 'ros'):
        raise ValueError('backend must be preview or ros')
    config = compose_hydra_base_config([
        'app=teleop_exe', 'teleoperation_modes=bimanual_quest', 'backends=kinematic'])
    config['backend']['command_hz'] = args.command_hz
    config['bimanual'].update(duration=args.duration, left_hand_enabled=False, right_hand_enabled=False)
    config['input']['serial'] = args.serial
    config['viewer'].update(enabled=args.viewer, port=args.viewer_port, wait_for_client=False)
    flow = build_bimanual_execution_flow(config)
    if backend == 'ros' and args.publish_hz is not None:
        horizon = (flow.period if args.interpolation_horizon_ms is None else
                   args.interpolation_horizon_ms / 1000.0)
        if horizon >= flow.timeout:
            raise ValueError('Interpolation horizon must be shorter than the target timeout')
    retargeters = (flow.pipeline.left_retargeter, flow.pipeline.right_retargeter)
    for retargeter in retargeters:
        if tuple(retargeter.robot_config.actuated_joints) != PROFILE_NAMES:
            raise ValueError('Joint script requires J1-J6 followed by joint_0-joint_15 per arm')
    mode = load_teleoperation_mode_config(config['teleoperation_mode'])
    alpha = float(config['bimanual'].get('output', {}).get(
        'arm_smoothing_alpha', mode.output.smoothing_alpha))
    mode = replace(mode, output=replace(mode.output, smoothing_alpha=alpha))
    flow.arm_output_filters = tuple(QposOutputFilter(r.qpos_init[:6], mode) for r in retargeters)
    if backend == 'ros':
        flow.backend_factory = lambda: JointConnection(
            flow.initial_qpos, flow.period, publish_hz=args.publish_hz,
            output_interpolation=args.output_interpolation,
            interpolation_horizon=(None if args.interpolation_horizon_ms is None else
                                   args.interpolation_horizon_ms / 1000.0),
            target_timeout=flow.timeout, namespace=getattr(args, 'namespace', 'crx5ia'))
    return flow, config


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('preview', 'ros'), default='ros',
                        help='preview uses no ROS; ros publishes joint targets (default: ros)')
    parser.add_argument('--namespace', default='crx5ia',
                        help='ROS namespace for joint_targets and joint_states (default: crx5ia)')
    parser.add_argument('--command-hz', type=float, default=20.0, help='Maximum target rate (default: 20)')
    parser.add_argument('--publish-hz', type=float, default=None,
                        help='Independent ROS output rate, e.g. 100; omitted keeps direct output')
    parser.add_argument('--output-interpolation', choices=['linear', 'cubic'], default='cubic',
                        help='Curve used with --publish-hz (default: cubic)')
    parser.add_argument('--interpolation-horizon-ms', type=float, default=None,
                        help='Target arrival horizon; defaults to 1000 / command-hz, not publish-hz')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Seconds after calibration; 0 runs until Ctrl+C')
    parser.add_argument('--serial', default=None, help='ADB headset serial')
    parser.add_argument('--viewer', action=argparse.BooleanOptionalAction, default=True,
                        help='Show the existing dual-arm web viewer (default: enabled)')
    parser.add_argument('--viewer-port', type=int, default=9219, help='Web viewer port (default: 9219)')
    args = parser.parse_args(argv)
    if not args.namespace.strip('/'):
        parser.error('--namespace must not be empty')
    if not math.isfinite(args.command_hz) or args.command_hz <= 0:
        parser.error('--command-hz must be finite and positive')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if args.publish_hz is not None and (not math.isfinite(args.publish_hz) or not 0 < args.publish_hz <= 500):
        parser.error('--publish-hz must be finite and in (0, 500]')
    if args.interpolation_horizon_ms is not None:
        if not math.isfinite(args.interpolation_horizon_ms) or args.interpolation_horizon_ms <= 0:
            parser.error('--interpolation-horizon-ms must be finite and positive')
        if args.publish_hz is None:
            parser.error('--interpolation-horizon-ms requires --publish-hz')
    if not 1 <= args.viewer_port <= 65535:
        parser.error('--viewer-port must be between 1 and 65535')
    visualizer = None
    try:
        flow, config = build_flow(args)
        if args.viewer:
            from retargeting_apps.visualization.execution.manager import create_optional_execution_visualizer

            visualizer = create_optional_execution_visualizer(config, flow)
        output = (f'/{args.namespace.strip("/") or "crx5ia"}/joint_targets'
                  if args.backend == 'ros' else 'local preview (no ROS output)')
        print(f'Quest joint teleoperation -> {output}; both arms, no LEAP output.', flush=True)
        flow.run()
    except KeyboardInterrupt:
        return 0
    except (ImportError, RuntimeError, ValueError) as exc:
        print(f'Joint teleoperation failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if visualizer is not None:
            visualizer.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
