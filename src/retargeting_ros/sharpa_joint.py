"""JointState transport for two Sharpa hands, optionally with both CRX arms."""

import math
import threading
import time

import numpy as np

from teleoperation.backends.base import BackendStepResult
from teleoperation.backends.sharpa_contract import joint_channels, named_positions


class SharpaJointBackend:
    def __init__(self, *, robot_names, initial_qpos, lower, upper, control_period,
                 startup_timeout=5., feedback_timeout=.5, target_timeout=.25,
                 publish_hz=None, interpolation_horizon=None,
                 crx_namespace='crx5ia', sharpa_namespace='sharpa', require_waypoint=False):
        self.channels = joint_channels(robot_names)
        self._size = sum(map(len, robot_names))
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        if (self.lower.shape != (self._size,) or self.upper.shape != (self._size,)
                or not np.isfinite(self.lower).all() or not np.isfinite(self.upper).all()
                or np.any(self.lower > self.upper)):
            raise ValueError('Expected finite model bounds for every joint')
        self._actual = self._validate_command(initial_qpos)
        self._target = self._actual.copy()
        horizon = control_period if interpolation_horizon is None else interpolation_horizon
        for value in (control_period, startup_timeout, feedback_timeout, target_timeout, horizon):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Periods and timeouts must be finite and positive')
        if publish_hz is not None:
            if not math.isfinite(publish_hz) or publish_hz <= 0:
                raise ValueError('publish_hz must be finite and positive')
            if horizon >= target_timeout:
                raise ValueError('Interpolation horizon must be shorter than the target timeout')
        self.control_period = control_period
        self._publish_hz, self._horizon = publish_hz, horizon
        self._output_timer = self._interpolator = self._pending_target = None
        self._last_published = self._actual.copy()
        self._output_generation = 0
        self._target_count = self._publish_count = self._rejected_samples = 0
        self._feedback_timeout, self._target_timeout = feedback_timeout, target_timeout
        self._lock = threading.RLock()
        self._closed = self._stopped = self._paused = False
        self._last_command_at = None
        self._spin_error = None
        self._stop_error = None
        self._output_armed = False
        self._require_waypoint = require_waypoint
        self._feedback = {key: dict(stamp=0, received=0., advanced=0., error='waiting for feedback')
                          for key in self.channels}
        self._publishers = {}
        self._context = self._node = self._executor = self._thread = self._watchdog = None
        try:
            import rclpy
            from rclpy.clock import Clock, ClockType
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from sensor_msgs.msg import JointState
        except ImportError as exc:
            raise RuntimeError('Source ROS Jazzy and use the ROS-compatible project venv') from exc
        self._message_type = JointState
        try:
            self._context = Context()
            rclpy.init(args=[], context=self._context)
            self._node = Node('retargeting_sharpa_joint', context=self._context)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            for key in self.channels:
                namespace = crx_namespace.strip('/') if key == 'arms' else f'{sharpa_namespace.strip("/")}/{key}'
                if not namespace or namespace.startswith('/'):
                    raise ValueError('Namespaces must not be empty')
                target = 'joint_targets' if key == 'arms' else 'joint_command'
                self._publishers[key] = self._node.create_publisher(JointState, f'/{namespace}/{target}', qos)
                self._node.create_subscription(
                    JointState, f'/{namespace}/joint_states',
                    lambda msg, key=key: self._receive_state(key, msg), qos)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._spin, daemon=True, name='sharpa_feedback')
            self._thread.start()
            deadline = time.monotonic() + startup_timeout
            if require_waypoint:
                if 'arms' not in self.channels:
                    raise ValueError('Gesture stop requires CRX arm channels')
                self._verify_waypoint(crx_namespace, deadline)
            while True:
                try:
                    with self._lock:
                        self._check_feedback()
                        self._target = self._validate_command(self._actual)
                        self._reset_output_locked(self._actual)
                    break
                except RuntimeError as exc:
                    if time.monotonic() >= deadline:
                        raise RuntimeError(f'Sharpa startup timeout: {exc}') from exc
                    time.sleep(.01)
            self._watchdog = self._node.create_timer(
                .02, self._watchdog_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            if self._publish_hz is not None:
                self._output_timer = self._node.create_timer(
                    1. / self._publish_hz, self._publish_tick,
                    clock=Clock(clock_type=ClockType.STEADY_TIME), autostart=False)
            self._node.get_logger().info(
                'Sharpa output: ' + ('direct solver targets' if publish_hz is None else
                f'{publish_hz:g} Hz linear interpolation; horizon {horizon * 1000:g} ms'))
            self._output_armed = True
        except BaseException:
            self.close()
            raise

    def _verify_waypoint(self, namespace, deadline):
        """Read the driver's immutable mode before any command (including hold)."""
        from rcl_interfaces.srv import GetParameters

        client = self._node.create_client(
            GetParameters, f'/{namespace.strip("/")}/joint_interpolation/get_parameters')
        future = None
        try:
            while not client.wait_for_service(timeout_sec=max(0., min(.05, deadline-time.monotonic()))):
                if time.monotonic() >= deadline:
                    raise RuntimeError('Gesture stop: CRX parameter service unavailable')
            future = client.call_async(GetParameters.Request(names=['method', 'ruckig_target_mode']))
            while not future.done():
                if time.monotonic() >= deadline:
                    raise RuntimeError('Gesture stop: CRX mode query timed out')
                time.sleep(.005)
            self._check_waypoint_response(future.result())
            self._node.get_logger().info('Gesture stop: CRX ruckig/waypoint verified')
        finally:
            if future is not None and not future.done():
                future.cancel()
            self._node.destroy_client(client)

    @staticmethod
    def _check_waypoint_response(response):
        values = getattr(response, 'values', ())
        # ParameterType.PARAMETER_STRING == 4; keep validation ROS-independent.
        if (len(values) != 2 or any(v.type != 4 for v in values)
                or [v.string_value for v in values] != ['ruckig', 'waypoint']):
            raise RuntimeError('Gesture stop requires CRX method=ruckig, ruckig_target_mode=waypoint')

    def _validate_command(self, qpos):
        values = np.asarray(qpos, dtype=float)
        if values.shape != (self._size,) or not np.isfinite(values).all():
            raise ValueError(f'Expected {self._size} finite joint positions')
        if np.any(values < self.lower - 1e-7) or np.any(values > self.upper + 1e-7):
            raise ValueError('Joint command exceeds model limits')
        return values.copy()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception as exc:
            with self._lock:
                self._spin_error = str(exc)
                self._stopped = True
                self._reset_output_locked(self._last_published)

    def _reset_output_locked(self, seed):
        """Cancel publication and invalidate work computed outside the lock."""
        self._output_generation += 1
        self._pending_target = self._last_command_at = None
        self._last_published = np.asarray(seed, dtype=float).copy()
        if self._output_timer is not None:
            self._output_timer.cancel()
        if self._publish_hz is not None:
            from retargeting_ros.joint_interpolation import JointCommandInterpolator

            self._interpolator = JointCommandInterpolator(seed, 'linear', num_joints=self._size)

    def _publish_tick(self):
        """Sample all joints at one time; publish each channel once per tick."""
        with self._lock:
            if (self._closed or self._stopped or self._paused
                    or self._last_command_at is None or self._publish_hz is None):
                return
            try:
                self._check_feedback()
                now = time.monotonic()
                if now - self._last_command_at > self._target_timeout:
                    raise RuntimeError('No fresh retargeting target')
            except RuntimeError as exc:
                self._pause_locked()
                self._node.get_logger().warning(str(exc))
                return
            pending, self._pending_target = self._pending_target, None
            interpolator, generation = self._interpolator, self._output_generation

        # Use the same interpolation helper as CRX. Do not hold up feedback or stop.
        try:
            if pending is not None:
                target, received_at = pending
                interpolator.set_target(target, received_at, self._horizon, now)
            values = interpolator.sample(now)
            self._validate_command(values)
        except (ValueError, FloatingPointError) as exc:
            with self._lock:
                if generation != self._output_generation:
                    return
                self._rejected_samples += 1
                self._pause_locked()
            self._node.get_logger().warning(f'Sharpa interpolation rejected: {exc}')
            return

        with self._lock:
            if (generation != self._output_generation or self._closed
                    or self._stopped or self._paused):
                return
            try:
                self._check_feedback()
                published_at = time.monotonic()
                if (published_at - self._last_command_at > self._target_timeout
                        or published_at - now >= 1. / self._publish_hz):
                    self._rejected_samples += 1
                    raise RuntimeError('Sharpa interpolation sample missed its deadline')
            except RuntimeError as exc:
                self._pause_locked()
                self._node.get_logger().warning(str(exc))
                return
            self._publish(values)
            interpolator.record_published(published_at, values)
            self._last_published = values.copy()
            self._publish_count += 1

    def _receive_state(self, key, message):
        with self._lock:
            state = self._feedback[key]
            try:
                names, indices = self.channels[key]
                values = named_positions(message.name, message.position, names)
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                age = (self._node.get_clock().now().nanoseconds - stamp) / 1e9
                if stamp <= 0 or not -.02 <= age <= self._feedback_timeout or stamp < state['stamp']:
                    raise ValueError('stale, future or backwards feedback timestamp')
            except (ValueError, TypeError) as exc:
                state['error'] = str(exc)
                return
            now = time.monotonic()
            if stamp > state['stamp']:
                state['advanced'] = now
            state.update(stamp=stamp, received=now, error=None)
            self._actual[indices] = values

    def _check_feedback(self):
        if self._stopped:
            raise RuntimeError(f'Sharpa output stopped: {self._spin_error or "stop requested"}')
        if (self._context is None or self._thread is None or self._node is None
                or not self._context.ok() or not self._thread.is_alive()):
            raise RuntimeError('ROS executor unavailable')
        now, ros_now = time.monotonic(), self._node.get_clock().now().nanoseconds
        for key, state in self._feedback.items():
            if state['error']:
                raise RuntimeError(f'{key}: {state["error"]}')
            if (now - state['received'] > self._feedback_timeout
                    or now - state['advanced'] > self._feedback_timeout
                    or not -.02 <= (ros_now - state['stamp']) / 1e9 <= self._feedback_timeout):
                raise RuntimeError(
                    f'{key}: stale or frozen feedback '
                    f'(received={now - state["received"]:.3f}s, '
                    f'advanced={now - state["advanced"]:.3f}s, '
                    f'stamp_age={(ros_now - state["stamp"]) / 1e9:.3f}s)'
                )
            if self._publishers[key].get_subscription_count() == 0:
                raise RuntimeError(f'{key}: command subscriber missing')

    def _publish(self, values):
        # Validate all groups before any publication. Topics are not atomic transport.
        values = self._validate_command(values)
        stamp = self._node.get_clock().now().to_msg()
        messages = []
        for key, (names, indices) in self.channels.items():
            msg = self._message_type()
            msg.header.stamp = stamp
            msg.name, msg.position = list(names), values[indices].tolist()
            messages.append((key, msg))
        for key, msg in messages:
            self._publishers[key].publish(msg)

    def get_joint_pos(self):
        with self._lock:
            self._check_feedback()
            return self._actual.copy()

    def get_target_joint_pos(self):
        with self._lock:
            return self._target.copy()

    def execute(self, qpos):
        values = self._validate_command(qpos)
        with self._lock:
            self._check_feedback()
            if self._paused:
                raise RuntimeError('Output paused; recalibrate before resuming')
            received_at = time.monotonic()
            if self._publish_hz is None:
                self._publish(values)
                self._last_published = values.copy()
                self._publish_count += 1
            else:
                self._pending_target = (values.copy(), received_at)
                if self._output_timer.is_canceled():
                    self._output_timer.reset()
            self._target = values
            self._last_command_at = received_at
            self._target_count += 1
            return BackendStepResult(command_qpos=values, actual_qpos=self._actual.copy(),
                                     diagnostics={'targets': self._target_count,
                                                  'publications': self._publish_count})

    def _pause_locked(self, *, strict=False):
        if self._paused:
            return
        self._paused = True
        self._reset_output_locked(self._actual)
        if not getattr(self, '_output_armed', True):
            return
        try:
            self._check_feedback()
            self._publish(self._actual)  # One measured hold target, never a repeated stale goal.
            self._target = self._actual.copy()
        except (RuntimeError, ValueError):
            if strict:
                raise
            pass  # Missing feedback: stop sending; drivers own their timeout policy.

    def _watchdog_tick(self):
        with self._lock:
            if self._closed or self._stopped or self._last_command_at is None:
                return
            try:
                self._check_feedback()
                if time.monotonic() - self._last_command_at > self._target_timeout:
                    raise RuntimeError('No fresh retargeting target')
            except RuntimeError as exc:
                self._pause_locked()
                self._node.get_logger().warning(str(exc))

    def pause_tracking(self):
        with self._lock:
            self._pause_locked()

    def resume_tracking(self):
        with self._lock:
            try:
                self._check_feedback()
                self._validate_command(self._actual)
            except (RuntimeError, ValueError):
                return False
            self._target = self._actual.copy()
            self._reset_output_locked(self._actual)
            self._paused = False
            return True

    def assert_tracking(self):
        with self._lock:
            if self._stopped:
                raise RuntimeError(f'Output stopped: {self._spin_error or "stop requested"}')
            try:
                self._check_feedback()
            except RuntimeError:
                self._pause_locked()
                return False
            return not self._paused

    def request_stop(self, reason):
        with self._lock:
            if self._stopped:
                if getattr(self, '_require_waypoint', False) and self._spin_error:
                    raise RuntimeError(f'Gesture stop: ROS executor failed: {self._spin_error}')
                return
            try:
                # Gesture sessions must report inability to deliver the hold.
                if getattr(self, '_require_waypoint', False):
                    self._check_feedback()
                self._pause_locked(strict=getattr(self, '_require_waypoint', False))
            except Exception as exc:
                self._stop_error = str(exc)
                raise
            finally:
                self._stopped = True
                self._reset_output_locked(self._last_published)

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
        try:
            with self._lock:
                try:
                    self._pause_locked()
                finally:
                    self._stopped = True
                    if self._watchdog is not None:
                        self._watchdog.cancel()
        finally:
            try:
                if self._executor is not None:
                    self._executor.shutdown()
            finally:
                try:
                    if self._thread is not None:
                        self._thread.join()
                finally:
                    try:
                        if self._node is not None:
                            self._node.destroy_node()
                    finally:
                        if self._context is not None:
                            self._context.try_shutdown()
        error = getattr(self, '_stop_error', None) or self._spin_error
        if getattr(self, '_require_waypoint', False) and error:
            raise RuntimeError(f'Gesture session stop failed: {error}')
