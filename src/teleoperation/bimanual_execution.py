"""Flat lifecycle owner for synchronized Quest solving and robot output."""
import time
import threading
import numpy as np

from teleoperation.bimanual import BimanualRetargetedFrame
from teleoperation.output import QposCommandLimiter, QposOutputFilter


class BimanualExecutionFlow:
    def __init__(self, *, source, pipeline, initial_qpos, backend_factory=None,
                 observer=None, command_hz=20., timeout=.25, duration=0.,
                 arm_output_filters: tuple[QposOutputFilter, QposOutputFilter] | None = None,
                 hand_output_filters: tuple[QposOutputFilter, QposOutputFilter] | None = None,
                 robot_dofs=(22, 22), arm_dofs=(6, 6),
                 command_limiters: tuple[QposCommandLimiter, QposCommandLimiter] | None = None,
                 stop_gesture=None):
        if not np.isfinite(command_hz) or command_hz <= 0:
            raise ValueError("command_hz must be positive")
        if not np.isfinite(duration) or duration < 0:
            raise ValueError("duration must be finite and non-negative")
        self.duration = float(duration)
        self._duration_expired = threading.Event()
        self._duration_timer = None
        self.source, self.pipeline = source, pipeline
        if (len(robot_dofs) != 2 or len(arm_dofs) != 2
                or any(isinstance(n, bool) or not isinstance(n, int) or n <= 0 for n in robot_dofs)
                or any(isinstance(a, bool) or not isinstance(a, int) or not 0 <= a <= n
                       for a, n in zip(arm_dofs, robot_dofs))):
            raise ValueError("Expected two positive robot dimensions and valid arm dimensions")
        self.robot_dofs, self.arm_dofs = tuple(robot_dofs), tuple(arm_dofs)
        self.robot_slices = (slice(0, robot_dofs[0]), slice(robot_dofs[0], sum(robot_dofs)))
        self.initial_qpos = np.asarray(initial_qpos, dtype=float).copy()
        if self.initial_qpos.shape != (sum(robot_dofs),) or not np.isfinite(self.initial_qpos).all():
            raise ValueError(f"expected {sum(robot_dofs)} finite initial positions")
        for label, filters, sizes in (
            ("arm", arm_output_filters, arm_dofs),
            ("hand", hand_output_filters, [n - a for n, a in zip(robot_dofs, arm_dofs)]),
            ("command", command_limiters, robot_dofs),
        ):
            if filters is not None:
                if len(filters) != 2:
                    raise ValueError(f"{label} filters must contain left and right filters")
                if any(f.previous_qpos.shape != (n,) for f, n in zip(filters, sizes)):
                    raise ValueError(f"{label} filter dimensions must match the configured joints")
        self.command_limiters = command_limiters
        self.arm_output_filters = arm_output_filters
        self.hand_output_filters = hand_output_filters
        self.backend_factory, self.observer = backend_factory, observer
        self.period, self.timeout = 1. / command_hz, timeout
        self.backend = None
        self.last_sequence = None
        self.last_command_at = None
        self.started_at = None
        self.tracking_paused = False
        self._recovery_started = None
        self._recovery_sequence = None
        self._last_qpos = self.initial_qpos.copy()
        self.command_count = self.stale_count = 0
        self.last_solve_ms = 0.0
        self.pair_solver = None
        self._closed = False
        self.stop_gesture = stop_gesture
        self.stop_latched = threading.Event()
        self.stop_reason = None
        self.stop_observer = None
        self._last_stop_report = None

    def latch_gesture_stop(self):
        """Gate output before cancelling workers; never resume this session."""
        if self.stop_latched.is_set():
            return
        self.stop_latched.set()
        self.stop_reason = 'Dual thumb/ring pinch'
        try:
            if self.backend is not None:
                self.backend.request_stop(self.stop_reason)
        finally:
            if self.pair_solver is not None:
                self.pair_solver.cancel()

    def _check_stop_gesture(self, sample=None):
        if self.stop_gesture is None:
            return False
        now = time.monotonic()
        detector = self.stop_gesture
        if sample is None:
            detector.poll(now)
        else:
            detector.update(sample, now)
        if detector.latched:
            self.latch_gesture_stop()
            report = (detector.state, min(int(detector.elapsed*4), int(detector.config.hold_s*4)))
            if report != self._last_stop_report:
                self._last_stop_report = report
                message = (f'{detector.state}: {detector.elapsed:.2f}/{detector.config.hold_s:g}s; '
                           f'{detector.reason}. New motion targets disabled; robot standstill not verified.')
                print(message, flush=True)
                if self.stop_observer is not None:
                    self.stop_observer(message)
        return detector.latched

    def start_solver(self):
        """Start optional solver peers before opening input or constructing a backend."""
        if self._closed:
            raise RuntimeError('Execution flow is closed')
        if self.pair_solver is not None:
            self.pair_solver.start()
            self.pipeline.pair_solver = self.pair_solver

    def _reset_output_filters(self, seed: np.ndarray) -> None:
        """Seed arm and hand smoothing from the current measured robot pose."""
        for i, joints in enumerate(self.robot_slices):
            qpos, arm_dof = seed[joints], self.arm_dofs[i]
            if self.arm_output_filters is not None:
                self.arm_output_filters[i].reset(qpos[:arm_dof])
            if self.hand_output_filters is not None:
                self.hand_output_filters[i].reset(qpos[arm_dof:])
            if self.command_limiters is not None:
                self.command_limiters[i].reset(qpos)

    def _filter_commands(self, result: BimanualRetargetedFrame) -> BimanualRetargetedFrame:
        """Filter and optionally limit commands in each robot's configured order."""
        if self.arm_output_filters is None and self.hand_output_filters is None and self.command_limiters is None:
            return result
        commands = []
        for i, raw in enumerate((result.left_qpos, result.right_qpos)):
            qpos = np.asarray(raw, dtype=float).copy()
            if qpos.shape != (self.robot_dofs[i],) or not np.isfinite(qpos).all():
                raise ValueError("bimanual filtering requires finite configured robot dimensions")
            arm_dof = self.arm_dofs[i]
            if self.arm_output_filters is not None:
                qpos[:arm_dof] = self.arm_output_filters[i].apply(qpos[:arm_dof])
            if self.hand_output_filters is not None:
                qpos[arm_dof:] = self.hand_output_filters[i].apply(qpos[arm_dof:])
            if self.command_limiters is not None:
                qpos = self.command_limiters[i].apply(qpos)
                # Keep smoothing state at the limited command, preventing windup.
                if self.arm_output_filters is not None:
                    self.arm_output_filters[i].reset(qpos[:arm_dof])
                if self.hand_output_filters is not None:
                    self.hand_output_filters[i].reset(qpos[arm_dof:])
            commands.append(qpos)
        self.pipeline.left_retargeter.previous_qpos = commands[0].copy()
        self.pipeline.right_retargeter.previous_qpos = commands[1].copy()
        return BimanualRetargetedFrame(
            left_qpos=commands[0], right_qpos=commands[1],
            left_observation=result.left_observation, right_observation=result.right_observation,
        )

    def step(self, sample):
        """Send at most one command for each complete synchronized frame."""
        if self._closed:
            raise RuntimeError('Execution flow is closed')
        if self._duration_expired.is_set():
            return None
        if self._check_stop_gesture(sample) or self.stop_latched.is_set():
            return None
        if self.pair_solver is not None and not self.pair_solver.started:
            self.start_solver()
            # Direct step callers must also discard input captured before startup.
            return None
        if not sample.complete or sample.source_index is None:
            self._pause_tracking()
            return None
        if self.last_sequence is not None and sample.source_index <= self.last_sequence:
            return None
        if sample.left.source_index != sample.right.source_index:
            raise ValueError("left and right samples must come from the same Quest frame")
        if self.tracking_paused:
            now = time.monotonic()
            if self._recovery_sequence is not None and sample.source_index <= self._recovery_sequence:
                return None
            self._recovery_sequence = sample.source_index
            if self._recovery_started is None:
                self._recovery_started = now
            if now - self._recovery_started < .3:
                return None
            if self.backend is not None:
                if self.backend.resume_tracking() is False:
                    self._recovery_started = None
                    return None
            if self._duration_expired.is_set() or self.stop_latched.is_set():
                return None
            # Resume setup can block: calibrate using a NEW sample on the next tick.
            self.pipeline.reset()
            self.tracking_paused = False
            self._recovery_started = None
            self.last_sequence = sample.source_index
            self.last_command_at = time.monotonic()
            print("Quest tracking recovered; recalibrating from current robot pose.", flush=True)
            return None
        if not self.pipeline.initialized:
            if self.backend is None and self.backend_factory is not None:
                self.backend = self.backend_factory()
                # Startup may take seconds. Acquire a new frame before calibration.
                return None
            seed = self._last_qpos if self.backend is None else self.backend.get_joint_pos()
            self.pipeline.left_retargeter.reset(seed[self.robot_slices[0]])
            self.pipeline.right_retargeter.reset(seed[self.robot_slices[1]])
            self._last_qpos = np.asarray(seed, dtype=float).copy()
            self._reset_output_filters(seed)
            if not self.pipeline.initialize(sample, seed[self.robot_slices[0]], seed[self.robot_slices[1]]):
                return None
            first_start = self.started_at is None
            if first_start:
                self.started_at = time.monotonic()
            if first_start and self.duration > 0:
                self._duration_timer = threading.Timer(self.duration, self._expire_duration)
                self._duration_timer.daemon = True
                self._duration_timer.start()
                print(f"Quest tracking initialized; automatic stop in {self.duration:g} seconds.", flush=True)
        started = time.monotonic()
        try:
            result = self.pipeline.step(sample)
        except RuntimeError:
            if self._duration_expired.is_set() or self.stop_latched.is_set():
                return None
            if self.pair_solver is not None and self.backend is not None:
                self.backend.request_stop('Parallel retargeting solver failed')
            raise
        self.last_solve_ms = (time.monotonic() - started) * 1000.0
        self.last_sequence = sample.source_index
        if result is None or self._duration_expired.is_set() or self.stop_latched.is_set():
            return None
        # Never refresh an obsolete input into a new ROS command after a slow solve.
        received_ns = getattr(getattr(sample.left, "raw", None), "received_monotonic_ns", None)
        age = time.monotonic() - started if received_ns is None else (time.monotonic_ns() - received_ns) / 1e9
        if age > self.source.max_age_s:
            self.stale_count += 1
            self.pipeline.left_retargeter.previous_qpos = self._last_qpos[self.robot_slices[0]].copy()
            self.pipeline.right_retargeter.previous_qpos = self._last_qpos[self.robot_slices[1]].copy()
            return None
        result = self._filter_commands(result)
        if self.backend is not None:
            if hasattr(self.backend, 'assert_tracking') and self.backend.assert_tracking() is False:
                self._pause_tracking()
                return None
            try:
                if self.stop_latched.is_set() or self._duration_expired.is_set():
                    return None
                self.backend.execute(result.qpos)
            except RuntimeError:
                if self._duration_expired.is_set() or self.stop_latched.is_set():
                    return None
                if hasattr(self.backend, 'assert_tracking') and self.backend.assert_tracking() is False:
                    self._pause_tracking()
                    return None
                raise
        self.command_count += 1
        self._last_qpos = result.qpos.copy()
        self.last_command_at = time.monotonic()
        if self.observer is not None:
            self.observer(result)
        return result

    def _pause_tracking(self):
        self._recovery_started = None
        self._recovery_sequence = None
        if self.started_at is None or self.tracking_paused:
            return
        if self.backend is not None:
            self.backend.pause_tracking()
        self.tracking_paused = True
        print("Quest tracking lost; holding robots, waiting for both hands.", flush=True)

    def _expire_duration(self):
        # Independent of the solver loop: request stop even if a solve is slow.
        self._duration_expired.set()
        if self.pair_solver is not None:
            self.pair_solver.cancel()
        if self.backend is not None:
            self.backend.request_stop("Quest session duration reached")

    def run(self):
        try:
            self.start_solver()
            self.source.open()
            next_command = time.monotonic()
            last_report = 0.
            while True:
                now = time.monotonic()
                if self._duration_expired.is_set():
                    print("Quest session duration reached; stopping teleoperation.", flush=True)
                    return
                self._check_stop_gesture()
                if self.stop_gesture is not None and self.stop_gesture.finished:
                    return
                if (not self.stop_latched.is_set()
                        and self.backend is not None and self.started_at is not None):
                    if now - self.started_at > .5:
                        if self.backend.assert_tracking() is False and not self.tracking_paused:
                            self._pause_tracking()
                    reference = self.last_command_at if self.last_command_at is not None else self.started_at
                    if not self.tracking_paused and now - reference > self.timeout:
                        self._pause_tracking()
                if now >= next_command:
                    try:
                        sample = self.source.read()
                    except StopIteration:
                        if self.stop_gesture is not None and self.stop_latched.is_set():
                            self.stop_gesture.interrupt('Input exhausted during confirmation')
                        return
                    self.step(sample)
                    next_command = max(now + self.period, time.monotonic())
                if now - last_report > 2.:
                    stats = self.source.stats
                    print(f"Quest frames={getattr(stats, 'accepted', 0)} "
                          f"initialized={self.pipeline.initialized} paused={self.tracking_paused} "
                          f"sequence={self.last_sequence} commands={self.command_count} "
                          f"stale={self.stale_count} solve_ms={self.last_solve_ms:.1f}", flush=True)
                    last_report = now
                time.sleep(.002)
        finally:
            self.close()

    def close(self):
        """Stop output first; always reap solver processes and close acquisition."""
        if self._closed:
            return
        self._closed = True
        if self._duration_timer is not None:
            self._duration_timer.cancel()
            self._duration_timer.join()
        try:
            if self.backend is not None:
                self.backend.close()
        finally:
            try:
                if self.pair_solver is not None:
                    self.pair_solver.close()
            finally:
                self.source.close()
