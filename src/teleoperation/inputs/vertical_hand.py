"""Finite, clock-driven bilateral wrist translation with a fixed hand gesture."""

from dataclasses import replace
import math
import time
from types import SimpleNamespace

import numpy as np

from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput
from teleoperation.types import BimanualSensorHandSample


class VerticalBimanualInput:
    """Move from zero to stroke and back; the flow supplies calibrated elapsed time.

    sensor_axes express world-up in each input frame. acknowledge is called only
    after a complete result has passed freshness checks and output submission.
    """

    def __init__(self, *, stroke=.20, period=8., cycles=3, hold_time=1.,
                 settle_timeout=10., max_age_s=.15):
        if any(not math.isfinite(v) or v <= 0 for v in
               (stroke, period, hold_time, settle_timeout, max_age_s)):
            raise ValueError('Stroke, period and timeouts must be finite and positive')
        if isinstance(cycles, bool) or not isinstance(cycles, int) or cycles < 1:
            raise ValueError('cycles must be a positive integer')
        if hold_time >= settle_timeout:
            raise ValueError('hold_time must be shorter than settle_timeout')
        if not math.isfinite(period * cycles):
            raise ValueError('Total motion duration must be finite')
        self.stroke, self.period, self.cycles = stroke, period, cycles
        self.hold_time, self.settle_timeout, self.max_age_s = hold_time, settle_timeout, max_age_s
        self.motion_duration = period * cycles
        self.sensor_axes = (np.array([0., 0., 1.]), np.array([0., 0., 1.]))
        self.elapsed = lambda: None  # Bound by the application; None means not calibrated.
        self._gesture = SyntheticBimanualInput(frames=1).read()
        self.keypoints = (self._gesture.left.keypoints_wrist, self._gesture.right.keypoints_wrist)
        self.max_ack_gap = max_age_s
        self.stats = SimpleNamespace(accepted=0)
        self.done = False
        self.last_elapsed = 0.
        self._settled_since = self._last_ack = None

    def open(self):
        self.stats.accepted = 0
        self.done = False
        self.last_elapsed = 0.
        self._settled_since = self._last_ack = None

    def displacement(self, elapsed):
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('Elapsed time must be finite and nonnegative')
        if elapsed >= self.motion_duration:
            return 0.
        phase = (elapsed % self.period) / self.period
        u = 2 * phase if phase <= .5 else 2 * (1 - phase)
        return self.stroke * u**3 * (10 + u * (-15 + 6 * u))

    def read(self):
        if self.done:
            raise StopIteration
        elapsed = self.elapsed()
        self.last_elapsed = 0. if elapsed is None else elapsed
        offset = self.displacement(self.last_elapsed)
        if self.last_elapsed > self.motion_duration + self.settle_timeout:
            raise RuntimeError('Vertical demo did not settle before its timeout')
        now_ns = time.monotonic_ns()
        index = self.stats.accepted
        self.stats.accepted += 1
        hands = []
        for hand, axis, points in zip((self._gesture.left, self._gesture.right),
                                     self.sensor_axes, self.keypoints):
            pose = np.eye(4)
            pose[:3, 3] = np.asarray(axis) * offset
            hands.append(replace(hand, keypoints_wrist=points,
                                 wrist_pose_sensor=pose, timestamp=now_ns / 1e9,
                                 source_index=index,
                                 raw=SimpleNamespace(received_monotonic_ns=now_ns)))
        return BimanualSensorHandSample(*hands)

    def acknowledge(self, *, reached):
        """Finish after continuous accepted endpoint targets and settled feedback."""
        elapsed = self.last_elapsed
        if elapsed < self.motion_duration:
            return
        if not reached or (self._last_ack is not None
                           and elapsed - self._last_ack > self.max_ack_gap):
            self._settled_since = None
        self._last_ack = elapsed
        if reached:
            if self._settled_since is None:
                self._settled_since = elapsed
            self.done = elapsed - self._settled_since >= self.hold_time

    def close(self):
        pass
