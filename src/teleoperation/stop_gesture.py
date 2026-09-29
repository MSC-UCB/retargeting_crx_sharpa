"""Backend-free, latched dual thumb/ring pinch confirmation on fresh frames."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class StopGestureConfig:
    hold_s: float = 2.
    pinch_enter_ratio: float = .20
    pinch_exit_ratio: float = .30
    other_tip_margin_ratio: float = .08
    fist_pip_angle_deg: float = 110.
    max_sample_age_s: float = .15
    max_gap_s: float = .15

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('Stop gesture settings must be finite and positive')
        if self.pinch_exit_ratio <= self.pinch_enter_ratio:
            raise ValueError('Pinch exit threshold must exceed entry threshold')
        if self.fist_pip_angle_deg >= 180:
            raise ValueError('Fist angle must be below 180 degrees')


class GestureStopDetector:
    """A candidate latches immediately; any later interruption ends the session."""
    def __init__(self, config=None):
        self.config = config or StopGestureConfig()
        self.state = 'RUNNING'
        self.reason = ''
        self._pinched = [False, False]
        self._sequence = self._received_ns = self._first_ns = None

    @property
    def latched(self):
        return self.state != 'RUNNING'

    @property
    def finished(self):
        return self.state in ('EXIT_CONFIRMED', 'EXIT_UNCONFIRMED')

    @property
    def elapsed(self):
        return 0. if self._first_ns is None else (self._received_ns-self._first_ns)/1e9

    def _invalid(self, reason):
        self._pinched = [False, False]
        if self.latched and not self.finished:
            self.state, self.reason = 'EXIT_UNCONFIRMED', reason
        return self.state

    def interrupt(self, reason):
        """End an in-progress confirmation without releasing its latch."""
        return self._invalid(reason)

    def poll(self, now):
        if self.finished:
            return self.state
        if not math.isfinite(now):
            return self._invalid('Invalid monotonic clock')
        if self._received_ns is not None and (
                now < self._received_ns/1e9 or now-self._received_ns/1e9 > self.config.max_gap_s):
            return self._invalid('No fresh gesture frame within the allowed gap')
        return self.state

    def _matches(self, hand, was_pinched):
        p, pose = hand.keypoints_wrist, hand.wrist_pose_sensor
        if (p is None or pose is None or np.shape(p) != (21, 3) or np.shape(pose) != (4, 4)
                or not np.isfinite(p).all() or not np.isfinite(pose).all()):
            return False
        width = np.linalg.norm(p[5]-p[17])
        if width < 1e-6:
            return False
        for start in (1, 5, 9, 13, 17):
            if np.any(np.linalg.norm(np.diff(p[start:start+4], axis=0), axis=1) < 1e-6):
                return False
        distance = np.linalg.norm(p[4]-p[16])/width
        if was_pinched:
            if distance >= self.config.pinch_exit_ratio:
                return False
        elif distance > self.config.pinch_enter_ratio:
            return False
        others = np.linalg.norm(p[[8, 12, 20]]-p[4], axis=1)/width
        if np.any(others < distance+self.config.other_tip_margin_ratio):
            return False
        angles = []
        for root in (5, 9, 17):
            a, b = p[root]-p[root+1], p[root+2]-p[root+1]
            cosine = np.dot(a, b)/(np.linalg.norm(a)*np.linalg.norm(b))
            angles.append(np.degrees(np.arccos(np.clip(cosine, -1., 1.))))
        return not all(angle < self.config.fist_pip_angle_deg for angle in angles)

    def update(self, sample, now):
        self.poll(now)
        if self.finished:
            return self.state
        hands = (sample.left, sample.right)
        sequence = sample.source_index
        stamps = [getattr(h.raw, 'received_monotonic_ns', None) for h in hands]
        if (not sample.complete or sequence is None or any(h.source_index != sequence for h in hands)
                or any(not isinstance(s, (int, np.integer)) for s in stamps)
                or stamps[0] != stamps[1]):
            return self._invalid('Missing or unsynchronized hands')
        stamp = stamps[0]
        age = now-stamp/1e9
        if not math.isfinite(age) or not 0 <= age <= self.config.max_sample_age_s:
            return self._invalid('Stale or future gesture frame')
        if self._sequence is not None:
            if sequence < self._sequence or stamp < self._received_ns:
                return self._invalid('Gesture frame moved backwards')
            if sequence == self._sequence:
                if stamp != self._received_ns:
                    return self._invalid('Repeated sequence with a different timestamp')
                return self.state
            if stamp <= self._received_ns:
                return self._invalid('Gesture timestamp did not advance')
            if (stamp-self._received_ns)/1e9 > self.config.max_gap_s:
                self._invalid('Gap between gesture frames')
                if self.finished:
                    return self.state
        self._sequence, self._received_ns = sequence, stamp
        self._pinched = [self._matches(h, active) for h, active in zip(hands, self._pinched)]
        if all(self._pinched):
            if not self.latched:
                self.state, self._first_ns = 'STOP_LATCHED', stamp
                self.reason = 'Dual thumb/ring pinch: output latched'
            if self.elapsed >= self.config.hold_s:
                self.state, self.reason = 'EXIT_CONFIRMED', 'Dual thumb/ring pinch confirmed'
        elif self.latched:
            self._invalid('Pinch released or gesture geometry invalid')
        return self.state
