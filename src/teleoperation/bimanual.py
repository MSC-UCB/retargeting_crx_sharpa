"""Synchronized bimanual Quest-to-robot retargeting primitives."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from retargeting.core import Retargeter
from retargeting.core.types import RetargetingHandObservation
from teleoperation.observation_mapping import HandObservationMapper
from teleoperation.types import BimanualSensorHandSample


@dataclass(frozen=True)
class BimanualRetargetedFrame:
    """The two independently solved arms from one synchronized Quest frame."""

    left_qpos: np.ndarray
    right_qpos: np.ndarray
    left_observation: RetargetingHandObservation
    right_observation: RetargetingHandObservation

    def __post_init__(self) -> None:
        for name in ("left_qpos", "right_qpos"):
            value = np.asarray(getattr(self, name), dtype=float).copy()
            value.setflags(write=False)
            object.__setattr__(self, name, value)

    @property
    def qpos(self) -> np.ndarray:
        """Return the concatenated command in left-then-right order."""
        return np.concatenate((self.left_qpos, self.right_qpos))


class BimanualRetargetingPipeline:
    """Map and solve both hands while keeping each robot's solver independent."""

    def __init__(
        self,
        *,
        left_mapper: HandObservationMapper,
        right_mapper: HandObservationMapper,
        left_retargeter: Retargeter,
        right_retargeter: Retargeter,
    ) -> None:
        self.left_mapper = left_mapper
        self.right_mapper = right_mapper
        self.left_retargeter = left_retargeter
        self.right_retargeter = right_retargeter
        self.pair_solver = None  # Optional peer owned and started by the execution flow.
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    def initialize(self, sample: BimanualSensorHandSample, left_qpos: np.ndarray, right_qpos: np.ndarray) -> bool:
        """Capture both relative wrist origins from one synchronized sample."""
        left_ok = self.left_mapper.initialize(sample.left, np.asarray(left_qpos, dtype=float))
        right_ok = self.right_mapper.initialize(sample.right, np.asarray(right_qpos, dtype=float))
        self._initialized = left_ok and right_ok
        return self._initialized

    def reset(self) -> None:
        self.left_mapper.reset()
        self.right_mapper.reset()
        self.left_retargeter.reset(self.left_retargeter.qpos_init)
        self.right_retargeter.reset(self.right_retargeter.qpos_init)
        self._initialized = False

    def step(self, sample: BimanualSensorHandSample) -> BimanualRetargetedFrame | None:
        """Return both arm commands, or ``None`` until both hands are available."""
        if not self._initialized:
            raise RuntimeError("BimanualRetargetingPipeline must be initialized before step().")
        left = self.left_mapper.map(sample.left)
        right = self.right_mapper.map(sample.right)
        if left is None or right is None:
            return None
        left = RetargetingHandObservation(
            keypoints_wrist=left.keypoints_wrist,
            wrist_pose_world=left.wrist_pose_world,
            timestamp=left.timestamp,
            handedness="left",
            keypoint_2d=left.keypoint_2d,
            raw=left.raw,
        )
        right = RetargetingHandObservation(
            keypoints_wrist=right.keypoints_wrist,
            wrist_pose_world=right.wrist_pose_world,
            timestamp=right.timestamp,
            handedness="right",
            keypoint_2d=right.keypoint_2d,
            raw=right.raw,
        )
        if self.pair_solver is None:
            left_result = self.left_retargeter.solve(left, previous_qpos=self.left_retargeter.previous_qpos)
            right_result = self.right_retargeter.solve(right, previous_qpos=self.right_retargeter.previous_qpos)
        else:
            left_result, right_result = self.pair_solver.solve(
                (left, right), (self.left_retargeter.previous_qpos, self.right_retargeter.previous_qpos),
                frame_id=sample.source_index)
        self.left_retargeter.previous_qpos = left_result.qpos.copy()
        self.right_retargeter.previous_qpos = right_result.qpos.copy()
        return BimanualRetargetedFrame(
            left_qpos=left_result.qpos,
            right_qpos=right_result.qpos,
            left_observation=left,
            right_observation=right,
        )


__all__ = ["BimanualRetargetedFrame", "BimanualRetargetingPipeline"]
