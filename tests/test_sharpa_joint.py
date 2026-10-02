"""Headless command routing, freshness, watchdog and lifecycle checks."""

import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from retargeting_ros.sharpa_joint import SharpaJointBackend
from teleoperation.backends.sharpa_contract import joint_channels, sharpa_names


def robot_names(arms=True):
    arm = tuple(f'J{i}' for i in range(1, 7)) if arms else ()
    return tuple(arm + sharpa_names(side) for side in ('left', 'right'))


@pytest.fixture
def backend():
    b = SharpaJointBackend.__new__(SharpaJointBackend)
    b.channels = joint_channels(robot_names())
    b._size = 56
    b.lower, b.upper = np.full(56, -2.), np.full(56, 2.)
    b._actual = b._target = np.zeros(56)
    b._lock = threading.RLock()
    b._closed = b._stopped = b._paused = False
    b._spin_error = b._last_command_at = None
    b._publish_hz, b._horizon = None, .05
    b._output_timer = b._interpolator = b._pending_target = None
    b._last_published = b._actual.copy()
    b._output_generation = 0
    b._target_count = b._publish_count = b._rejected_samples = 0
    b._feedback_timeout, b._target_timeout = .5, .25
    b._feedback = {key: dict(stamp=0, received=0., advanced=0., error='waiting') for key in b.channels}
    b.sent = {key: [] for key in b.channels}
    b._publishers = {key: NS(publish=b.sent[key].append, get_subscription_count=lambda: 1) for key in b.channels}
    b._context = NS(ok=lambda: True)
    b._thread = NS(is_alive=lambda: True)
    b._node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000,
                                                    to_msg=lambda: NS(sec=100, nanosec=0))),
                 get_logger=lambda: NS(warning=lambda message: None))
    b._message_type = lambda: NS(header=NS())
    b.control_period = .05
    return b


def feedback(b, key, *, stamp=100, reverse=False, value=.1):
    names = list(b.channels[key][0])
    values = [value + i / 1000 for i in range(len(names))]
    if reverse:
        names.reverse()
        values.reverse()
    return NS(name=names, position=values, header=NS(stamp=NS(sec=stamp, nanosec=0)))


def ready(b):
    for key in b.channels:
        b._receive_state(key, feedback(b, key, reverse=True))


@pytest.mark.parametrize('arms', [False, True])
def test_name_mapping_round_trip_and_arm_modes(arms):
    names = robot_names(arms)
    # Deliberately reorder profile fingers; transport stays canonical by name.
    a = 6 if arms else 0
    names = tuple(n[:a] + tuple(reversed(n[a:])) for n in names)
    channels = joint_channels(names)
    assert ('arms' in channels) is arms
    covered = np.concatenate([indices for _, indices in channels.values()])
    assert sorted(covered) == list(range(sum(map(len, names))))
    for side, offset in (('left', 0), ('right', len(names[0]))):
        expected, indices = channels[f'{side}_hand']
        assert tuple(names[0 if side == 'left' else 1][i-offset] for i in indices) == expected


def test_44_dim_leap_cannot_be_mistaken_for_44_dim_sharpa():
    leap = tuple(f'J{i}' for i in range(1, 7)) + tuple(f'joint_{i}' for i in range(16))
    with pytest.raises(ValueError, match='profile joint names'):
        joint_channels((leap, leap))


def test_feedback_name_order_and_three_topic_output(backend):
    b = backend
    ready(b)
    measured = b.get_joint_pos()
    assert measured[28] == pytest.approx(.106)
    assert measured[6] == measured[34] == .1
    target = np.arange(56) / 100.
    result = b.execute(target)
    np.testing.assert_array_equal(result.command_qpos, target)
    for key, (names, indices) in b.channels.items():
        msg = b.sent[key][0]
        assert msg.name == list(names)
        np.testing.assert_array_equal(msg.position, target[indices])


@pytest.mark.parametrize('kind', ['nan', 'length', 'limits'])
def test_bad_command_is_rejected_before_any_topic(backend, kind):
    ready(backend)
    q = np.zeros(56)
    if kind == 'nan':
        q[-1] = np.nan
    elif kind == 'length':
        q = q[:-1]
    else:
        q[-1] = 3
    with pytest.raises(ValueError):
        backend.execute(q)
    assert all(not msgs for msgs in backend.sent.values())


@pytest.mark.parametrize('kind', ['missing', 'duplicate', 'unknown', 'nan', 'old', 'future', 'backwards'])
def test_bad_hand_feedback_blocks_all_output(backend, kind):
    b = backend
    ready(b)
    msg = feedback(b, 'right_hand')
    if kind == 'missing':
        msg.name.pop()
    elif kind == 'duplicate':
        msg.name[-1] = msg.name[0]
    elif kind == 'unknown':
        msg.name[-1] = 'right_not_a_joint'
    elif kind == 'nan':
        msg.position[-1] = np.nan
    elif kind == 'future':
        msg.header.stamp.sec = 101
    elif kind == 'old':
        msg.header.stamp.sec = 98
    else:
        msg.header.stamp.sec, msg.header.stamp.nanosec = 99, 999_999_999
    b._receive_state('right_hand', msg)
    with pytest.raises(RuntimeError, match='right_hand'):
        b.execute(np.zeros(56))
    assert all(not msgs for msgs in b.sent.values())


def test_stream_freshness_is_independent_and_frozen_stamp_is_not_refreshed(backend):
    b = backend
    ready(b)
    b._feedback['left_hand']['advanced'] = time.monotonic() - 1
    ready(b)  # Same timestamp arriving repeatedly must remain stale.
    assert b.assert_tracking() is False
    assert b._paused
    assert b.resume_tracking() is False
    assert all(not msgs for msgs in b.sent.values())


def test_watchdog_holds_once_and_recovery_seeds_current_feedback(backend):
    b = backend
    ready(b)
    b.execute(np.zeros(56))
    b._last_command_at = time.monotonic() - 1
    b._watchdog_tick()
    assert b._paused
    assert b.assert_tracking() is False
    assert all(len(msgs) == 2 for msgs in b.sent.values())
    b._watchdog_tick()
    assert all(len(msgs) == 2 for msgs in b.sent.values())
    assert b.resume_tracking() is True
    np.testing.assert_array_equal(b.get_target_joint_pos(), b.get_joint_pos())
    b.request_stop('test')
    with pytest.raises(RuntimeError, match='stopped'):
        b.execute(np.zeros(56))


def test_close_is_idempotent_and_does_not_shutdown_global_ros(backend):
    b = backend
    ready(b)
    closed = []
    b._watchdog = NS(cancel=lambda: closed.append('timer'))
    b._executor = NS(shutdown=lambda: closed.append('executor'))
    b._thread.join = lambda: closed.append('thread')
    b._node.destroy_node = lambda: closed.append('node')
    b._context.try_shutdown = lambda: closed.append('context')
    b.close()
    b.close()
    assert closed == ['timer', 'executor', 'thread', 'node', 'context']
    assert all(len(msgs) == 1 for msgs in b.sent.values())


def test_cleanup_failure_still_closes_other_ros_resources(backend):
    b, closed = backend, []
    ready(b)
    def fail():
        closed.append('executor')
        raise RuntimeError('executor shutdown failed')
    b._watchdog = None
    b._executor = NS(shutdown=fail)
    b._thread.join = lambda: closed.append('thread')
    b._node.destroy_node = lambda: closed.append('node')
    b._context.try_shutdown = lambda: closed.append('context')
    with pytest.raises(RuntimeError, match='shutdown failed'):
        b.close()
    assert closed == ['executor', 'thread', 'node', 'context']
    assert b._closed and b._stopped


def test_gesture_does_not_report_success_after_executor_failure(backend):
    backend._strict_stop = True
    backend._stopped = True
    backend._spin_error = 'executor failed'
    with pytest.raises(RuntimeError, match='executor failed'):
        backend.request_stop('gesture')
