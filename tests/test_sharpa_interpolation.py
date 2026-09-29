"""Deterministic 100 Hz publication tests without ROS, devices or wall-clock sleeps."""

from types import SimpleNamespace as NS

import numpy as np
import pytest

from retargeting_ros import sharpa_joint
from retargeting_ros.joint_interpolation import JointCommandInterpolator
from teleoperation.backends.sharpa_contract import joint_channels
from test_sharpa_joint import backend, feedback, ready, robot_names


@pytest.fixture(params=[False, True], ids=['hands', 'arms_hands'])
def timed_backend(backend, request, monkeypatch):
    b, now = backend, [10.]
    monkeypatch.setattr(sharpa_joint, 'time', NS(monotonic=lambda: now[0]))
    b.channels = joint_channels(robot_names(request.param))
    b._size = 56 if request.param else 44
    b.lower, b.upper = np.full(b._size, -2.), np.full(b._size, 2.)
    b._actual = np.zeros(b._size)
    b._feedback = {key: dict(stamp=0, received=0., advanced=0., error='waiting') for key in b.channels}
    b.sent = {key: [] for key in b.channels}
    b._publishers = {key: NS(publish=b.sent[key].append, get_subscription_count=lambda: 1)
                     for key in b.channels}
    b._publish_hz = 100.
    timer = NS(canceled=True, resets=0)
    timer.cancel = lambda: setattr(timer, 'canceled', True)
    timer.is_canceled = lambda: timer.canceled

    def reset():
        timer.canceled = False
        timer.resets += 1

    timer.reset = reset
    b._output_timer = timer
    b._watchdog = b._executor = None
    b._thread.join = lambda: None
    b._node.destroy_node = lambda: None
    b._context.try_shutdown = lambda: None
    ready(b)
    b._target = b._actual.copy()
    b._reset_output_locked(b._actual)
    return b, now


def sent_vector(b, index=-1):
    qpos = np.empty(b._size)
    stamps = []
    for key, (names, indices) in b.channels.items():
        message = b.sent[key][index]
        assert message.name == list(names)
        qpos[indices] = message.position
        stamps.append(message.header.stamp)
    assert all(stamp is stamps[0] for stamp in stamps)
    return qpos


def test_latest_target_is_interpolated_at_100hz_with_common_timestamp(timed_backend):
    b, now = timed_backend
    seed = b.get_joint_pos()
    target = np.linspace(.5, 1., b._size)
    b.execute(target - .1)
    b.execute(target)
    assert all(not messages for messages in b.sent.values())
    assert b._output_timer.resets == 1
    for offset in (.01, .02, .03, .04, .05, .06):
        now[0] = 10. + offset
        b._publish_tick()
        phase = np.clip((offset - .01) / .04, 0., 1.)
        np.testing.assert_allclose(sent_vector(b), seed + phase * (target - seed), atol=1e-12)
    assert b._publish_count == 6 and b._target_count == 2
    assert b._last_command_at == 10.  # Publishing must not refresh the IK target.
    # A late tick samples once; it does not burst-replay the missed slots.
    now[0] = 10.19
    b._publish_tick()
    assert b._publish_count == 7
    np.testing.assert_array_equal(sent_vector(b), target)


def test_target_replacement_starts_at_last_published_position(timed_backend):
    b, now = timed_backend
    b.execute(np.ones(b._size))
    for t in (10.01, 10.02):
        now[0] = t
        b._publish_tick()
    start = sent_vector(b)
    now[0] = 10.025
    target = np.full(b._size, -.5)
    b.execute(target)
    now[0] = 10.03
    b._publish_tick()
    np.testing.assert_allclose(sent_vector(b), start + (.01 / .055) * (target - start))


def test_target_timeout_cancels_publication_despite_regular_ticks(timed_backend):
    b, now = timed_backend
    b.execute(np.ones(b._size))
    for tick in range(1, 25):
        now[0] = 10. + tick * .01
        b._publish_tick()
    now[0] = 10.251
    b._publish_tick()
    assert b._paused and b._output_timer.canceled
    assert b._pending_target is None and b._last_command_at is None
    np.testing.assert_array_equal(sent_vector(b), b.get_joint_pos())  # One hold.
    counts = [len(messages) for messages in b.sent.values()]
    now[0] = 10.3
    b._publish_tick()
    assert counts == [len(messages) for messages in b.sent.values()]


@pytest.mark.parametrize('action', ['pause_tracking', 'request_stop', 'close'])
def test_stop_during_sampling_invalidates_pending_curve(timed_backend, action):
    b, now = timed_backend
    b.execute(np.ones(b._size))
    sample = b._interpolator.sample

    def stop_during_sample(t):
        getattr(b, action)(*(['test'] if action == 'request_stop' else []))
        return sample(t)

    b._interpolator.sample = stop_during_sample
    now[0] = 10.01
    b._publish_tick()
    assert b._output_timer.canceled and b._pending_target is None
    assert b._publish_count == 0
    assert all(len(messages) == 1 for messages in b.sent.values())  # Only measured hold.
    np.testing.assert_array_equal(sent_vector(b), b._actual)
    now[0] = 10.02
    b._publish_tick()
    assert all(len(messages) == 1 for messages in b.sent.values())


def test_recovery_reseeds_interpolation_from_measured_joints(timed_backend):
    b, now = timed_backend
    b.execute(np.ones(b._size))
    b.pause_tracking()
    for key in b.channels:
        b._receive_state(key, feedback(b, key, value=.4))
    assert b.resume_tracking()
    seed = b.get_joint_pos()
    assert b._output_timer.canceled
    assert b._pending_target is None
    b.execute(np.full(b._size, .8))
    now[0] = 10.01
    b._publish_tick()
    np.testing.assert_array_equal(sent_vector(b), seed)


@pytest.mark.parametrize('failure', ['feedback', 'subscriber'])
def test_one_failed_channel_stops_all_interpolated_output(timed_backend, failure):
    b, now = timed_backend
    b.execute(np.ones(b._size))
    if failure == 'feedback':
        b._feedback['right_hand']['error'] = 'stale feedback'
    else:
        b._publishers['right_hand'].get_subscription_count = lambda: 0
    now[0] = 10.01
    b._publish_tick()
    assert b._paused and b._output_timer.canceled
    assert all(not messages for messages in b.sent.values())


@pytest.mark.parametrize('failure', ['expired_target', 'slow_sample', 'out_of_bounds'])
def test_invalid_interpolation_cannot_publish_a_motion_target(timed_backend, failure):
    b, now = timed_backend
    b.execute(np.ones(b._size))
    sample = b._interpolator.sample
    if failure == 'slow_sample':
        def delayed(t):
            now[0] += .02
            return sample(t)
        b._interpolator.sample = delayed
    elif failure == 'out_of_bounds':
        b._interpolator.sample = lambda t: np.full(b._size, 3.)
    now[0] = 10.06 if failure == 'expired_target' else 10.01
    b._publish_tick()
    assert b._rejected_samples == 1 and b._paused and b._output_timer.canceled
    assert b._publish_count == 0
    np.testing.assert_array_equal(sent_vector(b), b.get_joint_pos())


@pytest.mark.parametrize('size', [12, 44, 56])
def test_interpolator_keeps_configured_dimension_for_future_targets(size):
    curve = JointCommandInterpolator(np.zeros(size), 'linear', num_joints=size)
    with pytest.raises(ValueError, match='finite joint positions'):
        curve.set_target(np.zeros(size + 1), 1., .05, 1.)
    curve.set_target(np.ones(size), 1., .05, 1.)
    np.testing.assert_allclose(curve.sample(1.025), .5)


def test_gesture_stop_is_final_and_holds_only_once(timed_backend):
    b, now = timed_backend
    b._require_waypoint = True
    b.execute(np.ones(b._size))
    b.request_stop('gesture')
    b.request_stop('gesture again')
    assert not b.resume_tracking()
    with pytest.raises(RuntimeError, match='stopped'):
        b.execute(np.ones(b._size))
    b.close()
    assert all(len(messages) == 1 for messages in b.sent.values())
    assert b._output_timer.canceled and b._pending_target is None


def test_unverified_startup_cleanup_cannot_publish_hold(timed_backend):
    b, _ = timed_backend
    b._output_armed = False
    b.close()
    assert all(not messages for messages in b.sent.values())


def test_failed_gesture_hold_still_revokes_pending_output(timed_backend):
    b, now = timed_backend
    b._require_waypoint = True
    b.execute(np.ones(b._size))
    b._feedback['right_hand']['error'] = 'feedback lost'
    with pytest.raises(RuntimeError, match='feedback lost'):
        b.request_stop('gesture')
    assert b._stopped and b._output_timer.canceled and b._pending_target is None
    b._publish_tick()
    with pytest.raises(RuntimeError, match='feedback lost'):
        b.close()
    assert all(not messages for messages in b.sent.values())


def test_stop_from_another_thread_discards_sample_in_flight(timed_backend):
    import threading

    b, now = timed_backend
    b._require_waypoint = True
    b.execute(np.ones(b._size))
    sampling, release = threading.Event(), threading.Event()
    sample = b._interpolator.sample
    def blocked_sample(t):
        sampling.set()
        assert release.wait(2)
        return sample(t)

    b._interpolator.sample = blocked_sample
    now[0] = 10.01
    thread = threading.Thread(target=b._publish_tick)
    thread.start()
    try:
        assert sampling.wait(2)
        b.request_stop('gesture')
    finally:
        release.set()
        thread.join(2)
    assert not thread.is_alive()
    assert b._publish_count == 0
    assert all(len(messages) == 1 for messages in b.sent.values())
