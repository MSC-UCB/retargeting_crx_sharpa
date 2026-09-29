"""Waypoint preflight with no ROS installation or network required."""
import sys
from types import SimpleNamespace as NS

import pytest

from retargeting_ros.sharpa_joint import SharpaJointBackend


@pytest.mark.parametrize('values', [None, [], ['ruckig'], ['linear', 'waypoint'],
                                  ['ruckig', 'stream'], [None, 'waypoint']])
def test_unknown_or_unsupported_mode_is_rejected(values):
    response = None if values is None else NS(values=[
        NS(type=0 if v is None else 4, string_value=v or '') for v in values])
    with pytest.raises(RuntimeError, match='requires CRX'):
        SharpaJointBackend._check_waypoint_response(response)


@pytest.mark.parametrize('mode', ['waypoint', 'stream', 'unavailable', 'timeout'])
def test_query_validates_response_and_always_destroys_client(monkeypatch, mode):
    from retargeting_ros import sharpa_joint

    now, events = [10.], []
    def advance(*args, **kwargs):
        now[0] += .05
        return mode != 'unavailable'

    monkeypatch.setattr(sharpa_joint, 'time', NS(monotonic=lambda: now[0], sleep=advance))
    monkeypatch.setitem(sys.modules, 'rcl_interfaces.srv', NS(GetParameters=NS(Request=NS)))
    future = NS(done=lambda: mode != 'timeout', cancel=lambda: events.append('cancel'),
                result=lambda: NS(values=[NS(type=4, string_value=s) for s in ('ruckig', mode)]))
    def call(request):
        assert request.names == ['method', 'ruckig_target_mode']
        return future

    client = NS(wait_for_service=advance, call_async=call)
    def create(kind, name):
        assert name == '/crx5ia/joint_interpolation/get_parameters'
        return client

    b = SharpaJointBackend.__new__(SharpaJointBackend)
    b._node = NS(create_client=create, destroy_client=lambda c: events.append('destroy'),
                 get_logger=lambda: NS(info=lambda s: events.append('verified')))
    if mode == 'waypoint':
        b._verify_waypoint('/crx5ia/', 10.2)
        assert events == ['verified', 'destroy']
    else:
        with pytest.raises(RuntimeError):
            b._verify_waypoint('crx5ia', 10.2)
        assert events == (['cancel', 'destroy'] if mode == 'timeout' else ['destroy'])
