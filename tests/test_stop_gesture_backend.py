"""Gesture-enabled startup needs feedback, but no driver parameter service."""
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from retargeting_ros.sharpa_joint import SharpaJointBackend
from test_sharpa_joint import robot_names


@pytest.mark.parametrize('feedback_ready', [True, False])
def test_startup_without_parameter_service_preserves_feedback_requirement(monkeypatch, feedback_ready):
    from retargeting_ros import sharpa_joint

    events, sent = [], []
    context = NS(try_shutdown=lambda: events.append('context'))
    node = NS(
        create_publisher=lambda *a: NS(publish=sent.append),
        create_subscription=lambda *a: None,
        create_client=lambda *a: pytest.fail('Startup must not query driver parameters'),
        create_timer=lambda *a, **kw: NS(cancel=lambda: None),
        get_logger=lambda: NS(info=lambda message: None),
        destroy_node=lambda: events.append('node'),
    )
    executor = NS(add_node=lambda node: None, spin=lambda: None,
                  shutdown=lambda: events.append('executor'))
    thread = NS(start=lambda: None, join=lambda: events.append('thread'))
    modules = {
        'rclpy': NS(init=lambda **kw: None),
        'rclpy.clock': NS(Clock=lambda **kw: None, ClockType=NS(STEADY_TIME=1)),
        'rclpy.context': NS(Context=lambda: context),
        'rclpy.executors': NS(SingleThreadedExecutor=lambda **kw: executor),
        'rclpy.node': NS(Node=lambda *a, **kw: node),
        'rclpy.qos': NS(QoSProfile=lambda **kw: None,
                         ReliabilityPolicy=NS(RELIABLE=1), DurabilityPolicy=NS(VOLATILE=0)),
        'sensor_msgs.msg': NS(JointState=lambda: NS(header=NS())),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(sharpa_joint, 'threading', NS(RLock=sharpa_joint.threading.RLock,
                                                    Thread=lambda **kw: thread))
    clock = [10.]
    monkeypatch.setattr(sharpa_joint, 'time', NS(monotonic=lambda: clock[0],
                       sleep=lambda dt: clock.__setitem__(0, clock[0] + dt)))
    def check_feedback(self):
        if not feedback_ready:
            raise RuntimeError('waiting for feedback')
    monkeypatch.setattr(SharpaJointBackend, '_check_feedback', check_feedback)
    kwargs = dict(robot_names=robot_names(), initial_qpos=np.zeros(56),
                  lower=np.full(56, -2.), upper=np.full(56, 2.),
                  control_period=.05, publish_hz=100., startup_timeout=.02, strict_stop=True)
    if feedback_ready:
        b = SharpaJointBackend(**kwargs)
        try:
            assert b._output_armed and b._strict_stop
            assert not sent
        finally:
            # The regular stop/hold path is covered by the transport tests.
            monkeypatch.setattr(b, '_publish', lambda values: sent.append(values.copy()))
            b.close()
        assert len(sent) == 1
    else:
        with pytest.raises(RuntimeError, match='startup timeout.*waiting for feedback'):
            SharpaJointBackend(**kwargs)
        assert not sent
    assert events == ['executor', 'thread', 'node', 'context']
