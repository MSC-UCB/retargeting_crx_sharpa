"""Fresh-frame pinch geometry, latched flow lifecycle and public CLI contracts."""
from dataclasses import replace
from types import SimpleNamespace as NS
import subprocess
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from teleoperation.stop_gesture import GestureStopDetector, StopGestureConfig
from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput
from teleoperation.types import BimanualSensorHandSample
from teleoperation.bimanual_execution import BimanualExecutionFlow


def gesture_sample(t, sequence, *, pinch=(True, True), ratio=.05):
    base = SyntheticBimanualInput().read()
    hands = []
    for hand, active in zip((base.left, base.right), pinch):
        p = hand.keypoints_wrist.copy()
        if active:
            width = np.linalg.norm(p[5]-p[17])
            p[4] = p[16]+[ratio*width, 0, 0]
        hands.append(replace(hand, keypoints_wrist=p, source_index=sequence, timestamp=t,
                             raw=NS(received_monotonic_ns=round(t*1e9))))
    return BimanualSensorHandSample(*hands)


def test_exact_two_seconds_requires_new_frame_and_never_unlatches():
    d = GestureStopDetector()
    for i in range(20):
        t = 10+i*.1
        assert d.update(gesture_sample(t,i),t) == 'STOP_LATCHED'
    assert d.update(gesture_sample(11.99,20),11.99) == 'STOP_LATCHED'
    assert d.update(gesture_sample(12.,21),12.) == 'EXIT_CONFIRMED'
    assert d.update(gesture_sample(12.01,22,pinch=(False,False)),12.01) == 'EXIT_CONFIRMED'


@pytest.mark.parametrize('scale', [.7, 1., 1.5])
def test_rotation_scale_and_handedness_do_not_change_pinch(scale):
    d = GestureStopDetector()
    sample = gesture_sample(10.,0)
    rot = Rotation.from_euler('xyz',[.8,-1.2,.2]).as_matrix()
    sample = BimanualSensorHandSample(*[replace(h,keypoints_wrist=h.keypoints_wrist@rot.T*scale)
                                       for h in (sample.left,sample.right)])
    assert d.update(sample,10.) == 'STOP_LATCHED'


def test_hysteresis_and_bilateral_overlap():
    d = GestureStopDetector()
    d.update(gesture_sample(10,0,pinch=(True,False)),10)
    assert not d.latched
    d.update(gesture_sample(10.1,1,ratio=.25),10.1)  # right has not entered
    assert not d.latched
    d.update(gesture_sample(10.2,2),10.2)
    assert d.latched and d.elapsed == 0
    assert d.update(gesture_sample(10.3,3,ratio=.25),10.3) == 'STOP_LATCHED'
    assert d.update(gesture_sample(10.4,4,ratio=.31),10.4) == 'EXIT_UNCONFIRMED'
    assert d.update(gesture_sample(10.5,5),10.5) == 'EXIT_UNCONFIRMED'


@pytest.mark.parametrize('kind', ['open','single','index_pinch','fist','nan','degenerate','missing','future','stale','unsynced'])
def test_non_gestures_and_invalid_inputs_do_not_latch(kind):
    sample = gesture_sample(10,0)
    if kind=='open':sample=gesture_sample(10,0,pinch=(False,False))
    elif kind=='single':sample=gesture_sample(10,0,pinch=(True,False))
    else:
        left=sample.left;p=left.keypoints_wrist.copy()
        if kind=='index_pinch':p[4]=p[8]
        elif kind=='fist':
            for root in (5,9,17):p[root+2]=p[root]+[.001,0,0]
        elif kind=='nan':p[0,0]=np.nan
        elif kind=='degenerate':p[17]=p[5]
        elif kind=='missing':p=None
        left=replace(left,keypoints_wrist=p)
        if kind in ('future','stale'):left=replace(left,raw=NS(received_monotonic_ns=int((11 if kind=='future' else 9)*1e9)))
        if kind=='unsynced':left=replace(left,source_index=1)
        sample=BimanualSensorHandSample(left,sample.right)
    assert GestureStopDetector().update(sample,10)=='RUNNING'


@pytest.mark.parametrize('kind', ['missing','stale','backwards','same_stamp','duplicate_timeout','gap','release','bad_geometry'])
def test_bad_confirmation_ends_session_without_resume(kind):
    d=GestureStopDetector();first=gesture_sample(10,1);d.update(first,10)
    sample=gesture_sample(10.05,2);now=10.05
    if kind=='missing':sample=replace(sample,left=replace(sample.left,keypoints_wrist=None))
    elif kind=='stale':now=10.3
    elif kind=='backwards':sample=gesture_sample(10.05,0)
    elif kind=='same_stamp':sample=gesture_sample(10,2)
    elif kind=='duplicate_timeout':
        assert d.update(first,10.1)=='STOP_LATCHED'
        assert d.elapsed==0
        assert d.poll(10.151)=='EXIT_UNCONFIRMED'
        return
    elif kind=='gap':sample=gesture_sample(10.2,2);now=10.2
    elif kind=='release':sample=gesture_sample(10.05,2,pinch=(False,True))
    elif kind=='bad_geometry':
        p=sample.left.keypoints_wrist.copy();p[4]=p[8]
        sample=replace(sample,left=replace(sample.left,keypoints_wrist=p))
    assert d.update(sample,now)=='EXIT_UNCONFIRMED'
    assert d.latched


@pytest.mark.parametrize('settings',[{'hold_s':0},{'hold_s':float('nan')},{'pinch_exit_ratio':.1},{'fist_pip_angle_deg':180}])
def test_invalid_settings(settings):
    with pytest.raises(ValueError):StopGestureConfig(**settings)


def minimal_flow(monkeypatch, *, initialized=True):
    from teleoperation import bimanual_execution as module
    clock=[10.];events=[]
    monkeypatch.setattr(module,'time',NS(monotonic=lambda:clock[0],sleep=lambda dt:clock.__setitem__(0,clock[0]+dt)))
    source=NS(max_age_s=.15,close=lambda:events.append('source closed'),open=lambda:None,stats=NS(accepted=0))
    flow=BimanualExecutionFlow(source=source,pipeline=NS(initialized=initialized),initial_qpos=np.zeros(4),
                              robot_dofs=(2,2),arm_dofs=(0,0),stop_gesture=GestureStopDetector())
    return flow,clock,events


def test_candidate_blocks_startup_and_ik(monkeypatch):
    flow,clock,events=minimal_flow(monkeypatch,initialized=False)
    flow.backend_factory=lambda:pytest.fail('Candidate must not connect a backend')
    flow.pipeline.step=lambda sample:pytest.fail('Gesture must not be solved')
    flow.pair_solver=NS(started=False,start=lambda:pytest.fail('step must inspect gesture first'),cancel=lambda:events.append('cancel'))
    assert flow.step(gesture_sample(10,0)) is None
    assert flow.stop_latched.is_set() and events==['cancel']
    assert flow.command_count==0


@pytest.mark.parametrize('release',[False,True])
def test_run_continues_confirmation_without_backend_tracking_or_resume(monkeypatch,release):
    flow,clock,events=minimal_flow(monkeypatch)
    flow.started_at=9
    def read():
        i=flow.source.stats.accepted;flow.source.stats.accepted+=1
        return gesture_sample(clock[0],i,pinch=(not release or i==0,True))
    flow.source.read=read
    flow.pipeline.step=lambda sample:pytest.fail('No candidate should enter IK')
    def tracking():
        if flow.stop_latched.is_set():pytest.fail('Latched flow must skip tracking recovery')
        return True
    flow.backend=NS(request_stop=lambda r:events.append('stop'),assert_tracking=tracking,
                    resume_tracking=lambda:pytest.fail('Must not resume'),close=lambda:events.append('backend closed'))
    flow.last_command_at=10
    flow.run()
    assert flow.stop_gesture.state==('EXIT_UNCONFIRMED' if release else 'EXIT_CONFIRMED')
    assert events==['stop','backend closed','source closed']
    assert flow.command_count==0


def test_latch_during_solver_discards_reply_and_cancellation(monkeypatch):
    flow,clock,events=minimal_flow(monkeypatch)
    flow.backend=NS(request_stop=lambda r:events.append('stop'),execute=lambda q:pytest.fail('No late target'))
    flow.pair_solver=NS(started=True,cancel=lambda:events.append('cancel'))
    def solve(sample):
        flow.latch_gesture_stop()
        raise RuntimeError('Process solver cancelled')
    flow.pipeline.step=solve
    assert flow.step(gesture_sample(10,0,pinch=(False,False))) is None
    assert events==['stop','cancel'] and flow.command_count==0


def test_stop_error_still_cancels_and_cleans(monkeypatch):
    flow,clock,events=minimal_flow(monkeypatch)
    def stop(reason):raise RuntimeError('No fresh feedback for hold')
    flow.backend=NS(request_stop=stop,close=lambda:events.append('backend closed'))
    flow.pair_solver=NS(cancel=lambda:events.append('cancel'),close=lambda:events.append('solver closed'))
    try:
        with pytest.raises(RuntimeError,match='feedback'):flow.step(gesture_sample(10,0))
    finally:flow.close()
    assert events==['cancel','backend closed','solver closed','source closed']


@pytest.mark.parametrize('script,args',[
    ('run_crx_sharpa_joint_teleop.py',['--synthetic-frames','2']),
    ('run_sharpa_joint_teleop.py',[]),
    ('run_crx_sharpa_joint_teleop.py',['--stop-gesture-hold-s','nan']),
    ('run_crx_sharpa_joint_teleop.py',['--command-hz','5']),
])
def test_cli_rejects_unsupported_before_devices(script,args):
    r=subprocess.run([sys.executable,'scripts/'+script,'--stop-gesture','dual-thumb-ring-pinch',*args],capture_output=True,text=True)
    assert r.returncode==2 and 'error:' in r.stderr


def test_composition_only_live_quest_and_arms(monkeypatch):
    from retargeting_apps.sharpa_teleop import build_flow
    from test_sharpa_execution import arguments
    from retargeting_ros import sharpa_joint
    args=arguments(backend='ros',stop_gesture='dual-thumb-ring-pinch',stop_gesture_hold_s=2.)
    monkeypatch.setattr(sharpa_joint,'SharpaJointBackend',lambda **kw:NS(**kw))
    flow,_=build_flow(args,with_arms=True)
    try:
        assert flow.stop_gesture.config.hold_s==2
        assert flow.backend_factory().strict_stop
    finally:flow.close()
    with pytest.raises(ValueError,match='live Quest'):build_flow(args,with_arms=True,source=SyntheticBimanualInput())
    with pytest.raises(ValueError,match='live Quest'):build_flow(args,with_arms=False)


class PinchAfterCommands(SyntheticBimanualInput):
    """Test-only acquisition; never opens Quest or exposes a public CLI mode."""
    def __init__(self, flow, *, release=False):
        super().__init__(frames=200)
        self.flow, self.release = flow, release
        self.first_pinch = None
        self.closed = False

    def read(self):
        import time

        sample = super().read()
        now = time.monotonic()
        active = self.flow.command_count >= 5
        if active and self.first_pinch is None:
            self.first_pinch = now
        if self.release and self.first_pinch is not None and now-self.first_pinch > .3:
            active = False
        return gesture_sample(now, sample.source_index, pinch=(active, active))

    def close(self):
        self.closed = True


def test_actual_two_process_preview_confirms_and_reaps_workers():
    from retargeting_apps.sharpa_teleop import build_flow
    from test_sharpa_execution import arguments

    flow, _ = build_flow(arguments(stop_gesture='dual-thumb-ring-pinch',
                                   stop_gesture_hold_s=2.), with_arms=True)
    flow.source = PinchAfterCommands(flow)
    flow.run()
    assert flow.command_count == 5
    assert flow.stop_gesture.state == 'EXIT_CONFIRMED'
    assert flow.source.closed
    assert len(flow.pair_solver.cleanup) == 2
    assert all(not p['alive'] and p['exitcode'] == 0 for p in flow.pair_solver.cleanup)


@pytest.mark.parametrize('outcome,expected', [('EXIT_CONFIRMED', 0), ('EXIT_UNCONFIRMED', 2),
                                           ('error', 1), ('interrupt', 130)])
def test_main_exit_status_and_cleanup(monkeypatch, outcome, expected):
    from retargeting_apps import sharpa_teleop

    events = []
    def run():
        if outcome == 'error':
            raise RuntimeError('test failure')
        if outcome == 'interrupt':
            raise KeyboardInterrupt
    flow = NS(run=run, close=lambda: events.append('close'),
              stop_gesture=NS(latched=True, state=outcome, reason='test'))
    monkeypatch.setattr(sharpa_teleop, 'build_flow', lambda *a, **kw: (flow, {}))
    assert sharpa_teleop.main(['--no-viewer', '--stop-gesture', 'dual-thumb-ring-pinch']) == expected
    assert events == ['close']


def test_viewer_startup_failure_closes_flow(monkeypatch):
    from retargeting_apps import sharpa_teleop
    from retargeting_apps.visualization.execution import manager

    events = []
    flow = NS(close=lambda: events.append('close'))
    def fail(*args):
        raise RuntimeError('viewer failed')
    monkeypatch.setattr(sharpa_teleop, 'build_flow', lambda *a, **kw: (flow, {}))
    monkeypatch.setattr(manager, 'create_optional_execution_visualizer', fail)
    assert sharpa_teleop.main(['--viewer']) == 1
    assert events == ['close']
