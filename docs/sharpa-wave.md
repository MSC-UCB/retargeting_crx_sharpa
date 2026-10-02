# Sharpa Wave preview and mock validation

The CRX + Sharpa live Quest entrypoint defaults to ROS output, 20 Hz solving,
100 Hz publication, a fixed 50 ms interpolation horizon, and
`--stop-gesture dual-thumb-ring-pinch --stop-gesture-hold-s 2`.
Run `.venv/bin/python scripts/run_crx_sharpa_joint_teleop.py` to use these defaults.
Use `--backend preview` for preview or `--stop-gesture none` to disable the gesture.
Pinch each thumb to its ring fingertip on both hands. The first bilateral
candidate latches output immediately; holding for two seconds confirms exit.
Releasing after the latch exits without resuming. ROS startup does not query or
restrict the CRX interpolation method or target mode. The stop still sends one
measured-position hold when feedback is valid; removing the mode restriction
does not fix the previously observed stream-mode stop excursion. Additional
usage and validation notes are retained locally in `../stop_gesture.md`, which
is not tracked in Git.

The CRX + Sharpa scene uses `configs/bimanual/crx5ia_sharpa_wave.yaml`.
The hand-only scene uses `configs/bimanual/sharpa_wave.yaml`. Both use the same
local Sharpa models as the ROS execution scripts. The flange transforms in
`configs/sharpa_mounts.yaml` are provisional and must be checked against the
physical mounts before controlling real hardware.
The left CRX mount currently rotates the hand 180 degrees about `fanuc_flange`
Z; the right mount remains at zero rotation.

The CRX + Sharpa initial pose sets both J6 joints to zero. Joint order is J1–J6:

| Convention | Left (degrees) | Right (degrees) |
| --- | --- | --- |
| FANUC pendant | 0, 30, -60, 0, 60, 0 | -90, -30, 240, 0, -60, 0 |
| ROS / URDF | 0, 30, -30, 0, 60, 0 | -90, -30, 210, 0, -60, 0 |

The robot YAML files store ROS angles in radians; all hand joints start at zero.
Preview uses this configured pose. ROS teleoperation calibrates from measured
joint feedback and does not automatically move to it. The matching target is in
`dual_crx_control/scripts/move_to_default_pose.py`; run that separately when a
move to this pose is intended. Within dual_crx, `config/initial_pose.yaml` now
supplies the same pose to this script, mock startup and planned-motion scripts.
Retargeting keeps its own robot YAML files; keep their arm values consistent
with that ROS-degree config when changing the shared starting pose.

The optimization frame `<side>_retarget_wrist` has the same orientation as the
native Sharpa wrist: +Z follows extended fingers and +X points out of the palm
(the flexion direction). This matches the current Quest decoder. Its fixed
joint has zero rotation; it does not add a flange-mount correction. Synthetic
smoke input uses this same local basis. Regression tests pass an independent
WebXR skeleton through the Quest decoder and compare mapped finger directions
and flexion against FK for both hand-only and CRX-mounted models.

From the repository root, inspect the initial scene without Quest or ROS:

```bash
env -u PYTHONPATH .venv/bin/python scripts/view_bimanual_initial.py --config configs/bimanual/crx5ia_sharpa_wave.yaml
```

Open `http://localhost:9219` in the Windows browser. Inspect both bases,
flanges, palms, thumb directions, and the initial joint pose. For the hand-only
scene, pass `--config configs/bimanual/sharpa_wave.yaml` instead.

With Quest connected and authorized in WSL (`adb devices -l`), run a virtual
preview that publishes no robot commands:

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_crx_sharpa_joint_teleop.py --backend preview
```

The hand-only equivalent is
`scripts/run_sharpa_joint_teleop.py --backend preview`. Quest Browser connects
to the WebXR receiver on port 8765; the
viewer remains on port 9219. Both hands must be tracked for calibration.
Try wrist translation and rotation, each finger, tracking loss, and recovery.

To compare the Quest skeleton with the hand-only Sharpa meshes while tuning
their input scales, run:

```bash
env -u PYTHONPATH .venv/bin/python scripts/preview_sharpa_scale.py
```

Open `http://localhost:9219` and adjust the **Left Quest hand scale** and
**Right Quest hand scale** sliders. Each starts at its robot config's
`human_hand_scale` (currently 1.2 on both hands) and ranges from 0.50 to 2.00. Use
`--left-scale 1.1 --right-scale 1.3` to try other starting values. The sliders scale
Quest wrist-local keypoints on subsequent frames; they do not resize the
Sharpa URDF. This preview sends no ROS commands. On Ctrl+C, copy the printed
values to `configs/robots/sharpa_wave_left.yaml` and
`configs/robots/sharpa_wave_right.yaml` after checking several poses.

Add `--demo` to inspect the same sliders and an animated synthetic skeleton
without connecting Quest. The synthetic skeleton is for checking the preview
interface and must not be used to calibrate real tracking scale. Demo mode
keeps the configured hand-command smoothing. Quest keypoint markers now use a
0.006 m default diameter in execution and replay viewers, including this scale
preview. A viewer's `human_keypoint_size` setting can still override it.

## Synthetic 20 cm vertical motion

`run_crx_sharpa_vertical_demo.py` replaces Quest input with synchronized synthetic
hands while reusing the CRX + Sharpa mapping, two solver processes (30 ms per
side), output smoothing and ROS backend. It moves both wrist targets along
robot world Z: **start → 20 cm above start → start**, with fixed wrist orientation.
Each cycle takes 8 seconds; the default is 3 cycles. A quintic trajectory gives
zero target velocity and acceleration at the top and bottom.

Preview requires no Quest or ROS and starts from the configured robot pose:

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_crx_sharpa_vertical_demo.py \
  --backend preview --stroke-m 0.20 --period 8 --cycles 3
```

Open `http://localhost:9219`, or add `--no-viewer` for headless execution.
For ROS, first start both drivers in the intended ROS domain, then run:

```bash
.venv/bin/python scripts/run_crx_sharpa_vertical_demo.py \
  --backend ros --stroke-m 0.20 --period 8 --cycles 3 \
  --command-hz 20 --publish-hz 100 --interpolation-horizon-ms 50
```

ROS calibrates the wrist origins and orientations from fresh measured joints;
the motion clock starts after calibration. It does not issue a home command.
The fixed finger gesture is generated from the models' initial open-hand poses,
so hands starting in another gesture can move toward that gesture. Both hand
command channels remain active. This does not freeze measured finger joints.

Retargeting output uses **linear interpolation**, with a 50 ms horizon and
100 Hz publication; it does not select or change the CRX driver's downstream
interpolator. Preview displays solved targets at 20 Hz. Existing smoothing and
soft optimization objectives mean the Cartesian motion is approximate: offline
validation from the configured pose measured about 19.7 cm wrist travel and
7 mm maximum position error relative to the reference over one cycle.

After the final cycle, the script continues submitting the start-position target
until all measured joints are within `--tolerance-deg 2` of the solved command
for `--hold-time 1` second. It fails if this cannot finish within
`--settle-timeout 10` seconds. This joint tolerance does not guarantee an exact
Cartesian endpoint or the original joint configuration. Preview only requires
accepted endpoint targets for the hold. Tracking interruption aborts the demo;
Ctrl+C stops new commands without a return move. No collision checking is added.

The opt-in integration test launches isolated mock drivers on domain 192 with
localhost discovery and no viewer. It uses CRX `method:=linear`, so it does not
validate hardware or downstream Ruckig tuning:

```bash
# After sourcing ROS and the installed driver workspace:
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 SHARPA_VERTICAL_ROS_TEST=1 \
  .venv/bin/python -m pytest tests/test_sharpa_vertical_demo_ros.py -q -s
```

The one-cycle mock run measured 100.1 Hz on all three command topics (908
messages each), 178 solved targets with zero stale inputs, and left/right wrist
travel of 0.1969/0.1970 m. The final maximum feedback-to-command joint error was
0.000154 rad. Both solver processes and the ROS feedback thread exited cleanly.

A subsequent hardware recording with downstream **Ruckig stream** showed repeated
braking/reversal and an approximately 7.9-degree right J2 excursion after the
final measured-hold message. The linear mock test did not cover this behavior.
Evidence, offline reproduction and proposed fixes are recorded in the local
`vertical-demo-recording-analysis-20260928.md`, which is not tracked in Git.
Those fixes are not yet applied.
The operator subsequently reported good hardware motion with
`method:=ruckig ruckig_target_mode:=waypoint`. Waypoint avoids the stream mode's
per-message velocity/acceleration estimation while retaining Ruckig limits and
the current reference state during replanning. No new recording has yet been
analyzed to quantify that improvement; launch defaults have not been changed here.

## ROS mock and live input

For an isolated ROS mock test, source ROS Jazzy and installed copies of both
driver workspaces in each terminal. Start the Sharpa mock driver in one terminal:

```bash
export ROS_DOMAIN_ID=189
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch dual_sharpa_wave dual_sharpa.launch.py backend:=mock use_rviz:=false
```

To test arms and hands together, also start the CRX mock driver in another
terminal using the same domain:

```bash
ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false method:=linear input_rate_hz:=100.0
```

Then run one of these finite synthetic-input smoke tests from this repository,
with the same sourced ROS environment and domain:

```bash
.venv/bin/python scripts/run_sharpa_joint_teleop.py --backend ros --no-viewer --synthetic-frames 100
.venv/bin/python scripts/run_crx_sharpa_joint_teleop.py --backend ros --no-viewer --synthetic-frames 100 --stop-gesture none
```

Run the two commands separately. The first requires only Sharpa feedback; the
second requires Sharpa and CRX feedback. The scripts wait for complete, fresh
joint states and subscribers before publishing. They stop on input exhaustion.

Both Sharpa scripts default to **100 Hz linear-interpolated ROS output**, while
retargeting still targets 20 Hz. To state these settings explicitly for live
Quest input after starting the mock drivers:

```bash
.venv/bin/python scripts/run_crx_sharpa_joint_teleop.py --backend ros --command-hz 20 --publish-hz 100 --interpolation-horizon-ms 50
```

For hands only, use `scripts/run_sharpa_joint_teleop.py` with the same options.
`--publish-hz` and `--interpolation-horizon-ms` affect ROS output only; preview
continues to show each solved, smoothed target. The default interpolation horizon
is fixed at 50 ms for CRX + Sharpa and is `1000 / command-hz` ms for hands only.
It must be shorter than the 250 ms target timeout.
Smoothing runs once per solver target, not once per 100 Hz publication.

An independent steady-clock ROS timer samples the complete 44- or 56-joint
vector, then publishes the hand channels and optional arm channel with the same
timestamp. Separate topics are not an atomic transport. New solver targets
replace pending targets; missed timer slots are not replayed in bursts. The
last endpoint is held between fresh targets, but repeated publications do not
extend the lifetime of an old solver target. Tracking loss, stale feedback,
expired targets, and shutdown cancel interpolation. Recovery starts from fresh
measured joint positions.

Sharpa's driver `publish_rate_hz: 100.0` controls its feedback/update timer;
commands are consumed on receipt. Its SDK interpolation setting remains owned
by that driver. CRX uses `input_rate_hz:=100.0` for this incoming stream and keeps
its own downstream output frequency. No controller configuration is changed by
the retargeting scripts. To measure each command topic during a mock run:

```bash
ros2 topic hz /sharpa/left_hand/joint_command
ros2 topic hz /sharpa/right_hand/joint_command
ros2 topic hz /crx5ia/joint_targets
```

To run the opt-in automated mock test:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 SHARPA_ROS_TEST=1 .venv/bin/python -m pytest tests/test_sharpa_ros_integration.py -q
```

For live Quest input with mock drivers, omit `--synthetic-frames`;
`--duration 60` sets a finite session. Each side has its own solver, executed
concurrently in two persistent spawned processes, with a **30 ms time budget per side**.
Both hand-only and CRX+Sharpa entrypoints use this path in preview and ROS modes;
CLI arguments are unchanged. Workers build only their own side's model/optimizer,
use one numerical-library thread each, and start before input acquisition.
The parent maps each Quest frame, supplies the last valid filtered command as each
seed, waits for both matching results, and then applies the existing output policy.
This is not a fixed 60 ms wait, nor a hard 30 ms end-to-end deadline.
A solve that exceeds the input freshness threshold
(150 ms by default) can still cause a frame to be dropped. The Sharpa defaults
apply output smoothing (arm alpha 0.3, hand alpha 0.5) in preview and ROS modes,
without retargeting-side joint speed limiting. Smoothing is enabled independently
with `output.smooth_output_qpos`; `output.limit_joint_speed` defaults to false.
The downstream controller owns velocity limiting. Model joint-position bounds still apply.
A missing feedback stream pauses output
until fresh feedback and a new calibration are available. The solver target rate
is 20 Hz and the ROS publication target is 100 Hz. The flow's `commands` counter
counts solver targets, not interpolated ROS publications. Actual solver rate and
solve time are reported during execution; topic frequency must be measured under
solver and viewer load. Mock
results verify the software path only. Physical control requires confirmed
mounts, joint mapping, safety setup, and a separate device test.

Worker replies have a 250 ms timeout, independent of the 30 ms NLopt budget;
the 150 ms input-age check still rejects obsolete completed results. Worker failure,
timeout, invalid joint positions, or mismatched reply IDs stops the session rather
than switching to serial solving or publishing a partial pair. Recovery after a
tracking pause uses fresh measured seeds; no worker command queue is accumulated.
Timed shutdown cancels a pending wait, and the flow stops output before joining
workers. Cleanup waits up to 0.5 s for normal exit, then uses terminate/kill if needed.
On Linux, workers install a parent-death signal so a killed parent cannot leave a
native solver running. ROS, SDK connections, input devices, and the viewer stay in
the parent. Callers driving `flow.step()` manually should call `flow.start_solver()`
before acquiring a sample and always call `flow.close()` in a `finally` block;
otherwise the first `step()` starts workers and discards that pre-startup sample.

The offline comparison and integration measurements are recorded in the local
`crx-sharpa-teleop-latency-investigation.md`, which is not tracked in Git.
