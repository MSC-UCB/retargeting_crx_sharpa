# Sharpa Wave preview and mock validation

The CRX + Sharpa live Quest entrypoint defaults to ROS output, 20 Hz solving,
100 Hz publication, a fixed 50 ms interpolation horizon, and
`--stop-gesture dual-thumb-ring-pinch --stop-gesture-hold-s 2`.
Run `.venv/bin/python scripts/run_crx_sharpa_joint_teleop.py` to use these defaults.
Use `--backend preview` for preview or `--stop-gesture none` to disable the gesture.
Pinch each thumb to its ring fingertip on both hands. The first bilateral
candidate latches output immediately; holding for two seconds confirms exit.
Releasing after the latch exits without resuming. ROS requires verified CRX
`method=ruckig` and `ruckig_target_mode=waypoint`; other modes are rejected
before any command publication. Additional usage and validation notes are
retained locally in `../stop_gesture.md`, which is not tracked in Git.

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
