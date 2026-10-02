# Human-to-Robot Retargeting: Dual CRX + Quest

## Quick Start: Quest 3 + Dual CRX + Sharpa Wave

**Use Quest hand tracking with CRX arms, Sharpa Wave hands, or both through ROS
control or virtual preview.** Run the commands below from the repository root in WSL/Linux.
Complete [Install and Test](#install-and-test) first if `.venv` is not ready.

### Choose a ROS control mode (100 Hz)

All three entrypoints are listed below. Source ROS and the relevant workspaces,
use the same ROS domain as the drivers, and start the mock drivers first:
[CRX arms setup](#joint-only-output-to-ws_fanucdual_crx_control) or
[Sharpa hands / arms + hands setup](docs/sharpa-wave.md).
Connect Quest as described below, then run **one** of these commands.

**Dual CRX arms only (12 joints):**

```bash
.venv/bin/python scripts/run_crx_joint_teleop.py \
  --namespace crx5ia --command-hz 20 --publish-hz 100 \
  --output-interpolation cubic --interpolation-horizon-ms 50
```

**Dual Sharpa hands only (44 joints):**

```bash
.venv/bin/python scripts/run_sharpa_joint_teleop.py \
  --backend ros --command-hz 20 --publish-hz 100 --interpolation-horizon-ms 50
```

**Dual CRX arms + dual Sharpa hands (56 joints):**

```bash
.venv/bin/python scripts/run_crx_sharpa_joint_teleop.py
```

The combined script defaults to `--backend ros --command-hz 20 --publish-hz 100
--interpolation-horizon-ms 50 --stop-gesture dual-thumb-ring-pinch
--stop-gesture-hold-s 2`. All options remain overridable. The first bilateral
thumb-to-ring pinch latches output immediately; holding for 2 seconds confirms
exit. Use `--stop-gesture none` to disable it or `--backend preview` for preview.

These commands request 20 Hz solving and independent 100 Hz ROS publication,
with a 50 ms interpolation horizon. The arms-only command uses cubic
interpolation; both Sharpa entrypoints use linear interpolation by default.
Configure the CRX driver with `input_rate_hz:=100.0`; Sharpa uses its 100 Hz
driver update rate. Output smoothing follows the selected `configs/bimanual/`
profile: arm alpha 0.3 and hand alpha 0.5. Joint speed limiting remains disabled
in the Sharpa profiles.
Both Sharpa entrypoints solve the left and right sides in two persistent processes,
with a 30 ms NLopt time budget per side. Worker startup precedes input acquisition;
the flow owns cancellation and process cleanup. Command and publication rates are unchanged.

For a virtual Sharpa preview, follow the steps below. Preview updates at the
solver rate; the 100 Hz publisher runs in ROS mode. See the
[Sharpa Wave guide](docs/sharpa-wave.md) for topic-frequency checks and configuration.

### 1. Connect Quest 3

Enable developer mode and hand tracking on the headset, connect it by USB, and
accept the USB debugging authorization prompt. On WSL, attach the USB device to
WSL first, then check from the same WSL terminal used to run the application:

```bash
adb devices -l
```

The headset must appear with status `device`. If the list is empty, check USB
attachment to WSL; if it says `unauthorized`, accept the prompt in the headset.

### 2. Start the Sharpa preview

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_crx_sharpa_joint_teleop.py --backend preview
```

The application opens Quest Browser. Press **Start tracking** in the headset
and keep both hands visible and steady until the terminal reports
`initialized=True`. The initial wrist pose is used for calibration.

On the Windows/desktop browser, open **http://localhost:9219** to view the robots.
Try wrist translation and rotation, opening and closing each hand, and individual
finger movements. This mode runs a virtual preview without ROS robot commands.
Press **Ctrl+C** in the terminal to stop; restart the command to recalibrate.

### 3. Other useful preview modes

**Inspect the initial installation without Quest:**

```bash
env -u PYTHONPATH .venv/bin/python scripts/view_bimanual_initial.py --config configs/bimanual/crx5ia_sharpa_wave.yaml
```

Open the same viewer URL, inspect the scene, then stop this process before
starting live tracking so port 9219 is available.

**Track only the two Sharpa hands with fixed robot wrists:**

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_sharpa_joint_teleop.py --backend preview
```

**Adjust Quest hand scale against the Sharpa meshes (no ROS output):**

```bash
env -u PYTHONPATH .venv/bin/python scripts/preview_sharpa_scale.py
```

Open `http://localhost:9219` and adjust the left and right hand scale sliders
while moving both hands in Quest. Press Ctrl+C to print the final values; copy
them to `human_hand_scale` in `configs/robots/sharpa_wave_left.yaml` and
`configs/robots/sharpa_wave_right.yaml` after checking several poses. Both the
hand-only and CRX+Sharpa modes now default to 1.2 for each hand. Use
`--left-scale` and `--right-scale` to try another starting value. This preview
does not publish ROS commands. Add `--demo` to inspect the viewer without Quest;
synthetic hand size cannot calibrate real tracking.

The two `run_*_joint_teleop.py` preview commands also accept:

| Option | Purpose |
| --- | --- |
| `--duration 60` | Stop 60 seconds after initial calibration. |
| `--serial <adb-serial>` | Select a headset when multiple ADB devices are attached. |
| `--viewer-port 9220` | Open the viewer on a different port. |
| `--no-viewer` | Run without the desktop viewer. |

## Project Overview

This project builds on [retargeting](https://github.com/Mingrui-Yu/retargeting)
for the paper *Analyzing Key Objectives in Human-to-Robot Retargeting for Dexterous
Manipulation*. It retains the core algorithms, offline replay, benchmarking and
optional simulation workflows, with five additions for the CRX setup:

- Two CRX-5iA arms with LEAP hands, solved from the same input frame.
- Quest WebXR hand tracking over USB/ADB.
- Left/right ROS command channels carried in one synchronized 44-joint message.
- Direct targets at the configured command frequency, without interpolation.
- Separate left/right initial joint poses, base placements and hand mounts.

## Install and Test

Run commands from the repository root using system Python and a local virtual
environment. Python 3.10+ is supported; ROS Jazzy integration uses Python 3.12.

```bash
git submodule update --init --recursive
/usr/bin/python3 -m venv .venv
env -u PYTHONPATH .venv/bin/python -m pip install -e ".[dev,quest3,replay]" pin scikit-learn
env -u PYTHONPATH .venv/bin/python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
env -u PYTHONPATH .venv/bin/python -m pytest tests/test_bimanual_quest.py tests/test_bimanual_execution.py -q
```

If `venv` reports missing `ensurepip`, use the bootstrap instructions in
[Local Test Environment](docs/configuration-and-development.md#local-test-environment).
The `pin` distribution supplies Pinocchio. CPU PyTorch is sufficient for these
headless tests. The `replay` extra includes Viser's URDF loader and the COLLADA
loader needed by the CRX meshes. Optional MuJoCo workflows require `.[mujoco]` or `.[mujoco-web]`.
The `env -u PYTHONPATH` prefix isolates offline commands from inherited ROS paths;
omit it for ROS execution after sourcing the ROS environment.

## Offline Replay

Retarget the bundled trajectory and save artifacts under `outputs/`:

```bash
env -u PYTHONPATH .venv/bin/python -m retargeting_apps.main \
  app=offline_retarget end=200 run_name=quickstart_leap
```

Add `post.visualize.enabled=true` to open the Viser viewer. To view saved results
or compute benchmark statistics:

```bash
env -u PYTHONPATH .venv/bin/python -m retargeting_apps.main app=replay run_name=quickstart_leap
env -u PYTHONPATH .venv/bin/python -m retargeting_apps.main app=benchmark run_name=quickstart_leap
```

## Dual-Arm Quest Execution

Enable Quest developer mode and USB debugging, connect USB and accept the
headset authorization prompt. `adb devices -l` should list the device. If Linux
needs a device access rule, use `scripts/install_quest_udev_rule.sh`.

Preview both arms with a kinematic scene:

```bash
env -u PYTHONPATH .venv/bin/python -m retargeting_apps.main app=teleop_exe \
  teleoperation_modes=bimanual_quest viewer.enabled=true
```

The application opens Quest Browser. Enter the immersive session and keep both
hands tracked for calibration. The viewer defaults to port 9219. This preview
uses no ROS backend. Set `input.serial=<adb-serial>` to select a headset.
To view only the configured initial robot poses without starting Quest:

```bash
env -u PYTHONPATH .venv/bin/python scripts/view_bimanual_initial.py
```

For ROS output, first start your configured `dual_crx_ros2` gateway, then run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_fanuc/install/setup.bash
source ~/dual_crx_ros2/install/setup.bash
.venv/bin/python -m retargeting_apps.main app=teleop_exe \
  teleoperation_modes=bimanual_quest backends=dual_crx viewer.enabled=true
```

This command enables physical output when the gateway controls real hardware.
The default enables both arms and the right LEAP; the left LEAP is disabled.
Use `bimanual.left_hand_enabled=true` only when the gateway also has that hand
configured. `bimanual.duration=60` stops 60 seconds after initial calibration;
zero means unlimited. Ctrl+C releases the session. For gateway setup, tracking
loss and cleanup behavior, see [physical execution](docs/bimanual_physical.md).

The solid robot meshes show measured joints when using ROS, and solved joints
in preview. Red wrist markers show command targets. An absent hand has only
configured placeholder positions, not measured feedback.

The existing `python -m retargeting_apps.bimanual_quest` and
`scripts/run_bimanual_quest_preview.py` commands remain compatibility entrypoints
and call the same application. New code should use the unified command above.
Single-arm Quest modes remain available as `online_quest3_kinematic` and
`online_quest3_mujoco`.

## Joint-Only Output to `ws_fanuc/dual_crx_control`

Use the standalone script for the Python joint bridge's standard ROS topics.
It runs the existing Quest retargeting flow and controls **both CRX arms only**;
neither LEAP hand is enabled. No `dual_crx_ros2` gateway, custom messages, or new
backend configuration is required. The existing dual-arm Viser viewer starts
automatically in the same process.

First start the joint controllers and bridge in a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_fanuc/install/setup.bash
export ROS_DOMAIN_ID=185
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch dual_crx_control dual_arm.launch.py namespace:=crx5ia mock:=true rviz:=false
```

Mock bringup starts the dual-arm simulation without RViz in the example above.
Set `rviz:=true` when you want the dual-arm graphical feedback view.

Then run from this repository, using the same ROS domain:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_fanuc/install/setup.bash
export ROS_DOMAIN_ID=185
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
.venv/bin/python scripts/run_crx_joint_teleop.py --namespace crx5ia --command-hz 20
```

Open **http://localhost:9219** on the retargeting computer (or use its IP address
from another computer). The actual bound URL is printed in the terminal.
Use `--viewer-port 9220` to select another port, or `--no-viewer` for headless
operation. Robot meshes show measured arm joints once tracking starts; red wrist
markers show command targets. Before tracking, the scene shows configured initial
poses. Finger poses remain placeholders because this script does not control hands.

Optional arguments: `--serial <adb-serial>` and `--duration 60` (seconds after
calibration; default 0 runs until Ctrl+C). `--help` does not start ROS, Quest, or a viewer.
Use a Python 3.12 virtual environment compatible with Jazzy and retain ROS's
`PYTHONPATH` for this command. The script uses the existing model/profile settings
and arm smoothing alpha (default 0.3) without changing the registered backends.

To keep IK at approximately 20 Hz while publishing interpolated commands at
100 Hz, use the updated ws_fanuc bridge and match its expected input rate:

```bash
# Robot-stack terminal: start with isolated mock hardware.
ros2 launch dual_crx_control dual_arm.launch.py \
  namespace:=crx5ia mock:=true rviz:=false input_rate_hz:=100.0 method:=linear

# Retargeting terminal: same ROS environment/domain as above.
.venv/bin/python scripts/run_crx_joint_teleop.py \
  --namespace crx5ia --command-hz 20 --publish-hz 100 --output-interpolation cubic \
  --interpolation-horizon-ms 50 --duration 60
```

`--publish-hz` enables an independent steady-clock publisher in the existing ROS
executor thread. Omit it to retain direct publication at the IK target rate.
`--output-interpolation` accepts `linear` or `cubic` (default `cubic`). The horizon
defaults to `1000 / command-hz` milliseconds, not the faster publish period.
It must be shorter than the flow's target timeout (normally 250 ms). An IK
update submits a target; each publisher tick samples the curve at the current
monotonic time. Initial output starts at the measured pose, insufficient cubic
history uses a linear segment, and endpoints are explicitly held between updates.
The 0.5 arm output filter still runs once per IK result, not at 100 Hz.

Natural cubic uses recent published samples, as in `RobotRealHighFreq`; it can
overshoot and does not guarantee velocity continuity between rebuilt curves.
It adds no motion limits. A target deadline that has already expired is rejected
without jumping to it. A computation taking a full publish period is discarded
instead of sending an obsolete sample. These events may interrupt the output;
frequency alone is not evidence of smoother physical motion. Python scheduling
and solver load must be measured. Keep the downstream interpolator `linear` for
this comparison to avoid two cubic stages.

Pause, stop, reset and close cancel the background timer and discard its pending
curve. Fresh feedback is checked on every active tick; feedback failure stops
the publisher. No new IK target for the flow timeout suspends publication, even
though the last endpoint was being repeatedly sent. A fresh target can resume
from the last published position; tracking recovery explicitly reseeds from
measured feedback. Neither case commands a home pose. Cancelling publication
does not cancel motion already accepted by the downstream controller.

Commands extract `qpos[:6]` and `qpos[22:28]` from the internal 44-joint vector
and publish a named 12-joint `sensor_msgs/msg/JointState` to
`/crx5ia/joint_targets`. Feedback comes from `/crx5ia/joint_states`
(`JointState`) and is mapped by joint name. The core `dual_arm.launch.py`
interpolator consumes the target stream, publishes
`/crx5ia/interpolated_joint_commands` at 500 Hz, and holds the last accepted
target when input pauses.
Startup and tracking recovery use measured arm positions; finger positions in
the internal state are configuration placeholders. The script waits up to five
seconds for complete feedback and a command subscriber, and rejects feedback
older than 0.5 seconds or with a frozen source timestamp, even if cached messages
continue arriving. It does not publish a startup/home command.

Ctrl+C, session expiry, or tracking loss stops new targets; the controller can
still finish its last target. This interface has no gateway lease, software-stop
service, collision checking, or added speed limiting. Test with mock first;
physical operation uses `mock:=false` in the first terminal, after stopping mock
and checking both controllers and measured robot positions. The same script
then sends physical commands.

Run the script's headless checks, or explicitly opt in to the isolated ROS mock
test (domain 185, no Quest or physical hardware):

```bash
env -u PYTHONPATH .venv/bin/python -m pytest tests/test_crx_joint_script.py tests/test_crx_joint_interpolation.py -q
# After sourcing ROS and ws_fanuc:
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CRX_JOINT_ROS_TEST=1 \
  .venv/bin/python -m pytest tests/test_crx_joint_script.py -k ros_mock -q
```

The ROS test disables unrelated pytest plugin auto-loading so system ROS launch
testing plugins do not impose extra packages on the retargeting virtualenv.

## Frequency and Output Filtering

The default command frequency is **20 Hz**. Change `backend.command_hz=20` on the
CLI, or edit `configs/backends/dual_crx.yaml` for ROS and
`configs/backends/kinematic.yaml` for preview. The compatibility CLI's
`--command-hz` overrides this same backend setting; it has no separate default.

`src/teleoperation/bimanual_execution.py` schedules the loop. It takes the newest
complete Quest frame, solves each arm once, and sends at most one joint target
per accepted frame. Duplicate, incomplete or stale frames do not become new
targets. Slow solving can reduce the actual output frequency; this is not a
guarantee to process every native Quest frame.

The dual-arm path generates **no intermediate trajectory points** and does not
use the legacy high-frequency spline node or startup waypoint interpolation.
`use_high_freq_interp` is false in the recommended modes. The legacy Panda
interpolation node and optional offline MuJoCo startup waypoints remain available
for their existing workflows. Gateway controller frequency and limits are
separate from retargeting target frequency. Lease heartbeats and startup holds
are lifecycle messages, not extra solved input frames.

Physical commands use first-order low-pass smoothing with **alpha=0.3 for
arms and alpha=0.5 for hands**. Values live in
`configs/bimanual/crx5ia_coact_leap.yaml` under `output.arm_smoothing_alpha` and
`output.hand_smoothing_alpha`. The formula is
`q = alpha * target + (1 - alpha) * previous_command`. This filter changes the
values without adding frames. Set `teleoperation_mode.output.smooth_output_qpos=false`
to disable it, or set the desired alpha to 1. Preview continues to show raw
solutions.

## ROS Interface

The left and right logical channels share one atomic target message. Both sides
come from the same Quest sequence. Message-level synchronization is not a claim
of hardware synchronization. Interface definitions are supplied by the external
`dual_crx_interfaces` package.

| Topic | Message type | Direction / content |
| --- | --- | --- |
| `/dual_crx/teleop/bimanual/command` | `dual_crx_interfaces/msg/TeleopCommand` | Output: 44 named joint positions |
| `/dual_crx/state` | `dual_crx_interfaces/msg/SystemState` | Input: left/right arm feedback and control state |
| `/left_leap/state`, `/right_leap/state` | `dual_crx_interfaces/msg/LeapState` | Input: feedback for enabled hands |

`TeleopCommand.client_id` identifies the lease owner. `target` is a
`sensor_msgs/JointState` with positions in **radians**, in this exact order:

| Indices | `target.name` |
| --- | --- |
| 0–5 | `left_J1` … `left_J6` |
| 6–21 | `left_leap_joint_0` … `left_leap_joint_15` |
| 22–27 | `right_J1` … `right_J6` |
| 28–43 | `right_leap_joint_0` … `right_leap_joint_15` |

`target.header.stamp` is command generation time on the ROS clock, not a target
arrival deadline or the Quest capture time. `velocity`, `effort` and `frame_id`
are empty. Values must be finite. Disabled hands retain their slots in the
44-position vector but receive no hand-enable request from this application.

Command QoS uses KEEP_LAST depth 1; state subscriptions use depth 10. The integer
QoS profiles use rclpy's default RELIABLE reliability and VOLATILE durability.

| Service | Type | Purpose |
| --- | --- | --- |
| `/dual_crx/acquire_control` | `dual_crx_interfaces/srv/AcquireControl` | Acquire BOTH scope (`arm_scope=3`) |
| `/dual_crx/heartbeat` | `dual_crx_interfaces/srv/Heartbeat` | Renew the lease |
| `/dual_crx/release_control` | `dual_crx_interfaces/srv/ReleaseControl` | Release the lease |
| `/dual_crx/teleop/bimanual/enable` | `dual_crx_interfaces/srv/SetTeleop` | Enable/disable the session |
| `/dual_crx/teleop/bimanual/pause_tracking` | `dual_crx_interfaces/srv/SetTeleop` | Pause (`enabled=true`) / resume (`false`) tracking |
| `/dual_crx/stop` | `dual_crx_interfaces/srv/SoftwareStop` | Stop output |
| `/left_leap/enable`, `/right_leap/enable` | `std_srvs/srv/SetBool` | Enable/disable installed hands |

The right-only backend retains `/dual_crx/teleop/command` with 22 right-side joints
and RIGHT scope (`arm_scope=2`). It does not run concurrently with a BOTH session.

## Initial Poses and Layout

The historical filename `configs/bimanual/crx5ia_coact_leap.yaml` now selects two
LEAP-equipped CRX arms and remains supported for existing commands.

| Setting | Left | Right |
| --- | --- | --- |
| Robot config | `configs/robots/crx5ia_leap_paxini_left.yaml` | `configs/robots/crx5ia_leap_paxini.yaml` |
| Arm initial joints (rad) | `[0, 0, 0, 0, -pi/2, 0]` | `[-pi/2, 0, pi, 0, pi/2, 0]` |
| Base position (m) | `[0, 0.3, 0]` | `[0, -0.3, 0]` |
| Base RPY (rad) | `[0, 0, 0]` | `[0, 0, -pi/2]` |

Each robot config also holds its 16 finger initial positions. Hand mounts and
wrist frames remain defined by the corresponding URDF assets. The left hand has
its own mounting transform; see its [asset notes](assets/robots/crx5ia_leap_paxini_left/README.md).

Preview begins at these YAML poses. Physical startup and tracking recovery seed
from fresh measured robot joints and calibrate the Quest wrist origin there;
connecting does not command a move to the YAML home pose.

## Code Layout

| Package | Responsibility |
| --- | --- |
| `retargeting` | Pure algorithms, kinematics, configuration and metrics |
| `teleoperation` | Inputs, mapping, policies and single/dual-arm execution flows |
| `retargeting_apps` | Hydra/CLI composition, offline artifacts and visualization |
| `retargeting_ros` | ROS and hardware adapters |

Both ROS backend classes now live in `retargeting_ros.dual_crx`. ROS-free joint
mapping lives in `teleoperation.backends.dual_crx_contract`. The unused duplicate
publishers, COACT placeholder path, LEAP-only backend and synthetic bimanual
trajectory demo have been removed. The static initial-pose viewer is retained.
See [configuration and development](docs/configuration-and-development.md) for
other workflows. The design and migration notes are retained locally in
`reorganized.md`, which is not tracked in Git.

## Citation

```bibtex
@article{xin2026analyzing,
  title={Analyzing Key Objectives in Human-to-Robot Retargeting for Dexterous Manipulation},
  author={Xin, Chendong and Yu, Mingrui and Jiang, Yongpeng and Zhang, Zhefeng and Li, Xiang},
  journal={IEEE Robotics and Automation Practice},
  volume={1},
  pages={29--34},
  year={2026},
  doi={10.1109/RAP.2026.3656110}
}
```

## Contact

For questions, contact Mingrui Yu at [mingruiyu98@gmail.com](mailto:mingruiyu98@gmail.com).
