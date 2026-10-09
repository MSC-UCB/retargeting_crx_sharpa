# Configuration and Development Guide

## Configuration

Robot-specific, method-specific, and robot-method profile values are configured in YAML files instead of being edited directly in Python source.

| Config | Purpose |
| --- | --- |
| `configs/base.yaml` | Unified Hydra entry config; select an application with `app=<name>`. |
| `configs/app/offline_retarget.yaml` | Offline retargeting app defaults and required config groups. |
| `configs/app/replay.yaml` | Saved-artifact replay viewer defaults. |
| `configs/app/benchmark.yaml` | Benchmark app defaults. |
| `configs/robots/panda_leap_paxini.yaml` | Panda arm + Leap hand with Paxini fingertips. |
| `configs/robots/panda_shadow.yaml` | Panda arm + Shadow hand. |
| `configs/retargeting_methods/vector_wrist_joint.yaml` | Vector wrist joint method metadata and optimizer defaults. |
| `configs/retargeting_profiles/vector_wrist_joint_panda_leap_paxini.yaml` | Panda+Leap profile binding robot, method, target links, objective weights, retargeting runtime weights, and teleoperation command limits. |
| `configs/retargeting_profiles/vector_wrist_joint_panda_shadow.yaml` | Panda+Shadow profile binding robot, method, target links, objective weights, retargeting runtime weights, and teleoperation command limits. |
| `configs/solvers/nlopt_slsqp.yaml` | NLopt SLSQP backend and stopping/runtime parameters. |
| `configs/solvers/scipy_slsqp.yaml` | SciPy SLSQP backend and stopping/runtime parameters. |

Replay accepts an offline retarget runtime name and viewer overrides:

```bash
python -m retargeting_apps.main app=replay \
  run_name=quickstart_leap \
  viewer.port=8090 \
  viewer.no_robot_mesh=true
```

When adding a new robot, prefer this route:

1. Add robot assets under `assets/robots/<robot_name>/`.
2. Add a robot config under `configs/robots/<robot_name>.yaml`.
3. Put joints, frames, model paths, initial qpos, and hand scale in the config.
4. Put robot-method-specific target links, objective weights, and retargeting runtime weights in `configs/retargeting_profiles/<method>_<robot_name>.yaml`; keep command limits in its `teleoperation` section.
5. Reuse or add method-level optimizer defaults under `configs/retargeting_methods/`.

Avoid hard-coding robot-specific joint names, link names, URDF paths, or initial poses in core Python modules.

## Robot Assets

The current core/offline asset layout is:

```text
assets/
├── meshes/
│   ├── leap_hand/
│   ├── panda/
│   └── shadow_hand/
├── robots/
│   ├── panda_leap_paxini/
│   │   ├── manifest.yaml
│   │   ├── meshes/
│   │   ├── mjcf/
│   │   └── urdf/
│   └── panda_shadow/
└── scenes/
```

Robot configs should point to stable paths under `assets/robots/`. `panda_leap_paxini` is a self-contained portable
bundle: its manifest exposes both URDF and MJCF entry points, both descriptions resolve only bundle-local meshes, and
the bundle contains no symlinks. `panda_shadow` continues to reuse component meshes under `assets/meshes/` through
robot-local `panda` and `shadow_hand` symlinks. The shared component directories remain available for that layout and
compatibility with older assets.

For ROS robot description work, Xacro/URDF files are still available under `ws_ros2/src/my_robot_description/`.

## Data And Outputs

The repository includes a promoted replay fixture for tests and quickstart:

```text
tests/fixtures/avp_teleop_2025-01-16_20-27-43.npz
```

Large experiment data and full benchmark datasets are not bundled as the default quickstart path. The `data/` directory is treated as a local or historical experiment-data location. See [data/README.md](../data/README.md) for the current boundary.

New generated files should go under `outputs/`, for example:

```text
outputs/teleop/
outputs/simulation/
outputs/<run_name>/retargeting/
outputs/<run_name>/benchmark/
outputs/<run_name>/plots/
```

Saved offline retargeting, benchmark, and plot artifacts use this layout:

```text
outputs/<run_name>/
  retargeting/
    result.npz
    metadata.yaml
  benchmark/
    metrics.json
    summary.csv
  plots/
    benchmark_metric_means.png
    benchmark_metric_means.pdf
```

`result.npz` stores qpos, human keypoints, wrist poses, robot frame poses, and per-frame optimization errors. `metadata.yaml` stores the replay source, robot config, retargeting config, frame range, and result schema version.

`outputs/` is gitignored.

## ROS And RViz

ROS/RViz is optional and is not required for offline replay.

Legacy Panda ROS paths target Ubuntu 22.04 and ROS2 Humble. The dual-CRX gateway uses ROS2 Jazzy and system Python 3.12. Use a local virtual environment matching the selected ROS distribution.

Install ROS2 Humble by following the [official ROS2 Humble instructions](https://docs.ros.org/en/humble/Installation.html), then install the required ROS packages:

```bash
sudo apt-get install python3-colcon-common-extensions
sudo apt-get install ros-humble-xacro
sudo apt-get install ros-humble-robot-state-publisher
sudo apt-get install ros-humble-joint-state-publisher
sudo apt-get install ros-humble-joint-state-publisher-gui
```

Build the ROS2 workspace:

```bash
cd ws_ros2
colcon build --symlink-install
source install/setup.bash
```

Example RViz launch:

```bash
ros2 launch retargeting_benchmark rviz_vis_paxini.py
```

Python-level ROS integration lives in `src/retargeting_ros/`. Compatibility scripts are still kept under `ws_ros2/src/retargeting_benchmark/src/` so existing launch files and older user commands do not break immediately.

## Live Teleoperation

Live teleoperation is an advanced path. It may require ROS, a camera, Vision Pro, or Quest 3 live stream, optional visualization dependencies, and robot-specific setup.

The legacy compatibility entrypoint is:

```bash
source ws_ros2/install/setup.bash
python ws_ros2/src/retargeting_benchmark/src/main_robot_teleoperation.py
```

Generated teleoperation recordings now default to `outputs/teleop/`.

Optional live-input dependencies:

- RGB hand detection: `pip install -e ".[vision]"`
- Vision Pro streaming: `pip install -e ".[avp]"`
- Quest 3 USB/WebXR streaming: `pip install -e ".[quest3]"`
- MuJoCo-related paths: `pip install -e ".[mujoco]"`
- MuJoCo Web visualization: `pip install -e ".[mujoco-web]"`
- ROS/RViz/hardware: ROS2 Humble workspace and robot drivers

## Real Robot Control

### dual_crx_ros2 integration

For dual-arm Quest execution with an optional left LEAP, see
[bimanual physical execution](bimanual_physical.md). The right-only commands below remain supported.

The CRX+LEAP profile can publish directly to the dual-crx ROS 2 gateway when
the retargeting process runs in a Python 3.12 environment that can import both
ROS Jazzy and the retargeting dependencies. Select the opt-in backend with:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_fanuc/install/setup.bash
source ~/dual_crx_ros2/install/setup.bash
python -m retargeting_apps.main app=teleop_exe \
  retargeting_profiles=vector_wrist_joint_crx5ia_leap_paxini \
  teleoperation_modes=real_world \
  +inputs=quest3 +backends=dual_crx
```

The backend acquires the RIGHT lease, enables the hand and teleoperation
session, publishes `dual_crx_interfaces/msg/TeleopCommand` at the configured
rate, renews the lease, and stops/releases on shutdown. It is intentionally
not the default backend. Verify the combined interpreter first with:

```bash
python -c "import rclpy, hydra, pinocchio, nlopt; print('ROS + retargeting imports OK')"
```

Physical use still requires the installed hand model, bounds, frames, and
FANUC command path to be validated. Use the dual-crx fake launch for the first
end-to-end run.

Real robot control is lab-specific and is not required for offline replay. Confirm robot safety, ROS networking, drivers, and emergency-stop procedures before running any hardware command.

The original lab setup targeted a Franka Panda arm with a Leap hand. The IP addresses below are historical examples from that environment, not portable defaults.

1. Unlock the Panda arm and activate FCI in the robot desk UI.

2. Launch the Franka driver on the Franka control PC:

   ```bash
   ssh robotics@192.168.52.5
   cd franka_emika_panda/ws_ros2/
   source install/setup.bash
   ros2 launch franka_bringup low_level_joint_impedance_controller.launch.py arm_id:=fer robot_ip:=192.168.52.3
   ```

3. Launch Leap hand bringup:

   ```bash
   source .venv/bin/activate
   ros2 launch leap_hand leap_bringup.py
   ```

4. Prepare real robot ROS nodes:

   ```bash
   source .venv/bin/activate
   ros2 launch retargeting_benchmark real_prepare.py
   ```

5. Run the teleoperation entrypoint only after confirming the hardware state and ROS topics.

Leap hand hardware details are in [ws_ros2/src/leaphand_ros2_module/readme.md](../ws_ros2/src/leaphand_ros2_module/readme.md).

## Development

### Local Test Environment

Use a repository-local virtual environment created with system Python, without conda:
For a new Linux user, clone into that user's own directory and create a fresh
`.venv`; copied virtual environments retain paths to their original location.

```bash
git submodule update --init --recursive
/usr/bin/python3 -m venv .venv
env -u PYTHONPATH -u LD_LIBRARY_PATH .venv/bin/python -m pip install -e ".[dev,quest3]" pin scikit-learn
env -u PYTHONPATH -u LD_LIBRARY_PATH .venv/bin/python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
env -u PYTHONPATH -u LD_LIBRARY_PATH .venv/bin/python -m pytest tests/test_bimanual_quest.py tests/test_bimanual_execution.py -q
```

On systems without `ensurepip`, create the environment without pip and bootstrap pip inside it:

```bash
/usr/bin/python3 -m venv --without-pip .venv
curl -fL https://bootstrap.pypa.io/get-pip.py -o /tmp/retargeting-crx-get-pip.py
env -u PYTHONPATH -u LD_LIBRARY_PATH .venv/bin/python /tmp/retargeting-crx-get-pip.py
```

Then run the installation commands above. The `pin` distribution provides `pinocchio`;
`scikit-learn` and CPU PyTorch are needed by the retargeter imports. These tests require
no ROS installation, connected headset, viewer, or hardware. Optional simulation and
viewer tests require their respective extras. `.venv/` is ignored by Git.
The `env -u PYTHONPATH -u LD_LIBRARY_PATH` prefix prevents inherited Python and
native-library paths from contaminating the standalone test environment. Clearing
only `PYTHONPATH` can still load ROS's `libeigenpy.so` instead of the virtualenv's
version, causing Pinocchio imports to fail with
`undefined symbol: EIGENPY_ARRAY_APIPyArray_RUNTIME_VERSION`. Use both overrides
for headless checks in a ROS-sourced shell; retain ROS paths when doing explicit
ROS work. The overrides affect only the invoked command.

### Package Boundaries

The Python packages follow one dependency direction:

```text
retargeting_apps -> teleoperation -> retargeting
       |                              ^
       +------------------------------+

retargeting_apps -> retargeting_ros (lazy ROS backend selection)
retargeting_ros -> teleoperation / retargeting
```

- `retargeting` owns canonical domain types, core config, kinematics, optimizers, solvers, and pure metrics.
- `teleoperation` owns sensor-first inputs, observation mapping, output/command policies, robot backends, and flows.
- `retargeting_apps` is the only Hydra/CLI composition root and owns artifacts, reports, and replay visualization.
  It may lazily select backends from `retargeting_ros`; runtime inputs and flows never import ROS adapters.
- Every `configs/app/<id>.yaml` maps to `retargeting_apps.apps.<id>.run(config, argv)`. The dispatcher imports only the
  selected whitelisted task. `retargeting_apps.composition` builds complete flows; apps do not own per-frame loops.
- `retargeting_apps.offline_retargeting` and `retargeting_apps.benchmark_report` own artifact/report workflows. There
  is no `retargeting_apps.pipelines` facade.
- `retargeting_ros` owns optional ROS, RViz, and real-robot adapters. Compatibility scripts under `ws_ros2/` import
  these canonical packages but are not imported by them.

Core code must not import `teleoperation`, `retargeting_apps`, `retargeting_ros`, or optional runtime/viewer modules.
Keep `retargeting.core.*` as the public algorithm path; do not flatten it as part of unrelated changes.

The execution data path is:

```text
HandInput -> SensorHandSample -> HandObservationMapper -> RetargetingHandObservation
          -> Retargeter -> QposOutputFilter -> QposCommandLimiter -> RobotBackend.execute
```

`ExecutionFlow` owns this sequence for single-arm execution; `BimanualExecutionFlow` coordinates two independent
mapper/retargeter pairs in one loop for synchronized dual-arm execution. It also owns mapping initialization, missing-input
hold behavior, wall-clock pacing, source/command counters, passive observers, and the reset contract. AVP live and
archived acquisition share `teleoperation.inputs.avp.common.decode_avp_sample`; Quest transport and conversion live
under `teleoperation.inputs.quest3`. The optional `avp_stream` and `aiohttp` dependencies are imported only by live
input paths. Offline artifact generation uses `BatchRetargetFlow`, skips missing
samples, and never creates a robot backend or execution result.

After installation, both entry forms below use the same app registry and Hydra overrides:

```bash
retargeting app=offline_retarget end=1 run_name=smoke
python -m retargeting_apps.main app=offline_retarget end=1 run_name=smoke
```

### Headless Checks

Run the headless test suite from the repository root:

```bash
python -m pytest tests
```

Check the core and ROS adapter import boundary:

```bash
python -c "import retargeting; import retargeting_ros"
```

Check Hydra replay config composition without starting the viewer:

```bash
python -c "from retargeting_apps.main import compose_hydra_base_config; cfg = compose_hydra_base_config(['app=replay','run_name=quickstart_leap','viewer.port=8090']); print(cfg['run_name'], cfg['viewer']['port'])"
```

Check Hydra offline retarget config composition:

```bash
python -c "from retargeting_apps.main import compose_hydra_base_config; cfg = compose_hydra_base_config(['app=offline_retarget','end=1','run_name=smoke']); print(cfg['data'], cfg['run_name'])"
```

Default tests should not start ROS, RViz, cameras, Vision Pro live streaming, real robots, Open3D GUI, or MuJoCo viewer.

## Teleoperation Execution

`app=teleop_exe` is the unified execution path, not a saved-trajectory replay. Teleoperation setup selection lives in
`teleoperation_modes`; each setup composes `input`, `backend`, and pipeline policy. The same `ExecutionFlow` runs
online AVP, online Quest 3, archived AVP, MuJoCo, and pure kinematic execution. Dual-arm Quest selects
`teleoperation_modes=bimanual_quest` through this same app and uses `BimanualExecutionFlow`.
See the [README](../README.md#dual-arm-quest-execution) for preview and ROS commands,
[frequency settings](../README.md#frequency-and-output-filtering), and the
[ROS contract](../README.md#ros-interface). For live AVP into MuJoCo:

```bash
python -m retargeting_apps.main app=teleop_exe \
  teleoperation_modes=online_mujoco input.avp_ip=192.168.52.6
```

For Quest 3, connect an ADB-authorized headset over USB and run the kinematic
backend before attempting MuJoCo or hardware:

```bash
python -m retargeting_apps.main app=teleop_exe \
  teleoperation_modes=online_quest3_kinematic
```

`Quest3OnlineInput` owns the local WebXR receiver, ADB port forwarding, and
Quest Browser launch. It selects the newest complete configured hand, rejects
stale frames, converts WebXR's 25 joints to the 21-joint MANO order, then feeds
the same relative-wrist mapper and 23-DOF Panda+LEAP solver used by the other
execution modes. Runtime overrides include `input.hand_side`, `input.port`,
`input.adb`, `input.serial`, `input.max_age_s`, and `input.max_frames`.

The app takes the latest selected live-input frame, retargets it once, applies the actuator-range policy, sends the resulting qpos to
the configured backend, and advances one 20 Hz command period before accepting the next frame. With the default
`startup_move_frames=0`, there is no explicit target-speed limit. The MuJoCo backend's `0.002 s` physics timestep
makes one command period exactly 25 MuJoCo steps.

Robot-specific MJCF paths live under `simulation_model` in the robot config. Runtime timing and range behavior live
in `configs/backends/*.yaml`; viewer dependencies are intentionally absent from the backend. For headless diagnostics,
override `teleoperation_mode.pipeline.realtime=false` so the same fixed execution time is advanced without wall-clock
sleeping. Select `teleoperation_modes=online_kinematic` or `teleoperation_modes=offline_kinematic` for pure
command/actual qpos execution with no MuJoCo dependency.

For raw offline human input, select `input.mode=offline` and set `input.data`. This app loads only the input NPZ's
`stream_*` arrays and explicitly ignores any existing `retarget_qpos`. It initializes wrist alignment from the first
valid raw human frame and retargets that same frame. The default `offline_kinematic` setup uses
`startup_move_frames=0`, while `offline_mujoco` uses `startup_move_frames=1` and synchronously moves the actuator
target to the first valid retarget result before consuming the next source frame. The command policy selects a
shared integer waypoint count using `ceil(max(abs(delta) / (max_joint_speed / command_hz)))`; all joints therefore
finish together and every commanded increment respects its configured speed. After startup, each valid target is
sent directly without an explicit speed limit and advances one command period. Actuator ranges and MuJoCo force,
servo, contact, and dynamics constraints remain active in both phases.

Frames without a valid hand hold the preceding robot command for one period and do not consume the startup count.
Offline execution selects `input.start` and `input.end` as a contiguous interval. Set `input.loop=true` to repeat that
selected interval until Ctrl+C. Each repeated cycle resets the backend to the configured robot initial qpos, clears
retargeter/filter/startup temporal state, and initializes relative wrist alignment again from the first valid frame.
The configured `input.source_hz` must match the backend command rate until timestamp-based resampling is added. During
startup, one source frame can advance multiple command periods, so simulation time intentionally exceeds the source
timeline.

### Execution Web Visualization

`app=teleop_exe` can publish execution through a backend-aware Web viewer. With `viewer.type=auto`, a kinematic
backend uses the standard Viser URDF adapter and a MuJoCo backend uses the passive `mjviser` adapter. Install
`.[replay]` for standard Viser or `.[mujoco-web]` for the MuJoCo path:

```bash
pip install -e ".[mujoco-web]"
python -m retargeting_apps.main app=teleop_exe teleoperation_modes=offline_mujoco \
  viewer.enabled=true teleoperation_mode.pipeline.realtime=true input.loop=true
```

The adapter uses the same `MjModel` and `MjData` owned by `MujocoRobotBackend` and calls
`ViserMujocoScene.update_from_mjdata()` after every flow command period, including every startup interpolation
waypoint. It never calls `mj_step`; `ExecutionFlow` remains the sole owner of backend advancement. Viewer settings are
application configuration rather than simulator-backend configuration:

The returned mjviser tab group is extended with a read-only `Joint angles` tab. The adapter enumerates compiled
MuJoCo joint ids, resolves each hinge joint through `model.jnt_qposadr`, and displays the corresponding actual
`data.qpos` value in radians. The current Panda+Leap MJCF has 23 hinge joints, all of which are updated atomically
after every viewer frame. No input callback is registered, so these fields cannot modify simulation state.

The MuJoCo viewer also overlays the mapped Quest target wrist and the actual robot wrist. Both use the standard
red-X, green-Y, blue-Z axes; the target has a yellow origin and `Quest wrist (target)` label, while the actual frame
has a magenta origin and `Panda wrist (actual)` label. The read-only `Wrist diagnostics` tab reports their Euclidean
origin distance in centimetres and shortest relative rotation angle in degrees. These diagnostics are passive and
do not alter retargeting, commands, or MuJoCo advancement.

Both `mjviser` and standard `viser` execution adapters share the same human-hand renderer. It transforms each
canonical `RetargetingHandObservation` into robot-world MANO keypoints, skeleton segments, and a wrist frame, then
updates persistent Viser handles once per completed source frame. A source frame without a valid observation, or a
flow reset, hides the human nodes instead of leaving stale geometry visible. Robot state publication remains on the
command-period observer so startup interpolation is still shown at full resolution. The mjviser adapter mounts its
human nodes below mjviser's `/fixed_bodies` frame, which applies the same camera-tracking `scene_offset` used by the
MuJoCo geometry. Turning `Track camera` on or off therefore moves both layers together.

```yaml
viewer:
  enabled: false
  type: auto
  host: 0.0.0.0
  port: 9219
  wait_for_client: true
  keep_open_after_completion: false
  camera_distance: -1.0
  camera_azimuth: 120.0
  camera_elevation: 20.0
  human_keypoint_size: 0.005
  initial_camera_position: [0.6, 0.6, 0.5]
  initial_camera_look_at: [0.0, 0.0, 0.45]
```

With `wait_for_client=true`, no source frame is consumed until a browser connects. Use
`keep_open_after_completion=true` to retain the final scene until Ctrl+C. Binding `0.0.0.0` exposes the server on
all interfaces; bind `127.0.0.1` and forward the configured port over SSH when direct network exposure is not
appropriate. Offline execution with the recommended config can use `teleoperation_mode.pipeline.realtime=false` for
faster-than-wall-clock runs; explicitly
enable realtime pacing for a human-observable 20 Hz run. Viewer publication occurs before each period's remaining
sleep, so its cost is included in realtime pacing without changing simulated time. In continuous mode, the same
viewer remains bound to the existing `MjModel` and `MjData`; the app publishes the reset state before processing the
next cycle. MuJoCo simulation time restarts at zero for every cycle, and Ctrl+C closes the viewer without applying
`keep_open_after_completion`.


Physical dual-CRX startup (2026-09-08): the backend requires fresh complete arm/hand
feedback and seeds its startup target from measured positions after hand activation.
It holds that pose while waiting for the first input rather than commanding the
profile's simulation home. Startup failure and close attempt software stop, hand
torque disable and lease release. Hardware authority remains an operator-controlled
step in the dual-crx stack. Focused verification: `tests/test_dual_crx_startup.py`.


### Live Quest checkpoint (2026-09-08)

The operator reports successful brief physical Quest teleoperation. The supplied
log at 1788919906.231 records a right_J6 position-limit warning, followed at
1788919906.241 by the gateway stopping on Servo warning/halt or stale status.
Controller-loop overruns also occurred (one reported loop about 8.15 ms against
a 2 ms period); sustained timing and communication remain unresolved. This is
a working checkpoint, not full-workspace or long-duration acceptance. J6 bounds
remain -225 to +225 degrees, with the existing Servo 0.12 rad margin unchanged.
No collision-model or joint-limit expansion was made.

The existing browser viewer can be explicitly selected with
`viewer.enabled=true viewer.type=viser viewer.wait_for_client=false`; open
http://localhost:9219. With `backends=dual_crx` the command still controls hardware.
Use `backends=kinematic` for visualization without physical commands. The viewer
command was inspected in source; successful live viewer operation is not yet
confirmed. Run launches in foreground terminals at the operator's request.
