"""Compose a finite synthetic wrist-motion demo through the Sharpa teleop path."""

import argparse
import math
import time


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('preview', 'ros'), default='preview')
    parser.add_argument('--stroke-m', '--amplitude-m', dest='stroke_m', type=float, default=.20,
                        help='Upward travel from start, not +/- amplitude (default: 0.20 m)')
    parser.add_argument('--period', type=float, default=8., help='Seconds per up-and-down cycle')
    parser.add_argument('--cycles', type=int, default=3)
    parser.add_argument('--hold-time', type=float, default=1., help='Settled endpoint hold in seconds')
    parser.add_argument('--settle-timeout', type=float, default=10.)
    parser.add_argument('--tolerance-deg', type=float, default=2., help='ROS feedback/target tolerance')
    parser.add_argument('--command-hz', type=float, default=20.)
    parser.add_argument('--publish-hz', type=float, default=100.)
    parser.add_argument('--interpolation-horizon-ms', type=float, default=50.)
    parser.add_argument('--startup-timeout', type=float, default=5.)
    parser.add_argument('--crx-namespace', default='crx5ia')
    parser.add_argument('--sharpa-namespace', default='sharpa')
    parser.add_argument('--viewer', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--viewer-port', type=int, default=9219)
    # Reuse composition without opening Quest or enabling its duration timer.
    parser.set_defaults(config=None, adb=None, serial=None, duration=0.)
    args = parser.parse_args(argv)
    for name in ('stroke_m', 'period', 'hold_time', 'settle_timeout', 'tolerance_deg',
                 'command_hz', 'publish_hz', 'interpolation_horizon_ms', 'startup_timeout'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name.replace("_", "-")} must be finite and positive')
    if args.cycles < 1 or not math.isfinite(args.period * args.cycles):
        parser.error('cycles must be positive and total duration finite')
    if args.hold_time >= args.settle_timeout:
        parser.error('hold-time must be shorter than settle-timeout')
    if not 1 <= args.viewer_port <= 65535:
        parser.error('viewer-port must be in 1..65535')
    if not args.crx_namespace.strip('/') or not args.sharpa_namespace.strip('/'):
        parser.error('Namespaces cannot be empty')
    return args


def build_flow(args):
    import numpy as np
    from scipy.spatial.transform import Rotation

    from retargeting_apps.sharpa_teleop import build_flow as build_sharpa_flow
    from teleoperation.inputs.vertical_hand import VerticalBimanualInput

    source = VerticalBimanualInput(stroke=args.stroke_m, period=args.period, cycles=args.cycles,
                                  hold_time=args.hold_time, settle_timeout=args.settle_timeout)
    flow, config = build_sharpa_flow(args, with_arms=True, source=source)
    mappers = (flow.pipeline.left_mapper, flow.pipeline.right_mapper)
    axes, keypoints = [], []
    retargeters = (flow.pipeline.left_retargeter, flow.pipeline.right_retargeter)
    for side, mapper, retargeter in zip(('left', 'right'), mappers, retargeters):
        if not mapper.config.use_relative_wrist_alignment or not mapper.align_wrist_rotation:
            raise ValueError('Vertical demo requires relative wrist position and rotation alignment')
        placement = Rotation.from_euler('xyz', config['bimanual'][side]['placement']['rpy']).as_matrix()
        calibration = Rotation.from_euler('xyz', mapper.config.rotation_euler_xyz_deg,
                                           degrees=True).as_matrix()
        axes.append(calibration.T @ placement.T @ np.array([0., 0., 1.]))
        # Fixed open-hand geometry in wrist coordinates, matching the robot's
        # initial hand pose. Undo mapper scaling so world-thumb targets agree
        # with the wrist origin instead of introducing an artificial offset.
        model = retargeter.robot_adaptor.robot_model
        q = retargeter.robot_adaptor.forward_qpos(retargeter.qpos_init)
        wrist = model.get_frame_pose(retargeter.robot_config.wrist_frame_name, qpos=q).copy()
        points = np.zeros((21, 3))
        for finger, name in enumerate(('thumb', 'index', 'middle', 'ring', 'pinky')):
            base = model.get_frame_pose(f'{side}_{name}_DP', qpos=q)[:3, 3].copy()
            tip = model.get_frame_pose(f'{side}_{name}_fingertip', qpos=q)[:3, 3].copy()
            base = wrist[:3, :3].T @ (base - wrist[:3, 3]) / mapper.human_hand_scale
            tip = wrist[:3, :3].T @ (tip - wrist[:3, 3]) / mapper.human_hand_scale
            points[1 + 4 * finger:5 + 4 * finger] = [base / 3, 2 * base / 3, base, tip]
        keypoints.append(points)
    source.sensor_axes = tuple(axes)
    source.keypoints = tuple(keypoints)
    source.max_ack_gap = max(source.max_age_s, 2 * flow.period)

    def elapsed():
        if flow.tracking_paused:
            raise RuntimeError('Vertical demo interrupted; restart to recalibrate from current pose')
        return None if flow.started_at is None else max(0., time.monotonic() - flow.started_at)

    def observe(result):
        reached = True
        if source.last_elapsed >= source.motion_duration and flow.backend is not None:
            actual = flow.backend.get_joint_pos()
            reached = bool(np.all(np.abs(actual - result.qpos) <= math.radians(args.tolerance_deg)))
        source.acknowledge(reached=reached)

    source.elapsed = elapsed
    flow.observer = observe
    return flow, config


def main(argv=None):
    args = parse_args(argv)
    flow = visualizer = None
    try:
        flow, config = build_flow(args)
        if args.viewer:
            from retargeting_apps.visualization.execution.manager import create_optional_execution_visualizer

            monitor = flow.observer
            visualizer = create_optional_execution_visualizer(config, flow)
            display = flow.observer

            def observe(result):
                monitor(result)
                display(result)

            flow.observer = observe
        print(f'CRX + Sharpa synthetic vertical motion; backend={args.backend}; '
              f'stroke={args.stroke_m:g} m upward, period={args.period:g} s, cycles={args.cycles}; '
              'fixed finger gesture; no Quest input.', flush=True)
        flow.run()
        if not flow.source.done:
            raise RuntimeError('Vertical demo stopped before completing the endpoint hold')
        print('Vertical demo completed; stopped sending commands.', flush=True)
        return 0
    except KeyboardInterrupt:
        print('Interrupted; stopped new commands without a return move.', flush=True)
        return 130
    except (ImportError, RuntimeError, ValueError) as exc:
        print(f'Vertical demo failed: {exc}', flush=True)
        return 1
    finally:
        try:
            if flow is not None:
                flow.close()
        finally:
            if visualizer is not None:
                visualizer.close()
