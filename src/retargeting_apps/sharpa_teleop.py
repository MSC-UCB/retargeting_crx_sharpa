"""Compose hand-only or CRX+Sharpa preview and JointState execution."""

import argparse
import math


def build_flow(args, *, with_arms, source=None):
    import numpy as np

    from retargeting.config.io import load_config_source
    from retargeting_apps.composition import build_bimanual_execution_flow
    from retargeting_apps.main import compose_hydra_base_config
    from teleoperation.backends.sharpa_contract import joint_channels
    from teleoperation.parallel_solver import BimanualProcessSolver

    name = 'crx5ia_sharpa_wave' if with_arms else 'sharpa_wave'
    config = compose_hydra_base_config([
        'app=teleop_exe', 'teleoperation_modes=bimanual_quest', 'backends=kinematic',
        f'bimanual={name}',
    ])
    if args.config:
        config['bimanual'].update(load_config_source(args.config))
    config['bimanual']['duration'] = args.duration
    # Independent processes solve the two sides concurrently, 30 ms per side.
    config['solver']['params']['maxtime'] = 0.030
    config['backend']['command_hz'] = args.command_hz
    config['input'].update(adb=args.adb, serial=args.serial)
    config['viewer'].update(enabled=args.viewer, port=args.viewer_port, wait_for_client=False)
    flow = build_bimanual_execution_flow(config, source=source)
    retargeters = (flow.pipeline.left_retargeter, flow.pipeline.right_retargeter)
    names = tuple(tuple(r.robot_config.actuated_joints) for r in retargeters)
    channels = joint_channels(names)
    if ('arms' in channels) != with_arms:
        raise ValueError('Selected profiles do not match the script arm/hand mode')
    if flow.arm_dofs != ((6, 6) if with_arms else (0, 0)):
        raise ValueError('Profile arm_dof does not match the selected Sharpa mode')
    flow.pair_solver = BimanualProcessSolver(retargeters, timeout=flow.timeout)
    if args.backend == 'ros':
        from retargeting_ros.sharpa_joint import SharpaJointBackend

        publish_hz = getattr(args, 'publish_hz', 100.)
        horizon_ms = getattr(args, 'interpolation_horizon_ms', None)
        horizon = flow.period if horizon_ms is None else horizon_ms / 1000.
        if not math.isfinite(publish_hz) or publish_hz <= 0:
            raise ValueError('publish_hz must be finite and positive')
        if not math.isfinite(horizon) or not 0 < horizon < flow.timeout:
            raise ValueError('Interpolation horizon must be positive and shorter than the target timeout')
        limits = np.concatenate([r.optimizer.joint_limits for r in retargeters])
        flow.backend_factory = lambda: SharpaJointBackend(
            robot_names=names, initial_qpos=flow.initial_qpos,
            lower=limits[:, 0], upper=limits[:, 1], control_period=flow.period,
            startup_timeout=args.startup_timeout, target_timeout=flow.timeout,
            publish_hz=publish_hz, interpolation_horizon=horizon,
            crx_namespace=args.crx_namespace, sharpa_namespace=args.sharpa_namespace,
        )
    return flow, config


def main(argv=None, *, with_arms=True):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('preview', 'ros'), default='preview',
                        help='preview uses no ROS; ros publishes to already running drivers')
    parser.add_argument('--config', default=None, help='Optional bimanual YAML')
    parser.add_argument('--adb', default=None, help='ADB executable; defaults to PATH discovery')
    parser.add_argument('--serial', default=None)
    parser.add_argument('--viewer', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--viewer-port', type=int, default=9219)
    parser.add_argument('--command-hz', type=float, default=20.)
    parser.add_argument('--publish-hz', type=float, default=100.,
                        help='ROS linear-interpolation publication rate; independent of solving (default: 100)')
    parser.add_argument('--interpolation-horizon-ms', type=float, default=None,
                        help='ROS interpolation duration; defaults to 1000 / command-hz, normally 50 ms')
    parser.add_argument('--duration', type=float, default=0., help='Seconds after initialization; 0 is unlimited')
    parser.add_argument('--startup-timeout', type=float, default=5.)
    parser.add_argument('--crx-namespace', default='crx5ia')
    parser.add_argument('--sharpa-namespace', default='sharpa')
    parser.add_argument('--synthetic-frames', type=int, default=None,
                        help='Explicit finite smoke input instead of opening Quest (use ROS mock only)')
    args = parser.parse_args(argv)
    for name in ('command_hz', 'startup_timeout', 'publish_hz'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be finite and positive')
    if args.interpolation_horizon_ms is not None and (
            not math.isfinite(args.interpolation_horizon_ms) or args.interpolation_horizon_ms <= 0):
        parser.error('interpolation-horizon-ms must be finite and positive')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and nonnegative')
    if not 1 <= args.viewer_port <= 65535:
        parser.error('viewer-port must be in 1..65535')
    if args.synthetic_frames is not None and args.synthetic_frames < 1:
        parser.error('synthetic-frames must be positive')
    if not args.crx_namespace.strip('/') or not args.sharpa_namespace.strip('/'):
        parser.error('Namespaces cannot be empty')
    source = None
    if args.synthetic_frames is not None:
        from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput
        source = SyntheticBimanualInput(args.synthetic_frames)
        print('Synthetic smoke input selected; this does not validate Quest tracking.', flush=True)
    flow, config = build_flow(args, with_arms=with_arms, source=source)
    visualizer = None
    try:
        if args.viewer:
            from retargeting_apps.visualization.execution.manager import create_optional_execution_visualizer
            visualizer = create_optional_execution_visualizer(config, flow)
        print(f'Sharpa {"arms + hands" if with_arms else "hands only"}; backend={args.backend}; '
              'two solver processes, 30 ms per side', flush=True)
        flow.run()
    except KeyboardInterrupt:
        pass
    finally:
        if visualizer is not None:
            visualizer.close()


if __name__ == '__main__':
    main()
