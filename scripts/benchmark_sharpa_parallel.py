"""Offline prototype only: compare serial, threads and spawned Sharpa solvers.

Run from the repository root. No ROS, viewer, USB, SDK or live flow is started.
Each mode replays identical observations AND seeds from a serial reference.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import json
import multiprocessing as mp
import os
from pathlib import Path
import platform
import resource
import time
import traceback
from types import SimpleNamespace


def numeric_threads(count):
    if count:
        for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                     'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
            os.environ[name] = str(count)
    import torch

    if count:
        torch.set_num_threads(count)
        torch.set_num_interop_threads(count)


def build_offline_flow(frames=1):
    from retargeting_apps.sharpa_teleop import build_flow
    from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput

    args = SimpleNamespace(config=None, backend='preview', duration=0., command_hz=20.,
                           adb=None, serial=None, viewer=False, viewer_port=9219)
    flow, _ = build_flow(args, with_arms=True, source=SyntheticBimanualInput(frames=frames))
    assert flow.backend_factory is None
    return flow


def retargeters(flow):
    return flow.pipeline.left_retargeter, flow.pipeline.right_retargeter


def solve_one(retargeter, request):
    import numpy as np

    frame, observation, seed, budget_ms = request
    opt = retargeter.optimizer.opt._opt
    opt.set_maxtime(budget_ms / 1000.)
    start = time.perf_counter()
    result = retargeter.solve(observation, previous_qpos=seed.copy())
    elapsed = (time.perf_counter() - start) * 1000.
    q = result.qpos
    bounds = retargeter.optimizer.joint_limits
    if not np.isfinite(q).all() or np.any(q < bounds[:, 0] - 1e-6) or np.any(q > bounds[:, 1] + 1e-6):
        raise ValueError('Solver produced invalid joint positions')
    # Evaluate the returned (clipped/float32) qpos, not NLopt's internal endpoint.
    cost = float(retargeter.optimizer.opt._state.objective(q, np.empty(0)))
    return dict(frame=frame, qpos=q, solve_ms=elapsed, cost=cost,
                evaluations=opt.get_numevals(), status=opt.last_optimize_result(),
                peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.)


def process_worker(connection, side, native_threads):
    """Own one solver; EOF exits idle workers when the parent disappears."""
    try:
        numeric_threads(native_threads)
        # Reuse canonical composition, then release the unused side and flow.
        solver = retargeters(build_offline_flow())[side]
        connection.send(('ready', dict(pid=os.getpid(), peak_rss_mib=
            resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.)))
        while True:
            request = connection.recv()
            if request is None:
                return
            connection.send(('result', solve_one(solver, request)))
    except (EOFError, BrokenPipeError):
        pass
    except BaseException:
        try:
            connection.send(('error', traceback.format_exc()))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        connection.close()


class ProcessPair:
    """Bounded request/reply prototype; never queue another frame while solving."""

    def __init__(self, native_threads=1, timeout=10., startup_timeout=60.,
                 close_timeout=2., worker=process_worker):
        self.timeout, self.close_timeout = timeout, close_timeout
        self.processes, self.connections, self.ready, self.cleanup = [], [], [], []
        self.closed = False
        context = mp.get_context('spawn')
        try:
            for side in range(2):
                parent, child = context.Pipe()
                process = context.Process(target=worker, args=(child, side, native_threads),
                                          name=f'sharpa-benchmark-{side}')
                try:
                    process.start()
                except BaseException:
                    parent.close()
                    raise
                finally:
                    child.close()
                self.connections.append(parent)
                self.processes.append(process)
            deadline = time.monotonic() + startup_timeout
            self.ready = [self._receive(c, deadline, 'ready') for c in self.connections]
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _receive(connection, deadline, expected):
        if not connection.poll(max(0., deadline - time.monotonic())):
            raise TimeoutError(f'Worker timed out waiting for {expected}')
        try:
            kind, value = connection.recv()
        except EOFError as exc:
            raise RuntimeError('Worker exited without a reply') from exc
        if kind != expected:
            raise RuntimeError(f'Worker {kind}: {value}')
        return value

    def solve(self, requests):
        if self.closed:
            raise RuntimeError('Process pair already closed')
        try:
            if len(requests) != 2 or requests[0][0] != requests[1][0]:
                raise ValueError('Expected matching left/right frame IDs')
            deadline = time.monotonic() + self.timeout
            for connection, request in zip(self.connections, requests):
                connection.send(request)
            results = [self._receive(c, deadline, 'result') for c in self.connections]
            if any(r['frame'] != requests[0][0] for r in results):
                raise RuntimeError('Worker returned an obsolete frame')
            return results
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.closed:
            return
        self.closed = True
        for connection in self.connections:
            try:
                connection.send(None)
            except (OSError, EOFError):
                pass
            connection.close()
        deadline = time.monotonic() + self.close_timeout
        for process in self.processes:
            process.join(max(0., deadline - time.monotonic()))
        for process in self.processes:
            method = 'graceful'
            if process.is_alive():
                method = 'terminate'
                process.terminate()
                process.join(self.close_timeout)
            if process.is_alive():
                method = 'kill'
                process.kill()
                process.join(self.close_timeout)
            alive = process.is_alive()
            self.cleanup.append(dict(pid=process.pid, method=method, alive=alive, exitcode=process.exitcode))
            if not alive:
                process.close()
        if any(item['alive'] for item in self.cleanup):
            raise RuntimeError(f'Unable to reap benchmark workers: {self.cleanup}')


def reference_sequence(frames):
    """Frozen replay with finger flexion plus modest wrist translation/rotation."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    from teleoperation.types import BimanualSensorHandSample

    flow = build_offline_flow(frames)
    pipeline = flow.pipeline
    solvers = retargeters(flow)
    seeds = [r.qpos_init.copy() for r in solvers]
    flow.source.open()
    records = []
    try:
        for index in range(frames):
            sample = flow.source.read()
            hands = []
            for side, hand in enumerate((sample.left, sample.right)):
                phase = index * .07
                pose = np.eye(4)
                pose[:3, :3] = Rotation.from_rotvec(
                    [.08 * np.sin(phase), .06 * np.sin(phase * .8), 0.]).as_matrix()
                pose[:3, 3] = [.015 * np.sin(phase), (-1)**side * .01 * np.sin(phase),
                              .01 * np.sin(phase * .6)]
                hands.append(replace(hand, wrist_pose_sensor=pose))
            sample = BimanualSensorHandSample(*hands)
            if index == 0:
                assert pipeline.initialize(sample, *seeds)
            observations = [replace(mapper.map(hand), raw=None, timestamp=0., handedness=side)
                            for mapper, hand, side in zip(
                                (pipeline.left_mapper, pipeline.right_mapper), hands, ('left', 'right'))]
            requests = [(index, obs, seed.copy(), 0.) for obs, seed in zip(observations, seeds)]
            reference = [solve_one(r, request) for r, request in zip(solvers, requests)]
            records.append(dict(requests=requests, reference=reference))
            # Canonical arm/hand alphas, only to define common next-frame seeds.
            for side in range(2):
                alpha = np.r_[np.full(6, .5), np.full(22, .3)]
                seeds[side] = alpha * reference[side]['qpos'] + (1. - alpha) * seeds[side]
    finally:
        flow.source.close()
    return records


def summarize(rows):
    import numpy as np

    elapsed = [r['pair_ms'] for r in rows]
    return dict(samples=len(rows), pair_p50_ms=float(np.median(elapsed)),
                pair_p95_ms=float(np.percentile(elapsed, 95)), pair_max_ms=max(elapsed),
                over_50ms=sum(t > 50 for t in elapsed),
                evaluations_mean=float(np.mean([r['evaluations'] for r in rows])),
                maxtime_fraction=float(np.mean([s == 6 for r in rows for s in r['status']])),
                cost_gap_mean=float(np.mean([r['cost_gap'] for r in rows])),
                q_rmse_rad=float(np.sqrt(np.mean([r['q_mse'] for r in rows]))),
                q_max_abs_rad=max(r['q_max_abs'] for r in rows),
                worker_peak_rss_mib=[max(r['peak_rss_mib'][i] for r in rows) for i in range(2)])


def memory_snapshot(pid):
    """Linux current RSS/PSS, outside timed work; peak RSS is not added memory."""
    values = {}
    for line in Path(f'/proc/{pid}/smaps_rollup').read_text().splitlines():
        parts = line.split()
        if parts[0] in ('Rss:', 'Pss:'):
            values[parts[0][:-1].lower() + '_mib'] = float(parts[1]) / 1024.
    return dict(pid=pid, **values)


def run_case(mode, budget_ms, records, warmup, native_threads):
    import numpy as np

    start = time.perf_counter()
    pair = pool = None
    cleanup = []
    try:
        if mode == 'process':
            pair = ProcessPair(native_threads)
        else:
            solvers = retargeters(build_offline_flow())
            if mode == 'thread':
                pool = ThreadPoolExecutor(max_workers=2)
        startup = (time.perf_counter() - start) * 1000.
        rows = []
        for index, record in enumerate(records):
            requests = [(*req[:3], budget_ms) for req in record['requests']]
            start = time.perf_counter()
            if pair is not None:
                result = pair.solve(requests)
            elif pool is not None:
                futures = [pool.submit(solve_one, r, req) for r, req in zip(solvers, requests)]
                result = [future.result() for future in futures]
            else:
                result = [solve_one(r, req) for r, req in zip(solvers, requests)]
            elapsed = (time.perf_counter() - start) * 1000.
            if index < warmup:
                continue
            reference = record['reference']
            error = np.concatenate([r['qpos'] - ref['qpos'] for r, ref in zip(result, reference)])
            rows.append(dict(frame=index, pair_ms=elapsed, solve_ms=[r['solve_ms'] for r in result],
                             evaluations=[r['evaluations'] for r in result],
                             status=[r['status'] for r in result],
                             cost_gap=[r['cost'] - ref['cost'] for r, ref in zip(result, reference)],
                             q_mse=float(np.mean(error**2)), q_max_abs=float(np.max(np.abs(error))),
                             peak_rss_mib=[r['peak_rss_mib'] for r in result]))
        memory = dict(parent=memory_snapshot(os.getpid()), workers=[] if pair is None else
                      [memory_snapshot(p.pid) for p in pair.processes])
    finally:
        start = time.perf_counter()
        if pair is not None:
            pair.close()
            cleanup = pair.cleanup
        if pool is not None:
            pool.shutdown(wait=True, cancel_futures=True)
        shutdown = (time.perf_counter() - start) * 1000.
    return dict(mode=mode, budget_ms=budget_ms, startup_ms=startup, shutdown_ms=shutdown,
                cleanup=cleanup, memory=memory, summary=summarize(rows), rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=int, default=100, help='Measured frames per case')
    parser.add_argument('--warmup', type=int, default=20)
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--native-threads', type=int, default=1, help='0 keeps library defaults')
    parser.add_argument('--output', type=Path, required=True, help='New JSON artifact path')
    args = parser.parse_args()
    if min(args.frames, args.rounds) < 1 or min(args.warmup, args.native_threads) < 0:
        parser.error('frames/rounds must be positive; warmup/native-threads nonnegative')
    if args.output.exists():
        parser.error('output already exists; choose a new artifact path')
    numeric_threads(args.native_threads)
    import numpy as np
    import torch
    import nlopt

    metadata = dict(python=platform.python_version(), numpy=np.__version__, torch=torch.__version__,
                    nlopt=nlopt.__version__, cpu_count=os.cpu_count(),
                    cpu_affinity=len(os.sched_getaffinity(0)), native_threads=args.native_threads,
                    torch_threads=torch.get_num_threads(), torch_interop=torch.get_num_interop_threads(),
                    frames=args.frames, warmup=args.warmup, rounds=args.rounds,
                    start_method='spawn', scope='fixed observations and reference seeds; no live runtime')
    print(json.dumps(metadata), flush=True)
    records = reference_sequence(args.frames + args.warmup)
    cases = [('serial', 25.), ('thread', 25.), ('thread', 35.), ('thread', 40.),
             ('process', 25.), ('serial', 0.), ('thread', 0.), ('process', 0.)]
    results = []
    for repeat in range(args.rounds):
        for mode, budget in (cases if repeat % 2 == 0 else cases[::-1]):
            result = run_case(mode, budget, records, args.warmup, args.native_threads)
            result['round'] = repeat + 1
            results.append(result)
            print(json.dumps({k: v for k, v in result.items() if k != 'rows'}), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(dict(metadata=metadata, results=results), stream, indent=2, allow_nan=False)
    print(f'Wrote {args.output}', flush=True)


if __name__ == '__main__':
    main()
