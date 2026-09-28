"""Persistent, spawned left/right solvers owned by the execution flow."""

from dataclasses import replace
import math
import multiprocessing as mp
from multiprocessing.connection import wait
import os
import signal
import sys
import threading
import time
import traceback

import numpy as np


_SPAWN_LOCK = threading.Lock()
_THREAD_ENV = ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
               'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS')


def _exit_with_parent(parent_pid):
    """On Linux, also terminate a native call that cannot service Python/EOF."""
    if sys.platform == 'linux':
        import ctypes

        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
            raise OSError(ctypes.get_errno(), 'Cannot install solver parent-death signal')
        if os.getppid() != parent_pid:  # Close the race before prctl was installed.
            os._exit(1)
    else:
        def watch_parent():
            wait([mp.parent_process().sentinel])
            os._exit(1)

        threading.Thread(target=watch_parent, daemon=True, name='solver_parent_watch').start()


def _solver_worker(connection, config, parent_pid):
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # The flow owns terminal cancellation.
        _exit_with_parent(parent_pid)
        import torch
        from retargeting.core import Retargeter
        from retargeting.core.kinematics import RobotAdaptor, RobotPinocchio

        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        robot = config['robot_config']
        model = RobotPinocchio(robot.robot_file_path, robot.model.type)
        adaptor = RobotAdaptor(model, list(robot.actuated_joints))
        solver = Retargeter(robot_adaptor=adaptor, **config)
        connection.send(('ready', dict(pid=os.getpid(), joints=list(robot.actuated_joints),
                                       maxtime=solver.solver_config.params['maxtime'],
                                       torch_threads=torch.get_num_threads())))
        while True:
            request = connection.recv()
            if request is None:
                return
            tag, observation, seed = request
            # Parent-supplied seed is authoritative after filtering, stale drops and resets.
            result = solver.solve(observation, previous_qpos=seed)
            connection.send(('result', (tag, result)))
    except (EOFError, BrokenPipeError):
        pass
    except BaseException:
        try:
            connection.send(('error', traceback.format_exc()))
        except (OSError, EOFError):
            pass
    finally:
        connection.close()


class BimanualProcessSolver:
    """One in-flight pair, explicit seeds, no stale work queue or automatic fallback.

    start/solve/close belong to the flow thread. cancel is safe from its duration
    timer. Configuration is copied into workers; models/optimizers are never pickled.
    """

    def __init__(self, retargeters, *, timeout=.25, startup_timeout=30., close_timeout=.5):
        if len(retargeters) != 2:
            raise ValueError('Expected left and right retargeters')
        if any(not math.isfinite(v) or v <= 0 for v in (timeout, startup_timeout, close_timeout)):
            raise ValueError('Solver timeouts must be finite and positive')
        self.configs = tuple({key: getattr(r, key) for key in
                             ('robot_config', 'profile_config', 'method_config', 'solver_config')}
                            for r in retargeters)
        self.limits = tuple(r.optimizer.joint_limits.copy() for r in retargeters)
        self.timeout, self.startup_timeout, self.close_timeout = timeout, startup_timeout, close_timeout
        self.started = self.closed = self.failed = False
        self._cancelled = threading.Event()
        self._request_id = 0
        self._connections, self._processes = [], []
        self.ready, self.cleanup = [], []

    def start(self):
        if self.closed or self.failed or self._cancelled.is_set():
            raise RuntimeError('Process solver stopped; create a new execution flow')
        if self.started:
            return
        try:
            # Spawn imports NumPy/PyTorch before entering the target. Scope environment
            # overrides to child creation; do not change the parent's numerical pools.
            with _SPAWN_LOCK:
                previous = {key: os.environ.get(key) for key in _THREAD_ENV}
                try:
                    os.environ.update({key: '1' for key in _THREAD_ENV})
                    context = mp.get_context('spawn')
                    for side, config in zip(('left', 'right'), self.configs):
                        parent, child = context.Pipe()
                        process = context.Process(target=_solver_worker, args=(child, config, os.getpid()),
                                                  name=f'retargeting_solver_{side}', daemon=True)
                        try:
                            process.start()
                        except BaseException:
                            parent.close()
                            raise
                        finally:
                            child.close()
                        self._connections.append(parent)
                        self._processes.append(process)
                finally:
                    for key, value in previous.items():
                        if value is None:
                            os.environ.pop(key, None)
                        else:
                            os.environ[key] = value
            self.ready = self._receive_pair('ready', time.monotonic() + self.startup_timeout)
            for config, ready in zip(self.configs, self.ready):
                if ready['joints'] != list(config['robot_config'].actuated_joints):
                    raise RuntimeError('Worker joint order differs from the parent')
            self.started = True
        except BaseException:
            self.failed = True
            self.close()
            raise

    def _receive_pair(self, expected, deadline):
        pending = dict(enumerate(self._connections))
        results = [None, None]
        while pending:
            if self._cancelled.is_set():
                raise RuntimeError('Process solver cancelled')
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(f'Process solver timed out waiting for {expected}')
            handles = list(pending.values()) + [self._processes[i].sentinel for i in pending]
            readable = wait(handles, timeout=min(.01, remaining))
            for side, connection in list(pending.items()):
                if connection in readable:
                    try:
                        kind, value = connection.recv()
                    except (OSError, EOFError) as exc:
                        raise RuntimeError(f'Solver {side} exited without a reply') from exc
                    if kind != expected:
                        raise RuntimeError(f'Solver {side} {kind}: {value}')
                    results[side] = value
                    del pending[side]
                elif self._processes[side].sentinel in readable:
                    raise RuntimeError(f'Solver {side} exited without a reply')
        if self._cancelled.is_set():
            raise RuntimeError('Process solver cancelled')
        return results

    def solve(self, observations, seeds, *, frame_id):
        if not self.started or self.closed or self.failed or self._cancelled.is_set():
            raise RuntimeError('Process solver is not running')
        try:
            if len(observations) != 2 or len(seeds) != 2:
                raise ValueError('Expected a complete observation/seed pair')
            self._request_id += 1
            tag = (self._request_id, frame_id)
            deadline = time.monotonic() + self.timeout
            for connection, observation, seed, limits in zip(self._connections, observations, seeds, self.limits):
                seed = np.asarray(seed, dtype=float)
                if seed.shape != (len(limits),) or not np.isfinite(seed).all():
                    raise ValueError('Invalid solver seed')
                # Acquisition objects may own threads or contain unpicklable fields.
                connection.send((tag, replace(observation, raw=None), seed.copy()))
            replies = self._receive_pair('result', deadline)
            results = []
            for (reply_tag, result), limits in zip(replies, self.limits):
                if reply_tag != tag:
                    raise RuntimeError('Process solver returned an obsolete frame')
                q = np.asarray(result.qpos)
                if (q.shape != (len(limits),) or not np.isfinite(q).all()
                        or np.any(q < limits[:, 0] - 1e-7) or np.any(q > limits[:, 1] + 1e-7)):
                    raise RuntimeError('Process solver returned invalid joint positions')
                results.append(result)
            return tuple(results)
        except BaseException:
            # Let the flow stop output before spending time joining failed workers.
            self.failed = True
            raise

    def cancel(self):
        self._cancelled.set()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.cancel()
        for connection in self._connections:
            try:
                connection.send(None)
            except (OSError, EOFError):
                pass
            connection.close()
        deadline = time.monotonic() + self.close_timeout
        for process in self._processes:
            process.join(max(0., deadline - time.monotonic()))
        for process in self._processes:
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
            self.cleanup.append(dict(pid=process.pid, exitcode=process.exitcode, alive=alive, method=method))
            if not alive:
                process.close()
        if any(item['alive'] for item in self.cleanup):
            raise RuntimeError(f'Unable to reap solver workers: {self.cleanup}')
