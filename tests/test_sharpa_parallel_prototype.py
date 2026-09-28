"""Offline prototype process isolation, error propagation and bounded cleanup."""

import os
from pathlib import Path
import time

import pytest

from scripts.benchmark_sharpa_parallel import ProcessPair, reference_sequence, run_case


def fake_worker(connection, side, behavior):
    try:
        if behavior == 'startup_error' and side == 1:
            connection.send(('error', 'injected startup failure'))
            return
        connection.send(('ready', {'pid': os.getpid()}))
        while True:
            request = connection.recv()
            if request is None:
                return
            if behavior == 'hang' and side == 0:
                while True:
                    time.sleep(.1)
            if behavior == 'crash' and side == 0:
                os._exit(7)
            if behavior == 'error' and side == 0:
                connection.send(('error', 'injected solve failure'))
                return
            frame = request[0] - (1 if behavior == 'stale' else 0)
            connection.send(('result', {'frame': frame}))
    except (EOFError, BrokenPipeError):
        pass
    finally:
        connection.close()


def pair_for(behavior):
    return ProcessPair(native_threads=behavior, worker=fake_worker, timeout=.3,
                       startup_timeout=10., close_timeout=.3)


def assert_reaped(pair):
    assert pair.closed
    assert len(pair.cleanup) == 2
    assert all(not item['alive'] and item['exitcode'] is not None for item in pair.cleanup)
    assert all(not Path(f'/proc/{item["pid"]}').exists() for item in pair.cleanup)


@pytest.mark.parametrize('behavior, message', [
    ('hang', 'timed out'), ('crash', 'exited without'),
    ('error', 'injected solve failure'), ('stale', 'obsolete frame'),
])
def test_failed_request_closes_both_workers(behavior, message):
    pair = pair_for(behavior)
    try:
        with pytest.raises((RuntimeError, TimeoutError), match=message):
            pair.solve([(4,), (4,)])
        assert_reaped(pair)
        if behavior == 'hang':
            assert pair.cleanup[0]['method'] in ('terminate', 'kill')
    finally:
        pair.close()


def test_idle_worker_exits_on_parent_pipe_eof():
    pair = pair_for('normal')
    try:
        for connection in pair.connections:
            connection.close()
        for process in pair.processes:
            process.join(5.)
            assert not process.is_alive()
    finally:
        pair.close()
    assert_reaped(pair)
    assert all(item['exitcode'] == 0 for item in pair.cleanup)


def test_keyboard_interrupt_during_receive_closes_workers(monkeypatch):
    pair = pair_for('normal')

    def interrupted(*_args):
        raise KeyboardInterrupt

    monkeypatch.setattr(pair, '_receive', interrupted)
    try:
        with pytest.raises(KeyboardInterrupt):
            pair.solve([(1,), (1,)])
        assert_reaped(pair)
    finally:
        pair.close()


def test_partial_startup_failure_reaps_both_workers(monkeypatch):
    cleanups = []
    original = ProcessPair.close

    def record_close(self):
        original(self)
        cleanups.extend(self.cleanup)

    monkeypatch.setattr(ProcessPair, 'close', record_close)
    with pytest.raises(RuntimeError, match='injected startup failure'):
        pair_for('startup_error')
    assert len(cleanups) == 2
    assert all(not item['alive'] and not Path(f'/proc/{item["pid"]}').exists() for item in cleanups)


def test_actual_solvers_match_reference_and_processes_exit():
    records = reference_sequence(4)
    for mode in ('serial', 'thread', 'process'):
        result = run_case(mode, 0., records, warmup=1, native_threads=1)
        assert result['summary']['samples'] == 3
        assert result['summary']['q_max_abs_rad'] < 1e-6
        assert abs(result['summary']['cost_gap_mean']) < 1e-9
        if mode == 'process':
            assert len(result['cleanup']) == 2
            assert all(item['method'] == 'graceful' and item['exitcode'] == 0
                       and not item['alive'] for item in result['cleanup'])
