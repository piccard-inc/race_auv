#!/usr/bin/env python3
"""Run one command as a subreaper and do not exit with adopted children unreaped."""

from __future__ import annotations

import ctypes
import os
import signal
import subprocess
import sys
import time


PR_SET_CHILD_SUBREAPER = 36
FORWARDED_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


def enable_subreaper() -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))


def reap_nonblocking() -> int:
    reaped = 0
    while True:
        try:
            pid, _status = os.waitpid(-1, os.WNOHANG)
        except ChildProcessError:
            return reaped
        if pid == 0:
            return reaped
        reaped += 1


def main(arguments: list[str] | None = None) -> int:
    command = sys.argv[1:] if arguments is None else arguments
    if not command:
        print("command required", file=sys.stderr)
        return 64
    enable_subreaper()
    child = subprocess.Popen(command, start_new_session=False)

    def forward(signum: int, _frame: object) -> None:
        try:
            os.kill(child.pid, signum)
        except ProcessLookupError:
            pass

    previous = {signum: signal.signal(signum, forward) for signum in FORWARDED_SIGNALS}
    try:
        return_code = child.wait()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            reap_nonblocking()
            try:
                os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                break
            time.sleep(0.02)
        reap_nonblocking()
        return return_code
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
