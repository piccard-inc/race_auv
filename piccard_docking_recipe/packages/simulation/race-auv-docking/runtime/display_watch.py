#!/usr/bin/env python3
"""The trial's X display, observed (piccard-physical-ai #122): xdpyinfo every period_s, and each change of the
display's state written to display-watch.jsonl with the wall and monotonic clocks the collector's rows carry, so a
display death lines up with the telemetry. States: up, lost (xdpyinfo failed: its stderr is kept), unresponsive (no
answer within timeout_s), no_xdpyinfo.

A server replaced between two polls answers both, so reachability alone misses it (M3-0 v1.2: the host restarted
its X server under the trial and 1802 polls saw one state). Two identity checks close that:
- socket: each poll stats the server's socket (/tmp/.X11-unix/X<n>) and records a server_changed event when its
  (inode, ctime) differs from the last one seen; a missing socket is a socket state, recorded once per change;
- connection: one long-lived `xprop -root -spy WM_NAME` holds a connection while the display is up; a server that
  goes away breaks it, and its exit is recorded as connection_broken with the exit code and stderr. A new one is
  opened on the next poll that finds the display up.

Observation only: it restarts nothing and retries nothing, so a display loss still fails the trial closed through
the streams it stops. It ends on SIGINT/SIGTERM, after max_seconds or after max_polls probes (0: no bound), with a
stop record, and ends its xprop first."""
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path


def probe(display: str, timeout_s: float) -> tuple[str, str | None]:
    try:
        done = subprocess.run(["xdpyinfo", "-display", display], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                              timeout=timeout_s, check=False)
    except subprocess.TimeoutExpired:
        return "unresponsive", None
    except OSError as error:
        return "no_xdpyinfo", str(error)
    if done.returncode == 0:
        return "up", None
    return "lost", done.stderr.decode(errors="replace").strip()[-300:] or f"exit {done.returncode}"


def socket_path(display: str) -> Path | None:
    """The local socket of display ":N[.S]"; None for a display on another host (no local socket)."""
    match = re.fullmatch(r"(?:unix)?:(\d+)(?:\.\d+)?", display)
    return Path(f"/tmp/.X11-unix/X{match.group(1)}") if match else None


def identity(path: Path | None) -> dict | None:
    """The server socket's (inode, ctime): a server that re-creates its socket gets a new one."""
    if path is None:
        return None
    try:
        stat = os.stat(path)
    except OSError:
        return None
    return {"inode": stat.st_ino, "ctime_ns": stat.st_ctime_ns}


def open_connection(display: str) -> subprocess.Popen | None:
    try:
        return subprocess.Popen(["xprop", "-display", display, "-root", "-spy", "WM_NAME"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    except OSError:
        return None


def record(stream, **fields) -> None:
    stream.write(json.dumps({"wall_time_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns(), **fields},
                            separators=(",", ":")) + "\n")
    stream.flush()


def watch(output: Path, display: str, period_s: float, timeout_s: float, max_seconds: float,
          max_polls: int = 0, socket: Path | None = None) -> dict:
    stopping = []
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopping.append(True))
    socket = socket if socket is not None else socket_path(display)
    state, polls, changes = None, 0, 0
    socket_state, known, server_changes = None, None, 0
    connection, connection_breaks, no_xprop = None, 0, False
    deadline = time.monotonic() + max_seconds
    with output.open("a", encoding="utf-8") as stream:
        record(stream, event="start", display=display, period_s=period_s, timeout_s=timeout_s,
               socket=str(socket) if socket else None)
        try:
            while not stopping and time.monotonic() < deadline and not (max_polls and polls >= max_polls):
                if connection is not None and connection.poll() is not None:
                    detail = connection.stderr.read().decode(errors="replace").strip()[-300:]
                    if not stopping:  # a group signal ends it too: that is the teardown, not a break
                        record(stream, event="connection_broken", exit_code=connection.returncode, detail=detail)
                        connection_breaks += 1
                    connection = None
                status, detail = probe(display, timeout_s)
                polls += 1
                if status != state:
                    record(stream, event="state", state=status, previous=state, detail=detail)
                    state, changes = status, changes + 1
                current = identity(socket)
                present = "present" if current else "missing"
                if present != socket_state:
                    record(stream, event="socket", state=present, previous=socket_state, identity=current)
                    socket_state = present
                if current and known and current != known:
                    record(stream, event="server_changed", old=known, new=current, state=status)
                    server_changes += 1
                known = current or known
                if connection is None and status == "up" and not no_xprop and not stopping:
                    connection = open_connection(display)
                    if connection is None:
                        no_xprop = True
                        record(stream, event="connection", state="no_xprop")
                    else:
                        record(stream, event="connection", state="open")
                end = time.monotonic() + period_s
                while not stopping and time.monotonic() < end and not (max_polls and polls >= max_polls):
                    time.sleep(min(0.1, period_s))
        finally:
            if connection is not None and connection.poll() is None:
                connection.terminate()
                try:
                    connection.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    connection.kill()
                    connection.wait()
        summary = {"polls": polls, "changes": changes, "state": state, "server_changes": server_changes,
                   "connection_breaks": connection_breaks}
        record(stream, event="stop", **summary)
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--display", required=True)
    parser.add_argument("--period-s", type=float, default=1.0)
    parser.add_argument("--timeout-s", type=float, default=2.0)
    parser.add_argument("--max-seconds", type=float, required=True)
    parser.add_argument("--max-polls", type=int, default=0)
    parser.add_argument("--socket", type=Path, help="the server socket to stat (default: /tmp/.X11-unix/X<n>)")
    args = parser.parse_args(argv)
    watch(args.output, args.display, args.period_s, args.timeout_s, args.max_seconds, args.max_polls, args.socket)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
