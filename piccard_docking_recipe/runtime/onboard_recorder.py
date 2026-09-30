#!/usr/bin/env python3
"""Onboard front-camera clip for race-auv-docking/v1 (--onboard-camera-video, issue #86).

One ROS node, /piccard_media_recorder, with one subscription: the simulated front camera image. It
never publishes, never touches ground truth, and appears in the collector's consumption audit under media_nodes.
Frames are halved to 800x600 and placed on a 10 fps timeline by receive time: a camera frame is held until the
next one arrives, so the clip plays in real time whatever rate the camera renders at (the stamps are wall time; the
dev loop's software renderer delivers ~0.6 Hz), and video time maps to collector time by one offset.
cam_front-frames.csv keeps each camera frame's video frame, header stamp and receive wall time. A full queue drops
frames (counted) rather than back-pressuring ROS. Illustrative media only; exits 0 on every failure.
ROS imports are lazy so the offline tests run without ROS.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time

import media

NODE = "piccard_media_recorder"
TOPIC = "/race_auv/cam_front/stonefish/data/image_color"
RATE_HZ = 10  # the clip's timeline; vehicles/race_auv.scn declares cam_front at 10 Hz, 1600x1200
DOWNSCALE = 2
ENCODINGS = {"rgb8": "rgb24", "bgr8": "bgr24"}
QUEUE_FRAMES = 30
# The camera publisher is reliable. A best-effort reader of these 5.8 MB frames lost ~90% of them in the dev loop
# (0.1 Hz received vs 0.82 Hz published); a reliable keep-last-2 reader gets them all, and when it falls behind it
# overwrites its own history instead of holding the publisher back.
QOS_DEPTH = 2


def frame_bytes(encoding: str, width: int, height: int, step: int, data) -> bytes:
    """Row-strided 8-bit colour image -> packed pixels at 1/DOWNSCALE size (nearest; illustrative only)."""
    import numpy as np
    if encoding not in ENCODINGS:
        raise ValueError(f"unsupported encoding {encoding}")
    rows = np.frombuffer(data, dtype=np.uint8, count=step * height).reshape(height, step)
    image = rows[:, :width * 3].reshape(height, width, 3)
    return np.ascontiguousarray(image[::DOWNSCALE, ::DOWNSCALE]).tobytes()


def ffmpeg_command(pix_fmt: str, width: int, height: int, maxrate: int, path: Path) -> list[str]:
    return ["ffmpeg", "-nostdin", "-hide_banner", "-nostats", "-loglevel", "warning", "-n",
            "-f", "rawvideo", "-pix_fmt", pix_fmt, "-video_size", f"{width}x{height}", "-framerate", str(RATE_HZ),
            "-i", "pipe:0", "-c:v", "libx264", "-preset", "ultrafast", "-crf", "25",
            "-maxrate", f"{maxrate}k", "-bufsize", f"{2 * maxrate}k", "-pix_fmt", "yuv420p", str(path)]


class Recorder:
    """Encoder side, independent of ROS: accepts frames, writes them from a thread, and summarizes."""

    def __init__(self, output: Path, limit_seconds: float):
        self.output = output
        self.maxrate = media.maxrate_kbps(media.ONBOARD_BUDGET_BYTES, limit_seconds)
        self.queue: queue.Queue = queue.Queue(maxsize=QUEUE_FRAMES)
        self.frames = (output / media.ONBOARD["frames"]).open("w", encoding="utf-8", buffering=1)
        self.frames.write("video_frame,stamp_ns,receive_wall_ns\n")
        self.stats = {"received": 0, "written": 0, "dropped_queue_full": 0, "coalesced": 0, "video_frames": 0,
                      "rejected": {}}
        self.first_receive_ns = None
        self.source = None
        self.process = None
        self.writer = None
        self.error = None

    def offer(self, encoding, width, height, step, data, stamp_ns, receive_ns):
        self.stats["received"] += 1
        shape = (encoding, width, height)
        try:
            if self.source is None:
                self.start(*shape)
            elif shape != self.source:
                raise ValueError("image format changed")
            payload = frame_bytes(encoding, width, height, step, data)
        except (ValueError, OSError) as error:
            reason = str(error)[:80]
            self.stats["rejected"][reason] = self.stats["rejected"].get(reason, 0) + 1
            return
        try:
            self.queue.put_nowait((payload, stamp_ns, receive_ns))
        except queue.Full:
            self.stats["dropped_queue_full"] += 1

    def start(self, encoding, width, height):
        if encoding not in ENCODINGS:
            raise ValueError(f"unsupported encoding {encoding}")
        self.source = (encoding, width, height)
        self.process = subprocess.Popen(
            ffmpeg_command(ENCODINGS[encoding], width // DOWNSCALE, height // DOWNSCALE, self.maxrate,
                           self.output / media.ONBOARD["path"]),
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=(self.output / "onboard-ffmpeg.log").open("w"),
            start_new_session=True)  # stopped by closing its input, so every queued frame is encoded
        self.writer = threading.Thread(target=self.write, daemon=True)
        self.writer.start()

    def write(self):
        previous = None
        while (item := self.queue.get()) is not None:
            payload, stamp_ns, receive_ns = item
            if self.first_receive_ns is None:
                self.first_receive_ns = receive_ns
            index = round((receive_ns - self.first_receive_ns) * RATE_HZ / 1e9)
            if index < self.stats["video_frames"]:  # two camera frames in one 0.1 s slot: keep the first
                self.stats["coalesced"] += 1
                continue
            try:
                for _ in range(index - self.stats["video_frames"]):  # hold the last camera frame until this one
                    self.process.stdin.write(previous)
                self.process.stdin.write(payload)
            except (BrokenPipeError, ValueError, OSError) as error:  # ffmpeg exited early
                self.error = f"{type(error).__name__}: {error}"[:200]
                return
            self.frames.write(f"{index},{stamp_ns},{receive_ns}\n")
            self.stats["written"] += 1
            self.stats["video_frames"] = index + 1
            previous = payload

    def close(self, status: str) -> dict:
        # finish the file even if run_trial.sh escalates to TERM; its KILL is the backstop
        handlers = {signum: signal.signal(signum, signal.SIG_IGN) for signum in (signal.SIGINT, signal.SIGTERM)}
        if self.writer is not None:
            self.queue.put(None)
            self.writer.join(timeout=20)
        exit_code = None
        if self.process is not None:
            try:
                self.process.stdin.close()
            except OSError:
                pass
            try:
                exit_code = self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.process.kill()
                exit_code = self.process.wait()
        self.frames.close()
        summary = {"schema": "piccard.race-auv.onboard-recorder/v1", "status": status, "node": f"/{NODE}",
                   "topic": TOPIC, "qos": f"reliable, keep_last {QOS_DEPTH}", "timeline_fps": RATE_HZ,
                   "source": None if self.source is None else dict(zip(("encoding", "width", "height"), self.source)),
                   "downscale": DOWNSCALE, "maxrate_kbps": self.maxrate, "frames": self.stats,
                   "first_frame_receive_wall_ns": self.first_receive_ns,
                   "ffmpeg_exit_code": exit_code, "writer_error": self.error}
        (self.output / media.ONBOARD["summary"]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
        for signum, handler in handlers.items():
            signal.signal(signum, handler)
        return summary


def interrupt(signum, frame):
    raise KeyboardInterrupt  # run_trial.sh escalates INT -> TERM; both finish the clip and the summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit-seconds", type=float, required=True)
    args = parser.parse_args(argv)
    signal.signal(signal.SIGTERM, interrupt)
    recorder = Recorder(args.output, args.limit_seconds)
    status = "stopped"
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image

        rclpy.init()
        node = rclpy.create_node(NODE)

        def receive(msg):
            recorder.offer(msg.encoding, msg.width, msg.height, msg.step, msg.data,
                           msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec, time.time_ns())

        node.create_subscription(Image, TOPIC, receive, QoSProfile(
            depth=QOS_DEPTH, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE))
        deadline = time.monotonic() + args.limit_seconds
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.5)
            status = "stopped" if time.monotonic() < deadline else "limit_reached"
        except (KeyboardInterrupt, ExternalShutdownException):
            status = "stopped"
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except KeyboardInterrupt:  # stopped before the node was up
        status = "stopped"
    except Exception as error:  # never let media change the trial outcome
        status = f"error: {type(error).__name__}: {error}"[:300]
    recorder.close(status)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
