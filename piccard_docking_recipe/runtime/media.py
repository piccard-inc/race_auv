#!/usr/bin/env python3
"""media.json for race-auv-docking/v1 recordings (issue #86), after the recorders have stopped.

Per clip: path, bytes, sha256, ffprobe summary, a poster PNG at a stated seek time, sampled-frame content checks,
the GL renderer that drew the display, and the measured offset between the video clock and the collector clock.
Semantics follow alpha-rise-gains-only/media-delivery-v1: illustrative media only, never an input to selection,
publication_authorized always false. Never changes the trial outcome: every failure is recorded and it exits 0.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

SCHEMA = "piccard.race-auv.media/v1"
MEDIA_BUDGET_BYTES = 100_000_000  # issue #86: a 5-minute trial stays under ~100 MB of media
DISPLAY_BUDGET_BYTES = 60_000_000
ONBOARD_BUDGET_BYTES = 40_000_000  # the rest of MEDIA_BUDGET_BYTES
DISPLAY = {"name": "display", "path": "display.mp4", "poster": "display-poster.png", "frame_rate": "10/1",
           "budget_bytes": DISPLAY_BUDGET_BYTES,
           "encoding": {"codec": "libx264", "preset": "ultrafast", "crf": 25, "pixel_format": "yuv420p",
                        "frame_rate": "10/1", "cursor": "hidden", "rate_cap": "maxrate from DISPLAY_BUDGET_BYTES over "
                                                                                  "the recording limit"}}
ONBOARD = {"name": "cam_front", "path": "cam_front.mp4", "poster": "cam_front-poster.png", "frame_rate": "10/1",
           "frames": "cam_front-frames.csv", "summary": "onboard-recorder.json", "budget_bytes": ONBOARD_BUDGET_BYTES,
           "encoding": {"codec": "libx264", "preset": "ultrafast", "crf": 25, "pixel_format": "yuv420p",
                        "frame_rate": "10/1", "size": "800x600, every second pixel of the 1600x1200 camera",
                        "rate_cap": "maxrate from ONBOARD_BUDGET_BYTES over the recording limit"}}
CLOCK = {"hidden": "The frame carries no burned-in clock (the GUI, including Stonefish's simulation-time status bar, "
                   "is hidden); the only timestamp is synchronization: collector t = video time + "
                   "video_start_minus_collector_t0_s.",
         "shown": "The Stonefish status bar in the frame shows the simulator's own simulation time; synchronization "
                  "maps video time to collector t."}
SOFTWARE_RENDERERS = ("llvmpipe", "softpipe", "swrast", "software rasterizer")
SAMPLE_INTERVAL_S = 10.0
SAMPLE_SIZE = (300, 200)
BLACK_LUMA = 8.0
CLAIM = ("Illustrative simulation media only: not an input to selection or scoring and not evidence of physical "
         "behaviour. Publication is a separate human decision.")


def maxrate_kbps(budget_bytes: int, limit_seconds: float) -> int:
    """Bitrate cap so a recording that runs its whole time limit stays within its byte budget."""
    return max(100, int(budget_bytes * 8 / 1000 / limit_seconds))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1 << 20):
            digest.update(chunk)
    return digest.hexdigest()


def x11grab_start(log_text: str) -> float | None:
    """Wall-clock time (s) of the first grabbed frame: x11grab stamps frames with the wall clock."""
    match = re.search(r"Input #0, x11grab, from '[^']*':\s*\n\s*Duration: [^,]*, start: ([0-9]+\.[0-9]+)", log_text)
    return float(match.group(1)) if match else None


def renderer(glxinfo_text: str) -> dict:
    match = re.search(r"OpenGL renderer string:\s*(.+)", glxinfo_text)
    name = match.group(1).strip() if match else None
    return {"renderer": name,
            "software_renderer": None if name is None else any(s in name.lower() for s in SOFTWARE_RENDERERS)}


def probe_summary(probe: dict) -> dict:
    stream = next(s for s in probe["streams"] if s.get("codec_type", "video") == "video")
    return {"codec": stream["codec_name"], "width": int(stream["width"]), "height": int(stream["height"]),
            "pixel_format": stream["pix_fmt"], "frame_rate": stream["avg_frame_rate"],
            "decoded_frames": int(stream["nb_read_frames"]), "duration_seconds": float(probe["format"]["duration"])}


def collector_t0(telemetry: Path) -> float | None:
    """Wall-clock time (s) of the collector's t = 0, from its first telemetry row."""
    try:
        with telemetry.open(encoding="utf-8") as stream:
            row = json.loads(stream.readline())
        return row["wall_time_ns"] / 1e9 - row["t"]
    except (OSError, ValueError, KeyError, TypeError):
        return None


def poster_seek(mission: dict, starts: dict, offset: float | None, duration: float) -> tuple[float, str]:
    """Middle of the last commanded pose's dwell (the hold) in video time, else the middle of the clip."""
    poses = (mission or {}).get("poses") or []
    if poses and offset is not None and poses[-1]["label"] in starts:
        seek = starts[poses[-1]["label"]] + poses[-1]["dwell_s"] / 2 - offset
        if 0 <= seek <= duration:
            return round(seek, 3), f"middle of pose '{poses[-1]['label']}' (collector t mapped to video time)"
    return round(duration / 2, 3), "middle of the clip"


def frame_times(frames_csv: Path, t0: float | None) -> list[dict]:
    """Camera frames in the onboard clip: video frame, header stamp, receive wall time and collector t."""
    rows = []
    try:
        with frames_csv.open(encoding="utf-8") as stream:
            next(stream)
            for line in stream:
                frame, stamp_ns, receive_ns = (int(value) for value in line.split(","))
                rows.append({"frame": frame, "stamp_s": stamp_ns / 1e9, "receive_s": receive_ns / 1e9,
                             "collector_t": None if t0 is None else receive_ns / 1e9 - t0})
    except (OSError, ValueError, StopIteration):
        pass
    return rows


def onboard_synchronization(frames: list[dict], t0: float | None) -> dict:
    received = [row["receive_s"] for row in frames]
    latency = sorted(row["receive_s"] - row["stamp_s"] for row in frames)
    return {"frames": ONBOARD["frames"], "camera_frames": len(frames), "collector_t0_wall_time_s": t0,
            "video_first_frame_wall_time_s": received[0] if received else None,
            "video_start_minus_collector_t0_s": round(received[0] - t0, 3) if received and t0 is not None else None,
            "camera_rate_hz_measured": round((len(received) - 1) / (received[-1] - received[0]), 3)
            if len(received) > 1 and received[-1] > received[0] else None,
            "median_receive_minus_stamp_s": round(latency[len(latency) // 2], 4) if latency else None,
            "mapping": "collector t = video time + video_start_minus_collector_t0_s, to within one 0.1 s frame: each "
                       "camera frame sits at round((receive - first receive) x 10) and is held until the next "
                       "(cam_front-frames.csv)",
            "method": "the recorder's receive wall clock for the first camera frame against the collector's first "
                      "telemetry row (wall_time_ns - t); measured per trial, not assumed"}


def pose_starts(telemetry: Path) -> dict:
    starts = {}
    try:
        with telemetry.open(encoding="utf-8") as stream:
            for line in stream:
                if '"pose_start"' in line:
                    row = json.loads(line)
                    starts.setdefault(row["label"], row["t"])
    except (OSError, ValueError, KeyError):
        pass
    return starts


def hull_mask(rgb):
    """Pixels of the AUV hull: saturated yellow above water, green once the water has absorbed the red
    (hue 40-150 deg), well apart from the blue water, the station and the cyan GUI."""
    import numpy as np
    rgb = rgb.astype(np.float32) / 255
    high, low = rgb.max(-1), rgb.min(-1)
    spread = high - low + 1e-9
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    hue = np.where(high == r, (g - b) / spread % 6, np.where(high == g, (b - r) / spread + 2, (r - g) / spread + 4)) * 60
    saturation = np.where(high > 0, spread / (high + 1e-9), 0)
    return (hue >= 40) & (hue <= 150) & (saturation >= 0.45) & (high >= 0.2)


def frame_stats(frames, width: int, height: int, vehicle_in_view: bool = True) -> dict:
    """Content checks on sampled RGB frames: black frames, and whether the AUV hull sits in the middle third
    (a heuristic for the follow view: the hull is the only saturated yellow-green object in the scene)."""
    lumas, centred = [], []
    box = (slice(height // 3, 2 * height // 3), slice(width // 3, 2 * width // 3))
    for frame in frames:
        rgb = frame.astype(float)
        lumas.append(float((0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]).mean()))
        centred.append(float(hull_mask(frame[box]).mean()) >= 0.01)
    first = next((index for index, flag in enumerate(centred) if flag), None)
    if not vehicle_in_view:  # the onboard camera rides on the AUV
        return {"sampled_frames": len(lumas), "sample_interval_seconds": SAMPLE_INTERVAL_S,
                "black_frames": sum(value < BLACK_LUMA for value in lumas),
                "mean_luma": round(sum(lumas) / len(lumas), 2) if lumas else None,
                "method": "frames sampled every 10 s at 300x200; black = mean luma < 8"}
    return {"sampled_frames": len(lumas), "sample_interval_seconds": SAMPLE_INTERVAL_S,
            "black_frames": sum(value < BLACK_LUMA for value in lumas),
            "mean_luma": round(sum(lumas) / len(lumas), 2) if lumas else None,
            "vehicle_centred_fraction": round(sum(centred) / len(centred), 3) if centred else None,
            "first_centred_video_s": None if first is None else first * SAMPLE_INTERVAL_S,
            "method": "frames sampled every 10 s at 300x200; black = mean luma < 8; vehicle centred = >= 1% hull "
                      "pixels (hue 40-150 deg, saturation >= 0.45) in the middle third (heuristic)"}


def sample_frames(clip: Path):
    import numpy as np
    width, height = SAMPLE_SIZE
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", str(clip), "-vf",
                          f"fps=1/{SAMPLE_INTERVAL_S:g},scale={width}:{height}", "-f", "rawvideo", "-pix_fmt", "rgb24",
                          "-"], capture_output=True, check=True, timeout=300).stdout
    size = width * height * 3
    return [np.frombuffer(raw[i:i + size], dtype=np.uint8).reshape(height, width, 3)
            for i in range(0, len(raw) - size + 1, size)], width, height


def describe_clip(root: Path, spec: dict, poster_at, vehicle_in_view: bool = True) -> dict:
    """poster_at(duration) -> (seek seconds, basis)."""
    clip = root / spec["path"]
    item = {"name": spec["name"], "path": spec["path"], "status": "missing", "encoding": spec["encoding"]}
    if not clip.is_file() or clip.stat().st_size == 0:
        return item
    item.update(status="recorded", bytes=clip.stat().st_size, sha256=sha256_file(clip))
    try:
        probe = json.loads(subprocess.run(
            ["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0", "-show_entries",
             "stream=codec_name,codec_type,width,height,pix_fmt,avg_frame_rate,nb_read_frames:format=duration",
             "-of", "json", str(clip)], capture_output=True, check=True, timeout=300, text=True).stdout)
        item["probe"] = probe_summary(probe)
    except (subprocess.SubprocessError, ValueError, KeyError, StopIteration) as error:
        item.update(status="unreadable", error=f"{type(error).__name__}: {error}"[:300])
        return item
    seek, basis = poster_at(item["probe"]["duration_seconds"])
    poster = root / spec["poster"]
    try:
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{seek}", "-i", str(clip), "-frames:v", "1", str(poster)],
                       capture_output=True, check=True, timeout=120)
        item["poster"] = {"path": spec["poster"], "bytes": poster.stat().st_size, "sha256": sha256_file(poster),
                          "seek_seconds": seek, "basis": basis}
    except (subprocess.SubprocessError, OSError) as error:
        item["poster"] = {"path": None, "error": f"{type(error).__name__}"}
    try:
        item["content"] = frame_stats(*sample_frames(clip), vehicle_in_view=vehicle_in_view)
    except (subprocess.SubprocessError, ImportError, ValueError) as error:
        item["content"] = {"error": f"{type(error).__name__}: {error}"[:300]}
    return item


def build(root: Path, display_requested: bool, onboard_requested: bool = False) -> dict:
    try:
        mission = json.loads((root / "trial.json").read_text()).get("mission") or {}
    except (OSError, ValueError, AttributeError):
        mission = {}
    starts = pose_starts(root / "telemetry.jsonl")
    t0 = collector_t0(root / "telemetry.jsonl")
    clips = []
    if display_requested:
        log = (root / "ffmpeg.log").read_text(errors="replace") if (root / "ffmpeg.log").is_file() else ""
        start = x11grab_start(log)
        offset = None if start is None or t0 is None else round(start - t0, 3)
        item = describe_clip(root, DISPLAY, lambda duration: poster_seek(mission, starts, offset, duration))
        glx = (root / "glxinfo.txt").read_text(errors="replace") if (root / "glxinfo.txt").is_file() else ""
        view = {}
        try:
            view = json.loads((root / "follow-view.json").read_text())
        except (OSError, ValueError):
            view = {"status": "not_recorded"}
        gui = "hidden" if (view.get("hud") or {}).get("status") == "hidden" else "shown"
        item["source"] = {"kind": "x11grab of the Stonefish window", "window": "Stonefish Simulator, 1200x800",
                          **renderer(glx), "view": {"status": view.get("status"), "log": "follow-view.json",
                                                    "gui": gui, "clock": CLOCK[gui],
                                                    "setup": "trackball centre race_auv, look north ~22 deg down, "
                                                             "~3.1 m orbit (runtime/follow_view.sh)"}}
        item["synchronization"] = {
            "video_first_frame_wall_time_s": start, "collector_t0_wall_time_s": t0,
            "video_start_minus_collector_t0_s": offset,
            "mapping": "collector t = video time + video_start_minus_collector_t0_s",
            "method": "x11grab stream start from ffmpeg's log (wall clock of the first grabbed frame) against the "
                      "collector's first telemetry row (wall_time_ns - t); measured per trial, not assumed"}
        clips.append(item)
    if onboard_requested:
        sync = onboard_synchronization(frame_times(root / ONBOARD["frames"], t0), t0)
        offset = sync["video_start_minus_collector_t0_s"]  # the clip is on a real-time timeline, like the display
        item = describe_clip(root, ONBOARD, lambda duration: poster_seek(mission, starts, offset, duration),
                             vehicle_in_view=False)
        try:
            recorder = json.loads((root / ONBOARD["summary"]).read_text())
        except (OSError, ValueError):
            recorder = {"status": "not_recorded"}
        item["source"] = {"kind": "sensor_msgs/Image from the simulated front camera, encoded by a ROS node",
                          "node": recorder.get("node"), "topic": recorder.get("topic"),
                          "native": "1600x1200 at 10 Hz (vehicles/race_auv.scn cam_front)",
                          "recorder": {key: recorder.get(key) for key in ("status", "frames", "source", "qos",
                                                                          "ffmpeg_exit_code")},
                          "audit": "the node is listed under trial.json consumption audits' media_nodes"}
        item["synchronization"] = sync
        clips.append(item)
    specs = {DISPLAY["name"]: DISPLAY, ONBOARD["name"]: ONBOARD}
    total = sum(clip.get("bytes", 0) + clip.get("poster", {}).get("bytes", 0) for clip in clips)
    over = [clip for clip in clips if clip.get("bytes", 0) > specs[clip["name"]]["budget_bytes"]]
    if total > MEDIA_BUDGET_BYTES:
        over = clips
    for clip in over:  # a report over the API's media cap would be rejected; drop the clip, keep the record
        for name in (clip["path"], (clip.get("poster") or {}).get("path")):
            if name and (root / name).is_file():
                (root / name).unlink()
        clip["status"] = "discarded_over_budget"
    within = not over
    return {"schema": SCHEMA, "role": "illustrative_media_only", "publication_authorized": False,
            "claim_boundary": CLAIM,
            "requested": {"display_video": display_requested, "onboard_camera_video": onboard_requested},
            "budget": {"max_bytes": MEDIA_BUDGET_BYTES, "display_max_bytes": DISPLAY_BUDGET_BYTES,
                       "onboard_max_bytes": ONBOARD_BUDGET_BYTES,
                       "total_bytes": total, "within_budget": within},
            "clips": clips}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--display-video", type=int, choices=(0, 1), default=0)
    parser.add_argument("--onboard-video", type=int, choices=(0, 1), default=0)
    parser.add_argument("--display-maxrate-kbps", type=float, metavar="LIMIT_SECONDS",
                        help="print the display bitrate cap (kbit/s) for a recording limit and exit")
    args = parser.parse_args(argv)
    if args.display_maxrate_kbps is not None:
        print(maxrate_kbps(DISPLAY_BUDGET_BYTES, args.display_maxrate_kbps))
        return 0
    try:
        record = build(args.output, bool(args.display_video), bool(args.onboard_video))
    except Exception as error:  # never let media change the trial outcome
        record = {"schema": SCHEMA, "role": "illustrative_media_only", "publication_authorized": False,
                  "claim_boundary": CLAIM, "error": f"{type(error).__name__}: {error}"[:500]}
    (args.output / "media.json").write_text(json.dumps(record, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
