#!/usr/bin/env python3
"""Does display recording perturb the trial it records? Paired no-video / with-video comparison (#93 follow-up).

Recording is structurally isolated (same collector and launch, no ROS node), but x11grab + libx264 share the host's
CPUs with the simulator, and the first RISE L40S media reruns settled slower than their scored trials. This tool
compares pairs of trial artifact directories run with the same gains, mission and horizon, one without and one with
media.display_video, on:
- settling per step: per depth step for RISE (evaluation/phase_a_metrics.py of alpha-rise-gains-only), per judged
  pose step for RACE (tools/analysis/race_m2_metrics.py, the M2 v0.2 definition on the controller's tracking error);
  not applicable to other recipes;
- the collector's timing: the inter-arrival of the controller's set_point and value streams (and the collector's own
  setpoint_command, where the recipe publishes one), and gaps between telemetry rows longer than 1 s;
- the trial minimum distance (RACE: docking.json minimum_distance, the ground-truth dock-point distance), per pair
  with-video minus without-video (piccard-physical-ai #102).
The verdict follows the onboard load check's shape (comparable/reasons). Decision aid only: a few pairs are noisy.

Minimum-distance rule (#102): over the comparable pairs with both minima, a two-sided sign test on the per-pair
deltas (ties dropped). If p < 0.05 the verdict is directional_effect, even when settling and timing show nothing.
With six pairs only a 6-0 split reaches it (p = 0.031; 5-1 gives 0.219); with five or fewer pairs no split can
(5-0 gives 0.0625), so the three R-A0 pairs could never have triggered it. No threshold lives in the runtime.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys

SCHEMA = "piccard.media.recording-isolation-check/v1"
ROOT = Path(__file__).resolve().parents[2]
PHASE_A = ROOT / "packages/simulation/alpha-rise-gains-only/evaluation/phase_a_metrics.py"
RACE_M2 = ROOT / "tools/analysis/race_m2_metrics.py"
STREAMS = ("set_point", "value", "setpoint_command")
GAP_S = 1.0  # phase_a_metrics MAX_GAP_S: a longer gap is a measurement gap
DEFAULT_SETTLING_TOLERANCE_S = 10.0
DEFAULT_JITTER_RATIO = 1.2
DEFAULT_MIN_PAIRS = 3
SIGN_TEST_ALPHA = 0.05


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def phase_a_metrics():
    return load_module("phase_a_metrics", PHASE_A)


def race_m2_metrics():
    return load_module("race_m2_metrics", RACE_M2)


def trial_root(path: Path) -> Path:
    """A trial's output directory, or a downloaded artifact directory that holds it under output/."""
    if (path / "telemetry.jsonl").is_file():
        return path
    if (path / "output" / "telemetry.jsonl").is_file():
        return path / "output"
    raise FileNotFoundError(f"{path}: no telemetry.jsonl (or output/telemetry.jsonl)")


def load_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def percentile(sorted_values: list[float], fraction: float) -> float:
    return sorted_values[min(len(sorted_values) - 1, int(fraction * len(sorted_values)))]


def collector_timing(telemetry: Path, start, end) -> dict:
    """Inter-arrival statistics (collector t) per stream inside the mission window, and telemetry gaps."""
    times = {stream: [] for stream in STREAMS}
    rows = []
    with telemetry.open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            t = row.get("t")
            if not finite(t) or (finite(start) and finite(end) and not start <= t <= end):
                continue
            rows.append(t)
            if row.get("kind") in times:
                times[row["kind"]].append(t)
    result = {}
    for name, values in times.items():
        intervals = sorted(b - a for a, b in zip(values, values[1:]))
        if len(intervals) < 2:
            continue
        result[name] = {"count": len(values), "median_s": round(statistics.median(intervals), 4),
                        "p95_s": round(percentile(intervals, 0.95), 4), "max_s": round(intervals[-1], 4),
                        "jitter_sd_s": round(statistics.pstdev(intervals), 4)}
    rows.sort()
    gaps = [b - a for a, b in zip(rows, rows[1:])]
    result["telemetry"] = {"rows": len(rows), "max_gap_s": round(max(gaps), 4) if gaps else None,
                           "gaps_over_1s": sum(gap > GAP_S for gap in gaps)}
    return result


def race_settling(root: Path):
    """RACE: the judged steps of race_m2_metrics (M2 amendment v0.2 settling; the dive is not judged)."""
    try:
        report = race_m2_metrics().analyze(root)
    except (FileNotFoundError, ValueError, KeyError) as error:
        return {"error": str(error)[:200]}
    return [{"index": step["index"], "poses": step["poses"], "stepped_axes": step["stepped_axes"],
             "status": "settled" if step["settled"] else "not_settled", "settling_time_s": step["settling_time_s"],
             "settling_censored": not step["settled"]}
            for step in report["steps"] if step["judged"]]


def settling(root: Path, trial: dict):
    schema = str(trial.get("schema", ""))
    if schema.startswith("piccard.race-auv.trial/"):
        return race_settling(root)
    if not schema.startswith("piccard.alpha-rise.trial/"):
        return None  # settling is defined for the RISE Phase A and RACE M2 missions only
    try:
        report = phase_a_metrics().analyze(root)
    except ValueError as error:
        return {"error": str(error)[:200]}
    return [{"index": index, "from_m": step.get("reference_m"), "to_m": step.get("target_m"),
             "status": step.get("status"), "settling_time_s": step.get("settling_time_s"),
             "settling_censored": step.get("settling_censored"),
             "overshoot_percent_of_step": step.get("overshoot_percent_of_step")}
            for index, step in enumerate(report["depth_intervals"])]


def describe(path: Path) -> dict:
    root = trial_root(path)
    trial, runner = load_json(root / "trial.json"), load_json(root / "runner.json")
    start, end = trial.get("mission_start_t"), trial.get("mission_end_t")
    return {"path": str(path), "schema": trial.get("schema"), "status": trial.get("status"),
            "stop_reason": trial.get("stop_reason"), "video_requested": runner.get("video_requested"),
            "mission_sha256": trial.get("mission_profile_sha256") or trial.get("mission_sha256"),
            "expected_gains": trial.get("expected_gains"), "horizon_seconds": trial.get("horizon_seconds"),
            "mission_window_s": end - start if finite(start) and finite(end) else None,
            "timing": collector_timing(root / "telemetry.jsonl", start, end), "settling": settling(root, trial),
            "minimum_distance_m": minimum_distance(root)}


def minimum_distance(root: Path):
    """RACE: the trial's minimum ground-truth dock-point distance (docking.json); None where there is none."""
    value = (load_json(root / "docking.json").get("minimum_distance") or {}).get("distance_m")
    return value if finite(value) else None


def sign_test(deltas: list[float]) -> dict:
    """Two-sided sign test on per-pair deltas, ties dropped (exact binomial, p = 0.5)."""
    nonzero = [d for d in deltas if d != 0]
    n, lower = len(nonzero), sum(d < 0 for d in nonzero)
    k = min(lower, n - lower)
    p = min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n) if n else 1.0
    return {"pairs": n, "closer_with_video": lower, "farther_with_video": n - lower, "ties": len(deltas) - n,
            "p_two_sided": round(p, 4), "directional": bool(n) and p < SIGN_TEST_ALPHA}


def compare_steps(plain: list | None, video: list | None) -> dict:
    """Step by step (same index): the with-video settling time minus the no-video one."""
    if not isinstance(plain, list) or not isinstance(video, list):
        return {"applicable": False}
    deltas, slower, faster, rows = [], 0, 0, []
    for a, b in zip(plain, video):
        if "step_within_settling_band" in (a["status"], b["status"]):
            continue
        a_settled, b_settled = finite(a["settling_time_s"]), finite(b["settling_time_s"])
        if a_settled and b_settled:
            delta = b["settling_time_s"] - a["settling_time_s"]
            deltas.append(delta)
            slower += delta > 0
            faster += delta < 0
            rows.append({"index": a["index"], "delta_s": round(delta, 3)})
        elif a_settled and not b_settled:
            slower += 1
            rows.append({"index": a["index"], "delta_s": None, "note": "censored only with video"})
        elif b_settled and not a_settled:
            faster += 1
            rows.append({"index": a["index"], "delta_s": None, "note": "censored only without video"})
    compared = slower + faster + sum(1 for d in deltas if d == 0)
    return {"applicable": True, "steps": rows, "compared": compared,
            "median_delta_s": round(statistics.median(deltas), 3) if deltas else None,
            "slower_fraction": round(slower / compared, 3) if compared else None}


def compare_pair(index: int, plain: dict, video: dict, tolerance: float, jitter_ratio: float) -> dict:
    reasons = []
    if plain["video_requested"] is not False or video["video_requested"] is not True:
        reasons.append("the pair is not one no-video and one with-video run (runner.json video_requested)")
    for key in ("mission_sha256", "horizon_seconds", "schema"):
        if plain[key] != video[key]:
            reasons.append(f"{key} differs")
    if json.dumps(plain["expected_gains"], sort_keys=True) != json.dumps(video["expected_gains"], sort_keys=True):
        reasons.append("gains differ")
    steps = compare_steps(plain["settling"], video["settling"])
    timing = {}
    for stream in STREAMS:
        a, b = plain["timing"].get(stream), video["timing"].get(stream)
        if a and b and a["p95_s"] > 0:
            timing[stream] = {"p95_ratio": round(b["p95_s"] / a["p95_s"], 3),
                              "jitter_sd_ratio": round(b["jitter_sd_s"] / a["jitter_sd_s"], 3) if a["jitter_sd_s"] else None}
    gaps = (plain["timing"]["telemetry"]["gaps_over_1s"], video["timing"]["telemetry"]["gaps_over_1s"])
    slower = steps.get("applicable") and steps["compared"] > 0 and (
        (steps["median_delta_s"] is not None and steps["median_delta_s"] > tolerance)
        or (steps["median_delta_s"] is None and steps["slower_fraction"] >= 0.75))
    jitter = any(item["p95_ratio"] > jitter_ratio for item in timing.values())
    video_only_gaps = gaps[1] > 0 and gaps[0] == 0
    closest = (plain["minimum_distance_m"], video["minimum_distance_m"])
    return {"pair": index, "comparable": not reasons, "reasons": reasons, "settling": steps,
            "settles_slower_with_video": bool(slower), "timing": timing,
            "p95_jitter_over_ratio": jitter,
            "telemetry_gaps_over_1s": {"without_video": gaps[0], "with_video": gaps[1]},
            "video_only_telemetry_gaps": video_only_gaps,
            "minimum_distance_m": {"without_video": closest[0], "with_video": closest[1],
                                   "delta_m": round(closest[1] - closest[0], 4)
                                   if finite(closest[0]) and finite(closest[1]) else None},
            "effect": bool(slower) or jitter or video_only_gaps,
            "runs": {"without_video": plain, "with_video": video}}


def compare(pairs: list[tuple[dict, dict]], tolerance: float = DEFAULT_SETTLING_TOLERANCE_S,
            jitter_ratio: float = DEFAULT_JITTER_RATIO, min_pairs: int = DEFAULT_MIN_PAIRS) -> dict:
    reports = [compare_pair(index, plain, video, tolerance, jitter_ratio) for index, (plain, video) in enumerate(pairs)]
    usable = [report for report in reports if report["comparable"]]
    effects = sum(report["effect"] for report in usable)
    signs = sign_test([r["minimum_distance_m"]["delta_m"] for r in usable if r["minimum_distance_m"]["delta_m"] is not None])
    # Conservative for the policy: one clean pair never clears recording on scored trials.
    if len(usable) < min_pairs:
        verdict = "insufficient_pairs"
    elif effects * 2 > len(usable):
        verdict = "perturbed"
    elif signs["directional"]:
        verdict = "directional_effect"
    elif effects == 0:
        verdict = "no_effect"
    else:
        verdict = "inconclusive"
    reasons = [f"pair {report['pair']}: {reason}" for report in reports for reason in report["reasons"]]
    if len(usable) < min_pairs:
        reasons.append(f"{len(usable)} comparable pair(s); at least {min_pairs} needed")
    return {
        "schema": SCHEMA,
        "method": ("pairs of trials with the same gains, mission and horizon, without and with display video; settling "
                   "per depth step from phase_a_metrics (RISE) or per judged pose step from race_m2_metrics (RACE), "
                   "with-video minus without-video at the same step index; and "
                   "set_point/value/setpoint_command inter-arrival p95 and telemetry gaps inside the mission window; "
                   "and the trial minimum distance (RACE docking.json), with-video minus without-video, by a two-sided "
                   "sign test over the pairs"),
        "thresholds": {"settling_tolerance_s": tolerance, "p95_jitter_ratio": jitter_ratio, "min_pairs": min_pairs,
                       "telemetry_gap_s": GAP_S},
        "signals": {"comparable_pairs": len(usable), "pairs_with_an_effect": effects,
                    "pairs_settling_slower_with_video": sum(r["settles_slower_with_video"] for r in usable),
                    "pairs_over_p95_jitter_ratio": sum(r["p95_jitter_over_ratio"] for r in usable),
                    "pairs_with_video_only_telemetry_gaps": sum(r["video_only_telemetry_gaps"] for r in usable),
                    "minimum_distance_sign_test": signs},
        "verdict": verdict, "comparable": not reasons, "reasons": reasons,
        "rule": ("no_effect only when no comparable pair settles slower with video, exceeds the p95 jitter ratio on any "
                 "stream, or has video-only telemetry gaps, and the minimum-distance deltas are not one-directional "
                 f"(two-sided sign test p >= {SIGN_TEST_ALPHA}); perturbed when a majority of comparable pairs show one "
                 "of the settling/timing effects; directional_effect when the sign test gives p < "
                 f"{SIGN_TEST_ALPHA}; inconclusive otherwise"),
        "recommendation": {"perturbed": "scored trials never record; media reruns only",
                           "no_effect": "no evidence at this sample size that recording perturbs settling, collector "
                                        "timing or the minimum distance",
                           "directional_effect": "not cleared: the minimum distance moves the same way with video in "
                                                 "every pair; scored trials keep not recording",
                           "inconclusive": "not cleared: scored trials keep not recording; add pairs or keep media to "
                                           "reruns",
                           "insufficient_pairs": f"not cleared: run at least {min_pairs} comparable pairs"}[verdict],
        "claim_boundary": "decision aid for the recording policy; synthetic simulation, few pairs, not a statistical test",
        "pairs": reports,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pair", nargs=2, type=Path, action="append", required=True, metavar=("NO_VIDEO", "WITH_VIDEO"))
    parser.add_argument("--settling-tolerance-s", type=float, default=DEFAULT_SETTLING_TOLERANCE_S,
                        help="a pair settles slower when the median step delta exceeds this")
    parser.add_argument("--jitter-ratio", type=float, default=DEFAULT_JITTER_RATIO,
                        help="timing is perturbed when a stream's p95 inter-arrival grows by more than this factor")
    parser.add_argument("--min-pairs", type=int, default=DEFAULT_MIN_PAIRS)
    parser.add_argument("--output", type=Path, help="write the report here instead of stdout")
    args = parser.parse_args(argv)
    try:
        report = compare([(describe(plain), describe(video)) for plain, video in args.pair],
                         args.settling_tolerance_s, args.jitter_ratio, args.min_pairs)
    except (FileNotFoundError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
