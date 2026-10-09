"""python -m race_tag_graph replay --telemetry FILE | --bag DIR [--config YAML] --out DIR [--solver batch|isam2|both]
[--evaluate-against-truth]

Reads the sensor topics only, builds the graph, solves, writes trajectory.csv, landmarks.csv, dock_point.csv,
results.json and figures. --evaluate-against-truth (telemetry only, simulation) adds the scoring metrics and the lab
fuser comparison through the evaluation modules, after the solve.
"""
from __future__ import annotations

import argparse
import hashlib
import re
import sys
from pathlib import Path

from . import dock as dock_module
from .evaluate import is_development
from .factors import DEFAULT_CONFIG, build, load_config, load_layout
from .metrics import sensor_grounded
from .sensors import read_rosbag2, read_telemetry
from .solvers import solve_batch, solve_isam2
from .writers import results, write_dock_point, write_figures, write_landmarks, write_results, write_trajectory


OPAQUE_ID = re.compile(r"^[0-9a-fA-F-]{16,}$")


def trial_name(args) -> str | None:
    """--trial, or the telemetry's trial directory (output/telemetry.jsonl -> its parent). An opaque id (a UUID or
    a long hex directory name) is never recorded."""
    if args.trial:
        return args.trial
    if not args.telemetry:
        return None
    path = Path(args.telemetry).resolve()
    name = path.parent.parent.name if path.parent.name == "output" else path.parent.name
    return None if OPAQUE_ID.match(name) else name


def sha256(path: Path) -> str | None:
    path = Path(path)
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def replay(args) -> dict:
    config = load_config(args.config)
    stream = read_telemetry(args.telemetry) if args.telemetry else read_rosbag2(args.bag)
    problem = build(stream, config)
    solutions = []
    if args.solver in ("batch", "both"):
        solutions.append(solve_batch(problem))
    if args.solver in ("isam2", "both"):
        solutions.append(solve_isam2(problem))
    primary = solutions[0]
    layout = load_layout(config)
    dock = dock_module.fit_dock(primary.landmarks, layout, config)
    lab_fuser = truth = None
    if args.evaluate_against_truth:
        if not args.telemetry:
            raise SystemExit("--evaluate-against-truth needs --telemetry (simulation truth)")
        from .truth import read_truth  # evaluation side, after the solve
        truth = read_truth(args.telemetry)
        lab_fuser = truth.lab_fuser
    metrics = sensor_grounded(problem, primary, lab_fuser)
    if truth is not None:
        from .evaluate import scoring
        metrics.update(scoring(problem, primary, truth, dock))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    write_trajectory(out / "trajectory.csv", problem, solutions)
    write_landmarks(out / "landmarks.csv", solutions)
    dock_rows = write_dock_point(out / "dock_point.csv", problem, primary, dock)
    trial = trial_name(args)
    development = is_development(trial, config)
    inputs = {"source": Path(args.telemetry or args.bag).name, "source_sha256": sha256(args.telemetry or ""),
              "trial": trial, "development_trial": development, "headline_eligible": not development,
              "config": Path(args.config).name, "config_file_sha256": sha256(args.config),
              "solver_for_metrics": primary.solver, "dock_point_rows": dock_rows}
    document = results(problem, solutions, dock, metrics, inputs)
    write_results(out / "results.json", document)
    document["figures"] = write_figures(out, problem, solutions)
    return document


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="race_tag_graph", description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("replay", help="replay a telemetry file or a bag through the graph")
    source = run.add_mutually_exclusive_group(required=True)
    source.add_argument("--telemetry", type=Path)
    source.add_argument("--bag", type=Path, help="a rosbag2 directory (reader untested until a real bag arrives)")
    run.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    run.add_argument("--out", type=Path, required=True)
    run.add_argument("--solver", choices=("batch", "isam2", "both"), default="batch")
    run.add_argument("--evaluate-against-truth", action="store_true")
    run.add_argument("--trial", help="the trial's name, for the development-trial check (default: its directory)")
    args = parser.parse_args(argv)
    document = replay(args)
    print(f"{document['counts']['keyframes']} keyframes, {document['counts']['landmarks']} landmarks, "
          f"{document['counts']['detections']} detections; dock {document['dock']['status']}; wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
