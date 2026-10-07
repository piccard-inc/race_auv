from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("race_m3_metrics", ROOT / "race_m3_metrics.py")
m3 = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(m3)
STATION_DOCK = (3.59, 0.325, 3.91)
PARAMETERS = {"standoffs_m": [1.5, 0.5, 0.0], "speed_cap_mps": 0.1}


def approach(t: float, floor: float = 0.008) -> float:
    """Stand-off: 1.003 m closing at 0.05 m/s to the floor (0.02 m reached between rows 19.6 and 19.7 s)."""
    return max(1.003 - 0.05 * t, floor)


def drift_out(t: float) -> float:
    """Closes as approach(), holds 0.008 m until 30 s, then drifts out at 2 mm/s (0.02 m at 36 s)."""
    return approach(t) if t <= 30.0 else 0.008 + 0.002 * (t - 30.0)


def dock_row(t: float, standoff: float, vertical: float = 0.0) -> dict:
    """A collector dock row; rel is in the station dock frame (Rx(pi) of the station: +z shallower), so the world
    delta is (rel_x, -rel_y, -rel_z) and a vertical offset of v (z_W AUV - z_W station) is rel_z = -v."""
    rel = [-standoff, 0.0, -vertical]
    auv = [STATION_DOCK[0] + rel[0], STATION_DOCK[1] - rel[1], STATION_DOCK[2] - rel[2]]
    return {"kind": "dock", "t": round(t, 3), "distance_m": math.sqrt(sum(v * v for v in rel)),
            "auv_dock_point_m": auv, "station_dock_point_m": list(STATION_DOCK),
            "relative_position_in_station_dock_m": rel}


def make_trial(root: Path, profile=approach, *, vertical: float = 0.0, end_t: float = 60.0, skip=(), contacts=(),
               planner=None, planner_inputs=(None, None), status="completed", poses=None, extra_rows=(),
               trial_fields=None, labels=None) -> Path:
    root.mkdir(parents=True)
    rows = [dock_row(k / 10, profile(k / 10), vertical) for k in range(int(end_t * 10) + 1)
            if not any(a <= k / 10 < b for a, b in skip)]
    for row in rows if labels else ():  # the collector labels each dock row with its pose or planner stage
        row["label"] = labels(row["t"])
    rows += [{"kind": "contact", "t": t, "contact": "auv_station", "normal_force_n": force} for t, force in contacts]
    if poses:  # ground truth for the mesh placement: AUV Base at the origin, station Base at poses["station"]
        for k in range(int(end_t * 10) + 1):
            for kind, position in (("gt_auv", (0.0, 0.0, 0.0)), ("gt_station", poses["station"])):
                rows.append({"kind": kind, "t": k / 10, "x": position[0], "y": position[1], "z": position[2],
                             "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0})
    rows += list(extra_rows)
    rows.sort(key=lambda row: row["t"])
    (root / "telemetry.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    audits = {}
    for name, verdict in zip(("before_mission", "at_end"), planner_inputs):
        audits[f"consumption_audit_{name}"] = {"problems": [], **({"planner_inputs": {
            "node": "/race_auv/piccard_planner", "reads_only_fuser_tf_and_odometry": verdict}} if verdict is not None
            else {})}
    (root / "trial.json").write_text(json.dumps({
        "schema": "piccard.race-auv.trial/v1", "status": status,
        "stop_reason": "mission_complete" if status == "completed" else "wall_timeout",
        "mission_start_t": 0.0, "mission_end_t": end_t, "mission": {"schema": "piccard.race-auv.planner-mission/v1"},
        "comparison_context": {"trial_id": root.name, "stage": "m3-b", "arm": "planner"}, "planner": planner,
        **audits, **(trial_fields or {})}))
    (root / "docking.json").write_text("{}")
    return root


def cube(path: Path, half: float) -> None:
    corners = [(x, y, z) for x in (-half, half) for y in (-half, half) for z in (-half, half)]
    faces = ((1, 2, 4, 3), (5, 7, 8, 6), (1, 5, 6, 2), (3, 4, 8, 7), (1, 3, 7, 5), (2, 6, 8, 4))
    path.write_text("".join(f"v {x} {y} {z}\n" for x, y, z in corners) +
                    "".join("f " + " ".join(map(str, face)) + "\n" for face in faces))


def analyze(root: Path, **kwargs) -> dict:
    return m3.analyze(root, **kwargs)


class ClosureTests(unittest.TestCase):
    def test_closure_at_both_thresholds_and_a_full_hold(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t"))
        two, one = report["closure"]["2cm"], report["closure"]["1cm"]
        self.assertEqual((two["closure_t"], one["closure_t"]), (19.7, 19.9))  # 0.018 m, then the 0.008 m floor
        self.assertAlmostEqual(two["gap_at_closure_m"], 0.018, places=4)
        self.assertEqual((two["time_to_closure_s"], one["time_to_closure_s"]), (19.7, 19.9))  # the mission starts at 0
        for item in (two, one):
            self.assertTrue(item["closed"])
            self.assertTrue(item["docked"])
            self.assertTrue(item["hold"]["observed"])
            self.assertEqual(item["hold"]["fraction_within"], 1.0)
            self.assertIn(item["hold"]["samples"], (300, 301))  # 30 s of 10 Hz rows, the end row by rounding
        self.assertEqual(one["hold"]["max_gap_m"], 0.008)
        self.assertEqual(report["gap"]["minimum_m"], 0.008)
        self.assertFalse(report["excluded"])

    def test_speed_at_closure_and_over_the_final_approach(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t"))
        two = report["closure"]["2cm"]
        # rows 19.2 s (0.043 m) and 20.2 s (floor 0.008 m): 0.035 m in 1.0 s
        self.assertAlmostEqual(two["speed_at_closure_mps"], (approach(19.2) - approach(20.2)) / 1.0, places=4)
        self.assertAlmostEqual(two["velocity_at_closure_in_station_dock_mps"][0], 0.035, places=4)
        approach_ = two["final_approach"]
        self.assertEqual(approach_["from_t"], 14.1)  # 14.0 s is the last row farther than 0.3 m (0.303 m)
        self.assertTrue(approach_["crossed_standoff"])
        self.assertAlmostEqual(approach_["max_speed_mps"], 0.05, places=4)

    def test_closed_then_drifted_out_is_not_docked(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", drift_out))
        two, one = report["closure"]["2cm"], report["closure"]["1cm"]
        self.assertTrue(two["closed"] and one["closed"])
        self.assertFalse(two["docked"] or one["docked"])
        self.assertTrue(two["hold"]["observed"])
        self.assertAlmostEqual(two["hold"]["fraction_within"], 164 / 301, delta=0.01)  # 19.7..36.0 s of 19.7..49.7 s
        self.assertAlmostEqual(one["hold"]["fraction_within"], 112 / 301, delta=0.01)  # 19.9..31.0 s
        self.assertAlmostEqual(two["hold"]["max_gap_m"], drift_out(49.7), delta=0.0005)

    def test_a_hold_past_the_mission_or_across_a_row_gap_is_not_observed(self):
        with tempfile.TemporaryDirectory() as tmp:
            short = analyze(make_trial(Path(tmp) / "short", end_t=40.0))
            gap = analyze(make_trial(Path(tmp) / "gap", skip=((30.0, 31.5),)))
        for report in (short, gap):
            item = report["closure"]["2cm"]
            self.assertEqual(item["hold"]["fraction_within"], 1.0)
            self.assertFalse(item["hold"]["observed"])
            self.assertFalse(item["docked"])

    def test_the_hold_length_is_a_parameter(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", drift_out), hold_s=10.0)
        self.assertTrue(report["closure"]["2cm"]["docked"])  # 19.7..29.7 s stays at or under 0.018 m
        self.assertEqual(report["hold_s"], 10.0)

    def test_the_vertical_bias_keeps_the_gap_open(self):
        """M2 R-B: the AUV dock point held 2.19 cm shallow, so the gap never reaches 2 cm."""
        with tempfile.TemporaryDirectory() as tmp:
            biased = analyze(make_trial(Path(tmp) / "rb", vertical=-0.0219))
            small = analyze(make_trial(Path(tmp) / "small", vertical=-0.005))
        self.assertEqual([biased["closure"][name]["closed"] for name in ("2cm", "1cm")], [False, False])
        self.assertAlmostEqual(biased["gap"]["minimum_m"], math.hypot(0.008, 0.0219), places=4)
        self.assertEqual(biased["gap"]["vertical_offset_at_minimum_m"], -0.0219)
        item = small["closure"]["2cm"]
        self.assertEqual(item["vertical_offset_m"]["at_closure"], -0.005)
        self.assertEqual(item["vertical_offset_m"]["hold"]["median"], -0.005)

    def test_the_gap_series_is_one_row_per_second_of_the_mission(self):
        with tempfile.TemporaryDirectory() as tmp:
            series = analyze(make_trial(Path(tmp) / "t"))["gap"]["series_1s"]
        self.assertEqual([t for t, _ in series], [float(k) for k in range(61)])
        self.assertAlmostEqual(series[0][1], 1.003, places=4)


class ClearanceClosureTests(unittest.TestCase):
    """Protocol v1.3: the hover at the approach clearance, scored beside the 3D gap."""

    def test_the_hover_closes_where_the_3d_gap_cannot(self):
        """The AUV dock point 2 cm above the station's with the horizontal gap closing to 8 mm: the 3D gap never
        reaches 2 cm; at clearance 0.02 +- 0.005 both thresholds close, hold and are entered slowly."""
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", vertical=-0.02), clearance_m=0.02, clearance_tol_m=0.005)
        self.assertEqual([report["closure"][name]["closed"] for name in ("2cm", "1cm")], [False, False])
        self.assertEqual(report["clearance"], {"clearance_m": 0.02, "tolerance_m": 0.005, "computed": True,
                                               "planner_approach_clearance_m": None})
        two, one = report["closure_at_clearance"]["2cm"], report["closure_at_clearance"]["1cm"]
        self.assertEqual((two["closure_t"], one["closure_t"]), (19.7, 19.9))
        self.assertAlmostEqual(two["gap_at_closure_m"], 0.018, places=4)  # horizontal
        self.assertEqual(two["vertical_offset_m"]["at_closure"], -0.02)
        self.assertTrue(all(item["docked"] and item["docked_at_speed"] for item in (two, one)))
        self.assertEqual((two["hold"]["max_gap_m"], one["hold"]["max_gap_m"]), (0.018, 0.008))
        self.assertEqual(report["gap"]["minimum_horizontal_m"], 0.008)
        self.assertEqual(report["gap"]["vertical_offset_at_minimum_horizontal_m"], -0.02)

    def test_outside_the_vertical_tolerance_it_does_not_close(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", vertical=-0.012), clearance_m=0.02, clearance_tol_m=0.005)
        self.assertEqual([report["closure_at_clearance"][name]["closed"] for name in ("2cm", "1cm")], [False, False])

    def test_drifting_out_breaks_the_hold_at_clearance_too(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", drift_out, vertical=-0.02), clearance_m=0.02,
                             clearance_tol_m=0.005)
        item = report["closure_at_clearance"]["2cm"]
        self.assertTrue(item["closed"])
        self.assertFalse(item["docked"])

    def test_by_default_it_is_not_scored_and_the_3d_closure_is_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            default = analyze(make_trial(Path(tmp) / "d"))
            scored = analyze(make_trial(Path(tmp) / "s"), clearance_m=0.02, clearance_tol_m=0.005)
        self.assertIsNone(default["closure_at_clearance"])
        self.assertFalse(default["clearance"]["computed"])
        self.assertIn("reason", default["clearance"])
        self.assertEqual(default["closure"], scored["closure"])

    def test_the_trial_planner_clearance_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", planner={"approach_clearance_m": 0.02}))
        self.assertEqual(report["clearance"]["planner_approach_clearance_m"], 0.02)
        self.assertEqual(report["planner"]["timeline"]["approach_clearance_m"], 0.02)


class ControllerVariantTests(unittest.TestCase):
    def test_the_variant_and_its_patch_are_reported(self):
        patch = {"controller_variant": "keep_xy_integral", "patch_sha256": "de" * 32}
        with tempfile.TemporaryDirectory() as tmp:
            variant = analyze(make_trial(Path(tmp) / "v", trial_fields={
                "controller_variant": "keep_xy_integral", "source_pins": {"mvp_control_patch": patch}}))
            upstream = analyze(make_trial(Path(tmp) / "u", trial_fields={"controller_variant": "upstream"}))
            older = analyze(make_trial(Path(tmp) / "o"))
            result = m3.matrix([variant, upstream, older])
        self.assertEqual((variant["controller_variant"], variant["mvp_control_patch"]), ("keep_xy_integral", patch))
        self.assertEqual((upstream["controller_variant"], upstream["mvp_control_patch"]), ("upstream", None))
        self.assertIsNone(older["controller_variant"])
        self.assertEqual([row["controller_variant"] for row in result["trials"]],
                         ["keep_xy_integral", "upstream", None])
        self.assertIn("M3-C", variant["claim_boundary"])


def closes_drifts_and_returns(t: float) -> float:
    """Closes as approach(); 0.008 m until 25 s; out at 2 mm/s (past 2 cm at 31 s) to 0.028 m at 35 s; back in from
    40 s at 2 mm/s (2 cm at 44 s, 1 cm at 49 s), 0.008 m from 50 s."""
    if t <= 25.0:
        return approach(t)
    if t <= 35.0:
        return 0.008 + 0.002 * (t - 25.0)
    if t <= 40.0:
        return 0.028
    return max(0.028 - 0.002 * (t - 40.0), 0.008)


def planner_record(final_start: float) -> dict:
    """A planner trial.json block with its stages' start times, the final stage from final_start."""
    starts = (0.0, 2.0, 4.0, final_start)
    return {"node": "/piccard_planner", "parameters": PARAMETERS, "parameters_sha256": m3.canonical_sha256(PARAMETERS),
            "complete": True, "stages": [{"label": f"planner_standoff_{s:g}m", "standoff_m": s, "start_t": t}
                                         for s, t in zip((3.0, 1.5, 0.3, 0.0), starts)]}


def versioned(version: str | None, trial_id: str = "t") -> dict:
    return {"comparison_context": {"trial_id": trial_id, "stage": "m3-0", "arm": "planner",
                                   **({"protocol_version": version} if version else {})}}


class FirstFullHoldTests(unittest.TestCase):
    """Protocol v1.5 (piccard-inc/piccard-experiments#88): docked is the first hold_s window inside the final stage in
    which the criterion holds continuously, entered at <= the speed limit; the first closure stays reported."""

    def trial(self, tmp, name="t", profile=approach, final_start=10.0, version="v1.5", **kwargs):
        return analyze(make_trial(Path(tmp) / name, profile, planner=planner_record(final_start),
                                  planner_inputs=(True, True), trial_fields=versioned(version), **kwargs))

    def test_the_first_closure_that_holds_is_the_first_full_hold(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = self.trial(tmp)
        for name, entry in (("2cm", 19.7), ("1cm", 19.9)):
            item = report["closure"][name]
            full = item["first_full_hold"]
            self.assertTrue(item["docked"] and item["docked_any_window"] and item["docked_any_window_at_speed"])
            self.assertEqual((full["entry_t"], full["time_into_final_stage_s"]), (entry, round(entry - 10.0, 3)))
            self.assertEqual((full["window"]["from_t"], full["window"]["to_t"]), (entry, round(entry + 30.0, 3)))
            self.assertTrue(full["speed_ok"])
        self.assertEqual(report["criterion"]["scored"], "first_full_hold")
        self.assertEqual(report["criterion"]["final_stage"], {"start_t": 10.0, "label": "planner_standoff_0m",
                                                              "source": "planner.stages"})

    def test_a_later_full_hold_docks_where_the_first_closure_does_not(self):
        """The first closure's hold breaks at 31 s; the vehicle is back under 2 cm at 44 s and holds to the end."""
        with tempfile.TemporaryDirectory() as tmp:
            report = self.trial(tmp, profile=closes_drifts_and_returns, end_t=90.0)
        two = report["closure"]["2cm"]
        self.assertEqual((two["closure_t"], two["docked"]), (19.7, False))
        self.assertTrue(two["docked_any_window"])
        self.assertEqual(two["first_full_hold"]["entry_t"], 44.0)
        self.assertEqual(report["closure"]["1cm"]["first_full_hold"]["entry_t"], 49.0)
        self.assertLess(two["first_full_hold"]["speed_at_entry_mps"], 0.1)

    def test_the_window_starts_inside_the_final_stage(self):
        """Closed at 19.7 s, before a final stage starting at 30 s: the window starts at the stage start."""
        with tempfile.TemporaryDirectory() as tmp:
            report = self.trial(tmp, final_start=30.0, end_t=70.0)
        self.assertEqual(report["closure"]["2cm"]["first_full_hold"]["entry_t"], 30.0)
        self.assertEqual(report["closure"]["2cm"]["closure_t"], 19.7)  # the first closure is unchanged

    def test_a_window_past_the_mission_end_or_across_a_row_gap_does_not_count(self):
        with tempfile.TemporaryDirectory() as tmp:
            short = self.trial(tmp, "short", end_t=45.0)  # 19.7 + 30 > 45
            gapped = self.trial(tmp, "gapped", end_t=90.0, skip=((30.0, 31.5),))
        self.assertEqual(short["closure"]["2cm"]["first_full_hold"], {"computed": True, "found": False})
        self.assertFalse(short["closure"]["2cm"]["docked_any_window"])
        self.assertEqual(gapped["closure"]["2cm"]["first_full_hold"]["entry_t"], 31.5)  # restarts after the gap

    def test_the_entry_speed_scores_at_speed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_trial(Path(tmp) / "t", planner=planner_record(10.0), planner_inputs=(True, True),
                              trial_fields=versioned("v1.5"))
            strict = analyze(root, speed_limit_mps=0.03)  # 0.035 m/s at the 2 cm entry
        two = strict["closure"]["2cm"]
        self.assertEqual((two["docked_any_window"], two["first_full_hold"]["speed_ok"],
                          two["docked_any_window_at_speed"], two["first_full_hold_at_speed"]),
                         (True, False, False, None))

    def test_a_later_window_entered_slowly_docks_at_speed(self):
        """The criterion is existential: the first full window entered at 0.035 m/s (over a 0.03 m/s limit), the
        vehicle out and back at 2 mm/s, a second full window: docked at speed, on the second window's entry."""
        def fast_then_slow(t: float) -> float:
            if t <= 55.0:
                return approach(t)  # 2 cm at 19.7 s at 0.05 m/s; the 0.008 m floor to 55 s
            if t <= 65.0:
                return 0.008 + 0.002 * (t - 55.0)  # out past 2 cm at 61 s
            if t <= 70.0:
                return 0.028
            return max(0.028 - 0.002 * (t - 70.0), 0.008)  # back under 2 cm at 74 s
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", fast_then_slow, end_t=120.0, planner=planner_record(10.0),
                                        planner_inputs=(True, True), trial_fields=versioned("v1.5")),
                             speed_limit_mps=0.03)
        two = report["closure"]["2cm"]
        self.assertEqual((two["first_full_hold"]["entry_t"], two["first_full_hold"]["speed_ok"]), (19.7, False))
        self.assertTrue(two["docked_any_window"] and two["docked_any_window_at_speed"])
        later = two["first_full_hold_at_speed"]
        self.assertEqual((later["entry_t"], later["speed_ok"]), (74.0, True))
        self.assertLess(later["speed_at_entry_mps"], 0.003)
        self.assertEqual((later["window"]["from_t"], later["window"]["to_t"]), (74.0, 104.0))

    def test_the_hover_at_clearance_has_its_own_full_hold(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", vertical=-0.02, planner=planner_record(10.0),
                                        planner_inputs=(True, True), trial_fields=versioned("v1.5")),
                             clearance_m=0.02, clearance_tol_m=0.005)
        self.assertFalse(report["closure"]["2cm"]["docked_any_window"])  # the 3D gap never reaches 2 cm
        at = report["closure_at_clearance"]["2cm"]
        self.assertTrue(at["docked_any_window_at_speed"])
        self.assertEqual(at["first_full_hold"]["entry_t"], 19.7)
        self.assertEqual(at["first_full_hold"]["vertical_offset_m"]["at_entry"], -0.02)

    def test_a_pose_mission_final_stage_is_its_last_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", labels=lambda t: "approach_0p3m" if t < 25.0 else "contact",
                                        trial_fields={"mission": {"schema": "piccard.race-auv.pose-mission/v1",
                                                                  "poses": [{"label": "approach_0p3m"},
                                                                            {"label": "contact"}]}}))
        self.assertEqual(report["criterion"]["final_stage"], {
            "start_t": 25.0, "label": "contact", "source": "the first dock row of the mission's last pose"})
        self.assertEqual(report["closure"]["2cm"]["first_full_hold"]["entry_t"], 25.0)

    def test_without_a_final_stage_it_is_not_computed(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t"))
        full = report["closure"]["2cm"]["first_full_hold"]
        self.assertEqual((full["computed"], full["found"]), (False, False))
        self.assertIn("no final stage", full["reason"])
        self.assertTrue(report["closure"]["2cm"]["docked"])  # the first closure is scored as before

    def test_the_definition_scored_follows_the_protocol_version(self):
        self.assertEqual([m3.scored_criterion({"protocol_version": v}, {"start_t": None})["scored"]
                          for v in (None, "v1.3", "v1.4", "v1.5", "v1.10", "v2.0", "draft")],
                         ["first_closure", "first_closure", "first_closure", "first_full_hold", "first_full_hold",
                          "first_full_hold", "first_closure"])

    def test_the_matrix_counts_each_trial_by_its_own_definition(self):
        """The same drift-and-return run: not docked under v1.4 (its first closure's hold breaks), docked under v1.5
        (a later full hold)."""
        with tempfile.TemporaryDirectory() as tmp:
            roots = [make_trial(Path(tmp) / name, closes_drifts_and_returns, end_t=90.0, planner=planner_record(10.0),
                                planner_inputs=(True, True), trial_fields=versioned(version, name))
                     for name, version in (("v14", "v1.4"), ("v15", "v1.5"))]
            out = Path(tmp) / "out"
            self.assertEqual(m3.main([*map(str, roots), "--output-dir", str(out), "--matrix"]), 0)
            result = json.loads((out / "m3-matrix.json").read_text())
            markdown = (out / "m3-matrix.md").read_text()
        self.assertEqual(result["docked"]["2cm"], 0)
        self.assertEqual(result["docked_any_window"]["2cm"], 2)
        self.assertEqual(result["scored_docked"]["2cm"], 1)  # only the v1.5 trial is scored on the full hold
        self.assertEqual([row["criterion"] for row in result["trials"]], ["first_closure", "first_full_hold"])
        self.assertIn("| v14 | first_closure | 44.0 | yes | 44.0 | yes | 49.0 | yes | 49.0 | yes |", markdown)
        self.assertIn("| v15 | first_full_hold | 44.0 | yes | 44.0 | yes | 49.0 | yes | 49.0 | yes |", markdown)
        self.assertIn("Docked by the definition each trial is scored by, included trials: 2cm: 1, 1cm: 1 of 2",
                      markdown)


class LoadAndCouplingTests(unittest.TestCase):
    def test_load_during_the_hold_in_both_classes(self):
        contacts = [(19.0, 0.0), (20.0, 0.0), (20.1, 4.5), (20.15, 2.0), (25.3, 0.0), (55.0, 9.0)]
        with tempfile.TemporaryDirectory() as tmp:
            load = analyze(make_trial(Path(tmp) / "t", contacts=contacts))["closure"]["2cm"]["load"]
        self.assertEqual((load["events"], load["force_bearing_events"], load["near_miss_events"]), (4, 2, 2))
        self.assertEqual(load["max_normal_force_n"], 4.5)  # the 9 N event at 55 s is after the hold
        self.assertAlmostEqual(load["force_bearing_bin_fraction"], 1 / 60, places=4)

    def test_coupling_uses_the_v1_1_placement(self):
        with tempfile.TemporaryDirectory() as tmp:
            meshes = Path(tmp) / "meshes"
            meshes.mkdir()
            names = ("RACE_sim_Body.obj", "RACE_sim_LeftLeg.obj", "RACE_sim_RightLeg.obj", "RACE_sim_Couplink.obj",
                     "race_station_sim_v6.obj", "race_station_couplink_sim.obj")
            for name in names:
                cube(meshes / name, 0.05)
            coupling = m3.Coupling(meshes)
            root = make_trial(Path(tmp) / "t", poses={"station": (0.105, 0.0, 0.0)})
            report = analyze(root, coupling=coupling)
        self.assertEqual(sorted(report["coupling_provenance"]["meshes"]), sorted(names))
        self.assertEqual(report["coupling_provenance"]["tool"]["file"], "contact_margin_check_v1_1.py")
        item = report["closure"]["2cm"]["coupling"]
        self.assertAlmostEqual(item["at_closure"]["couplink_mm"], 5.0, delta=0.1)  # 0.105 m apart, 0.05 m half-widths
        self.assertAlmostEqual(item["at_closure"]["mesh_mm"], 5.0, delta=0.1)
        self.assertEqual(item["samples"], 31)
        self.assertAlmostEqual(item["hold_couplink_mm"]["max"], 5.0, delta=0.1)
        self.assertEqual(item["hold_samples_touching"], 0)
        self.assertAlmostEqual(report["gap"]["coupling_at_minimum"]["couplink_mm"], 5.0, delta=0.1)
        for key in ("coupling_at_minimum", "coupling_at_minimum_horizontal"):
            parts = report["gap"][key]["parts"]
            self.assertEqual(len(parts["pairs_mm"]), 8)  # four AUV parts, two station parts
            self.assertIn("RACE_sim_LeftLeg~race_station_sim_v6", parts["pairs_mm"])
            self.assertAlmostEqual(parts["closest_mm"], 5.0, delta=0.1)
            self.assertEqual(parts["pairs_mm"][parts["closest_pair"]], parts["closest_mm"])
        self.assertNotIn("parts", item["at_closure"])  # the hold samples stay whole-mesh

    def test_without_meshes_coupling_is_reported_as_not_computed(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t"))
        self.assertFalse(report["coupling_provenance"]["computed"])
        self.assertIsNone(report["closure"]["2cm"]["coupling"])
        self.assertIsNone(report["gap"]["coupling_at_minimum"])
        self.assertIsNone(report["gap"]["coupling_at_minimum_horizontal"])


class PlannerTests(unittest.TestCase):
    def planner(self, **overrides) -> dict:
        return {"node": "/piccard_planner", "parameters": PARAMETERS, "parameters_sha256": m3.canonical_sha256(PARAMETERS),
                "stages": [{"label": f"planner_standoff_{s:g}m", "standoff_m": s} for s in (3.0, 1.5, 0.3, 0.0)],
                "complete": True, **overrides}

    def test_the_parameter_hash_is_the_runtimes(self):
        runtime = importlib.util.spec_from_file_location(
            "race_mission", ROOT.parents[1] / "packages/simulation/race-auv-docking/runtime/mission.py")
        mission = importlib.util.module_from_spec(runtime)
        runtime.loader.exec_module(mission)
        self.assertEqual(m3.canonical_sha256(PARAMETERS), mission.parameters_sha256(PARAMETERS))

    def test_no_planner(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t"))
        self.assertEqual({key: report["planner"][key] for key in ("present", "parameters_sha256",
                                                                   "reads_only_fuser_tf_and_odometry", "problems")},
                         {"present": False, "parameters_sha256": None, "reads_only_fuser_tf_and_odometry": None,
                          "problems": []})
        self.assertFalse(report["excluded"])

    def test_a_planner_with_its_parameters_and_clean_audits(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", planner=self.planner(), planner_inputs=(True, True)))
        planner = report["planner"]
        self.assertTrue(planner["present"] and planner["parameters_hash_verified"])
        self.assertEqual((planner["stages_reached"], planner["complete"]), ([3.0, 1.5, 0.3, 0.0], True))
        self.assertTrue(planner["reads_only_fuser_tf_and_odometry"])
        self.assertEqual(planner["parameters_sha256"], m3.canonical_sha256(PARAMETERS))
        self.assertFalse(report["excluded"])

    def test_planner_problems_exclude_the_trial(self):
        cases = {"planner read outside the fuser TF and odometry": dict(planner=self.planner(),
                                                                        planner_inputs=(True, False)),
                 "a consumption audit has no planner_inputs verdict": dict(planner=self.planner(),
                                                                           planner_inputs=(True, None)),
                 "planner parameters_sha256 does not match its parameters": dict(
                     planner=self.planner(parameters_sha256="0" * 64), planner_inputs=(True, True)),
                 "audit reports planner_inputs but trial.json has no planner": dict(planner_inputs=(True, True))}
        with tempfile.TemporaryDirectory() as tmp:
            for index, (problem, kwargs) in enumerate(cases.items()):
                with self.subTest(problem=problem):
                    report = analyze(make_trial(Path(tmp) / f"t{index}", **kwargs))
                    self.assertIn(problem, report["planner"]["problems"])
                    self.assertTrue(report["excluded"])


EXAMPLE = json.loads((m3.PLANNER_EXAMPLE).read_text())["mission"]


def rendered(value):
    """The submit path's rendering: integral floats as integers (2.0 as 2)."""
    if isinstance(value, float) and value.is_integer():
        return int(value)
    if isinstance(value, list):
        return [rendered(v) for v in value]
    if isinstance(value, dict):
        return {k: rendered(v) for k, v in value.items()}
    return value


def mission_hash(mission: dict) -> str:
    return m3.hashlib.sha256((json.dumps(mission, indent=2, sort_keys=True) + "\n").encode()).hexdigest()


class DockCriterionTests(unittest.TestCase):
    def test_speed_ok_and_docked_at_speed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = make_trial(Path(tmp) / "t")
            default, strict = analyze(root), analyze(root, speed_limit_mps=0.03)
        self.assertEqual((default["speed_limit_mps"], strict["speed_limit_mps"]), (0.1, 0.03))
        for name in ("2cm", "1cm"):
            self.assertTrue(default["closure"][name]["speed_ok"])
            self.assertTrue(default["closure"][name]["docked_at_speed"])
        two = strict["closure"]["2cm"]  # 0.035 m/s at closure: over 0.03
        self.assertEqual((two["docked"], two["speed_ok"], two["docked_at_speed"]), (True, False, False))
        self.assertTrue(strict["closure"]["1cm"]["docked_at_speed"])  # 0.025 m/s
        self.assertIn("docked alone is gap and hold only", default["definitions"]["hold"])

    def test_no_closure_scores_no_speed(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", lambda t: 0.5))
        self.assertEqual({k: report["closure"]["2cm"][k] for k in ("closed", "speed_ok", "docked_at_speed")},
                         {"closed": False, "speed_ok": None, "docked_at_speed": False})


class TimeAndWordsTests(unittest.TestCase):
    def test_time_basis_names_the_clock_and_the_mission_window(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", trial_fields={"mission_start_t": 12.5, "mission_end_t": 60.0}))
        self.assertEqual(report["time_basis"], {"clock": "collector_monotonic_receive_seconds", "mission_start_t": 12.5,
                                                "mission_end_t": 60.0})
        self.assertAlmostEqual(report["closure"]["2cm"]["time_to_closure_s"], 19.7 - 12.5, places=3)
        for field in ("closure_t", "minimum_t", "handover_t", "stale_intervals", "setpoint_steps", "start_t"):
            self.assertIn(field, report["definitions"]["time_fields"])

    def test_plain_words(self):
        for (status, reason), words in {
                ("budget_censored", "fixed_wall_horizon"): ("ran to the horizon without completing",
                                                            "the fixed horizon was reached"),
                ("failed", "consumption_audit:planner_reads_other_inputs,x"): (
                    "failed", "the consumption audit found a problem (planner_reads_other_inputs, x)"),
                ("failed", "signal_2"): ("failed", "stopped by signal 2"),
                ("failed", "some_new_reason"): ("failed", "some new reason")}.items():
            with self.subTest(reason=reason):
                self.assertEqual((m3.plain(m3.STATUS_PLAIN, status), m3.plain(m3.STOP_REASON_PLAIN, reason)), words)

    def test_issue_references_name_their_repository(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t"))
        text = json.dumps({"claim": report["claim_boundary"], "definitions": report["definitions"]})
        self.assertNotRegex(text, r"(?<![a-z-])#\d")  # every #N is owner/repo#N
        self.assertIn("perception-accuracy claim", report["claim_boundary"])


class ProvenanceTests(unittest.TestCase):
    def trial(self, tmp, mission, bound, **context):
        root = make_trial(Path(tmp) / "t")
        trial = json.loads((root / "trial.json").read_text())
        trial["mission"], trial["mission_sha256"] = rendered(mission), mission_hash(rendered(mission))
        trial["comparison_context"].update(mission_sha256=bound, **context)
        (root / "trial.json").write_text(json.dumps(trial))
        return analyze(root)

    def test_a_planner_mission_binds_in_the_builder_form(self):
        """The builder hashes its own rendering (2.0, [3.0, ..., 0.0]); the runtime hashes what the submit path sent
        (2, [3, ..., 0]). The builder form casts the planner example's floats back."""
        with tempfile.TemporaryDirectory() as tmp:
            builder = self.trial(tmp, EXAMPLE, mission_hash(EXAMPLE), role="planner", protocol_version="v1.0")
        with tempfile.TemporaryDirectory() as tmp:
            runtime = self.trial(tmp, EXAMPLE, mission_hash(rendered(EXAMPLE)))
        with tempfile.TemporaryDirectory() as tmp:
            other = self.trial(tmp, EXAMPLE, "0" * 64)
        with tempfile.TemporaryDirectory() as tmp:
            unbound = self.trial(tmp, EXAMPLE, None)
        self.assertNotEqual(mission_hash(EXAMPLE), mission_hash(rendered(EXAMPLE)))  # the two forms differ
        self.assertEqual((builder["mission_matches_context"], builder["mission_hash_form"]), (True, "builder"))
        self.assertEqual((runtime["mission_matches_context"], runtime["mission_hash_form"]), (True, "runtime"))
        self.assertEqual((other["mission_matches_context"], other["mission_hash_form"]), (False, None))
        self.assertEqual((unbound["mission_matches_context"], unbound["mission_hash_form"]), (None, None))
        self.assertEqual((builder["context"]["role"], builder["context"]["protocol_version"]), ("planner", "v1.0"))

    def test_a_pose_mission_binds_in_the_builder_form(self):
        pose = {"label": "a", "x_m": -0.325, "y_m": 0.0, "z_m": 3.78, "roll_rad": 0.0, "pitch_rad": 0.0,
                "yaw_rad": 1.571, "dwell_s": 40}
        mission = {"schema": "piccard.race-auv.pose-mission/v1", "mission_id": "m", "poses": [pose]}
        with tempfile.TemporaryDirectory() as tmp:
            report = self.trial(tmp, mission, mission_hash(mission))
        self.assertEqual((report["mission_matches_context"], report["mission_hash_form"]), (True, "builder"))


class TimelineAndTagsTests(unittest.TestCase):
    PIVOT = {"derived": {"camera": "cam_front", "pivot_m": [0.456667, 0.0, 0.22], "dock_link": "dock_point",
                         "tags": {"tag36h11:146": [0.43, 0.0, 0.405], "tag36h11:541": [0.47, 0.0, 0.16],
                                  "tag36h11:558": [0.47, 0.0, 0.095]}}, "ok": True}

    def planner(self, **extra):
        return {"node": "/piccard_planner", "parameters": PARAMETERS, "parameters_sha256": m3.canonical_sha256(PARAMETERS),
                "handover_t": 3.0, "stages": [{"label": "planner_standoff_1p5m", "standoff_m": 1.5, "start_t": 3.0}],
                "stale_intervals": [{"from_t": 5.0, "to_t": 5.4, "reason": "age"}], "tag_pivot": self.PIVOT, **extra}

    def test_set_point_steps_from_the_trial_record(self):
        steps = [[3.0, -0.3, 4.0, 3.78, 1.57], [20.0, -0.31, 4.1, 3.78, 1.58]]
        rehold = {"t": 50.0, "received_t": 21.0, "samples": 150, "applied": True, "previous": 1.60, "heading": 1.57}
        final_along = {"phase": "hold", "side": -1, "reapproaches": [],
                       "arrivals": [{"t": 104.2, "error_m": -0.0042, "side": -1, "change_m": 0.0256,
                                     "received_t": 54.4}]}
        with tempfile.TemporaryDirectory() as tmp:
            planner = self.planner(setpoint_steps=steps, heading_rehold=rehold, final_along=final_along)
            report = analyze(make_trial(Path(tmp) / "t", planner=planner, planner_inputs=(True, True)))
        timeline = report["planner"]["timeline"]
        self.assertEqual(timeline["heading_rehold"], rehold)
        self.assertEqual(timeline["final_along"], final_along)  # protocol v1.5, from trial.json
        self.assertEqual(timeline["setpoint_steps"], {"source": "trial.json", "steps": steps})
        self.assertEqual((timeline["handover_t"], timeline["stages"][0]["start_t"]), (3.0, 3.0))
        self.assertEqual(timeline["stale_intervals"], [{"from_t": 5.0, "to_t": 5.4, "reason": "age"}])

    def test_set_point_steps_reconstructed_from_planner_rows(self):
        def row(t, updates, x):
            return {"kind": "planner", "t": t, "planner": {"state": "tracking", "setpoint_updates": updates,
                                                          "command": {"x_m": x, "y_m": 4.0, "z_m": 3.78, "yaw_rad": 1.5}}}
        rows = [row(3.0, {"x_m": 1, "y_m": 1, "z_m": 0, "yaw_rad": 1}, -0.3),
                row(3.2, {"x_m": 1, "y_m": 1, "z_m": 0, "yaw_rad": 1}, -0.3),
                row(9.0, {"x_m": 2, "y_m": 1, "z_m": 0, "yaw_rad": 1}, -0.35)]
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", planner=self.planner(), planner_inputs=(True, True),
                                        extra_rows=rows))
        self.assertEqual(report["planner"]["timeline"]["setpoint_steps"],
                         {"source": "telemetry", "steps": [[3.0, -0.3, 4.0, 3.78, 1.5], [9.0, -0.35, 4.0, 3.78, 1.5]]})

    def test_the_forward_tags_are_collinear_and_detections_are_counted(self):
        detections = [{"kind": "detections", "t": 10 + k / 10, "camera": "cam_front", "count": 3,
                       "ids": ["tag36h11:146", "tag36h11:541", "tag36h11:558"]} for k in range(5)]
        detections += [{"kind": "detections", "t": 10.05, "camera": "cam_down", "count": 0, "ids": []}]
        with tempfile.TemporaryDirectory() as tmp:
            report = analyze(make_trial(Path(tmp) / "t", planner=self.planner(), planner_inputs=(True, True),
                                        extra_rows=detections))
        tags = report["tags"]
        self.assertEqual(tags["source"], "trial.json planner.tag_pivot")
        self.assertEqual(tags["detections_in_mission"]["cam_front"],
                         {"rows": 5, "ids": {"tag36h11:146": 5, "tag36h11:541": 5, "tag36h11:558": 5}})
        self.assertEqual(tags["detections_in_mission"]["cam_down"], {"rows": 1, "ids": {}})
        self.assertLess(tags["forward_collinearity_m"], 0.02)  # all three on the station's centreline, y = 0

    def test_a_layout_off_the_line_is_not_collinear(self):
        self.assertAlmostEqual(m3.collinearity_m([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 0.25, 0.0]]), 0.25, places=4)
        self.assertIsNone(m3.collinearity_m([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))


class PerceptionTests(unittest.TestCase):
    def test_fused_minus_truth_by_range_and_by_tag_set(self):
        """Truth at 1.2 m straight ahead (in bin 1-1.5 m); the fused point is 5 cm long, 2 cm left, 1 cm low and
        its heading 0.1 rad left of the true one (relative yaw -0.05: the true dock frame heads +0.05 in base_link)."""
        rows = []
        for k in range(40):
            t = 10 + k / 10
            rows.append({"kind": "dock", "t": t, "distance_m": 0.8, "station_dock_in_auv_base_link_m": [1.2, 0.0, 0.0],
                         "relative_rpy_rad": [0.0, 0.0, -0.05], "relative_position_in_station_dock_m": [-0.8, 0, 0],
                         "auv_dock_point_m": [0, 0, 0], "station_dock_point_m": [0.8, 0, 0]})
            w, z = math.cos(0.075), math.sin(0.075)  # heading 0.15
            rows.append({"kind": "fused_dock", "t": t + 0.02, "position_m": [1.25, 0.02, -0.01],
                         "orientation_q": [0.0, 0.0, z, w]})
            if k < 20:
                rows.append({"kind": "detections", "t": t + 0.01, "camera": "cam_front", "count": 1,
                             "ids": ["tag36h11:146"]})
        with tempfile.TemporaryDirectory() as tmp:
            root = make_trial(Path(tmp) / "t", extra_rows=rows)
            out = Path(tmp) / "out"
            m3.main([str(root), "--output-dir", str(out)])
            report = json.loads((out / "t.m3.json").read_text())
            series = [json.loads(line) for line in (out / "t.m3-perception.jsonl").read_text().splitlines()]
        summary = report["perception"]
        self.assertEqual(summary["samples"], 40)
        cell = summary["by_true_range"]["1-1.5 m"]
        self.assertEqual(cell["n"], 40)
        self.assertAlmostEqual(cell["range_m"]["mean"], math.hypot(1.25, 0.02, 0.01) - 1.2, places=5)
        self.assertAlmostEqual(cell["lateral_m"]["mean"], 0.02, places=5)
        self.assertAlmostEqual(cell["vertical_m"]["mean"], -0.01, places=5)
        self.assertAlmostEqual(cell["yaw_rad"]["mean"], 0.1, places=5)
        self.assertEqual(cell["yaw_rad"]["sd"], 0.0)
        # detections stop after 11.91 s; the fuser keeps a tag 0.5 s, so the samples from 12.42 s have none
        self.assertEqual({k: v["n"] for k, v in summary["by_forward_tags"].items()}, {"tag36h11:146": 24, "none": 16})
        self.assertEqual(len(series), 40)
        self.assertEqual(series[0][6], "tag36h11:146")
        self.assertEqual(series[-1][6], "")


class MatrixTests(unittest.TestCase):
    def test_cli_writes_trial_reports_and_the_matrix(self):
        with tempfile.TemporaryDirectory() as tmp:
            docked = make_trial(Path(tmp) / "docked")
            drifted = make_trial(Path(tmp) / "drifted", drift_out)
            timed_out = make_trial(Path(tmp) / "timed-out", status="stopped")
            out = Path(tmp) / "out"
            self.assertEqual(m3.main([str(docked), str(drifted), str(timed_out), "--output-dir", str(out),
                                      "--matrix"]), 0)
            result = json.loads((out / "m3-matrix.json").read_text())
            report = json.loads((out / "docked.m3.json").read_text())
            markdown = (out / "m3-matrix.md").read_text()
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(m3.main([str(Path(tmp) / "missing"), "--output-dir", str(out)]), 2)
        self.assertEqual(report["schema"], m3.SCHEMA)
        self.assertIn("claim_boundary", report)
        self.assertEqual(set(report["definitions"]), {
            "gap", "closure", "closure_at_clearance", "speed", "final_approach", "hold", "load", "coupling",
            "vertical_offset", "planner",
            "exclusion", "dock_criterion", "time_fields", "frames_and_signs", "true_range", "planner_frame",
            "planner_stage", "setpoint_steps", "tags", "perception", "provenance", "plain_words", "controller_variant",
            "first_full_hold", "criterion", "simulator_seed"})
        self.assertIn("simulator_seed", report)  # carried from trial.json as recorded
        self.assertEqual(result["docked"], {"2cm": 1, "1cm": 1})
        self.assertEqual(result["docked_at_speed"], {"2cm": 1, "1cm": 1})  # 0.035 and 0.025 m/s at closure
        self.assertEqual(result["speed_limit_mps"], 0.1)
        self.assertEqual(result["included"], 2)
        rows = {row["label"]: row for row in result["trials"]}
        self.assertTrue(rows["timed-out"]["excluded"])
        self.assertFalse(rows["drifted"]["2cm"]["docked"])
        self.assertEqual(rows["docked"]["2cm"]["closure_t"], 19.7)
        self.assertIn("| docked |  | 0.0080 | 0.0000 | no | 19.7 |", markdown)
        self.assertIn("Docked at <= 0.1 m/s, included trials: 2cm: 1, 1cm: 1 of 2", markdown)
        self.assertEqual(result["at_clearance"], {"included": 0, **{key: {"2cm": 0, "1cm": 0} for key in (
            "docked", "docked_at_speed", "docked_any_window", "docked_any_window_at_speed", "scored_docked",
            "scored_docked_at_speed")}})
        self.assertNotIn("At the approach clearance", markdown)
        # no protocol_version and no final stage recorded: scored on the first closure, the full hold not computed
        self.assertEqual((result["scored_docked"], result["docked_any_window"]),
                         ({"2cm": 1, "1cm": 1}, {"2cm": 0, "1cm": 0}))
        self.assertEqual(rows["docked"]["criterion"], "first_closure")
        self.assertFalse(rows["docked"]["2cm"]["any_window"]["computed"])
        self.assertIn("| docked | first_closure | n/a | - | - | - |", markdown)

    def test_cli_scores_the_hover_at_clearance(self):
        with tempfile.TemporaryDirectory() as tmp:
            hover = make_trial(Path(tmp) / "hover", vertical=-0.02)
            level = make_trial(Path(tmp) / "level")
            out = Path(tmp) / "out"
            self.assertEqual(m3.main([str(hover), str(level), "--output-dir", str(out), "--matrix",
                                      "--clearance-m", "0.02", "--clearance-tol-m", "0.005"]), 0)
            result = json.loads((out / "m3-matrix.json").read_text())
            markdown = (out / "m3-matrix.md").read_text()
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                m3.main([str(hover), "--output-dir", str(out), "--clearance-tol-m", "-0.001"])
        self.assertEqual(result["docked"], {"2cm": 1, "1cm": 1})  # the level trial, on the 3D gap
        one, none = {"2cm": 1, "1cm": 1}, {"2cm": 0, "1cm": 0}
        self.assertEqual(result["at_clearance"], {  # the hovering one; no final stage recorded, so no full hold
            "included": 2, "docked": one, "docked_at_speed": one, "docked_any_window": none,
            "docked_any_window_at_speed": none, "scored_docked": one, "scored_docked_at_speed": one})
        rows = {row["label"]: row for row in result["trials"]}
        self.assertFalse(rows["hover"]["2cm"]["docked"])
        self.assertTrue(rows["hover"]["2cm"]["at_clearance"]["docked"])
        self.assertFalse(rows["level"]["2cm"]["at_clearance"]["closed"])  # 2 cm below the clearance target
        self.assertIn("| hover | 0.020 | 0.005 | 19.7 | yes | yes |", markdown)


if __name__ == "__main__":
    unittest.main()
