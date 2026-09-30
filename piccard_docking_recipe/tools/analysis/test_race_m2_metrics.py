from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("race_m2_metrics", ROOT / "race_m2_metrics.py")
metrics = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(metrics)
builder_spec = importlib.util.spec_from_file_location("build_phase_r_campaign",
                                                      ROOT.parent / "campaigns/build_phase_r_campaign.py")
builder = importlib.util.module_from_spec(builder_spec)
assert builder_spec.loader is not None
builder_spec.loader.exec_module(builder)


def pose(label: str, y: float, x: float = -0.325, z: float = 3.78, yaw: float = 1.571, dwell: float = 60) -> dict:
    return {"label": label, "x_m": x, "y_m": y, "z_m": z, "roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": yaw,
            "dwell_s": dwell}


def report(gain_set: str, settling_s: float | None = 50.0, *, label: str | None = None, repeat: int = 1,
           minimum_m: float = 1.2, contact: bool = False, saturation: float = 0.0, distance_error: float = 0.2,
           lateral: float = 0.1, stage: str = "a1", mission_key: str = "staged", excluded: bool = False,
           extra_steps: tuple = (), contact_class: str = "force_bearing") -> dict:
    """A minimal race_m2_metrics trial report for the selection rule."""
    steps = [{"index": 0, "poses": ["dive"], "stepped_axes": [], "gated": False, "judged": False, "settled": None,
              "settling_time_s": None},
             {"index": 1, "poses": ["approach_3m"], "stepped_axes": ["y"], "gated": True, "judged": True,
              "settled": True, "settling_time_s": 40.0},
             {"index": 2, "poses": ["approach_1p5m", "hold_1p5m"], "stepped_axes": ["y"], "gated": True, "judged": True,
              "settled": settling_s is not None, "settling_time_s": settling_s}, *extra_steps]
    poses = [{"label": name, "minimum_distance_m": minimum_m if name != "dive" else 7.0,
              "v0_1_settling": {"settled": False, "settling_time_s": None},
              "dwell_end": {"distance_error_m": distance_error, "lateral_error_m": lateral}}
             for name in ("dive", "approach_3m", "approach_1p5m", "hold_1p5m")]
    return {"label": label or f"pr-a1-{gain_set}-staged-r{repeat}",
            "context": {"gain_set": gain_set, "stage": stage, "mission_key": mission_key, "repeat": repeat},
            "excluded": excluded, "exclusion_reasons": ["route incomplete (x, y)"] if excluded else [],
            "poses": poses, "steps": steps, "minimum_distance_m": minimum_m, "route_completed": not excluded,
            "contacts": {"auv_station": {"events": 3, contact_class: {"events": 3}}} if contact else {},
            "thruster_saturation": {"surge_port": {"samples": 10, "at_limit_fraction": saturation}}}


def submitted(mission: dict) -> dict:
    """The mission as the submit path delivers it: integral floats re-rendered as integers (0.0 as 0)."""
    return json.loads(json.dumps(mission), parse_float=lambda text: int(float(text)) if float(text).is_integer()
                      else float(text))


class MissionBindingTests(unittest.TestCase):
    def test_the_builder_context_hash_binds_the_submitted_mission(self):
        jobs = [j for stage in ("a1", "a2") for j in builder.build(stage)["jobs"]] + builder.build("b", "v10")["jobs"]
        for job in jobs:
            mission = submitted(job["mission"])
            runtime = hashlib.sha256((json.dumps(mission, indent=2, sort_keys=True) + "\n").encode()).hexdigest()
            trial = {"mission": mission, "mission_sha256": runtime}
            self.assertNotEqual(runtime, job["context"]["mission_sha256"])  # roll and pitch 0.0 arrive as 0
            self.assertEqual(metrics.mission_binding(trial, job["context"]), (True, "builder"), job["trial_id"])
            self.assertEqual(metrics.mission_binding(trial, {"mission_sha256": runtime}), (True, "runtime"))

    def test_a_different_mission_or_no_context_hash(self):
        job = builder.build("a1")["jobs"][0]
        mission = submitted(job["mission"])
        mission["poses"][-1]["dwell_s"] = 61
        trial = {"mission": mission, "mission_sha256": "0" * 64}
        self.assertEqual(metrics.mission_binding(trial, job["context"]), (False, None))
        self.assertEqual(metrics.mission_binding(trial, {}), (None, None))


class SettlingTests(unittest.TestCase):
    def test_settles_to_its_final_value_despite_a_steady_bias(self):
        series = [(t / 10, 0.15 + 0.85 * math.exp(-(t / 10) / 20.0)) for t in range(1201)]
        result = metrics.steady_settling(series, 0.0, 120.0, 0.05)
        final = sum(e for t, e in series if t >= 110.0) / sum(1 for t, e in series if t >= 110.0)
        self.assertTrue(result["settled"])
        self.assertAlmostEqual(result["steady_state_error"], final, places=4)
        # the band is around the final value: 0.85 exp(-t / 20) <= 0.05 + (final - 0.15)
        self.assertAlmostEqual(result["settling_time_s"], 20.0 * math.log(0.85 / (0.05 + final - 0.15)), delta=0.1)

    def test_oscillation_or_no_tail_does_not_settle(self):
        wobble = [(t / 10, 0.2 * math.sin(2 * math.pi * (t / 10) / 20.0)) for t in range(1201)]
        self.assertFalse(metrics.steady_settling(wobble, 0.0, 120.0, 0.05)["settled"])
        self.assertIn("no controller samples", metrics.steady_settling([(1.0, 0.0)], 0.0, 120.0, 0.05)["reason"])

    def test_steps_group_same_command_poses_and_judge_stepped_axes(self):
        poses = [pose("dive", 0.0, dwell=40), pose("approach_3m", 3.995, x=-0.326, dwell=120),
                 pose("approach_1p5m", 5.495, dwell=120), pose("hold_1p5m", 5.495), pose("yaw_plus", 5.495, yaw=1.871),
                 pose("depth_plus", 5.495, yaw=1.871, z=4.28)]
        docked, t = {}, 0.0
        for item in poses:
            docked[item["label"]] = {"start_t": t, "end_t": t + item["dwell_s"]}
            t += item["dwell_s"]
        rows, previous = [], {"x_m": -0.325, "y_m": 0.0, "z_m": 3.78, "yaw_rad": 1.571}
        for item in poses:  # first-order approach to each command, 15 s time constant, 0.1 m short on y
            window = docked[item["label"]]
            for k in range(int(item["dwell_s"] * 10)):
                s = window["start_t"] + k / 10
                a = 1 - math.exp(-(s - window["start_t"]) / 15.0)
                row = {"kind": "value", "t": s}
                for axis, field in (("x", "x_m"), ("y", "y_m"), ("z", "z_m"), ("yaw", "yaw_rad")):
                    target = item[field] - (0.1 if axis == "y" else 0.0)
                    row[axis] = previous[field] + (target - previous[field]) * a
                rows.append(row)
            previous = {field: rows[-1][axis] for axis, field in (("x", "x_m"), ("y", "y_m"), ("z", "z_m"), ("yaw", "yaw_rad"))}
        steps = metrics.step_settling(rows, poses, docked)
        self.assertEqual([s["poses"] for s in steps], [["dive"], ["approach_3m"], ["approach_1p5m", "hold_1p5m"],
                                                        ["yaw_plus"], ["depth_plus"]])
        self.assertEqual([s["stepped_axes"] for s in steps], [[], ["y"], ["y"], ["yaw"], ["z"]])  # 1 mm of x is no step
        self.assertEqual([s["gated"] for s in steps], [False, True, True, False, False])
        self.assertFalse(steps[0]["judged"])
        self.assertTrue(all(s["settled"] for s in steps[1:]))
        self.assertAlmostEqual(steps[2]["axes"]["y"]["steady_state_error"], 0.1, places=2)
        self.assertEqual(steps[2]["start_t"], docked["approach_1p5m"]["start_t"])
        self.assertEqual(steps[2]["end_t"], docked["hold_1p5m"]["end_t"])


class SelectionTests(unittest.TestCase):
    def test_passes_rank_by_settling_then_distance_then_lateral(self):
        reports = [report("n", 60.0), report("p10", 40.0), report("v10", 40.0, distance_error=0.1),
                   report("i0", 40.0, distance_error=0.1, lateral=0.05), report("n", 70.0, repeat=2)]
        result = metrics.selection(reports, "staged")
        ranks = {r["label"]: r.get("rank") for r in result["trials"]}
        self.assertEqual(ranks, {"pr-a1-i0-staged-r1": 1, "pr-a1-v10-staged-r1": 2, "pr-a1-p10-staged-r1": 3,
                                 "pr-a1-n-staged-r1": 4, "pr-a1-n-staged-r2": 5})
        self.assertEqual(result["best"], {"label": "pr-a1-i0-staged-r1", "gain_set": "i0"})
        self.assertEqual(result["nominal_repeat_spread"]["settling_time_s"], 10.0)

    def test_contact_distance_saturation_settling_and_route_gate_the_ranking(self):
        reports = [report("p10", 10.0, contact=True), report("p2p5", 10.0, minimum_m=0.85),
                   report("v20", 10.0, saturation=0.2), report("i0", None), report("i0p5", 10.0, excluded=True),
                   report("n", 60.0)]
        result = metrics.selection(reports, "staged")
        rows = {r["gain_set"]: r for r in result["trials"]}
        self.assertFalse(rows["p10"]["pass"])
        self.assertEqual(rows["p10"]["station_contact_class"], "force_bearing")
        self.assertIsNone(rows["n"]["station_contact_class"])
        self.assertFalse(rows["p2p5"]["pass"])
        self.assertTrue(rows["v20"]["pass"])
        self.assertFalse(rows["v20"]["gates"]["saturation_ok"])
        self.assertEqual(rows["i0"]["gates"]["non_settling_poses"], ["approach_1p5m"])
        self.assertNotIn("i0p5", rows)
        self.assertEqual(result["excluded_rerun_once"], [{"label": "pr-a1-i0p5-staged-r1",
                                                          "reasons": ["route incomplete (x, y)"]}])
        self.assertEqual([r["gain_set"] for r in result["trials"] if r["eligible"]], ["n"])
        self.assertEqual(result["best"]["gain_set"], "n")

    def test_a_near_miss_also_fails_the_pass(self):
        [row] = metrics.selection([report("n", contact=True, contact_class="near_miss")], "staged")["trials"]
        self.assertEqual((row["pass"], row["station_contact"], row["station_contact_class"]), (False, True, "near_miss"))

    def test_only_steps_of_docking_x_or_y_are_gated(self):
        yaw = {"index": 3, "poses": ["yaw_plus"], "stepped_axes": ["yaw"], "gated": False, "judged": True,
               "settled": False, "settling_time_s": None}
        lateral = {**yaw, "poses": ["lateral_plus"], "stepped_axes": ["x"], "gated": True}
        rows = metrics.selection([report("n", extra_steps=(yaw,)), report("v10", extra_steps=(lateral,))], "staged")
        gates = {r["gain_set"]: r["gates"]["non_settling_poses"] for r in rows["trials"]}
        self.assertEqual(gates, {"n": [], "v10": ["lateral_plus"]})

    def test_other_stages_and_missions_are_left_out(self):
        reports = [report("n"), report("n", stage="holdout", label="pr-ho-n-lateral-r1"),
                   report("n", mission_key="1step", label="pr-a1-n-1step-r1")]
        self.assertEqual([r["label"] for r in metrics.selection(reports, "staged")["trials"]], ["pr-a1-n-staged-r1"])
        self.assertEqual([r["label"] for r in metrics.selection(reports, "1step")["trials"]], ["pr-a1-n-1step-r1"])

    def test_v0_1_definition_gates_every_pose_on_the_ground_truth_band(self):
        result = metrics.selection([report("n")], "staged", "v0.1")
        [row] = result["trials"]
        self.assertEqual(row["gates"]["non_settling_poses"], ["dive", "approach_3m", "approach_1p5m", "hold_1p5m"])
        self.assertIsNone(row["rank_keys"]["settling_time_s"])
        self.assertIsNone(result["best"])
        self.assertIn("no non-settling pose", result["rule"]["gates"])


STATION = {"kind": "gt_station", "t": 0.0, "x": 4.0, "y": 0.0, "z": 3.95, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


def auv(t: float, speed: float = 0.05) -> dict:
    return {"kind": "gt_auv", "t": t, "x": 3.2, "y": 0.325, "z": 3.95, "qx": 1.0, "qy": 0.0, "qz": 0.0, "qw": 0.0,
            "linear_mps": [speed, 0.0, 0.0]}


def event(t: float, force: float, pair: str = "auv_station") -> dict:
    return {"kind": "contact", "t": t, "contact": pair, "normal_force_n": force}


class ContactTests(unittest.TestCase):
    def test_force_bearing_contact_and_its_persistence(self):
        near = [event(25.0 + k / 2, 0.0) for k in range(10)]  # 25-29.5 s inside the margin, no load
        loaded = [event(30.0 + k / 2, 1.0 + k) for k in range(80) if not 60 <= k < 64]  # 30-70 s, a 2 s break
        final = {"label": "contact", "start_t": 20.0, "end_t": 70.0}
        result = metrics.contact_metrics([STATION, auv(29.9), *near, *loaded], [final])
        self.assertTrue(result["contact"])
        self.assertEqual((result["events"], result["max_normal_force_n"]), (86, 80.0))
        self.assertIn("lower bound", result["max_normal_force_note"])
        self.assertEqual((result["near_miss"]["events"], result["near_miss"]["first_t"], result["near_miss"]["last_t"]),
                         (10, 25.0, 29.5))
        first = result["force_bearing"]
        self.assertEqual((first["first_contact_t"], first["first_contact_after_final_pose_start_s"]), (30.0, 10.0))
        self.assertAlmostEqual(first["speed_at_first_contact_mps"], 0.05)
        self.assertEqual((first["events"], first["max_normal_force_n"]), (76, 80.0))
        persistence = first["persistence"]
        self.assertEqual((persistence["from_t"], persistence["bins"]), (30.0, 40))
        self.assertAlmostEqual(persistence["bins_with_contact_fraction"], 38 / 40)
        self.assertTrue(persistence["persists"])
        self.assertEqual(metrics.contact_metrics([STATION, auv(29.9)], [final]), {"contact": False})

    def test_near_misses_only_have_no_force_bearing_contact(self):
        rows = [STATION, auv(29.9), {"kind": "dock", "t": 29.95, "distance_m": 0.74},
                *[event(30.0 + k / 10, 0.0) for k in range(25)], {"kind": "dock", "t": 32.0, "distance_m": 0.72},
                *[event(32.05 + k / 10, 0.0) for k in range(5)]]
        result = metrics.contact_metrics(rows, [{"label": "approach_1p5m", "start_t": 20.0, "end_t": 70.0}])
        self.assertTrue(result["contact"])
        self.assertIsNone(result["force_bearing"])
        self.assertEqual((result["near_miss"]["events"], result["near_miss"]["seconds_with_events"],
                          result["near_miss"]["min_dock_point_distance_m"]), (30, 3, 0.72))
        self.assertEqual(result["max_normal_force_n"], 0.0)

    def test_contacts_are_split_into_classes_per_pair(self):
        rows = [event(10.0, 0.0), event(10.5, 3.0), event(11.0, 0.0), event(20.0, 25.0, "auv_tank")]
        split = metrics.contacts(rows)
        station = split["auv_station"]
        self.assertEqual((station["events"], station["force_bearing"]["events"], station["near_miss"]["events"]), (3, 1, 2))
        self.assertEqual(station["max_normal_force_n"], 3.0)
        self.assertEqual((split["auv_tank"]["force_bearing"]["max_normal_force_n"], split["auv_tank"]["near_miss"]),
                         (25.0, None))

    def test_force_bearing_contact_after_the_final_dwell_has_no_persistence(self):
        result = metrics.contact_metrics([event(80.0, 2.0)], [{"label": "contact", "start_t": 20.0, "end_t": 70.0}])
        self.assertIsNone(result["force_bearing"]["persistence"])
        self.assertIsNone(result["force_bearing"]["speed_at_first_contact_mps"])


if __name__ == "__main__":
    unittest.main()
