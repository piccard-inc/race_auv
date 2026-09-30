from __future__ import annotations

import hashlib
import importlib.util
import io
import json
from pathlib import Path
import re
import tempfile
import unittest
from unittest import mock


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


builder = load("build_phase_r_campaign", HERE / "build_phase_r_campaign.py")
extractor = load("extract_tank_floor", HERE / "extract_tank_floor.py")
runtime_mission = load("race_mission", ROOT / "packages/simulation/race-auv-docking/runtime/mission.py")
TRIAL_ID = re.compile(r"^pr-(a0|a1|ho|a2|b)-(n|p2p5|p10|i0|i0p5|v10|v20)-(1step|staged|lateral|depth|contact)"
                      r"(-[a-z]+)?-r\d$")
STAGES = {"a0": None, "a1": None, "holdout": "v10", "a2": None, "b": "v10"}


def dwell(mission: dict) -> float:
    return sum(pose["dwell_s"] for pose in mission["poses"])


class MissionTests(unittest.TestCase):
    def test_mission_files_validate_and_equal_their_definitions(self):
        names = sorted(path.name for path in builder.MISSION_DIR.glob("*.json"))
        self.assertEqual(names, sorted(name for name, *_ in builder.MISSIONS.values()))
        for key in builder.MISSIONS:
            mission, digest = builder.load_mission(key)  # raises if the file differs from its definition
            raw = (builder.MISSION_DIR / builder.MISSIONS[key][0]).read_bytes()
            self.assertEqual(runtime_mission.validate_mission(json.loads(raw)), mission)
            self.assertEqual(digest, hashlib.sha256(raw).hexdigest())  # the hash the job context binds

    def test_one_step_is_the_m1_mission_and_the_others_share_its_start(self):
        m1 = json.loads(builder.EXAMPLE_REQUEST.read_text())["mission"]
        missions = {key: builder.mission_document(key) for key in builder.MISSIONS}
        self.assertEqual(missions["1step"]["poses"], m1["poses"])
        for key in ("staged", "lateral", "depth", "contact"):
            self.assertEqual(missions[key]["poses"][0], m1["poses"][0])  # M1's 40 s dive
            self.assertEqual([p["label"] for p in missions[key]["poses"][1:3]], ["approach_3m", "approach_1p5m"])

    def test_poses_follow_the_m1_derivation(self):
        poses = {key: {p["label"]: p for p in builder.mission_document(key)["poses"]} for key in builder.MISSIONS}
        staged = poses["staged"]
        self.assertEqual((staged["approach_1p5m"]["x_m"], staged["approach_1p5m"]["y_m"]), (-0.325, 5.495))
        self.assertEqual(staged["approach_3m"]["y_m"], 3.995)
        self.assertEqual(poses["contact"]["approach_0p3m"]["y_m"], 6.695)
        self.assertEqual(poses["contact"]["contact"]["y_m"], 6.995)  # stand-off 0, inside the tank's y_C bound
        self.assertEqual((poses["lateral"]["lateral_plus"]["x_m"], poses["lateral"]["lateral_minus"]["x_m"]),
                         (0.175, -0.825))
        self.assertEqual((poses["lateral"]["yaw_plus"]["yaw_rad"], poses["lateral"]["yaw_minus"]["yaw_rad"]),
                         (1.871, 1.271))
        self.assertEqual((poses["depth"]["depth_plus"]["z_m"], poses["depth"]["depth_minus"]["z_m"]), (4.28, 3.28))
        self.assertEqual({key: dwell(builder.mission_document(key)) for key in builder.MISSIONS},
                         {"1step": 300, "staged": 340, "lateral": 880, "depth": 760, "depth2": 760, "contact": 520})


class FloorClearanceTests(unittest.TestCase):
    def test_depth_hold_v2_keeps_the_vehicle_0p3_m_above_the_floor(self):
        v1 = dict(builder.check_floor(builder.mission_document("depth2")))
        self.assertAlmostEqual(v1["depth_plus"], 0.328, places=3)
        clearance = {p["label"]: builder.floor_clearance(p) for p in builder.mission_document("depth")["poses"]}
        self.assertAlmostEqual(clearance["depth_plus"], 0.028, places=3)  # v1: the pose that rested on the floor
        with self.assertRaises(SystemExit):
            builder.check_floor(builder.mission_document("depth"))
        depth2 = {p["label"]: p for p in builder.mission_document("depth2")["poses"]}
        self.assertEqual((depth2["depth_plus"]["z_m"], depth2["depth_minus"]["z_m"], depth2["hold_1p5m"]["dwell_s"]),
                         (3.98, 3.28, 300))

    def test_every_mission_for_new_stages_clears_the_floor(self):
        for key in builder.MISSIONS:
            if key not in builder.RECORDED_ONLY:
                self.assertGreaterEqual(min(v for _, v in builder.check_floor(builder.mission_document(key))), 0.3, key)

    def test_a_set_point_within_0p3_m_of_the_floor_is_rejected(self):
        mission = builder.mission_document("depth2")
        mission["poses"][3] = builder.pose("depth_plus", 90, 1.5, depth_m=0.25)  # z_C 4.03: clearance 0.278 m
        with self.assertRaises(SystemExit) as raised:
            builder.check_floor(mission)
        self.assertIn("depth_plus 0.278 m", str(raised.exception))
        with self.assertRaises(SystemExit):  # the footprint leaves the tank interior
            builder.floor_clearance(builder.pose("outside", 10, 1.5, lateral_m=3.3))  # y_W -2.975, wall at -3.05

    def test_the_recorded_v1_mission_is_only_for_its_own_stage(self):
        self.assertEqual(len(builder.build("holdout", "p10")["jobs"]), 4)
        with mock.patch.dict(builder.STAGES, {"a2": lambda: [builder.job("a2", "n", "depth", 1, "probe")]}):
            with self.assertRaises(SystemExit):
                builder.build("a2")

    def test_the_floor_matches_where_the_m2_holdout_vehicle_rested(self):
        """Both M2 holdout depth runs rested on the floor with the Base at z_W 4.309 (x_W about 1.8)."""
        geometry = json.loads(builder.TANK_FLOOR.read_text())
        resting = builder.floor_z(geometry["interior_floor_triangles_world_m"], 1.8, 0.325) \
            - geometry["vehicle"]["lowest_point_below_base_m"]
        self.assertAlmostEqual(resting, 4.309, delta=0.003)

    def test_holdout2_reruns_the_depth_hold_holdout_on_v2(self):
        document = builder.build("holdout2", "p10")
        self.assertEqual([j["trial_id"] for j in document["jobs"]], ["pr-ho2-n-depth2-r1", "pr-ho2-p10-depth2-r1"])
        self.assertEqual((document["horizon_s"], document["wall_timeout_s"]), (790, 940))
        self.assertTrue(all(j["context"]["follow_up"] == "piccard-physical-ai#102" for j in document["jobs"]))
        with self.assertRaises(SystemExit):
            builder.build("holdout2", "n")

    def test_extractor_on_a_synthetic_world(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name, text in {
                "world/race_auv_test.scn": '<scenario><static name="tank" type="model"><physical>'
                                           '<mesh filename="objects/tank.obj"/><origin xyz="0 0 0" rpy="0 0 0"/>'
                                           '</physical><world_transform xyz="0 0 5" rpy="0 0 0"/></static></scenario>',
                # inner floor at z -0.5 (world 4.5) over x +-2, y +-1; outer floor at z 0 (world 5) over x +-3, y +-2
                "data/objects/tank.obj": "v -2 -1 -0.5\nv 2 -1 -0.5\nv 2 1 -0.5\nv -2 1 -0.5\n"
                                         "v -3 -2 0\nv 3 -2 0\nv 3 2 0\nv -3 2 0\nf 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\n",
                "vehicles/race_auv.scn": '<scenario><robot name="r"><base_link name="Vehicle" type="compound">'
                                         '<external_part name="Hull"><physical><mesh filename="parts/hull.obj"/>'
                                         '<origin rpy="3.14159265 0 0" xyz="0 0 0"/></physical>'
                                         '<compound_transform rpy="0 0 0" xyz="0 0 0"/></external_part></base_link>'
                                         '<world_transform xyz="-1 0 0" rpy="0 0 0"/></robot></scenario>',
                # hull x [-1, 0], y +-0.2, z [-0.15, 0.05]: turned about x, its lowest point is 0.15 below the origin
                "data/parts/hull.obj": "v -1 -0.2 -0.15\nv 0 0.2 0.05\nv 0 -0.2 -0.15\nf 1 2 3\n",
            }.items():
                (root / name).parent.mkdir(parents=True, exist_ok=True)
                (root / name).write_text(text)
            document = extractor.extract(root, "c0ffee", "d" * 64)
        self.assertEqual(len(document["interior_floor_triangles_world_m"]), 2)
        self.assertEqual({p[2] for t in document["interior_floor_triangles_world_m"] for p in t}, {4.5})
        self.assertEqual(document["interior_xy_bounds_world_m"], {"x": [-2.0, 2.0], "y": [-1.0, 1.0]})
        self.assertEqual(document["vehicle"]["lowest_point_below_base_m"], 0.15)
        self.assertEqual(document["vehicle"]["footprint_base_m"], {"x": [-1.0, 0.0], "y": [-0.2, 0.2]})
        self.assertEqual(document["source"]["world_of_stonefish"]["commit"], "c0ffee")


class BuilderTests(unittest.TestCase):
    def build(self, stage: str) -> dict:
        return builder.build(stage, STAGES[stage])

    def test_same_inputs_give_the_same_bytes(self):
        for stage, selected in STAGES.items():
            outputs = []
            for _ in range(2):
                with tempfile.TemporaryDirectory() as tmp:
                    out = Path(tmp) / "jobs.json"
                    argv = ["--stage", stage, "--out", str(out)] + (["--selected", selected] if selected else [])
                    with mock.patch("sys.stderr", new_callable=io.StringIO):
                        self.assertEqual(builder.main(argv), 0)
                    outputs.append(out.read_bytes())
            self.assertEqual(outputs[0], outputs[1], stage)

    def test_stage_sizes_trial_ids_and_horizons(self):
        expected = {"a0": (8, 330), "a1": (9, 370), "holdout": (4, 910), "a2": (2, 370), "b": (2, 550)}
        seen = set()
        for stage, (count, horizon) in expected.items():
            document = self.build(stage)
            self.assertEqual(document["recipe"], "race-auv-docking/v1")
            self.assertEqual(len(document["jobs"]), count, stage)
            self.assertEqual((document["horizon_s"], document["wall_timeout_s"]), (horizon, horizon + 150), stage)
            for item in document["jobs"]:
                self.assertRegex(item["trial_id"], TRIAL_ID)
                self.assertNotIn(item["trial_id"], seen)
                seen.add(item["trial_id"])
                self.assertGreaterEqual(document["horizon_s"], dwell(item["mission"]) + 30)
        self.assertEqual(len(seen), 25)  # the preregistered 25 trials

    def test_contexts_bind_the_trial(self):
        for stage in STAGES:
            for item in self.build(stage)["jobs"]:
                context = item["context"]
                self.assertEqual(context["trial_id"], item["trial_id"])
                self.assertEqual(context["campaign_id"], "phase-r-a-20260928")
                self.assertEqual(context["stage"], stage)
                self.assertIn("v0.2", context["protocol"])
                self.assertEqual(context["mission_sha256"], hashlib.sha256(builder.mission_bytes(item["mission"])).hexdigest())
                self.assertEqual(context["docking_xy_overrides"], builder.KNOBS[context["gain_set"]])
                for key in ("mission_key", "role", "repeat", "pose_derivation"):
                    self.assertIn(key, context)

    def test_only_docking_x_and_y_vary(self):
        nominal = json.loads(builder.EXAMPLE_REQUEST.read_text())["gains"]
        for stage in STAGES:
            for item in self.build(stage)["jobs"]:
                gains = json.loads(json.dumps(item["gains"]))
                for axis in ("x", "y"):
                    self.assertEqual(gains["docking"][axis], {**nominal["docking"][axis],
                                                              **builder.KNOBS[item["context"]["gain_set"]]})
                    gains["docking"][axis] = nominal["docking"][axis]
                self.assertEqual(gains, nominal, item["trial_id"])

    def test_media_only_on_the_r_a0_pairs(self):
        a0 = self.build("a0")["jobs"]
        media = [item.get("media") for item in a0]
        self.assertEqual(media, [None, {"display_video": True}] * 3 + [None, {"onboard_camera_video": True}])
        self.assertEqual({item["context"]["gain_set"] for item in a0}, {"n"})
        self.assertEqual({item["context"]["mission_key"] for item in a0}, {"1step"})
        for stage in ("a1", "holdout", "a2", "b"):
            self.assertFalse(any("media" in item or "media" in item["context"] for item in self.build(stage)["jobs"]))

    def test_stage_contents(self):
        a1 = [(i["context"]["gain_set"], i["context"]["mission_key"]) for i in self.build("a1")["jobs"]]
        self.assertEqual(a1, [("n", "staged"), *[(k, "staged") for k in builder.PROBES], ("n", "staged"), ("n", "1step")])
        holdout = [(i["context"]["gain_set"], i["context"]["mission_key"]) for i in self.build("holdout")["jobs"]]
        self.assertEqual(sorted(holdout), [("n", "depth"), ("n", "lateral"), ("v10", "depth"), ("v10", "lateral")])
        a2 = self.build("a2")["jobs"]
        self.assertEqual([i["mission"]["apriltag_tag_size"] for i in a2], ["lab_configured", "black_square_edge"])
        self.assertTrue(a2[1]["mission"]["mission_id"].endswith("-black-square-edge"))
        self.assertEqual({i["context"]["mission_key"] for i in self.build("b")["jobs"]}, {"contact"})
        self.assertEqual([i["context"]["gain_set"] for i in builder.build("b", "n")["jobs"]], ["n", "n"])

    def test_holdout_needs_a_selected_set_other_than_n(self):
        with self.assertRaises(SystemExit):
            builder.build("holdout", "n")
        with self.assertRaises(SystemExit):
            builder.build("holdout", None)

    def test_rerun_emits_the_one_job_with_its_stage_horizon(self):
        document = builder.rerun("a1", None, "pr-a1-p10-staged-r1")
        self.assertEqual((document["horizon_s"], document["wall_timeout_s"]), (370, 520))
        [item] = document["jobs"]
        self.assertEqual((item["trial_id"], item["context"]["trial_id"], item["context"]["rerun_of"]),
                         ("pr-a1-p10-staged-r1-rerun1", "pr-a1-p10-staged-r1-rerun1", "pr-a1-p10-staged-r1"))
        original = next(j for j in builder.build("a1")["jobs"] if j["trial_id"] == "pr-a1-p10-staged-r1")
        self.assertEqual((item["gains"], item["mission"]), (original["gains"], original["mission"]))
        with self.assertRaises(SystemExit):
            builder.rerun("a1", None, "pr-a1-p10-staged-r9")

    def test_a0b_six_interleaved_display_pairs_at_the_selected_gains(self):
        document = builder.build("a0b", "p10")
        jobs = document["jobs"]
        self.assertEqual((len(jobs), document["horizon_s"], document["wall_timeout_s"]), (12, 370, 520))
        roles = [j["context"]["role"] for j in jobs]
        plain, video = "isolation_display_without_video", "isolation_display_with_video"
        self.assertEqual(roles, [plain, video, video, plain] * 3)  # no-video, video, video, no-video, ...
        self.assertEqual([j["context"]["repeat"] for j in jobs], [r for r in range(1, 7) for _ in range(2)])
        self.assertEqual([j.get("media") for j in jobs], [{"display_video": True} if r == video else None for r in roles])
        self.assertEqual([j["trial_id"] for j in jobs[:4]],
                         ["pr-a0b-p10-staged-novideo-r1", "pr-a0b-p10-staged-video-r1", "pr-a0b-p10-staged-video-r2",
                          "pr-a0b-p10-staged-novideo-r2"])
        self.assertEqual(len({j["trial_id"] for j in jobs}), 12)
        for item in jobs:
            self.assertEqual(item["context"]["mission_key"], "staged")
            self.assertEqual(item["context"]["follow_up"], "piccard-physical-ai#102")
            self.assertEqual(item["gains"], builder.gains_for("p10"))
            self.assertEqual(item["gains"]["docking"]["x"]["p"], 10.0)
        with self.assertRaises(SystemExit):
            builder.build("a0b", None)

    def test_a0c_one_onboard_pair_at_the_selected_gains(self):
        document = builder.build("a0c", "p10")
        jobs = document["jobs"]
        self.assertEqual((len(jobs), document["horizon_s"]), (2, 370))
        self.assertEqual([(j["trial_id"], j["context"]["role"], j.get("media")) for j in jobs],
                         [("pr-a0c-p10-staged-novideo-r1", "isolation_onboard_without_video", None),
                          ("pr-a0c-p10-staged-onboard-r1", "isolation_onboard_with_video",
                           {"display_video": True, "onboard_camera_video": True})])
        for item in jobs:
            self.assertEqual((item["context"]["mission_key"], item["context"]["follow_up"]),
                             ("staged", "piccard-physical-ai#102"))
            self.assertEqual(item["gains"], builder.gains_for("p10"))
        self.assertEqual(jobs[0]["mission"], jobs[1]["mission"])
        with self.assertRaises(SystemExit):
            builder.build("a0c", None)

    def test_cli_writes_to_stdout_and_requires_a_stage(self):
        with mock.patch("sys.stdout", new_callable=io.StringIO) as stdout, \
                mock.patch("sys.stderr", new_callable=io.StringIO) as stderr:
            self.assertEqual(builder.main(["--stage", "a2"]), 0)
        self.assertEqual(len(json.loads(stdout.getvalue())["jobs"]), 2)
        self.assertIn("floor clearance phase-r-a-m-approach-staged-v1: minimum 0.528 m at approach_1p5m",
                      stderr.getvalue())
        with mock.patch("sys.stderr", new_callable=io.StringIO), self.assertRaises(SystemExit):
            builder.main([])


if __name__ == "__main__":
    unittest.main()
