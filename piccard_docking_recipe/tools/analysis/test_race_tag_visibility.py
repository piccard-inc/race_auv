from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("race_tag_visibility", ROOT / "race_tag_visibility.py")
vis = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(vis)
mission_module = vis.mission_module
REQUEST = ROOT.parents[1] / "packages/simulation/race-auv-docking/examples/drift-and-revisit-request.json"
FRONT = set(vis.FRONT_TAGS)
TANK_MARGIN_M = 1.0


def poses_by_label(mission: dict) -> dict:
    return {pose["label"]: pose for pose in mission["poses"]}


class GeometryTests(unittest.TestCase):
    def test_the_vertical_field_of_view_follows_the_image_shape(self):
        self.assertAlmostEqual(vis.VFOV_DEG, 66.2, places=1)

    def test_world_from_controller_inverts_the_missions_nominal_transform(self):
        for world in ((-3.8, 0.0, 3.78, 0.0), (1.0, 0.325, 3.78, 0.0), (-2.5, 2.0, 3.0, math.pi - 1e-6),
                      (0.3, -1.2, 2.0, -1.0)):
            controller = mission_module.controller_from_ground_truth(*world)
            back = vis.world_from_controller(*controller)
            for a, b in zip(back[:3], world[:3]):
                self.assertAlmostEqual(a, b, places=9)
            self.assertAlmostEqual(math.cos(back[3] - world[3]), 1.0, places=9)

    def test_the_station_tags_sit_where_the_ground_truth_puts_them(self):
        tags = vis.tag_geometry()
        # dock point (3.59, 0.325, 3.91); the dock frame is the station's turned about x, so its +z is up
        for tag, expected in (("tag36h11:146", (4.02, 0.325, 3.505)), ("tag36h11:541", (4.06, 0.325, 3.75)),
                              ("tag36h11:558", (4.06, 0.325, 3.815))):
            for a, b in zip(tags[tag][0], expected):
                self.assertAlmostEqual(a, b, places=6)
        # the forward tags' printed faces point back along -x, towards an approaching vehicle
        self.assertAlmostEqual(tags["tag36h11:146"][1][0], -1.0, places=5)

    def test_facing_the_station_at_the_stand_off_the_forward_tags_are_detectable(self):
        seen = vis.view(1.0, 0.325, 3.78, 0.0)
        detectable = {tag for tag, row in seen["cam_front"].items() if row["detectable"]}
        self.assertEqual(detectable, FRONT)
        self.assertFalse(any(row["detectable"] for row in seen["cam_down"].values()))

    def test_beyond_its_band_tag_541_is_not_detectable_while_146_still_is(self):
        seen = vis.view(0.3, 0.325, 3.78, 0.0)  # forward camera about 3.06 m from tag 541
        self.assertGreater(seen["cam_front"]["tag36h11:541"]["range_m"], 2.75)
        self.assertFalse(seen["cam_front"]["tag36h11:541"]["detectable"])
        self.assertTrue(seen["cam_front"]["tag36h11:146"]["detectable"])

    def test_turned_away_no_tag_is_in_either_cameras_widened_view(self):
        result = vis.pose_visibility({"label": "away", "x_m": -0.325, "y_m": 4.8, "z_m": 3.78, "roll_rad": 0.0,
                                      "pitch_rad": 0.0, "yaw_rad": -1.571, "dwell_s": 1})
        self.assertTrue(vis.out_of_view(result))

    def test_a_tag_just_outside_the_field_of_view_is_still_in_the_widened_view(self):
        # turn the vehicle until tag 146 sits between the forward camera's half-angle and the widened one (the
        # camera is 0.7 m ahead of cg_link, so the tag's azimuth grows faster than the yaw)
        half = vis.HFOV_DEG / 2
        found = None
        for tenth in range(0, 900):
            seen = vis.view(1.0, 0.325, 3.78, math.radians(tenth / 10))["cam_front"]["tag36h11:146"]
            if half + 1.0 < abs(seen["azimuth_deg"]) < half + vis.FOV_MARGIN_DEG - 1.0:
                found = seen
                break
        self.assertIsNotNone(found)
        self.assertFalse(found["detectable"])
        self.assertTrue(found["in_fov"])
        far = vis.view(1.0, 0.325, 3.78, math.radians(60.0))["cam_front"]["tag36h11:146"]
        self.assertGreater(abs(far["azimuth_deg"]), half + vis.FOV_MARGIN_DEG)
        self.assertFalse(far["in_fov"])


class DriftAndRevisitRequestTests(unittest.TestCase):
    def setUp(self):
        self.request = json.loads(REQUEST.read_text())
        self.mission = mission_module.validate_mission(self.request["mission"])
        self.results = {r["label"]: r for r in vis.check_mission(self.mission)}

    def test_it_is_a_pose_mission_with_black_square_edge_tags_and_a_drawn_seed(self):
        self.assertEqual(self.request["mission"]["schema"], mission_module.SCHEMA)
        self.assertEqual(self.request["mission"]["apriltag_tag_size"], "black_square_edge")
        self.assertIsNone(self.request["mission"]["seed"])
        self.assertLessEqual(len(self.mission["poses"]), mission_module.MAX_POSES)

    def test_every_leg_pose_is_out_of_view_on_both_cameras(self):
        legs = [label for label in self.results if label.startswith("leg_")]
        self.assertEqual(len(legs), 6)
        for label in legs:
            with self.subTest(pose=label):
                self.assertTrue(vis.out_of_view(self.results[label]))

    def test_every_stand_off_pose_has_the_three_forward_tags_detectable(self):
        for label in ("approach", "view_1", "return_1", "return_2", "hold"):
            with self.subTest(pose=label):
                self.assertEqual(set(self.results[label]["detectable"]["cam_front"]), FRONT)
                for tag, rng in self.results[label]["front_ranges_m"].items():
                    low, high = vis.DETECTION_RANGE_M[("cam_front", tag)]
                    self.assertLessEqual(rng, high - 0.35)  # room for drift and the controller's error
                    self.assertGreaterEqual(rng, low + 0.35)

    def test_two_legs_of_about_five_minutes_each_alternate_with_returns_in_view(self):
        poses = self.mission["poses"]
        runs, current = [], 0.0
        for pose in poses:
            if vis.out_of_view(self.results[pose["label"]]):
                current += pose["dwell_s"]
            elif current:
                runs.append(current)
                current = 0.0
        self.assertEqual(current, 0.0)  # the mission ends in view
        self.assertEqual(len(runs), 2)
        for seconds in runs:
            self.assertGreaterEqual(seconds, 270.0)
            self.assertLessEqual(seconds, 420.0)
        labels = [pose["label"] for pose in poses]
        self.assertEqual(labels[-1], "hold")
        for leg, back in (("leg_1_b", "return_1"), ("leg_2_b", "return_2")):
            self.assertEqual(labels.index(back), labels.index(leg) + 1)

    def test_every_pose_stays_well_inside_the_tank(self):
        # the dive is M1's (examples/m1-smoke-request.json), straight down from the start, 0.77 m inside y_C
        dive = poses_by_label(json.loads((REQUEST.parent / "m1-smoke-request.json").read_text())["mission"])["dive"]
        self.assertEqual(self.mission["poses"][0], dive)
        for pose in self.mission["poses"][1:]:
            with self.subTest(pose=pose["label"]):
                for key in ("x_m", "y_m"):
                    low, high = mission_module.TANK_BOUNDS_M[key]
                    self.assertGreaterEqual(pose[key] - low, TANK_MARGIN_M)
                    self.assertGreaterEqual(high - pose[key], TANK_MARGIN_M)
                self.assertLessEqual(pose["z_m"], mission_module.TANK_BOUNDS_M["z_m"][1] - 0.3)

    def test_the_dwells_fit_the_horizon_with_room_to_spare(self):
        self.assertEqual(self.request["horizon_s"], 1800)
        self.assertLessEqual(mission_module.mission_seconds(self.mission), self.request["horizon_s"] - 300)

    def test_the_command_line_prints_one_line_per_pose(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(vis.main([str(REQUEST)]), 0)
        lines = [json.loads(line) for line in out.getvalue().splitlines()]
        self.assertEqual([line["label"] for line in lines], [pose["label"] for pose in self.mission["poses"]])
        self.assertEqual(sum(line["out_of_view"] for line in lines), 6)


if __name__ == "__main__":
    unittest.main()
