"""The tag pivot (M3, #109 follow-up): derived from the station URDF and apriltag.yaml the fuser loads, and the
collector's fail-closed check of the mission's tag_pivot_m. No ROS."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import yaml

import collect_trial
import tag_pivot

PACKAGE = Path(__file__).resolve().parent.parent
EXAMPLE = json.loads((PACKAGE / "examples/m3-planner-request.json").read_text())
# The pinned station layout (race_station_description urdf/base.urdf at the source lock's commit; the same values the
# M3-0 trace's ground-truth tf_static records): the dock point under base_link, every tag under the dock point.
TAGS = {"apriltag25h9_02": ("-0.81 0.0 -0.21", "-1.5707963 0.0 -1.5707963"),
        "apriltag25h9_13": ("-0.26 0.37 -0.21", "-1.5707963 0.0 3.1415926"),
        "apriltag25h9_17": ("-0.26 -0.37 -0.21", "-1.5707963 0.0 0.0"),
        "apriltag36h11_146": ("0.430 0.0 0.405", "-1.5707963 0.0 -1.5707963"),
        "apriltag36h11_176": ("0.0 0.25 -0.012", "3.1415926 0.0 -1.5707963"),
        "apriltag36h11_185": ("0.0 -0.25 -0.012", "3.1415926 0.0 -1.5707963"),
        "apriltag36h11_541": ("0.470 0.0 0.160", "-1.5707963 0.0 -1.5707963"),
        "apriltag36h11_558": ("0.470 0.0 0.095", "-1.5707963 0.0 -1.5707963")}


def urdf(tags=TAGS, tag_parent="dock_point", extra="") -> str:
    joints = "".join(f'<link name="{name}"/><joint name="{name}_joint" type="fixed"><origin xyz="{xyz}" rpy="{rpy}"/>'
                     f'<parent link="{tag_parent}"/><child link="{name}"/></joint>'
                     for name, (xyz, rpy) in tags.items())
    return ('<robot name="race_station"><link name="base_link"/><link name="dock_point"/>'
            '<joint name="dock_point_joint" type="fixed"><origin xyz="-0.410 0.325 -0.04" rpy="3.1415926 0.0 0.0"/>'
            '<parent link="base_link"/><child link="dock_point"/></joint>' + extra + joints + "</robot>")


CONFIG = {"apriltag": {
    "object": {"urdf_package": "race_station_description", "urdf_filename": "urdf/base.urdf", "base_link_name": "",
               "tag_link_prefix": "apriltag", "dock_link_name": "dock_point"},
    "tags": [{"id": i, "family": "tag36h11", "size": s} for i, s in ((146, 0.15), (176, 0.15), (185, 0.15),
                                                                      (541, 0.05), (558, 0.05))],
    "cameras": [{"name": "cam_front", "enabled": True, "tags": [{"id": 146, "family": "tag36h11", "size": 0.15},
                                                                {"id": 541, "family": "tag36h11", "size": 0.05},
                                                                {"id": 558, "family": "tag36h11", "size": 0.05}]},
                {"name": "cam_down", "enabled": True, "tags": [{"id": 176, "family": "tag36h11", "size": 0.15},
                                                               {"id": 185, "family": "tag36h11", "size": 0.15}]}]}}


class DeriveTests(unittest.TestCase):
    def test_the_forward_cameras_tag_centroid_in_the_dock_frame(self):
        derived = tag_pivot.derive(CONFIG, urdf())
        self.assertEqual(derived["pivot_m"], [0.456667, 0.0, 0.22])
        self.assertEqual(derived["tags"], {"tag36h11:146": [0.43, 0.0, 0.405], "tag36h11:541": [0.47, 0.0, 0.16],
                                           "tag36h11:558": [0.47, 0.0, 0.095]})
        self.assertEqual((derived["camera"], derived["dock_link"]), ("cam_front", "dock_point"))

    def test_the_example_mission_carries_the_derived_pivot(self):
        self.assertEqual(EXAMPLE["mission"]["planner"]["tag_pivot_m"], tag_pivot.derive(CONFIG, urdf())["pivot_m"])

    def test_tags_under_an_intermediate_link_compose_the_chain(self):
        mount = ('<link name="mount"/><joint name="mount_joint" type="fixed">'
                 '<origin xyz="0.1 0.0 0.2" rpy="0 0 1.5707963"/><parent link="dock_point"/><child link="mount"/></joint>')
        moved = {name: (xyz, rpy) for name, (xyz, rpy) in TAGS.items()}
        derived = tag_pivot.derive(CONFIG, urdf(moved, tag_parent="mount", extra=mount))
        # in the dock frame: mount offset (0.1, 0, 0.2) plus the tag offset turned +90 deg about z
        self.assertEqual(derived["tags"]["tag36h11:146"], [0.1, 0.43, 0.605])

    def test_a_camera_without_its_own_list_uses_the_global_tags(self):
        config = copy.deepcopy(CONFIG)
        del config["apriltag"]["cameras"][0]["tags"]
        self.assertEqual(len(tag_pivot.derive(config, urdf())["tags"]), 5)

    def test_the_black_square_size_correction_does_not_matter(self):
        config = copy.deepcopy(CONFIG)
        for tag in config["apriltag"]["cameras"][0]["tags"]:
            tag["size"] = round(tag["size"] * 0.8, 6)
        self.assertEqual(tag_pivot.derive(config, urdf())["pivot_m"], [0.456667, 0.0, 0.22])

    def test_rejections(self):
        no_dock = copy.deepcopy(CONFIG)
        no_dock["apriltag"]["object"]["dock_link_name"] = ""
        two = copy.deepcopy(CONFIG)
        two["apriltag"]["cameras"].append(copy.deepcopy(two["apriltag"]["cameras"][0]))
        missing = {k: v for k, v in TAGS.items() if k != "apriltag36h11_541"}
        cases = {"no dock link": (no_dock, urdf()), "two forward cameras": (two, urdf()),
                 "a configured tag missing": (CONFIG, urdf(missing)),
                 "dock link not in the URDF": (CONFIG, urdf().replace("dock_point", "dock"))}
        for name, (config, text) in cases.items():
            with self.subTest(name), self.assertRaises(ValueError):
                tag_pivot.derive(config, text)


class TagRecordTests(unittest.TestCase):
    def test_every_cameras_tags_in_the_dock_frame(self):
        placed = tag_pivot.layout(CONFIG, urdf())
        self.assertEqual(sorted(placed["cameras"]), ["cam_down", "cam_front"])
        self.assertEqual([t["tag"] for t in placed["cameras"]["cam_front"]["tags"]],
                         ["tag36h11:146", "tag36h11:541", "tag36h11:558"])
        down = {t["tag"]: t["position_in_dock_m"] for t in placed["cameras"]["cam_down"]["tags"]}
        self.assertEqual(down, {"tag36h11:176": [0.0, 0.25, -0.012], "tag36h11:185": [0.0, -0.25, -0.012]})
        self.assertEqual({t["position_in_dock_m"][1] for t in placed["cameras"]["cam_front"]["tags"]}, {0.0})

    def installed(self, directory: Path, config) -> object:
        (directory / "race_auv_bringup/config/simulation").mkdir(parents=True)
        (directory / "race_auv_bringup/config/simulation/apriltag.yaml").write_text(yaml.safe_dump(config))
        (directory / "race_station_description/urdf").mkdir(parents=True)
        (directory / "race_station_description/urdf/base.urdf").write_text(urdf())
        return lambda package: str(directory / package)

    def test_sizes_in_both_conventions(self):
        corrected = copy.deepcopy(CONFIG)  # as prepare_candidate installs it for black_square_edge
        for camera in corrected["apriltag"]["cameras"]:
            for tag in camera["tags"]:
                tag["size"] = round(tag["size"] * collect_trial.BLACK_EDGE_FRACTION, 6)
        with tempfile.TemporaryDirectory() as tmp:
            black = collect_trial.tags_record(self.installed(Path(tmp) / "b", corrected), "black_square_edge")
            lab = collect_trial.tags_record(self.installed(Path(tmp) / "l", CONFIG), "lab_configured")
        for record, installed in ((black, "black_square_edge"), (lab, "lab_configured")):
            tag = record["cameras"]["cam_front"]["tags"][0]
            self.assertEqual((tag["tag"], tag["size_m"]), ("tag36h11:146", {"installed": installed,
                                                                             "lab_configured": 0.15,
                                                                             "black_square_edge": 0.12}))
            self.assertTrue(record["urdf"].endswith("race_station_description/urdf/base.urdf"))

    def test_an_unreadable_layout_is_recorded_not_raised(self):
        record = collect_trial.tags_record(lambda package: "/nonexistent", "black_square_edge")
        self.assertIn("FileNotFoundError", record["error"])


class CheckTests(unittest.TestCase):
    def installed(self, directory: Path, config=CONFIG, text=None) -> object:
        (directory / "race_auv_bringup/config/simulation").mkdir(parents=True)
        (directory / "race_auv_bringup/config/simulation/apriltag.yaml").write_text(yaml.safe_dump(config))
        (directory / "race_station_description/urdf").mkdir(parents=True)
        (directory / "race_station_description/urdf/base.urdf").write_text(text if text is not None else urdf())
        return lambda package: str(directory / package)

    def test_within_a_millimetre_passes_and_is_recorded(self):
        with tempfile.TemporaryDirectory() as tmp:
            share = self.installed(Path(tmp))
            record = collect_trial.pivot_record([0.4570, 0.0, 0.2195], share)
            self.assertTrue(record["ok"])
            self.assertEqual(record["derived"]["pivot_m"], [0.456667, 0.0, 0.22])
            self.assertTrue(record["derived"]["urdf"].endswith("race_station_description/urdf/base.urdf"))
            self.assertTrue(record["derived"]["apriltag_config"].endswith("config/simulation/apriltag.yaml"))
            self.assertEqual(record["mission_m"], [0.4570, 0.0, 0.2195])

    def test_a_tuned_offset_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            record = collect_trial.pivot_record([0.456667, 0.0, 0.222], self.installed(Path(tmp)))
            self.assertFalse(record["ok"])
            self.assertAlmostEqual(record["max_difference_m"], 0.002, places=9)

    def test_an_underivable_pivot_fails_closed_with_its_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            record = collect_trial.pivot_record([0.456667, 0.0, 0.22], self.installed(Path(tmp), text="<robot/>"))
            self.assertEqual((record["ok"], record["derived"]), (False, None))
            self.assertIn("ValueError", record["error"])

    def test_the_collector_checks_before_anything_moves(self):
        source = (Path(collect_trial.__file__)).read_text()
        run = source[source.index("        def run(self):"):]
        self.assertLess(run.index("pivot_record("), run.index("while not self.ready()"))
        self.assertIn('"tag_pivot_mismatch"', run)
        self.assertIn('"tag_pivot_underivable"', run)


if __name__ == "__main__":
    unittest.main()
