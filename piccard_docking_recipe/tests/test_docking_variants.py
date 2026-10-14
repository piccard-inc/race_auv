"""The opt-in docking variants on piccard/docking-recipe, checked against the defaults they leave alone.

Run from the repository root:  python3 -m unittest discover -s piccard_docking_recipe/tests  (needs PyYAML).

- Every default file the variants derive from, and every launch file the default bringup reads, is byte-identical
  to base da58963c, so the default bringup is unchanged.
- No default launch file reads a variant; only bringup_docking_simulation.launch.py does.
- Each YAML variant equals what the trial-time installer (the snapshot's runtime/prepare_candidate.py) produced from
  its default, the control variant with the planner study's p10 docking gains.
- Each launch variant is its default with only the documented file names swapped, behind a comment header.
"""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
BASE = "da58963c048228856dfdd5917da21a14edfa1985"
DEFAULTS = {  # sha256 at BASE
    "race_auv_config/mvp_control_config/config_sim.yaml": "24de8bd709e5e68b80d4a08e592c574d4821cd1f5e6ac2de3c16bbb9e1810b1d",
    "race_auv_config/mvp_mission_config/helm_sim.yaml": "184013d2054edd471632c68f479371f8613b803174de06012de6f7a626b458aa",
    "race_auv_bringup/config/bhv_params_sim.yaml": "d01cfbff8835b8fec96486fd70274fc76ca7af358a7ff064c04092e693997f10",
    "race_auv_bringup/config/simulation/apriltag.yaml": "465a3841b2078a1073eb371563c7deeabd8a5b1bbcb610444027620d7d35e328",
    "race_auv_bringup/launch/bringup_simulation.launch.py": "11216d8b1a878d15dbe22802812bcfc9343a7a42587346c3bb03ebb9393891ce",
    "race_auv_bringup/launch/include/simulation/mvp_control_sim.launch.py": "c4a6cadd4cdb7e1c18c091a3da0c73cd228f6877f79a5ffc068152876ffb12a7",
    "race_auv_bringup/launch/include/simulation/mvp_mission_sim.launch.py": "3ec548e543e0ef5493fb9acf6de50055a9007969c239b8874d40d22fd255eaef",
    "race_auv_bringup/launch/include/simulation/apriltag_sim.launch.py": "c6698507bd46bd9c3db329a9fea1fbe0b7093b06c157953288207a7b313e21fd",
}
CONTROL, HELM = "race_auv_config/mvp_control_config/", "race_auv_config/mvp_mission_config/"
BRINGUP, LAUNCH = "race_auv_bringup/config/", "race_auv_bringup/launch/"
SIMULATION = LAUNCH + "include/simulation/"
# The RACE docking planner study's docking gains: Piccard physical-ai
# packages/simulation/race-auv-docking/examples/m3-planner-request.json, gains.docking.
PLANNER_DOCKING = {
    "x": {"p": 10.0, "i": 0.2, "d": 0.0, "v": 5.0, "pid_min": -15, "pid_max": 15},
    "y": {"p": 10.0, "i": 0.2, "d": 0.0, "v": 5.0, "pid_min": -15, "pid_max": 15},
    "z": {"p": 15.0, "i": 1.0, "d": 0.0, "v": 5.0, "pid_min": -30, "pid_max": 30},
    "roll": {"p": 2.0, "i": 0.1, "d": 0.0, "v": 5.0, "pid_min": -10, "pid_max": 10},
    "pitch": {"p": 5.0, "i": 1.0, "d": 0.0, "v": 5.0, "pid_min": -20, "pid_max": 20},
    "yaw": {"p": 4.0, "i": 0.2, "d": 0.0, "v": 5.0, "pid_min": -15, "pid_max": 15},
}
LAUNCH_VARIANTS = {  # variant: (default, replacements)
    SIMULATION + "mvp_control_docking_sim.launch.py": (
        SIMULATION + "mvp_control_sim.launch.py",
        (("'config_sim.yaml')", "'config_sim_docking.yaml')"),)),
    SIMULATION + "mvp_mission_docking_sim.launch.py": (
        SIMULATION + "mvp_mission_sim.launch.py",
        (("'bhv_params_sim.yaml')", "'bhv_params_sim_docking.yaml')"), ("'helm_sim.yaml')", "'helm_sim_docking.yaml')"))),
    LAUNCH + "bringup_docking_simulation.launch.py": (
        LAUNCH + "bringup_simulation.launch.py",
        (("'mvp_control_sim.launch.py')", "'mvp_control_docking_sim.launch.py')"),
         ("'mvp_mission_sim.launch.py')", "'mvp_mission_docking_sim.launch.py')"),
         ("                         'apriltag_sim.launch.py')\n        ]),\n    )\n",
          "                         'apriltag_sim.launch.py')\n        ]),\n"
          "        launch_arguments={'config': os.path.join(get_package_share_directory(robot_bringup), 'config',\n"
          "                                                 'simulation', 'apriltag_black_square_edge.yaml')}.items(),\n"
          "    )\n"))),
}


def installer():
    """The trial-time installer as exported to this branch, which these variants replace."""
    path = ROOT / "piccard_docking_recipe/packages/simulation/race-auv-docking/runtime/prepare_candidate.py"
    spec = importlib.util.spec_from_file_location("snapshot_prepare_candidate", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load(path: str):
    return yaml.safe_load((ROOT / path).read_text())


def without_header(text: str) -> str:
    """A variant's text after its leading comment block and the blank line that ends it."""
    lines = text.splitlines(keepends=True)
    n = 0
    while n < len(lines) and lines[n].startswith("#"):
        n += 1
    assert n and lines[n] == "\n", "a variant starts with a comment header and one blank line"
    return "".join(lines[n + 1:])


class DefaultBringupTests(unittest.TestCase):
    def test_every_default_file_is_byte_identical_to_the_base(self) -> None:
        for path, digest in DEFAULTS.items():
            with self.subTest(path):
                self.assertEqual(hashlib.sha256((ROOT / path).read_bytes()).hexdigest(), digest)

    def test_no_default_launch_file_reads_a_variant(self) -> None:
        for launch in sorted((ROOT / LAUNCH).rglob("*.launch.py")):
            relative = launch.relative_to(ROOT).as_posix()
            if relative in LAUNCH_VARIANTS:
                continue
            with self.subTest(relative):
                text = launch.read_text()
                self.assertNotIn("_docking", text)
                self.assertNotIn("apriltag_black_square_edge", text)


class VariantTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.installer = installer()

    def test_the_control_variant_adds_only_the_planner_docking_mode(self) -> None:
        expected = load(CONTROL + "config_sim.yaml")
        self.assertNotIn("docking", expected["control_modes"])
        expected["control_modes"]["docking"] = PLANNER_DOCKING
        self.assertEqual(load(CONTROL + "config_sim_docking.yaml"), expected)

    def test_the_helm_variant_is_what_the_installer_wrote(self) -> None:
        expected = self.installer.install_helm(load(HELM + "helm_sim.yaml"))
        self.assertEqual(load(HELM + "helm_sim_docking.yaml"), expected)

    def test_the_behaviour_variant_is_what_the_installer_wrote(self) -> None:
        expected = load(BRINGUP + "bhv_params_sim.yaml")
        self.assertNotIn(self.installer.HELM_BEHAVIOR, expected)
        expected[self.installer.HELM_BEHAVIOR] = dict(self.installer.BHV_PARAMS)
        self.assertEqual(load(BRINGUP + "bhv_params_sim_docking.yaml"), expected)

    def test_the_apriltag_variant_is_what_the_installer_wrote(self) -> None:
        default = load(BRINGUP + "simulation/apriltag.yaml")
        expected = self.installer.corrected_apriltag(copy.deepcopy(default))
        variant = load(BRINGUP + "simulation/apriltag_black_square_edge.yaml")
        self.assertEqual(variant, expected)
        self.assertNotEqual(variant, default)
        self.assertEqual(variant["apriltag"]["detector_defaults"]["detector_backend"], "python")

    def test_each_launch_variant_is_its_default_with_only_the_documented_names_swapped(self) -> None:
        for variant, (default, replacements) in LAUNCH_VARIANTS.items():
            with self.subTest(variant):
                expected = (ROOT / default).read_text()
                for old, new in replacements:
                    self.assertEqual(expected.count(old), 1, old)
                    expected = expected.replace(old, new)
                self.assertEqual(without_header((ROOT / variant).read_text()), expected)

    def test_each_yaml_variant_keeps_its_default_text_after_its_header(self) -> None:
        """Comments and layout of the defaults survive: each variant only inserts or edits the lines it documents."""
        for variant, default in ((CONTROL + "config_sim_docking.yaml", CONTROL + "config_sim.yaml"),
                                 (HELM + "helm_sim_docking.yaml", HELM + "helm_sim.yaml"),
                                 (BRINGUP + "bhv_params_sim_docking.yaml", BRINGUP + "bhv_params_sim.yaml"),
                                 (BRINGUP + "simulation/apriltag_black_square_edge.yaml",
                                  BRINGUP + "simulation/apriltag.yaml")):
            with self.subTest(variant):
                body = without_header((ROOT / variant).read_text()).splitlines()
                original = (ROOT / default).read_text().splitlines()
                changed = len(original) - sum(1 for line in original if line in body)
                self.assertLessEqual(changed, 10, "a variant rewrites only the lines it documents")


if __name__ == "__main__":
    unittest.main()
