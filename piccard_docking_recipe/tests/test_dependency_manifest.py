"""The dependency manifest: the lab pins it must carry, full commits everywhere, a licence row per source, and no
private reference.  Run from the repository root:  python3 -m unittest discover -s piccard_docking_recipe/tests"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

import yaml

DEPENDENCIES = Path(__file__).resolve().parents[1] / "dependencies"
# The six lab pins requested for the docking study (piccard-inc/piccard-physical-ai#185), as full commits.
LAB_PINS = {
    "mvp_control": "6cfea2d5c1e371bd371aa6587066d27c617c55ad",
    "mvp_mission": "84cce537aa08267eb65bcf2b9ea65a27d81bd6f0",
    "mvp_msgs": "7aa616fab3bdd26dcd4ad178c4bfcd48c4231480",
    "mvp_utilities": "0c475d3ffe9ea8407caf640e70f9f7d8b86c3331",
    "stonefish_ros2": "a7c9be9c50c4e851d045d278ef3e641641673b70",
    "stonefish": "7d52673791834caa743907dde83a1949134ef2f0",
}


def repositories() -> dict:
    found = {}
    for name in ("workspace.repos", "libraries.repos"):
        for path, entry in yaml.safe_load((DEPENDENCIES / name).read_text())["repositories"].items():
            found[path] = entry
    return found


class ManifestTests(unittest.TestCase):
    def test_the_six_lab_pins(self) -> None:
        found = repositories()
        for name, commit in LAB_PINS.items():
            with self.subTest(name):
                self.assertEqual(found[name]["version"], commit)

    def test_every_version_is_a_full_commit_except_this_branch(self) -> None:
        for path, entry in repositories().items():
            with self.subTest(path):
                self.assertEqual(entry["type"], "git")
                self.assertRegex(entry["url"], r"^https://github\.com/[\w.-]+/[\w.-]+\.git$")
                if path == "race_auv":
                    self.assertEqual((entry["url"], entry["version"]),
                                     ("https://github.com/piccard-inc/race_auv.git", "piccard/docking-recipe"))
                else:
                    self.assertRegex(entry["version"], r"^[0-9a-f]{40}$")

    def test_every_source_has_a_licence_row(self) -> None:
        notes = (DEPENDENCIES / "DEPENDENCIES.md").read_text()
        licences = notes.split("## Licences", 1)[1]
        for path in repositories():
            with self.subTest(path):
                self.assertIn(path.split("/")[-1], licences)

    def test_nothing_private(self) -> None:
        for path in sorted(DEPENDENCIES.iterdir()):
            text = path.read_text()
            with self.subTest(path.name):
                self.assertIsNone(re.search(r"dkr\.ecr|amazonaws|\b\d{12}\b|/Users/|piccard-private", text))


if __name__ == "__main__":
    unittest.main()
