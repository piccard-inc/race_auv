"""Scoring and analysis stay apart from the vehicle's packages (#185 deliverable 5).

Run from the repository root:  python3 -m unittest discover -s piccard_docking_recipe/tests

- colcon never builds piccard_docking_recipe/: it holds COLCON_IGNORE and no package manifest.
- No ROS package of this repository refers to it: no file outside Markdown names it, and no Python file imports one
  of its modules.
- No ROS package's code names a ground-truth topic or frame outside its tests. That code is everything except the
  test directories; its configuration is not scanned, because the vehicle's own configuration is the lab's. Ground
  truth is used in the recipe, where the collector records it for scoring and the consumption audit fails a trial if
  a vehicle node consumes it.
- The submodules (race_auv_perception, race_auv_sim) are the lab's own repositories and are not checked here.
"""

from __future__ import annotations

import ast
import configparser
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RECIPE = ROOT / "piccard_docking_recipe"
PACKAGES = {"race_auv", "race_auv_bringup", "race_auv_config", "race_auv_description", "race_auv_docking_planner"}
MANIFESTS = ("package.xml", "setup.py", "setup.cfg", "CMakeLists.txt")
# As race_auv_docking_planner/test/test_ground_truth_boundary.py: what the trials kept for scoring.
GROUND_TRUTH_MARKERS = ("stonefish/odometry", "ground_truth", "/piccard/contact", "upstream_docking_pose",
                        "race_station/base_link")


def submodules() -> set:
    parser = configparser.ConfigParser()
    parser.read(ROOT / ".gitmodules")
    return {ROOT / parser[section]["path"] for section in parser.sections()}


def ros_packages() -> dict:
    """{name: directory} for every package.xml outside .git, the recipe and the submodules."""
    skip = {ROOT / ".git", RECIPE, *submodules()}
    found = {}
    for manifest in ROOT.rglob("package.xml"):
        if not any(manifest.is_relative_to(path) for path in skip):
            found[manifest.parent.name] = manifest.parent
    return found


def is_test(path: Path, package: Path) -> bool:
    return bool({"test", "tests"} & set(path.relative_to(package).parts))


def code_strings(tree) -> list:
    """Every string constant that is not a docstring."""
    docstrings = {id(node.body[0].value) for node in ast.walk(tree)
                  if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef)) and node.body
                  and isinstance(node.body[0], ast.Expr) and isinstance(node.body[0].value, ast.Constant)}
    return [node.value for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docstrings]


def imported(tree) -> set:
    """The top-level name of every module a file imports, at any depth."""
    names = {alias.name.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.Import)
             for alias in node.names}
    return names | {node.module.split(".")[0] for node in ast.walk(tree)
                    if isinstance(node, ast.ImportFrom) and node.module and node.level == 0}


class RecipeTests(unittest.TestCase):
    def test_colcon_never_builds_it(self):
        self.assertTrue((RECIPE / "COLCON_IGNORE").is_file())
        for name in MANIFESTS:
            with self.subTest(name):
                self.assertEqual([p.relative_to(ROOT).as_posix() for p in RECIPE.rglob(name)], [])


class VehiclePackageTests(unittest.TestCase):
    def setUp(self):
        self.packages = ros_packages()

    def test_every_package_is_checked(self):
        self.assertLessEqual(PACKAGES, set(self.packages))

    def test_no_package_refers_to_the_recipe(self):
        for package in self.packages.values():
            for path in package.rglob("*"):
                if path.is_file() and path.suffix != ".md":
                    with self.subTest(path=path.relative_to(ROOT).as_posix()):
                        self.assertNotIn(b"piccard_docking_recipe", path.read_bytes())

    def test_no_package_imports_a_recipe_module(self):
        modules = {path.stem for path in RECIPE.rglob("*.py") if not path.name.startswith("test_")}
        self.assertTrue({"collect_trial", "graph_audit", "docking_metric", "race_m3_metrics"} <= modules)
        for package in self.packages.values():
            for path in package.rglob("*.py"):
                with self.subTest(path=path.relative_to(ROOT).as_posix()):
                    self.assertEqual(imported(ast.parse(path.read_text())) & modules, set())

    def test_no_package_code_names_ground_truth(self):
        for package in self.packages.values():
            for path in package.rglob("*.py"):
                if is_test(path, package):
                    continue
                strings = code_strings(ast.parse(path.read_text()))
                for marker in GROUND_TRUTH_MARKERS:
                    with self.subTest(path=path.relative_to(ROOT).as_posix(), marker=marker):
                        self.assertFalse([text for text in strings if marker in text])

    def test_the_markers_find_the_recipes_own_ground_truth(self):
        audit = RECIPE / "packages/simulation/race-auv-docking/runtime/graph_audit.py"
        strings = " ".join(code_strings(ast.parse(audit.read_text())))
        for marker in ("stonefish/odometry", "ground_truth", "/piccard/contact", "upstream_docking_pose"):
            with self.subTest(marker):
                self.assertIn(marker, strings)


if __name__ == "__main__":
    unittest.main()
