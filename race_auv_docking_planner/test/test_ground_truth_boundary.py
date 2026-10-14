"""Ground truth is not an input, an import or a dependency of this package.

The planner's inputs are the EKF odometry and the TF tree, looked up for four edges: the AprilTag fuser's
race_auv/base_link -> race_station/dock_point, race_auv/world_ned -> race_auv/base_link through the EKF's odom, and
the static base_link -> auv_dock_point and base_link -> cg_link. Its outputs are the helm's direct_control set points
and its state record. These tests read the package's files; no ROS. Ground truth here is what the trials kept for
scoring: the simulator's odometry of the vehicle and the station, the contact topics and /piccard/ground_truth/*
(the station's own TF tree among them).
"""
import ast
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from race_auv_docking_planner import planner as planner_module  # noqa: E402

GROUND_TRUTH_MARKERS = ("stonefish/odometry", "ground_truth", "/piccard/contact", "upstream_docking_pose",
                        "race_station/base_link")
IMPORTS = {  # every module each file imports, at any depth
    "race_auv_docking_planner/__init__.py": set(),
    "race_auv_docking_planner/planner.py": {"__future__", "math", "statistics"},
    "race_auv_docking_planner/parameters.py": {"__future__", "hashlib", "json", "math", "typing", "yaml"},
    "race_auv_docking_planner/node.py": {"__future__", "json", "time", "race_auv_docking_planner", "rclpy",
                                         "nav_msgs", "std_msgs", "std_srvs", "tf2_ros", "mvp_msgs"},
    "launch/docking_planner.launch.py": {"os", "ament_index_python", "launch", "launch_ros"},
    "setup.py": {"glob", "setuptools"},
}
DEPENDENCIES = {"buildtool_depend": {"ament_python"},
                "exec_depend": {"rclpy", "nav_msgs", "std_msgs", "std_srvs", "tf2_ros_py", "mvp_msgs", "launch",
                                "launch_ros", "ament_index_python"},
                "test_depend": {"python3-pytest", "python3-yaml"}}


def source_files():
    return sorted(p for p in PACKAGE.rglob("*.py") if "test" not in p.relative_to(PACKAGE).parts)


def imported(tree) -> set:
    names = {alias.name.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
    return names | {node.module.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}


def code_strings(tree) -> list:
    """Every string constant that is not a docstring."""
    docstrings = {id(node.body[0].value) for node in ast.walk(tree)
                  if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef)) and node.body
                  and isinstance(node.body[0], ast.Expr) and isinstance(node.body[0].value, ast.Constant)}
    return [node.value for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docstrings]


class ImportTests(unittest.TestCase):
    def test_every_file_imports_exactly_what_it_needs(self):
        self.assertEqual({p.relative_to(PACKAGE).as_posix() for p in source_files()}, set(IMPORTS))
        for path in source_files():
            relative = path.relative_to(PACKAGE).as_posix()
            with self.subTest(relative):
                self.assertEqual(imported(ast.parse(path.read_text())), IMPORTS[relative])

    def test_the_decisions_import_no_ros(self):
        tree = ast.parse((PACKAGE / "race_auv_docking_planner/planner.py").read_text())
        self.assertEqual(imported(tree), {"__future__", "math", "statistics"})

    def test_the_node_imports_ros_only_inside_its_class_factory(self):
        tree = ast.parse((PACKAGE / "race_auv_docking_planner/node.py").read_text())
        top = {alias.name for node in tree.body if isinstance(node, ast.Import) for alias in node.names}
        top |= {node.module for node in tree.body if isinstance(node, ast.ImportFrom)}
        self.assertEqual(top, {"__future__", "json", "time", "race_auv_docking_planner.parameters",
                               "race_auv_docking_planner.planner"})


class NodeInterfaceTests(unittest.TestCase):
    TREE = ast.parse((PACKAGE / "race_auv_docking_planner/node.py").read_text())

    def calls(self, name: str) -> list:
        return [node for node in ast.walk(self.TREE) if isinstance(node, ast.Call)
                and (getattr(node.func, "attr", None) == name or getattr(node.func, "id", None) == name)]

    def test_the_only_subscription_is_the_ekf_odometry(self):
        self.assertEqual([(ast.unparse(c.args[0]), ast.unparse(c.args[1])) for c in self.calls("create_subscription")],
                         [("Odometry", "EKF_ODOMETRY")])
        self.assertEqual(planner_module.EKF_ODOMETRY, "/race_auv/odometry/filtered")

    def test_tf_is_read_through_one_listener_and_one_lookup(self):
        self.assertEqual(len(self.calls("TransformListener")), 1)
        self.assertEqual(len(self.calls("lookup_transform")), 1)
        self.assertEqual(self.calls("create_client"), [])
        self.assertEqual(self.calls("create_subscriber"), [])

    def test_the_outputs(self):
        self.assertEqual([(ast.unparse(c.args[0]), ast.unparse(c.args[1])) for c in self.calls("create_publisher")],
                         [("ControlProcess", "SETPOINT_TOPIC"), ("String", "STATE_TOPIC")])
        self.assertEqual([ast.unparse(c.args[1]) for c in self.calls("create_service")], ["START_SERVICE"])
        self.assertEqual((planner_module.SETPOINT_TOPIC, planner_module.STATE_TOPIC, planner_module.START_SERVICE),
                         ("/race_auv/mvp_helm/bhv_direct_control/desired_setpoints", "/piccard/planner/state",
                          "/piccard_planner/start"))

    def test_the_four_edges(self):
        self.assertEqual(set(planner_module.PLANNER_TF_LOOKUPS),
                         {("race_auv/world_ned", "race_auv/base_link"), ("race_auv/base_link", "race_station/dock_point"),
                          ("race_auv/base_link", "race_auv/auv_dock_point"), ("race_auv/base_link", "race_auv/cg_link")})


class GroundTruthNameTests(unittest.TestCase):
    def test_no_code_names_a_ground_truth_topic_or_frame(self):
        for path in source_files():
            for text in code_strings(ast.parse(path.read_text())):
                for marker in GROUND_TRUTH_MARKERS:
                    with self.subTest(path=path.name, marker=marker):
                        self.assertNotIn(marker, text)

    def test_no_configuration_names_one(self):
        for path in [*PACKAGE.glob("config/*"), PACKAGE / "package.xml", PACKAGE / "setup.cfg"]:
            text = path.read_text()
            for marker in GROUND_TRUTH_MARKERS:
                with self.subTest(path=path.name, marker=marker):
                    self.assertNotIn(marker, text)

    def test_the_launch_file_remaps_nothing(self):
        tree = ast.parse((PACKAGE / "launch/docking_planner.launch.py").read_text())
        keywords = {k.arg for node in ast.walk(tree) if isinstance(node, ast.Call) for k in node.keywords}
        self.assertNotIn("remappings", keywords)
        self.assertFalse({"SetRemap", "PushRosNamespace"} & {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)})


class DependencyTests(unittest.TestCase):
    def test_package_xml_depends_on_exactly_these(self):
        root = ET.parse(PACKAGE / "package.xml").getroot()
        found = {}
        for element in root:
            if element.tag.endswith("depend"):
                found.setdefault(element.tag, set()).add(element.text.strip())
        self.assertEqual(found, DEPENDENCIES)

    def test_no_simulator_or_station_package(self):
        every = set().union(*DEPENDENCIES.values())
        for name in every:
            with self.subTest(name):
                for forbidden in ("stonefish", "race_auv_sim", "race_station", "world_of_stonefish", "gazebo"):
                    self.assertNotIn(forbidden, name)


if __name__ == "__main__":
    unittest.main()
