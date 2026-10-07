"""The native trial path (run_native_trial.sh, native_request.py, run_trial.sh's native hooks). No ROS: the request
split, the planner parameter file, and the scripts' order and hooks."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import yaml

import mission as mission_module
import native_request

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parent
PLANNER_REQUEST = json.loads((PACKAGE / "examples/m3-planner-request.json").read_text())
POSE_REQUEST = json.loads((PACKAGE / "examples/m1-smoke-request.json").read_text())
# planner.parameters_sha256 of the scored M3 planner trials at the 0.1 m/s cap: the example's values, as the submit
# path wrote them (integral numbers as integers). race_auv_docking_planner pins the same hash for its v1.5 YAML.
TRIAL_PARAMETERS_SHA256 = "12c05989447431826f1e082169eb572512e0b654301b564138f0c18d5ba5cc4d"


def package_hash(parameters: dict) -> str:
    """race_auv_docking_planner.parameters.parameters_sha256: integral floats as integers, then the runtime's JSON."""
    return hashlib.sha256(json.dumps(native_request.canonical(parameters), sort_keys=True,
                                     separators=(",", ":")).encode()).hexdigest()


class SplitTests(unittest.TestCase):
    def split(self, request):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        target = Path(tmp.name) / "request"
        native_request.split(copy.deepcopy(request), target)
        return target

    def test_the_parts_and_the_limits(self):
        target = self.split(PLANNER_REQUEST)
        self.assertEqual(sorted(p.name for p in target.iterdir()),
                         ["context.json", "expected-gains.json", "limits.env", "mission.json"])
        self.assertEqual((target / "limits.env").read_text(), "HORIZON_SECONDS=1800\nWALL_TIMEOUT_SECONDS=1950\n")
        self.assertEqual(json.loads((target / "expected-gains.json").read_text()), PLANNER_REQUEST["gains"])
        self.assertEqual(json.loads((target / "context.json").read_text()), PLANNER_REQUEST["context"])

    def test_numbers_are_written_as_the_submit_path_writes_them(self):
        mission = json.loads((self.split(PLANNER_REQUEST) / "mission.json").read_text())
        self.assertEqual(mission, PLANNER_REQUEST["mission"])  # the same values
        self.assertEqual(mission["planner"]["standoffs_m"], [3, 1.5, 0.3, 0])
        self.assertIsInstance(mission["planner"]["estimate_filter_s"], int)
        # The collector hashes the mission as received; the planner package hashes integral floats as integers.
        self.assertEqual(mission_module.parameters_sha256(mission["planner"]), TRIAL_PARAMETERS_SHA256)
        self.assertNotEqual(mission_module.parameters_sha256(PLANNER_REQUEST["mission"]["planner"]),
                            TRIAL_PARAMETERS_SHA256)  # as written in the example: why the split rewrites numbers

    def test_a_pose_mission_and_no_context(self):
        request = {key: value for key, value in POSE_REQUEST.items() if key != "context"}
        target = self.split(request)
        self.assertEqual(json.loads((target / "context.json").read_text()), {})

    def test_refusals(self):
        bad = {"extra key": {**PLANNER_REQUEST, "image": "x"},
               "no horizon": {k: v for k, v in PLANNER_REQUEST.items() if k != "horizon_s"},
               "zero horizon": {**PLANNER_REQUEST, "horizon_s": 0},
               "long wall timeout": {**PLANNER_REQUEST, "wall_timeout_s": 3601},
               "invalid mission": {**PLANNER_REQUEST, "mission": {**PLANNER_REQUEST["mission"], "frame_id": "map"}},
               "gains not an object": {**PLANNER_REQUEST, "gains": []}}
        for name, request in bad.items():
            with self.subTest(name), tempfile.TemporaryDirectory() as tmp, self.assertRaises(ValueError):
                native_request.split(copy.deepcopy(request), Path(tmp) / "request")
        with tempfile.TemporaryDirectory() as tmp, self.assertRaises(FileExistsError):
            native_request.split(copy.deepcopy(PLANNER_REQUEST), Path(tmp))


class PlannerParamsTests(unittest.TestCase):
    def test_the_package_parameter_file(self):
        document = native_request.planner_params(copy.deepcopy(PLANNER_REQUEST["mission"]))
        parameters = document["piccard_planner"]["ros__parameters"]
        setpoint = parameters.pop("initial_setpoint")
        fallback = PLANNER_REQUEST["mission"]["fallback_pose"]
        self.assertEqual(setpoint, {key: float(fallback[key]) for key in native_request.SETPOINT_FIELDS})
        self.assertEqual(parameters, PLANNER_REQUEST["mission"]["planner"])
        for key, value in parameters.items():  # every value a double, as the node declares them
            with self.subTest(key):
                self.assertTrue(all(isinstance(v, float) for v in value) if isinstance(value, list)
                                else isinstance(value, float))
        self.assertEqual(package_hash(parameters), TRIAL_PARAMETERS_SHA256)

    def test_the_file_round_trips_and_a_pose_mission_has_none(self):
        with tempfile.TemporaryDirectory() as tmp:
            mission, out = Path(tmp) / "mission.json", Path(tmp) / "planner-params.yaml"
            mission.write_text(json.dumps(PLANNER_REQUEST["mission"]))
            self.assertEqual(native_request.main(["planner-params", "--mission", str(mission), "--output", str(out)]), 0)
            self.assertEqual(yaml.safe_load(out.read_text()), native_request.planner_params(PLANNER_REQUEST["mission"]))
            with self.assertRaises(SystemExit):
                native_request.main(["planner-params", "--mission", str(mission), "--output", str(out)])
        with self.assertRaises(ValueError):
            native_request.planner_params(copy.deepcopy(POSE_REQUEST["mission"]))


class ScriptTests(unittest.TestCase):
    NATIVE = (HERE / "run_native_trial.sh").read_text()
    TRIAL = (HERE / "run_trial.sh").read_text()

    def test_the_native_runner_runs_the_steps_in_order(self):
        steps = ['native_request.py" split', 'source "$parts/limits.env"', 'prepare_candidate.py" install',
                 'prepare_candidate.py" verify', 'native_request.py" planner-params',
                 'export PICCARD_PLANNER_PARAMS=', '"$script_dir/run_trial.sh"', 'status=$?', 'python3 "$metrics"',
                 'exit "$status"']
        positions = [self.NATIVE.index(step) for step in steps]
        self.assertEqual(positions, sorted(positions))
        self.assertIn('export PICCARD_NATIVE_WORKSPACE="$workspace"', self.NATIVE)
        self.assertIn('source "$workspace/install/setup.bash"', self.NATIVE)

    def test_the_metrics_path_is_this_repositorys(self):
        self.assertTrue((HERE / "../../../../tools/analysis/race_m3_metrics.py").resolve().is_file())

    def test_missing_arguments_stop_before_anything_runs(self):
        for argv in ([], ["--workspace", "/nonexistent"], ["--bogus"]):
            with self.subTest(argv=argv):
                result = subprocess.run(["bash", str(HERE / "run_native_trial.sh"), *argv], capture_output=True,
                                        text=True, timeout=30)
                self.assertEqual(result.returncode, 64, result.stderr)

    def test_run_trial_sources_the_image_only_without_a_native_workspace(self):
        head = self.TRIAL[:self.TRIAL.index("set -u")]
        self.assertIn('if [[ -z "${PICCARD_NATIVE_WORKSPACE:-}" ]]; then', head)
        for setup in ("/opt/ros/jazzy/setup.bash", "/opt/ros2_ws/install/setup.bash", "/opt/race_ws/install/setup.bash"):
            self.assertGreater(head.index(setup), head.index("PICCARD_NATIVE_WORKSPACE"))

    def test_run_trial_runs_the_planner_package_natively_and_the_script_in_the_image(self):
        block = self.TRIAL[self.TRIAL.index("if ((planner_mission)); then"):self.TRIAL.index("planner_pid=$!")]
        self.assertIn('ros2 launch race_auv_docking_planner docking_planner.launch.py params_file:="$PICCARD_PLANNER_PARAMS"',
                      block)
        self.assertIn('python3 "$script_dir/planner_fused_dock.py" --mission-profile-json "$mission"', block)
        self.assertLess(block.index("PICCARD_PLANNER_PARAMS"), block.index("planner_fused_dock.py"))


if __name__ == "__main__":
    unittest.main()
