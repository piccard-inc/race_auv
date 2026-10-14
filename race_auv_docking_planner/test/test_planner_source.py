"""The package runs the planner that ran: its code is protocol v1.5's verbatim, and its parameter file is the scored
trials' parameters. No ROS."""
import hashlib
import json
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from race_auv_docking_planner import parameters as parameters_module  # noqa: E402
from race_auv_docking_planner import planner as planner_module  # noqa: E402

# sha256 of piccard-physical-ai packages/simulation/race-auv-docking/runtime/planner_fused_dock.py at
# 468d65f6dc16132dce734524a4e959369449b813 (protocol v1.5), from the line 'NODE = "piccard_planner"' up to its ROS node
# section, trailing blank lines removed: the constants, the rigid transforms and StagePlanner (359 lines).
PLANNER_V1_5_SHA256 = "3930b148caa4a64f49c309ed8d713f098e687fb366467678339cbba274065258"
# planner.parameters_sha256 in the records of every scored planner trial at the 0.1 m/s speed cap (four, M3-A and
# M3-B); M3-B's two speed-cap trials (0.05 and 0.2 m/s) differ from these parameters in speed_cap_mps only.
TRIAL_PARAMETERS_SHA256 = "12c05989447431826f1e082169eb572512e0b654301b564138f0c18d5ba5cc4d"


class PlannerSourceTests(unittest.TestCase):
    def test_the_planner_is_protocol_v1_5_verbatim(self):
        source = (PACKAGE / "race_auv_docking_planner/planner.py").read_text()
        copied = source[source.index('NODE = "piccard_planner"'):]
        self.assertEqual(hashlib.sha256(copied.encode()).hexdigest(), PLANNER_V1_5_SHA256)

    def test_the_parameter_file_is_the_scored_trials(self):
        planner, _ = parameters_module.load_yaml(PACKAGE / "config/docking_planner_v1_5.yaml", planner_module.NODE)
        self.assertEqual(parameters_module.parameters_sha256(planner), TRIAL_PARAMETERS_SHA256)
        as_doubles = hashlib.sha256(json.dumps(planner, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        self.assertNotEqual(as_doubles, TRIAL_PARAMETERS_SHA256)  # why the hash writes 5.0 as 5

    def test_the_hash_writes_integral_numbers_as_integers_only(self):
        self.assertEqual(parameters_module.canonical({"a": 5.0, "b": [3.0, 1.5, 0.0], "c": 0.005, "d": True}),
                         {"a": 5, "b": [3, 1.5, 0], "c": 0.005, "d": True})


if __name__ == "__main__":
    unittest.main()
