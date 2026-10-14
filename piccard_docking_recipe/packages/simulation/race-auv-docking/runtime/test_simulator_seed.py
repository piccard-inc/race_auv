"""The simulator seed: the runner's choice, trial.json's record of it, the mission field and stonefish_seed_v1."""
import copy
import hashlib
import io
import json
from pathlib import Path
import re
import tempfile
import unittest
from contextlib import redirect_stdout

import mission as mission_module
import simulator_seed

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parent
PATCH = PACKAGE / "simulator-patches/stonefish_seed_v1.patch"
NOTES = PACKAGE / "simulator-patches/STONEFISH_SEED_V1.md"
PATCH_SHA256 = "6068dbfbad5067aee90ccd6abc4e9875ff79d2866e76b0f2996bc8bd6595f16d"
POSE_REQUEST = json.loads((PACKAGE / "examples/m1-smoke-request.json").read_text())
PLANNER_REQUEST = json.loads((PACKAGE / "examples/m3-planner-request.json").read_text())


def log(*lines: str) -> str:
    return "\n".join(["[INFO] [launch]: All log files can be found below", *lines, "[stonefish_simulator-1] ready"]) + "\n"


class ChooseTests(unittest.TestCase):
    def test_the_missions_seed_is_exported(self):
        self.assertEqual(simulator_seed.choose({"seed": 42}), {"exported": 42, "origin": "request"})
        self.assertEqual(simulator_seed.choose({"seed": 0}), {"exported": 0, "origin": "request"})
        self.assertEqual(simulator_seed.choose({"seed": 2 ** 32 - 1})["exported"], 2 ** 32 - 1)

    def test_a_null_seed_is_drawn_in_range(self):
        bounds = []
        chosen = simulator_seed.choose({"seed": None}, draw=lambda bound: bounds.append(bound) or 7)
        self.assertEqual((chosen, bounds), ({"exported": 7, "origin": "drawn"}, [2 ** 32]))
        drawn = simulator_seed.choose({"seed": None})
        self.assertTrue(0 <= drawn["exported"] <= simulator_seed.MAX_SEED)

    def test_an_invalid_seed_is_refused(self):
        for seed in (-1, 2 ** 32, True, 1.5, "7"):
            with self.subTest(seed=seed), self.assertRaises(ValueError):
                simulator_seed.choose({"seed": seed})


class RecordTests(unittest.TestCase):
    CHOSEN = {"exported": 42, "origin": "drawn"}

    def test_a_patched_simulator_using_the_exported_seed(self):
        record = simulator_seed.record(self.CHOSEN, log(
            "[stonefish_simulator-1] stonefish_seed_v1: Sensor noise seed 42 from STONEFISH_SEED",
            "[stonefish_simulator-1] stonefish_seed_v1: USBL noise seed 43 from STONEFISH_SEED"))
        self.assertEqual(record, {"value": 42, "support": "stonefish_seed_v1", "exported": 42, "origin": "drawn",
                                  "environment": "STONEFISH_SEED", "source": "STONEFISH_SEED", "applied": True,
                                  "generators": {"Sensor": 42, "USBL": 43}})

    def test_the_pinned_simulator_prints_nothing_and_the_value_is_null(self):
        record = simulator_seed.record(self.CHOSEN, log("[stonefish_simulator-1] Loading scenario..."))
        self.assertEqual(record, {"value": None, "support": "not_exposed_by_pinned_runner", "exported": 42,
                                  "origin": "drawn", "environment": "STONEFISH_SEED", "applied": False})

    def test_a_printed_seed_that_is_not_the_exported_one_is_recorded_as_printed(self):
        record = simulator_seed.record(self.CHOSEN, log(
            "[stonefish_simulator-1] stonefish_seed_v1: Sensor noise seed 9001 from random_device"))
        self.assertEqual((record["value"], record["support"], record["source"], record["applied"]),
                         (9001, "stonefish_seed_v1", "random_device", False))
        other = simulator_seed.record(self.CHOSEN, log(
            "[stonefish_simulator-1] stonefish_seed_v1: Sensor noise seed 41 from STONEFISH_SEED"))
        self.assertEqual((other["value"], other["applied"]), (41, False))

    def test_only_the_simulators_own_lines_count(self):
        record = simulator_seed.record(self.CHOSEN, log(
            "[other_node-2] stonefish_seed_v1: Sensor noise seed 42 from STONEFISH_SEED",
            "stonefish_seed_v1: Sensor noise seed 42 from STONEFISH_SEED"))
        self.assertIsNone(record["value"])

    def test_without_a_seed_file(self):
        record = simulator_seed.record(None, "")
        self.assertEqual((record["value"], record["exported"], record["origin"]), (None, None, None))

    def test_read_record_and_the_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            mission = output / "mission.json"
            mission.write_text(json.dumps({**POSE_REQUEST["mission"], "seed": 1234}))
            out = io.StringIO()
            with redirect_stdout(out):
                self.assertEqual(simulator_seed.main(["--mission-profile-json", str(mission), "--output", tmp]), 0)
            self.assertEqual(out.getvalue(), "1234\n")
            self.assertEqual(json.loads((output / "simulator-seed.json").read_text()),
                             {"exported": 1234, "origin": "request"})
            with self.assertRaises(SystemExit):  # one seed per trial output
                simulator_seed.main(["--mission-profile-json", str(mission), "--output", tmp])
            self.assertEqual(simulator_seed.read_record(output)["support"], "not_exposed_by_pinned_runner")
            (output / "launch.log").write_text(log(
                "[stonefish_simulator-1] stonefish_seed_v1: Sensor noise seed 1234 from STONEFISH_SEED"))
            self.assertEqual(simulator_seed.read_record(output)["value"], 1234)


class MissionSeedTests(unittest.TestCase):
    def test_both_mission_kinds_take_null_or_a_seed_in_range(self):
        for request in (POSE_REQUEST, PLANNER_REQUEST):
            for seed in (None, 0, 1234, 2 ** 32 - 1):
                with self.subTest(schema=request["mission"]["schema"], seed=seed):
                    mission_module.validate_mission({**copy.deepcopy(request["mission"]), "seed": seed})
            for seed in (-1, 2 ** 32, True, 1.5, "1"):
                with self.subTest(schema=request["mission"]["schema"], seed=seed), self.assertRaises(ValueError):
                    mission_module.validate_mission({**copy.deepcopy(request["mission"]), "seed": seed})

    def test_one_bound(self):
        self.assertEqual(simulator_seed.MAX_SEED, mission_module.MAX_SEED)
        self.assertEqual(mission_module.MAX_SEED, 2 ** 32 - 1)


class WiringTests(unittest.TestCase):
    def test_the_runner_exports_the_seed_before_the_launch(self):
        script = (HERE / "run_trial.sh").read_text()
        choose = script.index('STONEFISH_SEED="$(python3 "$script_dir/simulator_seed.py" --mission-profile-json "$mission" '
                              '--output "$output")"')
        export = script.index("export STONEFISH_SEED\n")
        launch = script.index('ros2 launch "$script_dir/launch/docking_sim.launch.py"')
        self.assertLess(choose, export)
        self.assertLess(export, launch)

    def test_the_collector_records_the_seed_at_the_start_and_again_at_the_end(self):
        source = (HERE / "collect_trial.py").read_text()
        self.assertIn("simulator_seed=simulator_seed.read_record(self.root)", source)
        finish = source[source.index("        def finish(self):"):]
        self.assertIn('self.metadata["simulator_seed"] = simulator_seed.read_record(self.root)', finish)
        self.assertLess(finish.index('self.metadata["simulator_seed"]'), finish.index('("trial.json", self.metadata)'))
        self.assertNotIn('{"value": None, "support": "not_exposed_by_pinned_runner"}', source)


class PatchTests(unittest.TestCase):
    TEXT = PATCH.read_text()

    def test_the_patch_is_the_recorded_one(self):
        self.assertEqual(hashlib.sha256(PATCH.read_bytes()).hexdigest(), PATCH_SHA256)
        self.assertIn(PATCH_SHA256, NOTES.read_text())

    def test_it_changes_only_the_two_seeding_lines_files(self):
        files = re.findall(r"^diff --git a/(\S+) b/\1$", self.TEXT, re.MULTILINE)
        self.assertEqual(files, ["Library/src/comms/USBL.cpp", "Library/src/sensors/Sensor.cpp"])
        removed = [line[1:] for line in self.TEXT.splitlines() if line.startswith("-") and not line.startswith("---")]
        self.assertEqual(removed, ["std::mt19937 USBL::randomGenerator(randomDevice());",
                                   "std::mt19937 Sensor::randomGenerator(randomDevice());"])
        self.assertIn('+std::mt19937 Sensor::randomGenerator(stonefishSeed("Sensor", 0, randomDevice));', self.TEXT)
        self.assertIn('+std::mt19937 USBL::randomGenerator(stonefishSeed("USBL", 1, randomDevice));', self.TEXT)

    def test_its_printed_line_is_the_one_the_record_reads(self):
        fmt = re.search(r'std::fprintf\(stderr, "(stonefish_seed_v1: %s noise seed %llu from %s)\\n"', self.TEXT).group(1)
        line = "[stonefish_simulator-1] " + fmt.replace("%s", "Sensor", 1).replace("%llu", "42").replace("%s", "STONEFISH_SEED")
        self.assertEqual(simulator_seed.record({"exported": 42, "origin": "request"}, line + "\n")["value"], 42)
        self.assertIn('std::getenv("STONEFISH_SEED")', self.TEXT)

    def test_the_notes_carry_the_file_hashes(self):
        notes = NOTES.read_text()
        for digest in ("74ea906e4319061b0e50d5fe1e67ba4e6aecc700adc907f37a08c0906a5d6a22",
                       "b71b3f00ef4bfaf8bf9127a67f7e7f800a9a85dae6bbf9370d4ea02e2e837440",
                       "278a4d32cfef391fd74f7aac6b9705f5af407f00b37feb1fed78216bc343e60a",
                       "84394e87dd3c202bb55e96a2d35686576ffff29fa761df10164c4252088fe532"):
            self.assertIn(digest, notes)


if __name__ == "__main__":
    unittest.main()
