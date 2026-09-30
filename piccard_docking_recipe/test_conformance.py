import copy
import hashlib
import json
from pathlib import Path
import re
import unittest

from jsonschema import Draft202012Validator

import mission as mission_module
import prepare_candidate

ROOT = Path(__file__).resolve().parent
FIXTURE = json.loads((ROOT / "conformance-v1.json").read_text())
SCHEMA = json.loads((ROOT / "request-v1.schema.json").read_text())
RECIPE = json.loads((ROOT / "recipe-v1.json").read_text())
LOCK = json.loads((ROOT / "source-lock-v1.json").read_text())
VALIDATOR = Draft202012Validator(SCHEMA)
LIMITS = {axis: tuple(value) for axis, value in RECIPE["fixed_execution"]["configured_output_limits"].items()}


def apply_patch(document, operations):
    result = copy.deepcopy(document)
    for operation in operations:
        parts = [part.replace("~1", "/").replace("~0", "~") for part in operation["path"].split("/")[1:]]
        parent = result
        for part in parts[:-1]:
            parent = parent[int(part)] if isinstance(parent, list) else parent[part]
        key = parts[-1]
        if operation["op"] == "remove":
            del parent[int(key) if isinstance(parent, list) else key]
        elif isinstance(parent, list):
            parent[int(key)] = operation["value"]
        else:
            parent[key] = operation["value"]
    return result


def runtime_accepts(request) -> bool:
    try:
        prepare_candidate.validate(request["gains"], LIMITS)
        mission_module.validate_mission(request["mission"])
    except (ValueError, KeyError, TypeError):
        return False
    return True


class RaceConformanceTests(unittest.TestCase):
    def test_contract_source_hashes_are_exact(self):
        for record in FIXTURE["contract_sources"].values():
            self.assertEqual(hashlib.sha256((ROOT / record["path"]).read_bytes()).hexdigest(), record["sha256"])

    def test_schema_and_runtime_agree_with_every_case(self):
        base = FIXTURE["request"]["base"]
        for case in FIXTURE["request"]["cases"]:
            document = apply_patch(base, case["patch"])
            with self.subTest(case=case["id"], layer="schema"):
                errors = [error.message for error in VALIDATOR.iter_errors(document)]
                self.assertEqual(not errors, case["accept_schema"], errors)
            if case["accept_runtime"] is not None:
                with self.subTest(case=case["id"], layer="runtime"):
                    self.assertEqual(runtime_accepts(document), case["accept_runtime"])

    def test_schema_limits_bounds_and_modes_match_the_runtime(self):
        defs = SCHEMA["$defs"]
        for axis, (low, high) in LIMITS.items():
            properties = defs[f"axis_{axis}"]["properties"]
            self.assertEqual((properties["pid_min"]["const"], properties["pid_max"]["const"]), (low, high))
        modes = defs["gains"]["properties"]
        self.assertEqual({mode: tuple(modes[mode]["required"]) for mode in modes}, prepare_candidate.MODES)
        for mode, axes in prepare_candidate.MODES.items():
            for axis in axes:
                expected = "axis_free_limits" if (mode, axis) in prepare_candidate.FREE_LIMIT_AXES else f"axis_{axis}"
                self.assertEqual(modes[mode]["properties"][axis]["$ref"], f"#/$defs/{expected}")
        pose = defs["pose"]["properties"]
        for key, (low, high) in {**mission_module.TANK_BOUNDS_M, **mission_module.ANGLE_BOUNDS_RAD,
                                 "dwell_s": mission_module.DWELL_BOUNDS_S}.items():
            self.assertEqual((pose[key]["minimum"], pose[key]["maximum"]), (low, high))
        self.assertEqual(set(defs["pose"]["required"]), mission_module.POSE_FIELDS)
        self.assertEqual(defs["mission"]["properties"]["apriltag_tag_size"]["enum"],
                         list(mission_module.TAG_SIZE_CONVENTIONS))
        gain = defs["gain"]["allOf"][1]
        self.assertEqual(gain["maximum"], prepare_candidate.GAIN_CEILING)
        for field in ("p", "i", "d", "v"):
            self.assertEqual(RECIPE["development_bounds"][field], gain)

    def test_recipe_records_the_source_lock_and_base_image(self):
        self.assertEqual(RECIPE["source_lock_sha256"],
                         hashlib.sha256((ROOT / "source-lock-v1.json").read_bytes()).hexdigest())
        self.assertEqual(RECIPE["images"]["base"]["reference"], LOCK["target"]["base_image"])
        for source in LOCK["sources"]:
            self.assertEqual(RECIPE["fixed_source"][source["name"]], source["commit"])
        self.assertEqual([s["commit"] for s in RECIPE["overlay_sources"]], [s["commit"] for s in LOCK["sources"]])

    def test_dockerfile_fetches_exactly_the_locked_archives(self):
        dockerfile = (ROOT / "Dockerfile").read_text()
        default_base = re.search(r"^ARG BASE_IMAGE=(\S+)$", dockerfile, re.M).group(1)
        self.assertEqual(default_base, LOCK["target"]["base_image"])
        fetched = dict(re.findall(r"codeload\.github\.com/[^/]+/[^/]+/tar\.gz/([0-9a-f]{40})\s*\\?\s*"
                                  r"(?:-o \S+ && \\\s*echo ')?([0-9a-f]{64})", dockerfile))
        self.assertEqual(fetched, {source["commit"]: source["archive_sha256"] for source in LOCK["sources"]})
        for source in LOCK["sources"]:
            self.assertIn(f"{source['repository'].replace('https://github.com', 'https://codeload.github.com')}"
                          f"/tar.gz/{source['commit']}", dockerfile)

    def test_entrypoint_is_executor_compatible(self):
        entrypoint = RECIPE["fixed_execution"]["entrypoint"]
        self.assertRegex(entrypoint, r"^/opt/piccard-[a-z0-9][a-z0-9-]{0,62}/runtime/run_campaign_trial\.sh$")
        self.assertIn(f'ENTRYPOINT ["{entrypoint}"]', (ROOT / "Dockerfile").read_text())
        self.assertTrue((ROOT / "runtime/run_campaign_trial.sh").is_file())


if __name__ == "__main__":
    unittest.main()
