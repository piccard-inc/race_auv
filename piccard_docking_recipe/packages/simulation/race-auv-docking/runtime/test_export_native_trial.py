"""export_native_trial.py and run_native_trial.sh's export step (physical-ai#266), offline: a fake AWS CLI that checks
the SHA-256 on put as S3 does. No network, no AWS."""
import base64
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import uuid

import export_native_trial as ent

HERE = Path(__file__).resolve().parent
NATIVE = (HERE / "run_native_trial.sh").read_text()
ENV = {ent.PROFILE_ENV: "native-runs-writer", ent.ORGANIZATION_ENV: "piccard", ent.PROJECT_ENV: "race-auv-docking"}
PREFIX = "native-runs/piccard/race-auv-docking/trial-7/"


def b64(data: bytes) -> str:
    return base64.b64encode(hashlib.sha256(data).digest()).decode()


class FakeAws:
    """put-object stores the body when its --checksum-sha256 matches (S3's check on write); head-object returns it."""

    def __init__(self, profile, *, put_error=(), not_stored=(), length_off=(), checksum_off=(), missing_status=403):
        self.profile, self.calls, self.store, self.missing_status = profile, [], {}, missing_status
        self.put_error, self.not_stored = set(put_error), set(not_stored)
        self.length_off, self.checksum_off = set(length_off), set(checksum_off)

    def call(self, argv, timeout):
        self.calls.append((argv, timeout))
        key = argv[argv.index("--key") + 1]
        name = key.rsplit("/", 1)[-1]
        if argv[2] == "put-object":
            if name in self.put_error:
                return 254, "", "An error occurred (AccessDenied) when calling the PutObject operation"
            data = Path(argv[argv.index("--body") + 1]).read_bytes()
            checksum = argv[argv.index("--checksum-sha256") + 1]
            if b64(data) != checksum:
                return 254, "", "An error occurred (BadDigest)"
            if name not in self.not_stored:
                self.store[key] = (len(data), checksum)
            return 0, json.dumps({"ChecksumSHA256": checksum}), ""
        if argv[2] == "head-object":
            if key not in self.store:  # without ListBucket S3 answers a missing key with 403
                reason = "Forbidden" if self.missing_status == 403 else "Not Found"
                return 254, "", f"An error occurred ({self.missing_status}) when calling the HeadObject operation: {reason}"
            length, checksum = self.store[key]
            length += name in self.length_off
            checksum = b64(b"other") if name in self.checksum_off else checksum
            return 0, json.dumps({"ContentLength": length, "ChecksumSHA256": checksum}), ""
        raise AssertionError(f"unexpected call {argv}")


def make_trial(root: Path, name="trial-7") -> Path:
    trial = root / name
    (trial / "scored").mkdir(parents=True)
    (trial / "request").mkdir()
    (trial / "trial.json").write_text('{"outcome": "completed"}\n')
    (trial / "telemetry.jsonl").write_bytes(b'{"kind": "event"}\n' * 50)
    (trial / "scored" / "trial-7.m3.json").write_text('{"docked": true}\n')
    (trial / "request" / "limits.env").write_text("HORIZON_SECONDS=1800\n")
    return trial


def tree(trial: Path) -> dict:
    return {p.relative_to(trial).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in trial.rglob("*") if p.is_file() and not p.is_symlink()}


class Run:
    """main() with a fake client, capturing the printed lines."""

    def __init__(self, trial, argv=(), environ=None, size_of=None, **fake):
        self.fakes = []

        def factory(profile):
            self.fakes.append(FakeAws(profile, **fake))
            return self.fakes[-1]

        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            self.code = ent.main(["--trial-dir", str(trial), *argv], environ=ENV if environ is None else environ,
                                 client_factory=factory, size_of=size_of)
        self.lines, self.stderr = out.getvalue().splitlines(), err.getvalue()
        record = trial / ent.RECORD
        self.record = json.loads(record.read_text()) if record.is_file() else None
        self.calls = [argv for fake in self.fakes for argv, _ in fake.calls]


class Base(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name).resolve()
        self.trial = make_trial(self.root)


class WrittenManifestTests(Base):
    def test_verified(self):
        before = tree(self.trial)
        run = Run(self.trial)
        self.assertEqual(run.code, 0)
        self.assertEqual(run.lines[-1], f"UPLOAD verified {ent.DEFAULT_BUCKET}/{PREFIX}")
        self.assertEqual(run.record["status"], "verified")
        self.assertEqual(run.record["counts"], {"files": 5, "verified": 5, "failed": 0})
        self.assertEqual(run.fakes[0].profile, "native-runs-writer")
        sums = (self.trial / "SHA256SUMS").read_text()
        self.assertEqual(sums, "".join(f"{digest}  ./{path}\n" for path, digest in sorted(before.items())))
        keys = [argv[argv.index("--key") + 1] for argv in run.calls if argv[2] == "put-object"]
        self.assertEqual(keys, [PREFIX + path for path in sorted(before)] + [PREFIX + "SHA256SUMS"])
        self.assertEqual(run.record["manifest"]["sha256"], hashlib.sha256(sums.encode()).hexdigest())

    def test_the_trial_files_stay_byte_identical_and_only_two_files_are_added(self):
        for fake in ({}, {"put_error": {"trial.json"}}, {"checksum_off": {"telemetry.jsonl"}}):
            with self.subTest(fake=fake):
                trial = make_trial(self.root, f"t-{len(list(self.root.iterdir()))}")
                before = tree(trial)
                Run(trial, ["--run-id", "trial-7"], **fake)
                after = tree(trial)
                self.assertEqual({path: after[path] for path in before}, before)
                self.assertEqual(set(after) - set(before), {"SHA256SUMS", "upload.json"})
                self.assertNotIn("upload.json", (trial / "SHA256SUMS").read_text())

    def test_the_exact_calls_and_no_other_verb(self):
        run = Run(self.trial)
        body = self.trial / "trial.json"
        checksum = b64(body.read_bytes())
        key = PREFIX + "trial.json"
        self.assertIn(["aws", "s3api", "put-object", "--region", "eu-north-1", "--bucket", ent.DEFAULT_BUCKET,
                       "--key", key, "--body", str(body), "--checksum-algorithm", "SHA256", "--checksum-sha256",
                       checksum, "--output", "json"], run.calls)
        self.assertIn(["aws", "s3api", "head-object", "--region", "eu-north-1", "--bucket", ent.DEFAULT_BUCKET,
                       "--key", key, "--checksum-mode", "ENABLED", "--output", "json"], run.calls)
        for argv in run.calls:
            self.assertEqual(argv[argv.index("--region") + 1], "eu-north-1")
        self.assertEqual({tuple(argv[:3]) for argv in run.calls},
                         {("aws", "s3api", "put-object"), ("aws", "s3api", "head-object")})
        for argv in run.calls:
            self.assertTrue(argv[argv.index("--key") + 1].startswith(PREFIX))
            for word in ("delete", "list", "sync", "cp", "--recursive", "rm"):
                self.assertNotIn(word, argv)

    def test_a_put_is_followed_by_its_head_one_file_at_a_time(self):
        run = Run(self.trial)
        verbs = [argv[2] for argv in run.calls]
        self.assertEqual(verbs, ["put-object", "head-object"] * 5)

    def test_an_existing_sha256sums_is_refused_never_rewritten(self):
        (self.trial / "SHA256SUMS").write_text("x\n")
        run = Run(self.trial)
        self.assertEqual(run.code, 3)
        self.assertEqual((self.trial / "SHA256SUMS").read_text(), "x\n")
        self.assertEqual(run.calls, [])
        self.assertRegex(run.record["reasons"][0], "already exists")
        self.assertEqual(run.lines[-1], f"UPLOAD failed {ent.DEFAULT_BUCKET}/{PREFIX}")

    def test_a_symbolic_link_is_neither_followed_nor_listed(self):
        (self.trial / "link.json").symlink_to(self.trial / "trial.json")
        (self.trial / "linked-dir").symlink_to(self.trial / "scored")
        run = Run(self.trial)
        self.assertEqual(run.record["manifest"]["not_listed"], ["link.json", "linked-dir"])
        self.assertNotIn("link", (self.trial / "SHA256SUMS").read_text())
        self.assertEqual(run.record["status"], "verified")


class FailureTests(Base):
    def check_failed(self, run, name, pattern):
        self.assertEqual(run.code, 3)
        self.assertEqual(run.record["status"], "failed")
        self.assertTrue(run.lines[-1].startswith(f"UPLOAD failed {ent.DEFAULT_BUCKET}/"))
        bad = [item for item in run.record["files"] if item["result"] != "verified"]
        self.assertEqual([item["path"] for item in bad], [name])
        self.assertRegex(" ".join(bad[0]["reasons"]), pattern)
        self.assertEqual(run.record["counts"], {"files": 5, "verified": 4, "failed": 1})

    def test_a_size_mismatch(self):
        self.check_failed(Run(self.trial, length_off={"trial.json"}), "trial.json", "ContentLength")

    def test_a_checksum_mismatch(self):
        self.check_failed(Run(self.trial, checksum_off={"telemetry.jsonl"}), "telemetry.jsonl", "ChecksumSHA256")

    def test_an_object_missing_on_head_reads_as_403_without_list_bucket(self):
        run = Run(self.trial, not_stored={"limits.env"})
        self.check_failed(run, "request/limits.env", "missing or forbidden.*\\(403\\)")
        item = next(item for item in run.record["files"] if item["path"] == "request/limits.env")
        self.assertTrue(item["head"]["missing_or_forbidden"])

    def test_a_404_on_head_is_the_same_failure(self):
        self.check_failed(Run(self.trial, not_stored={"limits.env"}, missing_status=404), "request/limits.env",
                          "missing or forbidden.*\\(404\\)")

    def test_the_region_comes_from_the_argument_or_the_environment(self):
        run = Run(self.trial, ["--region", "eu-west-1"])
        self.assertEqual({argv[argv.index("--region") + 1] for argv in run.calls}, {"eu-west-1"})
        run = Run(make_trial(self.root, "r-2"), ["--run-id", "trial-7"], environ={**ENV, ent.REGION_ENV: "us-east-2"})
        self.assertEqual({argv[argv.index("--region") + 1] for argv in run.calls}, {"us-east-2"})
        run = Run(make_trial(self.root, "r-3"), ["--region", "nowhere"])
        self.assertEqual(run.calls, [])
        self.assertIn("region 'nowhere' is not a region name", run.record["reasons"])

    def test_a_put_error_skips_that_files_head(self):
        run = Run(self.trial, put_error={"trial.json"})
        self.check_failed(run, "trial.json", "put-object exited 254")
        heads = [argv for argv in run.calls if argv[2] == "head-object"]
        self.assertNotIn(PREFIX + "trial.json", [argv[argv.index("--key") + 1] for argv in heads])

    def test_a_file_over_five_gib_is_refused_never_sent(self):
        def size_of(path):
            return ent.SINGLE_PART_MAX_BYTES + 1 if path.name == "telemetry.jsonl" else path.stat().st_size

        run = Run(self.trial, size_of=size_of)
        self.check_failed(run, "telemetry.jsonl", "over the single-part limit")
        self.assertNotIn(PREFIX + "telemetry.jsonl", [argv[argv.index("--key") + 1] for argv in run.calls])

    def test_the_overall_bound_stops_further_calls(self):
        clock = iter([0.0] + [ent.OVERALL_BOUND_S + 1.0] * 50)
        fake = FakeAws("p")
        entries = [{"path": "trial.json", "sha256": hashlib.sha256((self.trial / "trial.json").read_bytes()).hexdigest()}]
        results = ent.upload(self.trial, entries, ent.DEFAULT_BUCKET, PREFIX, fake, monotonic=lambda: next(clock))
        self.assertEqual(fake.calls, [])
        self.assertRegex(results[0]["reasons"][0], "overall bound")


class SkippedTests(Base):
    def test_no_profile_calls_nothing_and_still_writes_the_manifest(self):
        environ = {key: value for key, value in ENV.items() if key != ent.PROFILE_ENV}
        run = Run(self.trial, environ=environ)
        self.assertEqual(run.fakes, [])
        self.assertEqual(run.code, 0)
        self.assertEqual(run.lines[-1], f"UPLOAD skipped(no profile) {ent.DEFAULT_BUCKET}/{PREFIX}")
        self.assertEqual(run.record["status"], "skipped(no profile)")
        self.assertTrue((self.trial / "SHA256SUMS").is_file())

    def test_no_profile_and_no_prefix_prints_a_dash(self):
        run = Run(self.trial, environ={})
        self.assertEqual(run.lines[-1], "UPLOAD skipped(no profile) -")
        self.assertEqual(run.fakes, [])


class SegmentTests(Base):
    def test_invalid_segments_call_nothing(self):
        cases = {"organization with a slash": ["--organization", "a/b"],
                 "dot-dot": ["--project", "a..b"],
                 "a leading dot": ["--project", ".hidden"],
                 "a job UUID as run id": ["--run-id", str(uuid.uuid4())],
                 "a bucket name": ["--bucket", "Not_A_Bucket"]}
        for name, argv in cases.items():
            with self.subTest(name):
                trial = make_trial(self.root, f"s-{len(list(self.root.iterdir()))}")
                run = Run(trial, argv)
                self.assertEqual(run.calls, [])
                self.assertEqual(run.code, 3)
                self.assertTrue(run.lines[-1].startswith("UPLOAD failed"))

    def test_a_missing_organization_with_a_profile_fails(self):
        run = Run(self.trial, environ={ent.PROFILE_ENV: "p", ent.PROJECT_ENV: "x"})
        self.assertEqual(run.calls, [])
        self.assertEqual(run.lines[-1], "UPLOAD failed -")
        self.assertIn("organization is not set", run.record["reasons"])

    def test_a_profile_that_is_not_a_simple_name_fails(self):
        run = Run(self.trial, environ={**ENV, ent.PROFILE_ENV: "x y"})
        self.assertEqual(run.calls, [])
        self.assertEqual(run.code, 3)

    def test_listed_paths(self):
        for bad in ("/abs", "a/../b", "a//b", "./x", "a\\b", "a\tb", ""):
            with self.subTest(bad):
                self.assertIsNotNone(ent.path_problem(bad))
        for good in ("trial.json", "scored/trial-7.m3.json", "kit/Isaac-Sim Python/5.0/kit.log"):
            self.assertIsNone(ent.path_problem(good))

    def test_the_run_id_defaults_to_the_directorys_name(self):
        run = Run(self.trial)
        self.assertEqual(run.record["prefix"], PREFIX)
        run = Run(make_trial(self.root, "m3-revisit-1"), environ={**ENV, ent.RUN_ID_ENV: "from-env"})
        self.assertEqual(run.record["prefix"], "native-runs/piccard/race-auv-docking/from-env/")

    def test_the_profile_is_the_only_aws_setting_but_the_configs_location(self):
        with mock.patch.dict(os.environ, {"AWS_PROFILE": "other", "AWS_DEFAULT_PROFILE": "other",
                                          "AWS_ANY_STATIC_CREDENTIAL": "x", "AWS_REGION": "us-east-2",
                                          "AWS_CONFIG_FILE": "/etc/aws-config", "HOME": "/root"}):
            env = ent.AwsCli("native-runs-writer").env
        self.assertEqual(env["AWS_PROFILE"], "native-runs-writer")
        self.assertEqual(env["AWS_PAGER"], "")
        self.assertEqual(env["HOME"], "/root")
        self.assertEqual({key for key in env if key.startswith("AWS_")}, {"AWS_PROFILE", "AWS_PAGER", "AWS_CONFIG_FILE"})


class ExistingManifestTests(Base):
    def own_sums(self, listed=("trial.json", "scored/trial-7.m3.json")):
        text = "".join(f"{hashlib.sha256((self.trial / p).read_bytes()).hexdigest()}  ./{p}\n" for p in listed)
        (self.trial / "SHA256SUMS").write_text(text)
        return text

    def test_a_matching_manifest_is_uploaded_as_it_is(self):
        text = self.own_sums()
        run = Run(self.trial, ["--existing-sha256sums"])
        self.assertEqual(run.code, 0, run.stderr)
        self.assertEqual((self.trial / "SHA256SUMS").read_text(), text)
        keys = [argv[argv.index("--key") + 1] for argv in run.calls if argv[2] == "put-object"]
        self.assertEqual(keys, [PREFIX + "trial.json", PREFIX + "scored/trial-7.m3.json", PREFIX + "SHA256SUMS"])
        self.assertEqual(run.record["manifest"]["mode"], "existing")

    def test_a_mismatch_refuses_everything(self):
        text = self.own_sums()
        (self.trial / "trial.json").write_text("changed\n")
        run = Run(self.trial, ["--existing-sha256sums"])
        self.assertEqual(run.calls, [])
        self.assertEqual(run.code, 3)
        self.assertEqual((self.trial / "SHA256SUMS").read_text(), text)
        self.assertRegex(run.record["reasons"][0], "trial.json: SHA-256 differs")

    def test_a_listed_file_that_is_missing_refuses_everything(self):
        self.own_sums()
        (self.trial / "trial.json").unlink()
        run = Run(self.trial, ["--existing-sha256sums"])
        self.assertEqual(run.calls, [])
        self.assertRegex(run.record["reasons"][0], "not a regular file")

    def test_no_manifest_and_a_manifest_that_lists_the_record(self):
        run = Run(self.trial, ["--existing-sha256sums"])
        self.assertRegex(run.record["reasons"][0], "missing")
        (self.trial / "upload.json").write_text("{}")
        self.own_sums(("trial.json", "upload.json"))
        with contextlib.redirect_stdout(io.StringIO()) as out, contextlib.redirect_stderr(io.StringIO()):
            code = ent.main(["--trial-dir", str(self.trial), "--existing-sha256sums"], environ=ENV,
                            client_factory=FakeAws)
        self.assertEqual(code, 3)
        self.assertEqual(out.getvalue().splitlines()[-1], f"UPLOAD failed {ent.DEFAULT_BUCKET}/{PREFIX}")
        self.assertEqual((self.trial / "upload.json").read_text(), "{}")  # listed, so never overwritten


class FinalLineTests(Base):
    def test_an_exception_inside_the_exporter_still_ends_with_the_line(self):
        def broken(profile):
            raise RuntimeError("boom")

        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = ent.main(["--trial-dir", str(self.trial)], environ=ENV, client_factory=broken)
        self.assertEqual(code, 3)
        self.assertEqual(out.getvalue().splitlines()[-1], "UPLOAD failed -")
        self.assertIn("RuntimeError: boom", err.getvalue())

    def test_a_missing_directory(self):
        with contextlib.redirect_stdout(io.StringIO()) as out, contextlib.redirect_stderr(io.StringIO()):
            code = ent.main(["--trial-dir", str(self.root / "absent")], environ=ENV, client_factory=FakeAws)
        self.assertEqual((code, out.getvalue().splitlines()[-1]), (3, "UPLOAD failed -"))


class RunnerTests(unittest.TestCase):
    """run_native_trial.sh's export step, run as written with a stand-in exporter: the exit status stays the trial's."""

    TAIL = NATIVE[NATIVE.index("export_trial() {"):]
    STUBS = {"verified": 'print("UPLOAD verified b/p/")',
             "failed": 'print("UPLOAD failed b/p/"); raise SystemExit(3)',
             "raises": 'raise RuntimeError("boom")',
             "usage": "raise SystemExit(2)",
             "killed": "import os, signal; os.kill(os.getpid(), signal.SIGKILL)"}

    def run_tail(self, stub: str | None, status: int, real=False):
        with tempfile.TemporaryDirectory() as tmp:
            scripts, output = Path(tmp) / "runtime", Path(tmp) / "trial-7"
            scripts.mkdir()
            make_trial(Path(tmp))
            if real:
                shutil.copy(HERE / "export_native_trial.py", scripts)
            elif stub is not None:
                (scripts / "export_native_trial.py").write_text(stub + "\n")
            script = (f'set -eo pipefail\nscript_dir="{scripts}"\noutput="{output}"\nrun_id=""\norganization=""\n'
                      f'project=""\nstatus={status}\n{self.TAIL}')
            env = {key: value for key, value in os.environ.items() if not key.startswith("PICCARD_NATIVE_RUNS_")}
            return subprocess.run(["bash", "-c", script], capture_output=True, text=True, timeout=60, env=env)

    def test_the_exit_status_and_the_last_line_in_every_outcome(self):
        for status in (0, 1, 3):
            for name, stub in [*self.STUBS.items(), ("absent", None)]:
                with self.subTest(status=status, outcome=name):
                    result = self.run_tail(stub, status)
                    self.assertEqual(result.returncode, status, result.stderr)
                    last = result.stdout.splitlines()[-1]
                    self.assertTrue(last.startswith("UPLOAD "), result.stdout)
                    if name in ("raises", "usage", "killed", "absent"):
                        self.assertRegex(last, r"^UPLOAD failed - \(export_native_trial.py exited \d+\)$")

    def test_the_real_exporter_without_a_profile(self):
        result = self.run_tail(None, 1, real=True)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(result.stdout.splitlines()[-1], "UPLOAD skipped(no profile) -")

    def test_the_export_runs_last_outside_set_e(self):
        steps = ['python3 "$metrics"', "export_trial() {", 'export_native_trial.py" --trial-dir "$output"',
                 "set +e\nexport_trial\nset -e\n", 'exit "$status"']
        positions = [NATIVE.index(step) for step in steps]
        self.assertEqual(positions, sorted(positions))
        self.assertTrue(NATIVE.rstrip().endswith('exit "$status"'))


if __name__ == "__main__":
    unittest.main()
