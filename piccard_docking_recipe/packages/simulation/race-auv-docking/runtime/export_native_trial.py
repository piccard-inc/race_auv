#!/usr/bin/env python3
"""A native trial's export: SHA256SUMS over the trial directory, then its upload to the operator's artifact bucket
(physical-ai#266, piccard-api#306). Standard library only; the upload goes through the AWS CLI v2.

  export_native_trial.py --trial-dir DIR [--run-id R] [--organization O] [--project P] [--bucket B] [--region R]
                         [--existing-sha256sums]

run_native_trial.sh calls it last, after the metrics. It also runs on its own on a copied-down trial directory.

1. The manifest.
   - Written (the default): SHA256SUMS lists, as `sha256sum` writes them (`<hex>  ./<path>`, sorted by path), every
     regular file under DIR except, at its top, SHA256SUMS, upload.json and their `.<name>.tmp` while being written.
     Symbolic links and other non-regular entries are not followed or listed; the record names them. A DIR that already holds SHA256SUMS is refused, never
     rewritten. The trial's own files are only read.
   - Existing (--existing-sha256sums): DIR's SHA256SUMS is used as it is, never rewritten. Every file it lists must be
     a regular file under DIR with that SHA-256, or nothing is uploaded.
2. The upload, only with a profile in PICCARD_NATIVE_RUNS_AWS_PROFILE (a named profile, never a key; without one the
   status is `skipped(no profile)` and the AWS CLI is never called). Every listed file, then SHA256SUMS itself, goes to
   the key native-runs/<organization>/<project>/<run-id>/<path> in the bucket by `aws s3api put-object` with its SHA-256
   (`--checksum-algorithm SHA256 --checksum-sha256`), so S3 checks it on write. That is one PutObject call: a file
   over 5 GiB is refused, never sent in parts, since a multipart checksum is not the file's SHA-256. Each object is
   then read back by `aws s3api head-object --checksum-mode ENABLED`: its ContentLength must be the file's size and
   its ChecksumSHA256 the listed SHA-256. Nothing is listed, deleted or read outside the prefix. The bucket and its
   region have no defaults: they come from --bucket or PICCARD_NATIVE_RUNS_BUCKET and --region or
   PICCARD_NATIVE_RUNS_REGION, set by the operator, and every call names that region explicitly, never the profile's.
   With a profile but no valid bucket or region the status is `failed` (bucket/region not configured) and the AWS CLI
   is never called. The operator's role grants PutObject and GetObject only, no ListBucket, so S3 answers a read-back of a
   missing object with 403, not 404: either is recorded as that file missing or forbidden.
3. The record: upload.json at DIR's top (outside SHA256SUMS), and a last line
   `UPLOAD <verified|failed|skipped(no profile)> <bucket>/<prefix>`, or `-` in place of the prefix when it is not known.

The status is `verified` only when every object read back matches. The exit status is 0 for verified or skipped and
3 for failed (2 for a usage error). run_native_trial.sh never lets it change the trial's own exit status, and an
upload that fails never changes the export.
"""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

SCHEMA = "piccard.race-auv.native-upload/v1"
SUMS = "SHA256SUMS"
RECORD = "upload.json"
PREFIX_ROOT = "native-runs"
PROFILE_ENV = "PICCARD_NATIVE_RUNS_AWS_PROFILE"
BUCKET_ENV = "PICCARD_NATIVE_RUNS_BUCKET"
REGION_ENV = "PICCARD_NATIVE_RUNS_REGION"
ORGANIZATION_ENV = "PICCARD_NATIVE_RUNS_ORGANIZATION"
PROJECT_ENV = "PICCARD_NATIVE_RUNS_PROJECT"
RUN_ID_ENV = "PICCARD_NATIVE_RUNS_RUN_ID"
VERIFIED, FAILED, SKIPPED = "verified", "failed", "skipped(no profile)"
EXIT = {VERIFIED: 0, SKIPPED: 0, FAILED: 3}
SINGLE_PART_MAX_BYTES = 5 * 1024 ** 3  # PutObject's limit
PUT_TIMEOUT_S = 600
HEAD_TIMEOUT_S = 60
OVERALL_BOUND_S = 1800
# The profile is a name in the operator's AWS config. For the calls, every other AWS_ variable is dropped (static
# credentials included, which would otherwise take precedence over the profile), but for where the config lives.
AWS_ENV_KEPT = ("AWS_CONFIG_FILE", "AWS_SHARED_CREDENTIALS_FILE", "AWS_CA_BUNDLE")
PROFILE_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}")
SEGMENT_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")
UUID_RE = re.compile(r"[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}")
BUCKET_RE = re.compile(r"[a-z0-9][a-z0-9.-]{1,61}[a-z0-9]")
REGION_RE = re.compile(r"[a-z]{2}(?:-[a-z]+)+-[0-9]")
# How the CLI reports a 403 or a 404 from HeadObject.
MISSING_OR_FORBIDDEN_RE = re.compile(r"\((?:403|404)\)|Forbidden|Not Found")
SUMS_LINE_RE = re.compile(r"([0-9a-f]{64}) [ *](.+)")


class ManifestError(Exception):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def checksum_base64(hex_digest: str) -> str:
    """The SHA-256 as S3 writes ChecksumSHA256: the 32 bytes, base64."""
    return base64.b64encode(bytes.fromhex(hex_digest)).decode()


def segment_problem(name: str, value) -> str | None:
    """Why value cannot be one segment of the prefix, or None. A run id never takes a job's UUID."""
    if not isinstance(value, str) or not value:
        return f"{name} is not set"
    if not SEGMENT_RE.fullmatch(value) or ".." in value:
        return f"{name} {value!r} is not one safe path segment ([A-Za-z0-9][A-Za-z0-9._-]*, no '..')"
    if name == "run id" and UUID_RE.search(value):
        return "run id looks like a job UUID; use the trial's own label"
    return None


def path_problem(path: str) -> str | None:
    """Why a listed path cannot be a key under the prefix, or None."""
    parts = path.split("/")
    if not path or path.startswith("/") or "\\" in path:
        return f"{path!r} is not a relative POSIX path"
    if any(part in ("", ".", "..") for part in parts):
        return f"{path!r} has an empty, '.' or '..' component"
    if any(ord(char) < 32 or ord(char) == 127 for char in path):
        return f"{path!r} has a control character"
    if len(path.encode()) > 900:
        return f"{path!r} is too long for a key under the prefix"
    return None


def manifest_path(line_path: str) -> str:
    return line_path[2:] if line_path.startswith("./") else line_path


def write_manifest(trial_dir: Path) -> tuple[list[dict], list[str]]:
    """Write SHA256SUMS over trial_dir (rule in the module docstring); return its entries and what was not listed."""
    sums = trial_dir / SUMS
    if sums.exists() or sums.is_symlink():
        raise ManifestError(f"{SUMS} already exists in the trial directory; it is never rewritten "
                            "(use --existing-sha256sums to upload it as it is)")
    entries, unlisted = [], []
    for root, directories, files in os.walk(trial_dir, followlinks=False):
        directories.sort()
        here = Path(root)
        for name in sorted(files) + [d for d in directories if (here / d).is_symlink()]:
            path = here / name
            relative = path.relative_to(trial_dir).as_posix()
            if here == trial_dir and name in (SUMS, RECORD, f".{SUMS}.tmp", f".{RECORD}.tmp"):
                continue
            if path.is_symlink() or not path.is_file():
                unlisted.append(relative)
                continue
            problem = path_problem(relative)
            if problem:
                raise ManifestError(problem)
            entries.append({"path": relative, "sha256": sha256_file(path)})
    entries.sort(key=lambda entry: entry["path"])
    text = "".join(f"{entry['sha256']}  ./{entry['path']}\n" for entry in entries)
    temporary = trial_dir / f".{SUMS}.tmp"
    temporary.write_text(text)
    os.replace(temporary, sums)
    return entries, sorted(unlisted)


def read_manifest(trial_dir: Path) -> list[dict]:
    """DIR's own SHA256SUMS, every listed file checked against it; raises on any difference."""
    sums = trial_dir / SUMS
    if sums.is_symlink() or not sums.is_file():
        raise ManifestError(f"{SUMS} is missing from the trial directory")
    entries, seen, problems = [], set(), []
    for number, line in enumerate(sums.read_text().splitlines(), 1):
        if not line.strip():
            continue
        match = SUMS_LINE_RE.fullmatch(line)
        if not match:
            raise ManifestError(f"{SUMS} line {number} is not `<sha256>  <path>`")
        relative = manifest_path(match.group(2))
        problem = path_problem(relative)
        if problem:
            raise ManifestError(f"{SUMS} line {number}: {problem}")
        if relative in seen or relative in (SUMS, RECORD):
            raise ManifestError(f"{SUMS} line {number}: {relative!r} cannot be listed here")
        seen.add(relative)
        path = trial_dir / relative
        if path.is_symlink() or not path.is_file():
            problems.append(f"{relative}: listed but not a regular file in the trial directory")
        elif sha256_file(path) != match.group(1):
            problems.append(f"{relative}: SHA-256 differs from {SUMS}")
        entries.append({"path": relative, "sha256": match.group(1)})
    if problems:
        raise ManifestError("; ".join(problems))
    return entries


class AwsCli:
    """The AWS CLI v2 under the operator's named profile. Tests replace call()."""

    def __init__(self, profile: str):
        self.env = {key: value for key, value in os.environ.items()
                    if not key.startswith("AWS_") or key in AWS_ENV_KEPT}
        self.env.update(AWS_PROFILE=profile, AWS_PAGER="")

    def call(self, argv: list[str], timeout: float) -> tuple[int, str, str]:
        try:
            proc = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, env=self.env)
        except subprocess.TimeoutExpired:
            return 124, "", f"timed out after {timeout:.0f} s"
        except OSError as error:
            return 127, "", str(error)
        return proc.returncode, proc.stdout, proc.stderr


def put_argv(region: str, bucket: str, key: str, path: Path, sha256: str) -> list[str]:
    return ["aws", "s3api", "put-object", "--region", region, "--bucket", bucket, "--key", key, "--body", str(path),
            "--checksum-algorithm", "SHA256", "--checksum-sha256", checksum_base64(sha256), "--output", "json"]


def head_argv(region: str, bucket: str, key: str) -> list[str]:
    return ["aws", "s3api", "head-object", "--region", region, "--bucket", bucket, "--key", key,
            "--checksum-mode", "ENABLED", "--output", "json"]


def upload(trial_dir: Path, entries: list[dict], bucket: str, prefix: str, client, *, region: str,
           size_of=None, monotonic=time.monotonic) -> list[dict]:
    """Put then read back each file; one result per file, in the manifest's order with SHA256SUMS last."""
    size_of = size_of or (lambda path: path.stat().st_size)
    deadline = monotonic() + OVERALL_BOUND_S
    results = []
    for entry in entries:
        path = trial_dir / entry["path"]
        key = f"{prefix}{entry['path']}"
        size = size_of(path)
        item = {"path": entry["path"], "key": key, "size": size, "sha256": entry["sha256"], "put": None,
                "head": None, "result": FAILED, "reasons": []}
        results.append(item)
        if size > SINGLE_PART_MAX_BYTES:
            item["reasons"].append(f"refused: {size} bytes is over the single-part limit of {SINGLE_PART_MAX_BYTES}")
            continue
        for name, argv, bound in (("put", put_argv(region, bucket, key, path, entry["sha256"]), PUT_TIMEOUT_S),
                                  ("head", head_argv(region, bucket, key), HEAD_TIMEOUT_S)):
            left = deadline - monotonic()
            if left <= 0:
                item["reasons"].append(f"{name} not attempted: the overall bound of {OVERALL_BOUND_S} s was reached")
                break
            code, stdout, stderr = client.call(argv, min(bound, left))
            item[name] = {"exit": code}
            if code != 0:
                if name == "head" and MISSING_OR_FORBIDDEN_RE.search(stderr):
                    item["head"]["missing_or_forbidden"] = True
                    item["reasons"].append("head-object: missing or forbidden (403 or 404; the role has no "
                                           f"ListBucket, so a missing object reads as 403): {stderr.strip()[-300:]}")
                else:
                    item["reasons"].append(f"{name}-object exited {code}: {stderr.strip()[-300:]}")
                break
            if name == "head":
                try:
                    reply = json.loads(stdout)
                    length, checksum = int(reply["ContentLength"]), reply.get("ChecksumSHA256")
                except (ValueError, KeyError, TypeError) as error:
                    item["reasons"].append(f"head-object reply unreadable: {error!r}")
                    break
                item["head"].update(content_length=length, checksum_sha256=checksum)
                if length != size:
                    item["reasons"].append(f"ContentLength {length} is not the file's {size} bytes")
                if checksum != checksum_base64(entry["sha256"]):
                    item["reasons"].append(f"ChecksumSHA256 {checksum!r} is not the listed SHA-256")
        if not item["reasons"]:
            item["result"] = VERIFIED
    return results


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def export(trial_dir: Path, *, existing: bool, run_id, organization, project, bucket, region,
           environ=None, client_factory=AwsCli, size_of=None, monotonic=time.monotonic) -> dict:
    """The whole step; returns the record (also written to upload.json). Never raises for an upload problem."""
    environ = os.environ if environ is None else environ
    profile = environ.get(PROFILE_ENV) or None
    record = {"schema": SCHEMA, "status": FAILED, "started_utc": utc_now(), "finished_utc": None,
              "trial_dir": trial_dir.name, "manifest": {"mode": "existing" if existing else "written", "path": SUMS,
                                                        "files": None, "sha256": None, "not_listed": []},
              "profile_env": PROFILE_ENV, "profile": profile, "bucket": bucket, "region": region, "prefix": None,
              "bounds": {"single_part_max_bytes": SINGLE_PART_MAX_BYTES, "put_timeout_s": PUT_TIMEOUT_S,
                         "head_timeout_s": HEAD_TIMEOUT_S, "overall_s": OVERALL_BOUND_S},
              "counts": {"files": 0, "verified": 0, "failed": 0}, "files": [], "reasons": []}
    problems = [problem for problem in (segment_problem("organization", organization),
                                        segment_problem("project", project), segment_problem("run id", run_id))
                if problem]
    # No defaults (the Claude CTO's ruling: no bucket name or region on a public surface); the operator sets both.
    if not bucket:
        problems.append(f"bucket/region not configured: no bucket (--bucket or {BUCKET_ENV})")
    elif not isinstance(bucket, str) or not BUCKET_RE.fullmatch(bucket):
        problems.append(f"bucket/region not configured: {bucket!r} is not a bucket name")
    if not region:
        problems.append(f"bucket/region not configured: no region (--region or {REGION_ENV})")
    elif not isinstance(region, str) or not REGION_RE.fullmatch(region):
        problems.append(f"bucket/region not configured: {region!r} is not a region name")
    if not problems:
        record["prefix"] = f"{PREFIX_ROOT}/{organization}/{project}/{run_id}/"
    try:
        if existing:
            entries = read_manifest(trial_dir)
        else:
            entries, record["manifest"]["not_listed"] = write_manifest(trial_dir)
        record["manifest"].update(files=len(entries), sha256=sha256_file(trial_dir / SUMS))
    except (ManifestError, OSError) as error:
        record["reasons"].append(f"manifest: {error}")
        return record
    if profile is None:
        record["status"] = SKIPPED
        return record
    if not PROFILE_RE.fullmatch(profile):
        problems.append(f"{PROFILE_ENV} is not a simple named profile")
    if problems:
        record["reasons"].extend(problems)
        return record
    entries = [*entries, {"path": SUMS, "sha256": record["manifest"]["sha256"]}]
    record["files"] = upload(trial_dir, entries, bucket, record["prefix"], client_factory(profile), region=region,
                             size_of=size_of, monotonic=monotonic)
    verified = sum(item["result"] == VERIFIED for item in record["files"])
    record["counts"] = {"files": len(entries), "verified": verified, "failed": len(entries) - verified}
    record["status"] = VERIFIED if verified == len(entries) else FAILED
    if record["status"] == FAILED:
        record["reasons"].append(f"{len(entries) - verified} of {len(entries)} objects not verified")
    return record


def write_record(trial_dir: Path, record: dict) -> None:
    record["finished_utc"] = utc_now()
    temporary = trial_dir / f".{RECORD}.tmp"
    temporary.write_text(json.dumps(record, indent=2) + "\n")
    os.replace(temporary, trial_dir / RECORD)


def record_is_listed(trial_dir: Path) -> bool:
    """Whether the directory's SHA256SUMS lists upload.json, which then must not be overwritten."""
    sums = trial_dir / SUMS
    if sums.is_symlink() or not sums.is_file():
        return False
    for line in sums.read_text(errors="replace").splitlines():
        match = SUMS_LINE_RE.fullmatch(line)
        if match and manifest_path(match.group(2)) == RECORD:
            return True
    return False


def final_line(record: dict) -> str:
    prefix = f"{record['bucket']}/{record['prefix']}" if record.get("prefix") else "-"
    return f"UPLOAD {record['status']} {prefix}"


def main(argv: list[str] | None = None, *, environ=None, client_factory=AwsCli, size_of=None) -> int:
    environ = os.environ if environ is None else environ
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--trial-dir", type=Path, required=True)
    parser.add_argument("--run-id", default=None, help=f"default {RUN_ID_ENV}, else the trial directory's name")
    parser.add_argument("--organization", default=None, help=f"default {ORGANIZATION_ENV}")
    parser.add_argument("--project", default=None, help=f"default {PROJECT_ENV}")
    parser.add_argument("--bucket", default=None, help=f"default {BUCKET_ENV}; no default beyond it")
    parser.add_argument("--region", default=None, help=f"default {REGION_ENV}; no default beyond it")
    parser.add_argument("--existing-sha256sums", action="store_true",
                        help=f"upload the directory's own {SUMS} as it is, after checking every listed file")
    args = parser.parse_args(argv)
    trial_dir = args.trial_dir
    record = {"status": FAILED, "bucket": None, "prefix": None}
    try:
        if not trial_dir.is_dir() or trial_dir.is_symlink():
            raise ManifestError(f"{trial_dir} is not a trial directory")
        trial_dir = trial_dir.resolve()
        record = export(trial_dir, existing=args.existing_sha256sums,
                        run_id=args.run_id or environ.get(RUN_ID_ENV) or trial_dir.name,
                        organization=args.organization or environ.get(ORGANIZATION_ENV),
                        project=args.project or environ.get(PROJECT_ENV),
                        bucket=args.bucket or environ.get(BUCKET_ENV) or None,
                        region=args.region or environ.get(REGION_ENV) or None,
                        environ=environ, client_factory=client_factory, size_of=size_of)
        if record_is_listed(trial_dir):
            record["reasons"].append(f"{RECORD} is listed in {SUMS}, so the record is not written")
        else:
            write_record(trial_dir, record)
    except Exception as error:  # the last line is printed whatever happened
        record["status"] = FAILED
        print(f"export_native_trial: {type(error).__name__}: {error}", file=sys.stderr, flush=True)
    for reason in record.get("reasons", []):
        print(f"export_native_trial: {reason}", file=sys.stderr, flush=True)
    print(final_line(record), flush=True)
    return EXIT[record["status"]]


if __name__ == "__main__":
    sys.exit(main())
