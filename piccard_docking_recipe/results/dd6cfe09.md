# Results at `dd6cfe09`: the verification trial

The verification trial ran at this branch's commit `dd6cfe09c2bbf2620d80b2bdc984fb7c3ef2c029`, on 2026-10-08. It was
one planner-arm trial on the documented path (RUN.md steps 1–5), with `examples/m3-planner-request.json` unchanged and
Stonefish built with `stonefish_seed_v1`, on an NVIDIA L40S with a headless NVIDIA X server. The acceptance was stated
before it ran.

**Attempt 2 (`verify-planner-2`): a valid run, a non-match on one figure.**
- **Simulator seed:** 2449946163, drawn and applied (`support: stonefish_seed_v1`).
- **Outcome:** docked at clearance at 2 cm and at 1 cm, at speed, by first full hold (2 cm entry at 450.8 s, 1 cm at
  453.3 s); not docked by the 3D criterion. Minimum gap 0.0215 m, vertical offset there −0.0215 m.
- **Against the acceptance:** a non-match under the preregistered rule on one figure only. The 1 cm entry, at 453.3 s,
  is earlier than the page's range for docked planner trials, 606.9–975.9 s. Every other figure is within tolerance.
  There is no rerun under this release.
- **Scoring:**
  - RUN.md step 5 at this commit did not score the clearance criterion: the runner passed no `--clearance-tol-m`, and
    the example's context named no protocol version. The result was re-scored with the scorer at this commit, using
    the report's settings: `--clearance-m 0.02 --clearance-tol-m 0.01`, protocol v1.5.
  - The re-score is an analysis step outside the preregistered path, and the record keeps both results.
  - Step 5 now scores this way (`native_request.py scoring`).
- **Deviations from RUN.md at this commit,** both since added to RUN.md:
  - the apriltag module link and `.pth`;
  - a check that `python3` imports Ubuntu's NumPy 1.26, after removing a pip NumPy 2 that the host's machine image
    carried.

  The run resumed once after that removal.

**Attempt 1 (`verify-planner-1`): an invalid run, not a trial.** The same machine image's pip NumPy 2 shadowed
Ubuntu's NumPy 1.26 under `cv_bridge`, so both AprilTag detectors died at startup. The collector now fails such a
trial before the mission (the perception gate).

**Where the results are:**
- **The page:** https://piccard.science/experiments/race-auv-docking-planner-2026-10
- **The canonical record:** Piccard's, in piccard-inc/piccard-physical-ai at `683545fdc3a37c78cb6afc52656b583fcf7a9a64`,
  `packages/simulation/race-auv-docking/provenance/verification-185/verify-planner-2/README.md`, SHA-256
  `7fcb669fc278f4d8f50ecebe0bef751825f2b1a3a52566d30ab2a2025f8818b6`.
