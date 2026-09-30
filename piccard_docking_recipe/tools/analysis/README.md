# Alpha Rise S0 readout

`alpha_rise_s0_readout.py` performs deterministic, offline analysis of the
frozen selected-gain S0 artifacts. It never calls a provider or simulator.

For a native result and trace:

```bash
python3 tools/analysis/alpha_rise_s0_readout.py \
  --result /path/to/output/S0/s0-result.json \
  --trace /path/to/output/S0.surge.jsonl \
  --source-revision <exact-merged-physical-commit> \
  --output /new/path/s0-readout.json \
  --plot /new/path/s0-response.svg
```

For a pre-result startup failure, replace `--result` and `--trace` with
`--startup-error /path/to/output/s0-startup-error.json` and omit `--plot`.

The JSON readout binds the analyzer revision/hash, input hashes, frozen image,
the canonical selected-gains source bytes, S0 profile, all 72
expected/readback comparisons (24 values
across three sources), trace continuity, exact result fields, and the explicit
separation between API transport evidence and scientific assessment. The SVG
uses only the Python standard library, is derived diagnostic output, and is
never treated as authoritative evidence.

An input reporting `eligible` is labeled
`native_reported_eligible_pending_full_frozen_validation`; this tool enumerates
its own independent checks and the remaining API/full-science validation. It
does not upgrade the native claim or infer transport success.

# RACE onboard recorder load check

`race_onboard_load_check.py` compares race-auv-docking trial artifact directories run with and without
`media.onboard_camera_video` (#86, #92). It checks whether the recorder's reliable `cam_front` subscription costs
the AprilTag detector front-camera images. It is offline and uses only the standard library; it never calls the
API or a simulator.

```bash
python3 tools/analysis/race_onboard_load_check.py \
  --with-onboard /path/to/job-with/ --without-onboard /path/to/job-without/ \
  [--with-onboard ... --without-onboard ...] [--threshold 0.8] [--output report.json]
```

Each directory is a trial's `output/` or a downloaded artifact directory that holds it.

The detector republishes detections on a timer, so detection counts say nothing about image delivery. Every
detection row in `telemetry.jsonl` carries its source image's stamp, so the tool counts distinct stamps per
camera inside the mission window: the images the detector processed. `cam_down` shares the simulator and the host
but not the recorder, so the `cam_front` / `cam_down` ratio within a run controls for load.

The verdict is `drop` when the mean ratio of the with-onboard runs is below `threshold` times that of the
without-onboard runs. The fallback is `BEST_EFFORT` in `runtime/onboard_recorder.py`, or keeping onboard video out
of scored trials.

The report also lists, per run:
- the recorder's measured camera rate and frame counts;
- the audit problems and hold distance;
- whether the runs are comparable: same mission and gains, and the recorder in `media_nodes` exactly in the
  with-onboard runs.

It is a decision aid for the recorder's QoS, not a perception metric. One pair of runs is noisy, so pass several
of each.

On the arm64 dev loop (two reliable-QoS recorder runs against three runs without it), the ratios were 0.54 and
1.01, relative 0.54: `drop`. The staging check on the L40S decides.

# Recording isolation check

`recording_isolation_check.py` compares paired trials, one without and one with `media.display_video` and
otherwise identical (gains, mission, horizon). It asks whether recording perturbs the trial it records. Recording
is structurally isolated: the same collector and launch run, with no extra ROS node. But x11grab and libx264 share
the host's CPUs with the simulator, and the first RISE L40S media reruns settled slower than their scored trials.
Scored trials don't record until this check, run on three pairs on the L40S, shows no effect. It is offline and
uses only the standard library.

```bash
python3 tools/analysis/recording_isolation_check.py \
  --pair /path/to/no-video-1/ /path/to/with-video-1/ \
  --pair /path/to/no-video-2/ /path/to/with-video-2/ \
  --pair /path/to/no-video-3/ /path/to/with-video-3/ \
  [--settling-tolerance-s 10] [--jitter-ratio 1.2] [--min-pairs 3] [--output report.json]
```

It compares three things:
- **Settling per step.** Matched by step index. A step censored in only one run counts as slower or faster. The
  source depends on the recipe; other recipes report `settling: null`.
  - alpha-rise-gains-only: per depth step, from `packages/simulation/alpha-rise-gains-only/evaluation/phase_a_metrics.py`.
  - race-auv-docking: per judged pose step, from `race_m2_metrics.py` (the M2 v0.2 definition, below). This is
    the R-A0(a) comparison.
- **The collector's timing inside the mission window.** Inter-arrival of the controller's `set_point` and `value`
  streams, and of the collector's own `setpoint_command` where the recipe publishes one: median, p95, max and
  standard deviation. Also gaps between telemetry rows longer than 1 s.
- **The trial minimum distance** (#102). For RACE this is `docking.json` `minimum_distance`, the ground-truth
  dock-point distance. Each pair gives with-video minus without-video, and a two-sided sign test runs over the
  comparable pairs, with ties dropped.

The verdict is deliberately conservative, because one clean pair must never clear recording on scored trials. A
comparable pair shows an effect if any of these holds:
- it settles slower with video (median step delta above the tolerance);
- a stream's p95 inter-arrival grows by more than the jitter ratio;
- telemetry gaps appear only with video.

With that:
- `perturbed` when a majority of comparable pairs show one of these effects;
- `directional_effect` when the minimum-distance sign test gives p < 0.05, meaning recording moves the vehicle's
  closest approach the same way in every pair. With six pairs only a 6–0 split reaches it (p = 0.031; 5–1 gives
  0.219). With five pairs or fewer, no split can (5–0 gives 0.0625). This result is not cleared.
- `no_effect` only when no comparable pair shows an effect and the sign test gives p ≥ 0.05;
- `inconclusive` otherwise, which is not cleared;
- `insufficient_pairs` below `--min-pairs`, also not cleared. The signals are still reported.

`comparable` and `reasons` flag pairs whose video flags, mission, gains, horizon or recipe don't match.

The rule uses the p95 rather than the standard deviation, because a single stall dominates the standard deviation;
both are reported. It is a decision aid for the recording policy, not a statistical test.


# RACE M2 docking metrics

`race_m2_metrics.py` computes the fixed metrics of the M2 Phase R-A preregistration (piccard-experiments #87, v0.1
as amended by v0.2 on 2026-09-28) for race-auv-docking trial directories. It reads `trial.json`, `docking.json`
and `telemetry.jsonl`, is offline and uses only the standard library. Campaign and missions:
`packages/simulation/race-auv-docking/campaigns/phase-r-a-20260928/`.

```bash
python3 tools/analysis/race_m2_metrics.py /path/to/job-1/ [/path/to/job-2/ ...] --output-dir out/ \
  [--matrix] [--selection-mission staged|1step] [--settling-definition v0.2|v0.1]
```

It writes `<trial>.metrics.json` and `.md` per trial. With `--matrix` it also writes `matrix.json` and `.md`.

**Per pose, from ground truth in the station dock-point frame.**
- Overshoot past the commanded stand-off (m and % of the step) and the minimum dock-point distance.
- The dwell-end distance, stand-off, lateral, vertical and orientation errors.
- The dwell-end position errors split into controller error and frame drift. Ground truth = controller + drift,
  using `docking.json` `relative_position_in_station_dock_m` and `controller_frame_drift_at_end`.
- **Which reference each field uses.** The dwell-end stand-off, lateral, vertical and distance *errors* are measured
  against the **commanded** dock point: the commanded pose mapped through the measured controller frame, including
  its vertical origin. `distance_m`, the minimum distance and `relative_position_in_station_dock_m` are measured
  against the **station** dock point. Both use the same dock-point offsets as `runtime/docking_metric.py` and
  `contact-margin.json`.
  - Example: in M2 R-B the dock points were 2.1–2.2 cm apart vertically, while the vertical errors read +0.004 m
    (N) and −0.016 m (p 10). The difference is the frame's vertical offset.

**Per step, the v0.2 settling (default).** A step is one pose, or several consecutive poses with the same command.
- It is judged on each axis whose command moves by more than the band. The measure is the controller's own
  tracking error: its cg_link `value` minus the command, in its world_ned. Docking x is lateral; docking y is the
  approach axis, positive when short of the command.
- An axis settles when the error stays within ±0.05 m (±0.05 rad for yaw) of its final value, the mean of the last
  10 s, for at least the last 10 s.
- The settling time runs from the step start to the start of that final in-band run.
- The final value is the steady-state error, reported on every axis.
- The first step, the dive from the surface, is not judged.

`--settling-definition v0.1` keeps the preregistered ground-truth band for the record: ±0.05 m of the commanded
dock point on each axis, and ±0.05 rad. At nominal gains M1 never enters it.

**Per trial.**
- Thruster saturation: the fraction of commands whose modelled force (the pinned `config_sim.yaml` polynomial)
  is at the limit.
- `cam_front` tag visibility: distinct image stamps with a tag ÷ distinct stamps the detector saw.
- The fused range ratio.
- Contacts per pair (AUV–station, AUV–tank) in two classes (amendment v0.3, #100):
  - **force-bearing**: normal force > 0, meaning the meshes touch;
  - **near-miss**: normal force exactly 0. These are manifold points inside Bullet's contact margin, about 2 cm.
    An offline mesh check (#100) found every zero-force interval 8–22 mm from the station with no crossing; the
    loaded dev push was at 0.0–0.2 mm.

  Each class reports its event count, first and last time, seconds with events, and the ground-truth dock-point
  distance while the events occur.
- Route completion.
- Exclusion: an audit problem, a non-finite critical stream stop or an incomplete route. Such a trial is re-run
  once with `build_phase_r_campaign.py --rerun`.
- Whether the mission matches the job context's `mission_sha256`, and in which form (`mission_hash_form`). The
  builder hashes the committed file. The submit path re-renders `0.0` as `0`, so trial.json's own hash (the
  `runtime` form) differs, and the tool also checks the builder's form (every pose coordinate and angle a float).

**R-B contact.**
- The first **force-bearing** contact: its time, speed, and alignment to the station dock point.
- Its persistence: at least 90 % of the 1 s bins contain force-bearing contact, from that contact (or the final
  pose's start) to the end of the final dwell.
- The near-miss statistics, reported separately.
- The maximum normal force, a **lower bound**. The scenario's contact monitors keep one manifold point per
  simulation step (`history points="1"`), so a loaded point can be dropped in favour of an unloaded one.

**Selection (`--matrix`).** It takes the stage-a1 trials on the selection mission.
- Pass: no AUV–station contact of either class, and a minimum distance of at least 0.9 m on every pose. The
  matrix names the class as `station_contact_class`.
- Gates: saturation ≤ 0.15 on every thruster, no non-settling step of docking x or y, and a completed route.
- Ranking: settling on `approach_1p5m`, then the dwell-end distance error at `hold_1p5m`, then the |lateral error|
  there.
- It reports the spread between the N repeats.

`test_race_m2_metrics.py` runs on synthetic trials. `test_race_m2_metrics_m1.py` runs on
`fixtures/race-m1-8c47cbcc/`, the M1 staging trial (job 8c47cbcc), trimmed by `fixtures/trim_race_trial.py`:
- only the rows and fields the tool and the isolation check's timing read, rounded to 1e-6 except `t` and `stamp`,
  and gzipped;
- the JSON files verbatim;
- source and fixture sha256 in `SOURCE.json`.

It reproduces the staging numbers: minimum distance 0.737 m, hold dwell-end distance 1.722 m, overshoot 0.764 m
(13.9 %). Under v0.2, the approach step settles at 195.3 s with a steady-state stop-short of 0.158 m. At the hold
end, 0.147 m of the 0.165 m lateral error is frame drift.
