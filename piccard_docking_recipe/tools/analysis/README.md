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

# RACE M3 dock-criterion metrics

`race_m3_metrics.py` computes the M3 dock-criterion metrics (piccard-physical-ai #108, protocol piccard-experiments
#88) for race-auv-docking trial directories: pose missions and planner missions alike. It reads `trial.json`,
`docking.json` and `telemetry.jsonl`, and reuses the loaders of `race_m2_metrics.py`.

```bash
python3 tools/analysis/race_m3_metrics.py /path/to/job-1/ [/path/to/job-2/ ...] --output-dir out/ \
  [--hold-s 30] [--speed-limit-mps 0.1] [--meshes /path/to/world_of_stonefish/meshes] [--matrix] \
  [--clearance-m 0.02 --clearance-tol-m 0.005]
```

It writes, per trial:
- `<trial>.m3.json`, with a `definitions` block and a `claim_boundary`;
- `<trial>.m3-perception.jsonl`, the full-rate fused-minus-truth series.

With `--matrix` it also writes `m3-matrix.json` and `m3-matrix.md`.

All quantities are ground truth, from the collector's dock rows. The dock points use the offsets of
`runtime/docking_metric.py`, the same ones `race_m2_metrics` and contact-margin v1.1 use.
- **Gap:** the dock-point distance. It is reported as a 1 s series, its minimum, and the vertical offset and
  (with meshes) the coupling separation at that minimum. The minimum horizontal gap (along and lateral in the
  station dock frame) is reported too. With meshes, both minima name every AUV part's separation from every station
  part and the closest pair: on M3-0 v1.2 that was the left leg over the station frame, 3.5 mm.
- **Closure:** the first dock row with a gap of 0.02 m or less, and separately 0.01 m or less. `time_to_closure_s`
  counts from the mission start (the dive's start).
- **Speed at closure:** the speed of the AUV dock point relative to the station dock point: a central difference
  of the relative position over ±0.5 s, in the station dock frame. Also its maximum over the final approach, which
  runs from the last crossing of the 0.3 m stand-off to closure.
- **Hold:** the `hold_s` after each closure (default 30 s). It reports the fraction of dock rows within the
  threshold and the maximum and mean gap. **Docked** means every row of the whole hold is within the threshold,
  and the hold is observed: inside the mission window, with no dock-row gap over 1 s.
- **Dock criterion:** the lab's criterion also bounds the entry speed. `speed_ok` is the speed at closure against
  `--speed-limit-mps` (default 0.1 m/s; null without a speed), and `docked_at_speed` is docked and `speed_ok`. The
  matrix counts `docked` and `docked_at_speed` per threshold. The planner's `speed_cap_mps` caps the commanded set
  point; it is not this limit.
- **Closure at clearance (protocol v1.3):** the simulated controller cannot descend onto the dock without a depth
  step, so v1.3 holds the AUV dock point `approach_clearance_m` above the station's, and #88 redefines the
  criterion as the hover there.
  - With `--clearance-m c --clearance-tol-m tol` (tol > 0), `closure_at_clearance` scores, per threshold, the
    first dock row whose horizontal gap is within the threshold and whose `vertical_offset_m` is within tol of −c.
  - Speed, hold (every hold row meeting both), load and coupling follow the same logic as closure. Its gap fields
    hold the horizontal gap.
  - The 3D-gap `closure` is reported alongside, unchanged. Without a tolerance (the default) it is not scored.
  - `clearance.planner_approach_clearance_m` is the trial's planner value.
  - The matrix adds `at_clearance` per threshold and counts, and a second markdown table.
- **First full hold (protocol v1.5):** #88 preregistered this as v1.5's definition of docked, for trials run after
  it: a `hold_s` window inside the final stage in which the criterion holds continuously, entered at
  `--speed-limit-mps` or less. The criterion is existential: any such window counts, not only the first.
  - **What is scored.** Each closure, 3D and at clearance, per threshold, carries:
    - `first_full_hold`, the first full window: its entry, its time into the final stage, the entry speed and
      `speed_ok`, and the window's gap, vertical offset, load and coupling;
    - `first_full_hold_at_speed`, the first full window entered at the speed limit or less, with the same fields.
      It is null when no full window was, or when a window's speed cannot be measured;
    - `docked_any_window` (a full window exists) and `docked_any_window_at_speed`
      (`first_full_hold_at_speed` exists).
  - **The window.** Every dock row in it must meet the criterion, and the window must be observed: inside the
    mission, with no dock-row gap over 1 s. A window starts at the first row of a run, or at the first row after a
    dock-row gap: where the vehicle entered.
  - **The final stage.** For a planner mission it is the last planner stage (trial.json `planner.stages`). For a pose
    mission it is the last pose: from the first dock row carrying its label. Without either, `first_full_hold` is
    not computed and says why.
  - **Which definition a trial is scored by.** `criterion.scored` is `first_full_hold` from `protocol_version`
    v1.5, and `first_closure` before it or without a version. Both results are always reported.
    `criterion.final_stage` says where the final stage starts, and from what record.
  - **The matrix.** It adds per trial:
    - `criterion`;
    - `any_window` per threshold, 3D and at clearance, with the first full window's entry and the at-speed one's;
    - totals for `docked_any_window`, `docked_any_window_at_speed`, `scored_docked` and `scored_docked_at_speed`,
      where "scored" uses each trial's own definition;
    - a markdown table.
  - M3-0 v1.4, for example, stays scored on its first closure: not docked. It reports a 2 cm at-clearance full hold
    from collector t 484.3, which is informational.
- **Load during the hold:** contact events in the v0.3 classes, the maximum normal force (a lower bound: history
  1) and the fraction of 0.5 s bins with a force-bearing event.
- **Coupling (with `--meshes`):**
  - The separations are whole-mesh and couplink to couplink, at closure and at each second of the hold.
  - The meshes are placed by `contact_margin_check_v1_1.py`, imported. That file is the tool that produced
    `contact-margin-v1.1.json`, byte-identical to the served copy (sha256 `87e18278…`).
  - `--meshes` is a directory holding the six collision meshes of the pinned world_of_stonefish `d51d59e`
    (`data/parts/…` and `data/objects/…`). This path needs numpy.
- **Vertical offset:** z_W(AUV dock point) − z_W(station dock point). World z points down, so a negative value
  means the AUV dock point is shallower.
- **Time basis:** every `*_t` field is collector time, seconds on the collector's monotonic clock at receipt.
  `time_basis` holds `mission_start_t` (the dive's start) and `mission_end_t`; mission time is
  `t - mission_start_t`. `time_to_closure_s` is already mission time.
- **Frames and signs:** `definitions.frames_and_signs` gives both vertical conventions. The dock rows' relative
  position has z up, while `vertical_offset_m` uses world z down, so one position reads +0.0229 m and −0.0229 m.
  `true_range` is measured from the AUV's base_link, 0.395 m aft of its dock point and 0.13 m above it, so it is
  not the gap.
- **Run provenance:** `context.role` and `context.protocol_version` come from the job context (the campaign builder
  writes `protocol_version` for the M3 planner arm: v1.5 from #88). `mission_matches_context` tells
  whether the context's `mission_sha256` is this trial's mission, and `mission_hash_form` says in which form:
  - `runtime`: trial.json's hash of the bytes the runtime received;
  - `builder`: the builder's rendering. The submit path renders integral floats as integers, so this form casts
    back every value the builder wrote as a float.
- **Controller variant:** `controller_variant` and `mvp_control_patch`, from trial.json. `upstream` is the lab's
  mvp_control as pinned; `keep_xy_integral` is Piccard's variant (M3-C). The matrix carries it per trial. null means the
  trial was recorded before the field existed, and ran upstream.
- **Plain words:** `status_plain` and `stop_reason_plain`; `budget_censored` reads "ran to the horizon without
  completing".
- **Planner timeline:** `planner.timeline` holds:
  - `handover_t` and each stage's `start_t` (`stage` is an index into `standoffs_m`);
  - `stale_intervals`;
  - `setpoint_steps`, as `[t, x, y, z, yaw]` of cg_link in `race_auv/world_ned`. They come from trial.json when
    the collector recorded them, else from the planner telemetry rows (`source` says which);
  - `heading_rehold` (v1.2): the final stage's heading re-hold from trial.json. It records `samples`, `applied`,
    `previous`, `heading`, and `received_t` in collector time;
  - `approach_clearance_m` (v1.3) from trial.json;
  - `final_along` (v1.5) from trial.json: the final stage's along phase (`approach` or `hold`) and side, each
    arrival (`t` on the planner's clock, `error_m`, `side`, `change_m`, `received_t`) and each re-approach.

  `definitions.planner_frame` and `planner_stage` give the error frame L (level station frame at the held heading),
  and the stage-advance rule with the lateral band `band_m + s · band_rad`.
- **Tags:** the forward camera's tag layout in the station dock frame, from trial.json `tags` or, for older trials,
  the planner's `tag_pivot` record. It includes `forward_collinearity_m`, the tags' largest distance from one line:
  8.3 mm for cam_front's 146/541/558, which lie on the station's vertical centreline. It also counts the detection
  rows per camera and tag id in the mission window.
- **Perception:** at each `fused_dock` sample in the mission, the fused station dock point minus ground truth, in
  `race_auv/base_link`: range, lateral, vertical and yaw. The truth is the dock row nearest in time, within 0.1 s.
  - `perception.by_true_range` bins the samples by true range, and `by_forward_tags` by the cam_front ids seen
    within the fuser's 0.5 s detection age. Each bin gives n, mean and sd.
  - The series goes to `<trial>.m3-perception.jsonl` as
    `[t, true_range_m, range_m, lateral_m, vertical_m, yaw_rad, forward_tags]`.
  - The claim boundary excludes perception accuracy: these numbers describe the simulation's rendered tags and the
    pinned fuser, not the lab's hardware.
- **Planner provenance:**
  - `trial.json` `planner` is `{node, parameters, parameters_sha256}`. `parameters_sha256` is the sha256 of the
    parameters as JSON with sorted keys and no spaces. The block is null or absent when no planner ran.
  - Each consumption audit's `planner_inputs.reads_only_fuser_tf_and_odometry` is read too.
  - A missing verdict, a false one or a hash mismatch excludes the trial.

On the M2 R-B contact trials, neither run closes at 2 cm:
- N reaches a minimum gap of 0.067 m, p10 0.024 m.
- At the minimum, the AUV dock point sits 21.9 mm shallow, with a couplink separation of 19.3 mm (N) and 19.4 mm
  (p10). That is contact-margin v1.1's recorded-pose result.

# RACE tag visibility

`race_tag_visibility.py` says which station tags each RACE camera can see from a pose, offline (#265, D2). It
checks a pose mission's poses: `in_fov` lists the tags whose centres fall inside a camera's field of view widened by
10 deg (any range, any facing), so a pose with none on either camera is out of view; `detectable` lists those inside
the unwidened field of view, facing the camera and inside the range band where the trials' detector found them
reliably. The geometry is the pinned simulation's (cameras from world_of_stonefish d51d59e `vehicles/race_auv.scn`
and `/tf_static`; the station and its tags from ground truth); the module's docstring gives the sources and the
check against recorded trials. Standard library only.

```bash
python3 tools/analysis/race_tag_visibility.py packages/simulation/race-auv-docking/examples/drift-and-revisit-request.json
```

It prints one JSON line per pose: the nominal Stonefish-world pose, `in_fov` and `detectable` per camera, the forward
camera's ranges to tags 146, 541 and 558, and `out_of_view`. The poses are commanded set points mapped through
`runtime/mission.py`'s nominal frame, so the EKF's drift moves the vehicle away from them; the field-of-view margin and
the stand-off's distance from the range limits are what absorb it.
