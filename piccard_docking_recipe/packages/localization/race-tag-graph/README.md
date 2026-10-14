# race_tag_graph: an offline tag-aided pose graph for RACE

`race_tag_graph` replays RACE's own sensor topics through a GTSAM pose graph and writes a drift-corrected
trajectory and the station's dock point. It runs offline on a recorded run, with no ROS needed. The work item is
piccard-inc/piccard-physical-ai#265, from the SOS Lab decision in piccard-research discussion #35.

It is built to the formulation in the lab's own `LOCALIZATION_ARCHITECTURE.md` (the `gtsam_dock_fuser`, phases
1–2), with one difference: tag detections enter as 3D points, not corner projections. It runs on simulation
telemetry today. A reader for a real rosbag2 is included but untested until a real bag arrives.

## The formulation (`race_tag_graph/factors.py`, the only place it lives)

- **Keyframes.** One per image with tag detections, and at least every `keyframes.max_interval_s`. Each starts at
  the EKF pose interpolated at its stamp.
- **Gauge.** A prior on the first keyframe at the EKF's first pose.
- **Odometry, once, as motion.**
  - One `BetweenFactorPose3` per consecutive keyframe pair, from the EKF's own increment.
  - The EKF's absolute pose never enters as a measurement.
  - Its noise is a random walk in time and distance, so the total over an interval doesn't depend on how many
    keyframes cut it.
  - The collector's telemetry records no EKF covariance. With a bag that carries it, `odometry.noise:
    ekf_covariance_difference` uses it. That path is untested.
- **Tags as fixed landmarks, estimated jointly.**
  - Each tag is a `Point3` in the odom frame, initialised at its first sighting. It is never a surveyed pose.
  - Every later 3D detection constrains it through the camera extrinsic from `/tf_static`.
- **The detection's noise model is anisotropic.**
  - Across the camera ray: a bearing error (`bearing_sigma_rad · r + lateral_floor_m`).
  - Along the ray: a range error growing as r² / (f · s), with s the tag's size.
  - Beyond `range_informative_sigma_m` (about 3.0 m for the 0.12 m tags, 1.75 m for the 0.04 m tags) a detection
    constrains its **bearing only**. Far ranges carry a systematic error that averaging many detections does not
    remove; bearings stay accurate.
- **No range-scale parameter.** A range convention error is reported in `range_scale_residual`, not absorbed.
- **Outliers.**
  - A robust kernel (Huber by default).
  - A causal gate: a detection whose EKF-projected position lies more than `gate.base_m + gate.per_m · r` from the
    tag's running median is recorded and not added.
- **Tag orientation: opt-in (`orientation.enabled`), off by default.**
  - Range-gated (`orientation.max_range_m`), with a flip gate.
  - It needs orientations in the detections. The collector's telemetry records positions only, so with telemetry
    it raises.

**Solvers** (`race_tag_graph/solvers.py`) consume the same factors:
- `solve_batch`: Levenberg–Marquardt.
- `solve_isam2`: one ISAM2 update per keyframe, in time order. This is the path to the lab's node.

On the synthetic test run they agree to well under a centimetre.

## The dock point (`race_tag_graph/dock.py`)

- **Output.** `race_station/dock_point` in `race_auv/base_link` per keyframe, the semantics of the lab fuser's
  `/race_station/dock_point/pose`.
- **Station layout:** `config/station_layout.yaml`.
  - Taken from the lab's station URDF, `race_station_description/urdf/base.urdf` at GSO-soslab/race_station
    `100bc5a7` (pinned in `packages/simulation/race-auv-docking/source-lock-v1.json`), the file the lab's apriltag
    fuser loads. It is operational geometry, not ground truth.
  - The tag sizes are the detector's configured black-square-edge sizes, from
    `race_auv_bringup/config/simulation/apriltag_black_square_edge.yaml`.
- **The fit.** Level-constrained in the gravity-aligned odom frame: yaw, the dock frame's vertical sign (chosen by
  residual), and translation.
- **Yaw needs horizontal spread among the observed tags.**
  - The forward camera's tags (146, 541, 558) sit on the station's vertical centreline, 4 cm apart horizontally.
  - With only those the yaw is unobserved, and the status is `yaw_unobserved`: no dock pose, only the tags' pivot.
  - The flat tags 176 and 185, 0.5 m apart, would fix it. Only the downward camera (cam_down) can see them.
- **Disclosure: on our data the dock output is the pivot only.**
  - **Across all 59 local RACE trials, cam_down never detected a tag** (#265's drift audit, D0; rechecked on the
    telemetry: no cam_down detection array with a tag in any of the 59 files).
  - The drift-and-revisit mission (D2) adds no pass over the flat tags either.
  - So on everything we hold, the graph's dock output is `yaw_unobserved`: the pivot position, no dock pose.
  - **A question for Yuewei:** does cam_down see the flat tags (176, 185) in his bag?

## The estimator reads sensors only (`race_tag_graph/sensors.py`)

**Allowlist.** The builder accepts only a `SensorStream`, and the readers fill it only from:
- `/race_auv/odometry/filtered`;
- `/cam_front|cam_down/apriltag_detection/detections3d`;
- `/tf_static`.

**Refused.** Asking for anything else raises `TruthAccessError`:
- the ground-truth topics (`/piccard/ground_truth/*`);
- the simulator's odometry;
- the contact topics;
- the collector's truth records (`gt_*`, `dock`, `alignment`);
- the lab fuser's `/race_station/dock_point/pose`.

**Skipped unparsed.** In a telemetry file, truth records are skipped by kind before they are parsed.

**Evaluation side only.** Truth and the lab fuser's output are read by `truth.py` and used by `evaluate.py`, which
no estimator module imports (a test checks it).

**Readers:**
- `read_telemetry(path)`: the Piccard collector's `telemetry.jsonl` (`runtime/collect_trial.py`). Tested.
- `read_rosbag2(path)`: **UNTESTED until Yuewei's bag arrives.**
  - It reads a rosbag2 (sqlite3 or mcap) through the `rosbags` library, which is not installed by default.
  - Message types: `nav_msgs/msg/Odometry`, `vision_msgs/msg/Detection3DArray` and `tf2_msgs/msg/TFMessage`.
  - **Why `vision_msgs/Detection3DArray`, not `apriltag_msgs/AprilTagDetectionArray`** (the issue's wording):
    `apriltag_msgs` carries 2D image corners, not a tag position. The lab's detector publishes its solved 3D tag
    poses on `/cam_*/apriltag_detection/detections3d` as `Detection3DArray`, and the position landmark factors need
    those. A bag with only `apriltag_msgs` detections cannot feed this formulation; corner factors would be the
    lab's projection formulation, not built here.
  - Only its message conversions are unit-tested, on stand-in objects.

## Metrics (`results.json`)

Every metric carries a `result_class`, and the page inherits it.

**`sensor-grounded`** (`metrics.py`): what a real bag can give, with no truth.

| Metric | What it reports |
|---|---|
| `resighting_residuals` | Before (EKF poses, landmarks at first sighting) and after the solve. |
| `drift_at_resightings` | At a re-sighting after `metrics.revisit_span_s` out of view: the jump of the tag's projected position since its last sighting, through the EKF and through the solve. Re-anchoring is not counted as improvement. |
| `dock_point_stability` | The dock point (or pivot) fitted per in-view segment, and its spread. Includes the lab fuser's when evaluated. |
| `range_scale_residual` | Measured over predicted range after the solve. |

**`scoring`** (`evaluate.py`, simulation only, against the simulator's truth):

| Metric | What it reports |
|---|---|
| `position_error_vs_truth` | After first-pose alignment, as the #265 drift audit and `runtime/docking_metric.py` do. |
| `error_at_resightings_vs_truth` | The same error at the re-sighting keyframes. |
| `dock_point_error_vs_truth` | The candidate's error, and the lab fuser's. |

## Replay from a fresh checkout

```
git clone https://github.com/piccard-inc/piccard-physical-ai && cd piccard-physical-ai
python3 -m venv .venv && . .venv/bin/activate
pip install -r packages/localization/race-tag-graph/requirements.txt
export PYTHONPATH=packages/localization/race-tag-graph
python -m race_tag_graph replay --telemetry path/to/telemetry.jsonl --out out --solver both [--evaluate-against-truth]
python -m race_tag_graph replay --bag path/to/rosbag2_dir --out out        # pip install rosbags; untested
```

**Outputs** (`out/`):
- `trajectory.csv`: EKF and solved poses per keyframe;
- `landmarks.csv`;
- `dock_point.csv`;
- `results.json`;
- `trajectory.png`.

**Tests:** `python3 packages/localization/race-tag-graph/tests/test_graph.py`, and the same for `test_sensors.py` and
`test_outputs.py`, or pytest. They run on synthetic telemetry (`tests/synthetic.py`); no recorded trial data is
included.

## Configuration (`config/default.yaml`)

| Section | Keys |
|---|---|
| `keyframes` | `max_interval_s`, `merge_s` |
| `prior` | the first pose's sigmas |
| `odometry` | `noise` (`model` or `ekf_covariance_difference`), the random-walk rates in time and distance, the floors |
| `detections` | `tags`; `max_range_m`; the bearing, range and size-scaled range model, `focal_px`; `range_informative_sigma_m`, `bearing_only_range_sigma_m`; `robust`; `gate` |
| `orientation` | `enabled`, `max_range_m`, `sigma_rad`, `flip_gate_rad` |
| `isam2` | the relinearisation settings |
| `batch` | iterations and tolerance |
| `dock` | `layout`, `frame_up`, `yaw_min_baseline_m`, `up_min_residual_ratio` |
| `metrics` | `visible_s`, `revisit_span_s` |

**How the values were set.** They are a-priori engineering values, not fitted to any trial.

**Disclosure: the range model was chosen after looking at one trial.**
- **What happened.** The size-scaled range model and its bearing-only gate were introduced after the first replay of
  one recorded ≈×1.0 trial, `pr-m3a-p10-contact-r1`, under a flat range model.
- **The observation, no cause attributed.** On that trial, measured against the simulator's truth, far detections
  read long even though it is a ≈×1.0 trial: tag 146's detections by about +2.7 cm at 3–6 m and about +13 cm at
  6–12 m, while their bearing stayed accurate.
- **What limits it.** The form (r² / f s) is the physical pose-from-apparent-size relation. Its coefficient (0.5 px)
  and threshold (5 cm) were set from that relation; no parameter was swept against any trial.
- **The development trial.** `pr-m3a-p10-contact-r1` is the DEVELOPMENT trial: `evaluation.development_trials` in
  `config/default.yaml`. A replay of it records `development_trial: true` and `headline_eligible: false` in
  `results.json`, and `evaluate.headline_trials` leaves it out.
- **The headline set** is the other ≈×1.0 trials plus D2's three drift-and-revisit trials, held out.
- **No evaluation numbers are stated here.** They come from D3, under its protocol.

## The later upgrade path: corner projection factors

The lab's own `LOCALIZATION_ARCHITECTURE.md` (race_auv root; on piccard-inc/race_auv `piccard/docking-recipe` at
`cd70dbee`, last changed in `1a8e112e`) specifies the `gtsam_dock_fuser` with tag **corner projection factors**:
`GenericProjectionFactorCal3_S2` per corner, per tag, per camera, fed by `apriltag_msgs/AprilTagDetectionArray` from
tagslam's `sync_and_detect`. This package uses the 3D detection array instead, one solved position per tag: the
input that the current detector publishes and that position landmarks need. Corner projection factors are the later
upgrade path; they would replace `landmark_factor` in `factors.py`, with the keyframes, the odometry, the gauge and
both solvers unchanged.

The same document plans to carry `apriltag_msgs/AprilTagDetectionArray` on the `detections3d` topics and to remove
the legacy `vision_msgs` one. A bag recorded after that change carries corners there, and `read_rosbag2` refuses it
with that reason rather than misreading it. Which message type his bag carries is a question for Yuewei.

## Not here

- No live-vehicle use.
- No new optimizer.
- No IMU preintegration.
- No accuracy claim beyond simulation.
- No range-scale estimation.
