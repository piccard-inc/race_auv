# Piccard race-auv-docking recipe: snapshot for the SOS Lab

Exported from piccard-inc/piccard-physical-ai at commit c66a66ace608683af496d9ab65305d51e9eeaa28 (committed 2026-10-09) by
`tools/release/export_race_auv_docking_snapshot.py`. It is the layer that ran the trials reported at
piccard.science/experiments/race-auv-docking-approach-2026-09 and
piccard.science/experiments/race-auv-docking-planner-2026-10. Nothing here runs against the lab's hardware. The
container base image is private, so the Dockerfile is for reading.

## Layout

The paths are the repository's own, so the tools and tests run from this directory unchanged.

- `packages/simulation/race-auv-docking/`: the recipe; its `README.md` describes every file.
  - The Dockerfile, the recipe, the request schema, the source lock and the conformance cases.
  - The example requests. `examples/m3-default-request.json` reads the tag sizes as the black-square edge
    (`black_square_edge`); `examples/m1-smoke-request.json` keeps the lab's configured sizes (`lab_configured`).
    `examples/drift-and-revisit-request.json` (#265) takes the tags out of view for two legs of about five minutes
    and back, reading them as the black-square edge.
  - `runtime/`: install, launch, the scenario wrapper, the collector, the mission validator, the docking metric,
    the media recorders, the supervisors, the simulator seed, the native runner (`run_native_trial.sh`)
    and its export (`export_native_trial.py`).
  - `simulator-patches/`: `stonefish_seed_v1`, the optional Stonefish patch that makes the sensor-noise seed
    settable and printed.
  - `campaigns/`: the M2 missions and the tank floor.
- `packages/localization/race-tag-graph/`: `race_tag_graph`, an offline GTSAM pose graph over the vehicle's own
  sensor topics (EKF odometry, the 3D tag detections, `/tf_static`) with the tags as jointly estimated landmarks and
  the dock point as output; batch and ISAM2 solvers over one factor builder; its `README.md` describes it. Its tests
  run on synthetic telemetry.
- `tools/campaigns/`: the mission and jobs-file builder, the tank-floor extractor, and their tests.
- `tools/analysis/`: the M2 and M3 metrics, the contact-margin check, the recording-isolation check, the
  onboard-load check and the tag-visibility check, with their `README.md` and the metrics, onboard-load and
  tag-visibility tests.

Below, a path that starts with neither `packages/` nor `tools/` is relative to `packages/simulation/race-auv-docking/`.

## The lab's repositories are used unmodified

Every trial builds these sources from pinned archives (sha256 in `source-lock-v1.json`). No file of theirs is edited:

| source | repository | commit |
|---|---|---|
| `race_auv` | https://github.com/piccard-inc/race_auv | `da58963c048228856dfdd5917da21a14edfa1985` |
| `race_auv_sim` | https://github.com/GSO-soslab/race_auv_sim | `3601a30f49c7b8ddac2845c41c99af8af92c0e65` |
| `race_auv_perception` | https://github.com/GSO-soslab/race_auv_perception | `5944d5a6ebd44601bcc5579e91b82d5d1b3735cb` |
| `race_station` | https://github.com/GSO-soslab/race_station | `100bc5a7ac253172b527c036d637da5482eaf0f3` |
| `world_of_stonefish` | https://github.com/GSO-soslab/world_of_stonefish | `d51d59e77211a436a0c3617c30a80c36337cdbbc` |
| `apriltag` | https://github.com/AprilRobotics/apriltag | `dc6316dc37520e56819d64741b54a41b50c92866` |

The build changes only which parts are built or kept, as `source-lock-v1.json` records per source:
- `race_auv`: real-vehicle configuration removed by runtime/sanitize_seed.py
- `race_auv_sim`: none
- `race_auv_perception`: race_auv_apriltag_cuda COLCON_IGNOREd; third_party (isaac_ros_nitros) and scripts removed; the cuAprilTags path is never built
- `race_station`: empty top-level race_station package COLCON_IGNOREd in this build only
- `world_of_stonefish`: none; shadows the base image's 55139d4 copy (asserted at build, recorded per trial)
- `apriltag`: none; built for the system Python 3.12. race_auv_camera_pkg's python backend calls apriltag.estimate_tag_pose(detection, size, fx, fy, cx, cy), which v3.4.5 lacks (every detection then falls back to an identity pose, as race_auv_camera_pkg/Jetson.md warns); the image build solves a pose on the pinned tag texture

`race_auv` is built from this fork (piccard-inc/race_auv) at `da58963`, which is also a commit of
GSO-soslab/race_auv `jazzy-devel-perception`. `runtime/sanitize_seed.py` deletes the real-vehicle configuration files
from the container's copy, so that a trial can never load hardware settings.

The base image is `piccard/alpha-rise-gains-only-staging@sha256:f11a47bbe889d10338652488192977edc771112f648421e293bb991cf99aae2c`, with the private registry host removed from the reference. The image itself is
private. It holds Stonefish 7d52673, stonefish_ros2 a7c9be9, mvp_msgs 7aa616f, mvp_control 6cfea2d, mvp_mission 84cce53, mvp_utilities 0c475d3 and world_of_stonefish 55139d4 in /opt/ros2_ws.

## How `direct_control` is installed (answer to the setup question)

The repository files leave `bhv_direct_control` commented out and define no direct-control PID. At trial start,
`runtime/prepare_candidate.py` edits the **container's copies** of three files and reads them back before the
controller is enabled (`recipe-v1.json` → `fixed_execution.helm`). `config_sim.yaml` is edited in both the source
and the install tree; the other two only in the install tree.

1. `race_auv_config/mvp_control_config/config_sim.yaml` (`install()`): `control_modes.flight` and a new
   `control_modes.docking` are both written from the trial request's gains.
   - The campaign requests carry the upstream flight values verbatim. The builder copies them from
     `examples/m1-smoke-request.json`, which holds the pinned `config_sim.yaml` values.
   - They set docking z, roll, pitch and yaw equal to flight. Only the docking x and y PIDs and output limits
     differ between trials.
   - So "flight gains unchanged" holds by the content of every request. The trial records the gains and reads them
     back (`trial.json` → `gains_verified`); there is no install-time guard. `runtime/check_image.py` checks only
     that the recipe's output limits equal the pinned file.
   - The docking mode is x, y, z, roll, pitch, yaw of `cg_link` in `world_ned`.
2. `race_auv_config/mvp_mission_config/helm_sim.yaml` (`install_helm()`): a `direct_control` state is added to the
   finite-state machine. It has control mode `docking` and transitions to `start` and `kill`, and `start` and `kill`
   each gain a transition to it. `behaviors.bhv_direct_control` is also added, with `plugin: helm/DirectControl`
   and `priority: {direct_control: 1}`.
3. `race_auv_bringup/config/bhv_params_sim.yaml`: `bhv_direct_control` gets its parameters,
   `default_bhv_world_link: world_ned` and `default_bhv_child_link: cg_link`.

The installer refuses to run if `direct_control` or `bhv_direct_control` is already present in either file.

`runtime/collect_trial.py` then:
1. publishes `mvp_msgs/ControlProcess` set points on `/race_auv/mvp_helm/bhv_direct_control/desired_setpoints` at
   1 Hz;
2. enables the controller;
3. changes the helm state to `direct_control` through the change-state service;
4. waits for the reported control mode `docking` before the mission clock starts.

Each pose (`runtime/mission.py`, `piccard.race-auv.pose-mission/v1`) is held for its dwell.

The controller sees only `/race_auv/odometry/filtered` and the AprilTag fuser.
- `runtime/launch/docking_sim.launch.py` mirrors `bringup_simulation.launch.py`, with the simulator's ground truth
  remapped to `/piccard/ground_truth/*`.
- The consumption audit in `collect_trial.py` fails the trial if a ground-truth edge or publisher reaches the
  vehicle's TF tree or a controller input.
- Ground truth scores (`runtime/docking_metric.py`, `tools/analysis/race_m2_metrics.py`) and never navigates.

## Running the checks

The export ran these from this directory, with Python 3, PyYAML and jsonschema (and, for the localization package,
gtsam 4.3a0, NumPy and Matplotlib: its `requirements.txt`), and they passed:

    PYTHONPATH=packages/simulation/race-auv-docking/runtime python3 packages/simulation/race-auv-docking/test_conformance.py
    python3 tools/campaigns/test_build_phase_r_campaign.py
    python3 tools/analysis/test_race_m2_metrics.py
    python3 tools/analysis/test_race_m3_metrics.py
    python3 tools/analysis/test_race_onboard_load_check.py
    python3 tools/analysis/test_race_tag_visibility.py
    PYTHONPATH=packages/localization/race-tag-graph python3 packages/localization/race-tag-graph/tests/test_sensors.py
    PYTHONPATH=packages/localization/race-tag-graph python3 packages/localization/race-tag-graph/tests/test_graph.py
    PYTHONPATH=packages/localization/race-tag-graph python3 packages/localization/race-tag-graph/tests/test_outputs.py

`tools/analysis/README.md` describes the tools' command lines. They recompute the metrics from a trial's served
outputs.

## Changed from the commit, and left out

Changed, and nothing else:
- The private registry host and account id are removed from the base-image reference in `Dockerfile`,
  `recipe-v1.json` and `source-lock-v1.json`.
- `recipe-v1.json`'s `source_lock_sha256` and `conformance-v1.json`'s recipe hash are refreshed to the scrubbed
  files, so `test_conformance.py` holds here.
- `conformance-v1.json` drops the contract source of a file left out below (the controller-variant recipe).

Left out:
- `packages/simulation/race-auv-docking/fixtures/`: trial outputs
- `packages/simulation/race-auv-docking/runtime/test_race_runtime.py`: reads fixtures/
- `packages/simulation/race-auv-docking/runtime/test_planner.py`: reads fixtures/ (replays of recorded trials, which carry ground truth); the lab branch's race_auv_docking_planner package holds the planner's fixture-free tests
- `packages/simulation/race-auv-docking/Dockerfile.dockerignore`: the private build's context filter
- `packages/simulation/race-auv-docking/native_bridge/`: source-only internal sensor/EKF bridge candidate; not used by the lab recipe
- `packages/simulation/race-auv-docking/controller-variants/`: Piccard's internal controller variants (keep_xy_integral, piccard-experiments#88 M3-C, retired; actuator_fix_v1, native actuator correction): separate modifications of the lab's GPL-3.0 mvp_control, left out of the lab snapshot: its patch and build helper
- `packages/simulation/race-auv-docking/Dockerfile.controller-variant`: Piccard's internal controller variants (keep_xy_integral, piccard-experiments#88 M3-C, retired; actuator_fix_v1, native actuator correction): separate modifications of the lab's GPL-3.0 mvp_control, left out of the lab snapshot: its image build
- `packages/simulation/race-auv-docking/Dockerfile.controller-variant.dockerignore`: Piccard's internal controller variants (keep_xy_integral, piccard-experiments#88 M3-C, retired; actuator_fix_v1, native actuator correction): separate modifications of the lab's GPL-3.0 mvp_control, left out of the lab snapshot: its build's context filter
- `packages/simulation/race-auv-docking/recipe-keepxyint-v1.json`: Piccard's internal controller variants (keep_xy_integral, piccard-experiments#88 M3-C, retired; actuator_fix_v1, native actuator correction): separate modifications of the lab's GPL-3.0 mvp_control, left out of the lab snapshot: its recipe (a copy of recipe-v1.json's hashes, which the scrub changes); its contract source is dropped here
- `packages/simulation/race-auv-docking/provenance/`: Piccard build evidence (raw image build logs, in-image records and registry receipts for its internal controller-variant images), not lab material
- `packages/simulation/race-auv-docking/provenance/native-harness/`: Piccard run evidence from its internal native no-physics harness (host and container logs and receipts), not lab material
- `tools/analysis/fixtures/`: trial outputs
- `tools/analysis/test_race_m2_metrics_m1.py`: runs on the M1 trial fixture
- `tools/analysis/test_recording_isolation_check.py`: runs on RISE trials through the RISE evaluation package, which is not part of this recipe
