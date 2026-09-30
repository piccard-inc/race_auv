# Piccard race-auv-docking recipe: snapshot for the SOS Lab

Exported 2026-09-29 from piccard-inc/piccard-physical-ai at commit 3c713b44996cd90a767f82072521018a7d98d366
(`packages/simulation/race-auv-docking/` plus the campaign builder and analysis tools). It is the layer that ran
the trials reported at piccard.science/experiments/race-auv-docking-approach-2026-09. Nothing here is executable
against the lab's hardware; the container base image is private, so the Dockerfile is for reading.

## The lab's repositories are used unmodified

Every trial builds these sources from pinned archives (sha256 in `source-lock-v1.json`) and never edits them:

| source | repository | commit |
|---|---|---|
| `race_auv` | https://github.com/piccard-inc/race_auv | `da58963c048228856dfdd5917da21a14edfa1985` |
| `race_auv_sim` | https://github.com/GSO-soslab/race_auv_sim | `3601a30f49c7b8ddac2845c41c99af8af92c0e65` |
| `race_auv_perception` | https://github.com/GSO-soslab/race_auv_perception | `5944d5a6ebd44601bcc5579e91b82d5d1b3735cb` |
| `race_station` | https://github.com/GSO-soslab/race_station | `100bc5a7ac253172b527c036d637da5482eaf0f3` |
| `world_of_stonefish` | https://github.com/GSO-soslab/world_of_stonefish | `d51d59e77211a436a0c3617c30a80c36337cdbbc` |
| `apriltag` | https://github.com/AprilRobotics/apriltag | `dc6316dc37520e56819d64741b54a41b50c92866` |

`race_auv` is built from this fork (piccard-inc/race_auv) at `da58963`, the same commit as GSO-soslab/race_auv
`jazzy-devel-perception`. The only build-time change is `runtime/sanitize_seed.py`, which deletes the real-vehicle
configuration files from the container so a trial can never load hardware settings.

## How `direct_control` is installed (answer to the setup question)

The repository files leave `bhv_direct_control` commented out and define no direct-control PID. At trial start,
`runtime/prepare_candidate.py` edits the **container's copies** of three files and reads them back before the
controller is enabled (`recipe-v1.json` → `fixed_execution.helm`):

1. `race_auv_config/mvp_control_config/config_sim.yaml`: `control_modes.flight` and a new `control_modes.docking`
   are both written from the trial request's `gains.json` (`install()`). The campaign requests carry the upstream
   flight values verbatim (the builder copies them from `examples/m1-smoke-request.json`, which is the pinned
   `config_sim.yaml`), and set docking z, roll, pitch and yaw equal to flight; only the docking x and y PIDs and
   output limits differ between trials. So "flight gains unchanged" holds by the content of every request, which
   the trial records and reads back (`trial.json` → `gains_verified`), not by an install-time guard;
   `runtime/check_image.py` checks only that the recipe's output limits equal the pinned file. The docking mode is
   x, y, z, roll, pitch, yaw of `cg_link` in `world_ned`.
2. `race_auv_config/mvp_mission_config/helm_sim.yaml` (`install_helm()`): a `direct_control` state is added to
   the finite-state machine (control mode `docking`, transitions to `start` and `kill`; `start` and `kill` gain a
   transition to it), and `behaviors.bhv_direct_control` is added with `plugin: helm/DirectControl` and
   `priority: {direct_control: 1}`.
3. `race_auv_bringup/config/bhv_params_sim.yaml`: `bhv_direct_control` gets its parameters,
   `default_bhv_world_link: world_ned` and `default_bhv_child_link: cg_link`.

The installer refuses to run if `direct_control` or `bhv_direct_control` is already present in either file.

`runtime/collect_trial.py` then publishes `mvp_msgs/ControlProcess` set points on
`/race_auv/mvp_helm/bhv_direct_control/desired_setpoints` at 1 Hz, enables the controller, changes the helm state to
`direct_control` through the change-state service, and waits for the reported control mode `docking` before the
mission clock starts. Each pose (`runtime/mission.py`, `piccard.race-auv.pose-mission/v1`) is held for its dwell.

The controller sees only `/race_auv/odometry/filtered` and the AprilTag fuser. `runtime/launch/docking_sim.launch.py`
mirrors `bringup_simulation.launch.py` with the simulator's ground truth remapped to `/piccard/ground_truth/*`, and
the consumption audit in `collect_trial.py` fails the trial if a ground-truth edge or publisher reaches the vehicle's
TF tree or a controller input. Ground truth scores (`runtime/docking_metric.py`, `tools/analysis/race_m2_metrics.py`)
and never navigates.

## Contents

- `README.md`, `recipe-v1.json`, `request-v1.schema.json`, `conformance-v1.json`, `source-lock-v1.json`, `Dockerfile`,
  `examples/m1-smoke-request.json`, `test_conformance.py`
- `runtime/`: install, launch, scenario wrapper (upstream `race_auv_test.scn` unchanged plus two contact monitors),
  collector, mission validator, docking metric, media recorders, supervisors
- `campaigns/phase-r-a-20260928/`: the M2 missions and their derivation; `campaigns/tank-floor-v1.json`
- `tools/campaigns/`: mission and jobs-file builder, tank-floor extractor, tests
- `tools/analysis/`: M2 metrics, recording-isolation check, onboard-load check, README

Omitted: test fixtures that carry trial outputs, and one runtime test that depends on them. Known limits of this hand-made first export: the tools resolve the package by its repository-relative path and do not run from this flattened layout, and `test_conformance.py`'s two source hashes no longer match the scrubbed `recipe-v1.json` and `source-lock-v1.json` (the private registry host was removed from both). A scripted re-export will fix the layout and refresh the hashes; the code itself is unchanged.
