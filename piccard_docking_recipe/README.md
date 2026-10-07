# piccard_docking_recipe: not for the vehicle

This directory is Piccard's layer for the simulation trials: running them, recording them, and scoring them against
ground truth. Nothing in it runs on the vehicle, and no vehicle package depends on it.
- **colcon skips it.** It holds `COLCON_IGNORE` and no package manifest.
- **`tests/test_separation.py` checks the separation.** No ROS package of this repository refers to this directory or
  imports one of its modules. No package's code, outside its tests, names a ground-truth topic or frame.

| Path | What it is |
|---|---|
| `RUN.md` | The documented trial path, from a fresh checkout to a scored result. |
| `packages/simulation/race-auv-docking/runtime/` | The trial layer. See below. |
| `packages/simulation/race-auv-docking/` (the rest) | The trial recipe: the request schema and its conformance cases, the example requests, the campaign missions, the source lock, the image's Dockerfile (for reading), and the optional Stonefish seed patch. |
| `tools/analysis/` | The scoring: `race_m3_metrics.py` (the M3 dock criterion), `race_m2_metrics.py`, `contact_margin_check_v1_1.py`, and the onboard-load and recording-isolation checks. |
| `tools/campaigns/` | The campaign builder and the tank-floor extractor. |
| `dependencies/` | The pinned sources, the build, the patches by hash, the licences. |
| `tests/` | This branch's checks: the docking variants, the dependency manifest, the separation. |
| `SNAPSHOT.md` | The piccard-physical-ai commit this directory was exported from. |

The trial layer in `runtime/`:
- the runners (`run_native_trial.sh`, `run_trial.sh`), the trial launch and the scenario wrapper with its contact
  monitors;
- the collector (`collect_trial.py`): it flies the dive or the poses, starts the planner, and records telemetry and
  ground truth;
- the consumption audit (`graph_audit.py`): it fails a trial if a vehicle node consumes ground truth;
- the ground-truth dock metric (`docking_metric.py`);
- the candidate check, the seed record, the tag-pivot derivation and the media recorders;
- `planner_fused_dock.py`, the planner as it ran inside Piccard's image. The vehicle-side package is
  `race_auv_docking_planner`.

Ground truth is:
- the simulator's odometry of the vehicle and the station (`*/stonefish/odometry`);
- the contact topics (`/piccard/contact/*`);
- the TF of the station bringup and of the ground-truth pose node, which the trial launch remaps onto
  `/piccard/ground_truth/*`.

In this repository, only this directory uses it.

## What a field deployment needs, and what it does not

Nothing here has run on the vehicle; the M3 report's results are simulation results. This section separates what the
docking planner depends on from what the simulation and scoring add.

**Needed on the vehicle**
- **The MVP stack:** mvp_control, mvp_mission, mvp_msgs, mvp_utilities. The trials ran the commits pinned in
  `dependencies/`.
- **The vehicle's packages:** `race_auv`, `race_auv_description` and `race_auv_config`, and `race_auv_bringup`'s
  vehicle launch files, not the simulation ones.
- **TF and localization.** The planner reads four edges:
  - `race_auv/world_ned → race_auv/base_link`, from the EKF;
  - `race_auv/base_link → race_auv/cg_link` and `→ race_auv/auv_dock_point`, from the vehicle description;
  - `race_auv/base_link → race_station/dock_point`, from the AprilTag fuser.

  It also reads the EKF odometry on `/race_auv/odometry/filtered`; its age gates every tick.
- **A fuser configuration for the vehicle.** The vehicle's `race_auv_bringup/config/apriltag.yaml` runs the detector
  only ("No fuser, no TF"). The trials' fuser configuration is the simulation's
  `config/simulation/apriltag_black_square_edge.yaml`. On the vehicle, one is needed for the cameras as mounted and
  the station as built. Detection in the trials used the CPU backend.
- **`race_auv_docking_planner`, on a computer that sees that TF and odometry.** Its parameter file holds the v1.5
  values, and `initial_setpoint` must equal the pose the controller holds when the planner starts.
  - `tag_pivot_m` (0.456667, 0, 0.22) is the centroid of the simulated station's forward-camera tags. A real station
    needs it derived from its own tag layout; `runtime/tag_pivot.py` derives it offline from a fuser configuration
    and the station URDF.
- **A docking control mode, and the helm's `direct_control` state with `bhv_direct_control`.**
  - The planner publishes its set points on `/race_auv/mvp_helm/bhv_direct_control/desired_setpoints`.
  - This branch has the docking control mode and `direct_control` only as simulation variants
    (`config_sim_docking.yaml`, `helm_sim_docking.yaml`, `bhv_params_sim_docking.yaml`). The vehicle's equivalents
    are the lab's to make; we have not read the vehicle's configuration.
- **Something that commands the hold pose, then calls `/piccard_planner/start`** (`std_srvs/Trigger`). In simulation
  the collector does this; on the vehicle, an operator or a mission.
- **Gains and planner parameters tuned on the vehicle.** The p10 docking gains and the v1.5 planner parameters were
  chosen in simulation.

**Not needed on the vehicle**
- **All of `piccard_docking_recipe/`:**
  - the runners, the trial launch and the scenario wrapper;
  - the collector, the consumption audit and the docking metric;
  - the M2 and M3 metrics and the campaign tools;
  - the candidate check, the seed record and the seed patch;
  - the media recorders, the Dockerfile and the tests.
- **The simulator and its world:** Stonefish, `stonefish_ros2`, `world_of_stonefish`, `race_auv_sim` with its
  ground-truth pose node, and the `race_station` simulation bringup.
- **The simulation configuration and launch files:**
  - the `*_sim.yaml` files and their docking variants;
  - `config/simulation/`;
  - `bringup_simulation.launch.py` and `bringup_docking_simulation.launch.py` with `launch/include/simulation/`.
