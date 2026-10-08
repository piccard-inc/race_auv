# Running a RACE docking trial: from a fresh checkout to a scored result

This page is the one documented way to run the RACE docking trials of Piccard's M3 report
(https://piccard.science/experiments/race-auv-docking-planner-2026-10) on your own machine. It needs no Piccard
image, account or service: Ubuntu 24.04, ROS 2 Jazzy, a GPU-capable X display or Xvfb, and the sources below.

## The path

```bash
# 0. Ubuntu 24.04 with ROS 2 Jazzy (ros-jazzy-ros-base) and the apt packages listed in
#    piccard_docking_recipe/dependencies/DEPENDENCIES.md ("Target").

# 1. Every source, pinned.
mkdir -p ~/race/libs ~/race/ws/src && cd ~/race
BRANCH=https://raw.githubusercontent.com/piccard-inc/race_auv/refs/heads/piccard/docking-recipe
curl -fsSLO "$BRANCH/piccard_docking_recipe/dependencies/libraries.repos"
curl -fsSLO "$BRANCH/piccard_docking_recipe/dependencies/workspace.repos"
vcs import libs < libraries.repos
vcs import ws/src < workspace.repos

# 2. Stonefish and apriltag, installed to /usr/local: from ~/race/libs, the commands in DEPENDENCIES.md ("Libraries"),
#    including apriltag's last block, which makes its Python module importable and checks it (the install alone
#    leaves it where Ubuntu's python3 does not look).
#    To record the simulator seed, apply stonefish_seed_v1 to Stonefish first (see "The simulator seed" below).

# 3. The workspace changes, then the build (DEPENDENCIES.md, "Workspace").
cd ~/race/ws/src
touch race_auv/race_auv_perception/race_auv_apriltag_cuda/COLCON_IGNORE race_station/race_station/COLCON_IGNORE
rm -rf race_auv/race_auv_perception/third_party race_auv/race_auv_perception/scripts
rm -f race_auv/race_auv_config/mvp_control_config/config.yaml race_auv/race_auv_config/mvp_mission_config/helm.yaml
cd ~/race/ws && source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y --skip-keys "dwe_camera_driver race_auv_apriltag_cuda"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --merge-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. One trial. The planner arm:
cd ~/race/ws/src/race_auv/piccard_docking_recipe
packages/simulation/race-auv-docking/runtime/run_native_trial.sh --workspace ~/race/ws \
  --request packages/simulation/race-auv-docking/examples/m3-planner-request.json --output ~/race/trials/planner-1
#    The control arm (the poses, no planner): --request .../examples/m3-contact-request.json

# 5. The scored result.
cat ~/race/trials/planner-1/scored/planner-1.m3.json
```

A trial takes up to its request's wall timeout (1950 s for both examples). `run_native_trial.sh` starts its own Xvfb
display; to watch the simulator, set `PICCARD_EXTERNAL_DISPLAY=1` and `DISPLAY` to your display.

## What step 4 does

`run_native_trial.sh` (in `piccard_docking_recipe/packages/simulation/race-auv-docking/runtime/`) is the native
counterpart of the runner that produced the report's trials. In order:

1. **Splits the request** into gains, mission and context. Numbers are written as Piccard's job service writes them.
2. **Checks the committed docking setup** (`prepare_candidate.py install`, then `verify`). On this branch it confirms
   that the committed docking variants are what the trials used. It writes nothing unless the request's gains differ
   from the committed ones.
3. **Runs the trial** (`run_trial.sh`).
   - The launch is `runtime/launch/docking_sim.launch.py` with the docking variant: the lab's simulation bringup with
     the docking control and helm files and the black-square-edge AprilTag configuration, plus the station.
   - The scenario is `runtime/scenario/race_auv_docking_trial.scn`: the lab's `race_auv_test.scn` unchanged, plus
     AUV–station and AUV–tank contact monitors for scoring.
   - A planner mission runs `race_auv_docking_planner`, with its parameters taken from the mission. A pose mission runs
     no planner.
   - The collector (`runtime/collect_trial.py`) checks that the controller has the requested gains, flies the dive
     (and, for a pose mission, the poses), starts the planner after the dive, and writes `trial.json`, `docking.json`
     and `telemetry.jsonl`.
4. **Scores it** with `tools/analysis/race_m3_metrics.py` into `<output>/scored/`.

**Ground truth stays out of the vehicle.** The station's own `robot_state_publisher` and the ground-truth pose node run
with `/tf` and `/tf_static` remapped to `/piccard/ground_truth/tf` and `/piccard/ground_truth/tf_static`.
- This leaves the AprilTag fuser as the only source of `race_station/dock_point` in the TF tree the planner reads.
- If the station bringup in `bringup_simulation.launch.py` is uncommented without that remapping, the station's TF
  gives `race_station/dock_point` a second, simulator-placed parent.
- The collector audits every run for this: which nodes consume ground truth and who publishes into the station's
  frames.

## The simulator seed

Every trial now records `simulator_seed` in its `trial.json`. What we verified:

- **What the seed covers.** The pinned Stonefish (`7d52673`) seeds its sensor-noise generator from
  `std::random_device` when it loads. Each run gets a new seed, which it neither prints nor lets you set. In the RACE
  vehicle (`world_of_stonefish` `vehicles/race_auv.scn`) the seed reaches only the pressure sensor's noise (2.0 Pa)
  and the DVL's (velocity 0.01 m/s, altitude 0.03 m). The scene sets the IMU's noise to zero.
- **Making it settable.** `stonefish_seed_v1` seeds that generator from `STONEFISH_SEED` and prints the seed it used.
  The patch is in `piccard_docking_recipe/packages/simulation/race-auv-docking/simulator-patches/`, listed by
  SHA-256 in `DEPENDENCIES.md`.
- **What the runner does.** It chooses the seed (the mission's `seed`, or a random draw) and exports it.
  - With the patch, `trial.json` records the seed the simulator printed (`support: stonefish_seed_v1`).
  - Without it, the value is null (`support: not_exposed_by_pinned_runner`), next to the seed that was exported and
    ignored.
- **A seed fixes the sequence of sensor-noise draws, not the trajectory.** The simulator, the EKF, the controller, the
  AprilTag fuser and the renders run asynchronously, so which draw serves which sample depends on timing. Two runs
  with one seed share their noise, not their path.
- **The report's trials recorded no seed, in either arm.** They ran on the pinned Stonefish. Each `trial.json` holds
  `simulator_seed: {"value": null, "support": "not_exposed_by_pinned_runner"}`.

## Where results go

Experiment results stay out of this repository. The canonical record is Piccard's (with hashes) and the public report
page. Each experiment leaves one pointer here: a short results note keyed to the exact commit of this branch it ran
on, with the page link, the record hash and the simulator seed. Per-trial metric summaries (small JSON, no hosts or
costs) are added beside the code only if the lab asks.

## Where things are

| Path | What it is |
|---|---|
| `race_auv_docking_planner/` | The planner: a ROS 2 package with its v1.5 parameters and launch file. It reads the EKF odometry and the TF tree only. |
| `race_auv_config/.../*_docking.yaml`, `race_auv_bringup/.../*docking*` | The docking setup, as opt-in variants beside the unchanged defaults. |
| `piccard_docking_recipe/dependencies/` | The pinned sources, the build, the patches by hash, the licences. |
| `piccard_docking_recipe/packages/simulation/race-auv-docking/` | Piccard's trial recipe: the runner, the launch, the scenario wrapper, the collector and the example requests. Its `README.md` describes every file. |
| `piccard_docking_recipe/tools/analysis/` | The scoring: the M3 and M2 metrics and their checks. |
