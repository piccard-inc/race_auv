# race-auv-docking/v1

A bounded RACE AUV docking trial in Stonefish, run by the campaign executor like `alpha-rise-gains-only/v1`.
One trial means:

1. Install one two-mode gain candidate and a `direct_control` helm state in a disposable container.
2. Start the pinned RACE vehicle stack, docking station and docking world.
3. Fly an ordered list of `cg_link` poses in the controller's `world_ned`, holding each pose for its dwell time.
4. Record telemetry, `trial.json` and `docking.json`.

Claim boundary: synthetic simulation development only. There is no physical docking, contact-mechanics,
perception-accuracy or lab-acceptance claim. Ground truth is used for scoring only.

Tracking: piccard-inc/piccard-physical-ai#79 (milestone piccard-experiments #87).

## Layout

| Path | Role |
|---|---|
| `Dockerfile` | `FROM` the pinned RISE base. Adds an `/opt/race_ws` overlay from sha256-checked tarballs (`source-lock-v1.json`), plus AprilRobotics apriltag at `dc6316d` (the first commit whose Python binding has `estimate_tag_pose`) for the CPU detector. |
| `recipe-v1.json` | Recipe descriptor. Covers the fixed source, fixed execution, image digests, overlay sources and notes: tag-size bias evidence, build dependencies, the controller frame and per-trial cost. `controller_variant` is `upstream`: the lab's mvp_control as pinned. |
| `recipe-keepxyint-v1.json`, `Dockerfile.controller-variant`, `controller-variants/` | The controller-variant arm (M3-C, piccard-experiments #88): Piccard's modification of mvp_control, built FROM the recipe image; see "Controller variant" below. |
| `request-v1.schema.json` | The executor claim request: `gains`, `mission`, `horizon_s`, `wall_timeout_s`, `context`. |
| `conformance-v1.json`, `test_conformance.py` | Accept/reject cases for the schema and the runtime validators. Each case states both verdicts. They differ where the runtime is stricter, and for an integer `seed`: the request schema rejects it, while the runtime accepts it for native runs. |
| `examples/m1-smoke-request.json` | The M1 smoke request. |
| `examples/m3-default-request.json` | The M1 smoke request with `apriltag_tag_size` `black_square_edge`, the tag-size convention for M3 requests (#107). |
| `examples/m3-planner-request.json` | An M3 planner mission (#109) at the M2 selection p10: M1's dive as the fallback pose, then the planner, with the values proposed on piccard-experiments #88. |
| `examples/m3-contact-request.json` | The M3 report's control arm at p10: Phase R-A's contact pose mission with black-square-edge tag sizes, the mission and gains the scored control-arm trials recorded. |
| `campaigns/phase-r-a-20260928/` | The M2 Phase R-A pose missions and their derivation (#96). The jobs files come from `tools/campaigns/build_phase_r_campaign.py`; the metrics from `tools/analysis/race_m2_metrics.py`. |
| `campaigns/tank-floor-v1.json` | The tank's interior floor and the AUV's footprint, from the pinned world_of_stonefish (`tools/campaigns/extract_tank_floor.py`). The builder checks every pose for 0.3 m of floor clearance with it (#102). |
| `runtime/run_campaign_trial.sh` | Container entrypoint: install → verify → `run_trial.sh`. |
| `runtime/run_native_trial.sh`, `runtime/native_request.py` | The same trial without the image, on a colcon workspace built from `piccard-inc/race_auv` `piccard/docking-recipe`: request → install and verify → `run_trial.sh` → `race_m3_metrics`. See "Run natively". |
| `runtime/prepare_candidate.py` | Validates and installs the gains, helm state and tag-size variant, or on a `piccard/docking-recipe` workspace checks the committed variants and writes only differing gains. Verifies them before launch. |
| `runtime/mission.py` | Validator of both mission kinds: `piccard.race-auv.pose-mission/v1` and `piccard.race-auv.planner-mission/v1`. |
| `runtime/planner_fused_dock.py` | The M3 planner node (#109): a staged approach on the tag-fused station dock point, publishing `direct_control` set points. It hears only the TF tree and the EKF odometry. |
| `runtime/tag_pivot.py` | The planner's tag pivot: the forward camera's tag centroid in the station dock frame, derived from the `apriltag.yaml` and station URDF the fuser loads. The collector checks the mission's `tag_pivot_m` against it. |
| `runtime/launch/docking_sim.launch.py` | The trial's launch file. It mirrors upstream `bringup_simulation.launch.py` with no rviz, joystick or C2, and adds the station and the ground-truth node. |
| `runtime/scenario/race_auv_docking_trial.scn` | The upstream `race_auv_test.scn`, included unchanged, plus AUV↔station and AUV↔tank contact monitors. |
| `runtime/collect_trial.py` | Readiness, the perception gate (planner missions), consumption audit, gain readback, `direct_control`, the pose loop or the planner handover, telemetry, `trial.json` and `docking.json`. |
| `runtime/graph_audit.py` | The consumption audit `collect_trial.py` runs: what the helm and controller consume, and whether ground truth reaches the vehicle stack. It needs no YAML or ROS, so the native bridge's evidence check recomputes it from an archived graph. |
| `runtime/docking_metric.py` | Pure-Python dock-point geometry, frame validation and the `docking.json` summary. |
| `runtime/check_image.py` | Build-time check against the real pinned files. |
| `runtime/simulator_seed.py` | The simulator seed: chosen before the launch (the mission's, else drawn), exported as `STONEFISH_SEED`, and recorded in `trial.json` from what the simulator printed. |
| `simulator-patches/stonefish_seed_v1.patch`, `STONEFISH_SEED_V1.md` | The Stonefish patch that makes the sensor-noise seed settable and printed. The docking image does not carry it yet; see "Simulator seed" below. |

## Request

`gains` follows `piccard.race-auv.gains/v1` and has two modes. Each axis is `{p, i, d, v, pid_min, pid_max}`.

- `flight`: `u, v, z, roll, pitch, yaw`. Output limits must equal the pinned `config_sim.yaml`: u ±50, v ±15,
  z ±30, roll ±10, pitch ±20, yaw ±15.
- `docking`: `x, y, z, roll, pitch, yaw` of `cg_link` in `world_ned`.
  - `z`, `roll`, `pitch` and `yaw` limits must equal the flight limits.
  - `x` and `y` limits come from the request (`pid_min < 0 < pid_max`). The runtime has no x/y defaults.

The gain check is well-formedness only: finite values, `p`/`i`/`d`/`v` in [0, 10000]. The M1 smoke uses these
gains, recorded in its `context`:

- upstream flight gains;
- docking z, roll, pitch and yaw copied from flight;
- the labelled "Piccard nominal starting point" for docking x/y (p 5, i 0.2, d 0, v 5, ±15). This is not tuning.

`mission` is one of two kinds, told apart by its `schema` literal. The first is `piccard.race-auv.pose-mission/v1`:

- `frame_id` is `world_ned` and `child_frame_id` is `cg_link`.
- `poses` holds 1–20 entries of `{label, x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad, dwell_s}`. Labels are unique.
- `seed` is `null` in a request: the request schema allows nothing else. A native run's mission may give an integer
  in [0, 4294967295]. Null makes the runner draw one. See "Simulator seed".
- `apriltag_tag_size` is `lab_configured` (the default) or `black_square_edge`.

The second is `piccard.race-auv.planner-mission/v1` (M3, #109). It has the same header, plus:

- `fallback_pose`: one pose, flown like a pose-mission pose. It is the dive: the tags are not visible from the surface.
- `planner`: the parameters of `runtime/planner_fused_dock.py`, all required:
  - `standoffs_m`: strictly decreasing and ending at 0;
  - `tick_hz`, `band_m`, `band_rad`, `vertical_band_m`, `approach_clearance_m`, `settle_s`, `speed_cap_mps`,
    `final_stage_s`, `final_deadband_m`, `final_along_deadband_m`, `final_along_interval_s`, `final_arrival_m`,
    `final_reapproach_s`, `max_estimate_age_s`, `max_frozen_estimate_s`, `estimate_filter_s`,
    `heading_window_s`, `setpoint_deadband_m` and `setpoint_deadband_rad`. `mission.py` also holds each deadband to
    at most its band: `setpoint_deadband_m` to `vertical_band_m` and `band_m`, `setpoint_deadband_rad` to
    `band_rad` (protocol v1.3). It holds `final_arrival_m` below `final_along_deadband_m` (protocol v1.5);
  - `tag_pivot_m`: three numbers, the forward camera's tag centroid in the station dock frame.
  - The station estimate is a level station at a heading, plus its dock point. The fuser cannot give the heading at
    short range: on the approach it sees only the forward camera's three tags, all on the station's vertical
    centreline, and with three it solves on their centres, which leaves yaw about that line unconstrained (M3-0:
    fused yaw sd 35–44° at 1.5 m, piccard-experiments #88).
    - The heading is the circular median of the fused heading over `heading_window_s`. The window fills from the
      fallback pose's dwell (the planner observes while it waits), so the handover goal has a full window. From the
      second stage on, the heading is held. On the M3-0 rerun, cam_front saw only tag 146 at the handover (7.45 m),
      and the single-tag solve flips between two headings about 13° apart. The first sample alone set the stage-0
      goal 11° off; the window's median would have been 2.5° off.
    - At the final stage's first fresh tick (protocol v1.2), the held heading is replaced once. The new value is the
      circular median of the fresh fused headings of the stage before it (0.3 m), within `heading_window_s`. There
      tags 541 and 558 both solve, without the three-tag degeneracy of 1.5 m. With fewer than 5 fresh samples the
      held heading is kept. Either way the heading is held from then on, so single-tag flips near the dock never
      reach the command. The yaw set point steps to it once, at that stage start.
      - On the v1.1 run the held heading was 1.88° off. Over the 0.457 m pivot, that put the re-anchored dock point
        about 1.5 cm across.
      - Each state record carries `heading_rehold`: `t`, `samples`, `applied`, `previous` and `heading`.
    - The dock point is re-anchored: the fused pose's image of `tag_pivot_m` (which the tag centres pin well),
      minus the pivot rotated by the level station at the heading. It is low-passed with `estimate_filter_s`.
    - `tag_pivot_m` is station construction, not station pose. At trial start the collector derives it from the
      `apriltag.yaml` and station URDF the fuser loads (`runtime/tag_pivot.py`). The trial fails closed
      (`tag_pivot_mismatch`, `tag_pivot_underivable`) if the mission's value differs by more than 1 mm.
  - The stage target places the AUV dock point stand-off s behind the station estimate and `approach_clearance_m`
    above it, in every stage including the final one (protocol v1.3). The vertical error and band are around that
    target.
    - Why: on the M3-0 v1.2 run, with the dock points level, the AUV's legs met the station frame at its rail
      entrance, 0.81 m before the dock point. Swept along the approach at that depth, the legs interfere by
      4.2–6.6 mm from 0.68 to 0.13 m of stand-off. At 2.2 cm above, v1.1's depth, they clear by 15.7–22.7 mm.
    - The controller cannot descend onto the dock without a depth step, so the clearance holds to the end. The
      sim's reachable docked state is the hover at the clearance; race_m3_metrics scores it as
      `closure_at_clearance`.
    - `vertical_band_m` is the stage-advance test around the clearance, not a clearance guarantee. On the M3-0
      v1.3 run the vehicle held +2.19 cm (2.16–2.25) in the final stage. The legs' tightest point, at 29 cm of
      stand-off, cleared by 8.9 mm; at the band's lower edge (+1 cm) that would be about −1 mm.
  - A stage other than the last ends after `settle_s` with the fused error inside the band:
    - `band_m` along;
    - `vertical_band_m` vertical (protocol v1.2; `band_m` in the final stage);
    - `band_rad` in heading;
    - `band_m + s * band_rad` lateral at stand-off s. The lateral target error is s times the error in the
      estimated station heading.

    v1.1 held `band_m` (5 cm) vertical as well, and its M3-0 run reached the dock 2.2 cm shallow: inside that
    band, so it was never corrected.
  - A tick is stale when an input is missing or older than `max_estimate_age_s` (`age`), or when the fused dock
    point's stamp advances but its transform stays bit-identical for longer than `max_frozen_estimate_s`
    (`frozen`: a frozen render keeps the fuser publishing one pose under fresh stamps). A stale tick holds the set
    point and touches no clock.
  - mvp_control zeroes an axis's integral whenever its set point changes, and the depth integral carries the
    buoyancy. So the held set point changes in discrete steps, each axis only past its deadband
    (`setpoint_deadband_m`, `_rad` for yaw), at a stage start and in a refinement step alike (protocol v1.3): x, y
    and yaw when a stage starts; then the axes a fresh fused error is outside the band on. A refinement check waits
    until the vehicle has been at its set point for `settle_s`: cg_link within `band_m` of it on x and y and within
    `vertical_band_m` on z (v1.3).
    - Why: v1.2 stepped a flagged axis whatever the difference, and its arrival gate held z to `band_m` (5 cm). On
      its M3-0 run each z step's integral reset raised the AUV 50 cm, and it took about 150 s to settle inside
      1 cm. The gate reopened at about +100 s and measured the controller's own tail (1.1–1.9 cm either way) as
      station error. It then stepped z by 0.1–1.1 mm, 13 times in 1450 s of stage 0.
    - In the final stage (protocol v1.2), x and y follow the live goal, walked, with no settle wait. Yaw keeps the
      stage start's, depth is kept, and `final_stage_s` still counts from the stage start.
    - From protocol v1.4, each world axis re-targets on its own, when its goal differs from its held value by more
      than `final_deadband_m`. v1.2–v1.3 used the Euclidean x/y difference and moved both axes together.
    - Why: mvp_control zeroes only the changed axis's integral, and the along integral is what brings the vehicle
      in. On the M3-0 v1.3 run the final stage moved x and y together 47 times, a median of 2.2 s apart. The
      along integral (time constant about 50 s) never built, and the vehicle crept on P alone at 0.3 mm/s. It
      ended 4.7 cm short of a set point within 0.2 cm of the true dock point.
    - The lateral axis keeps that rule. The along axis is the world axis closer to the approach (y at the RACE
      station's heading). v1.4 gated it on the fused along error past `final_along_deadband_m` and an interval.
    - What the v1.4 runs showed. On both M3-0 v1.4 runs the vehicle moved at most 0.5 mm/s below about 0.6 of x/y
      effort, which is a 6 cm error on P alone. So what closes the last centimetres is the integral wound up on the
      stage start's approach, and it also carried the vehicle 2 cm past the dock. The error gate then held a
      command that the EKF frame had drifted 6.8 cm off the true dock point, until the vehicle had been pulled out.
    - From protocol v1.5 the along axis moves through phases:
      - **Approach**, from the stage start. It is re-targeted only when its command is stale: the goal past
        `final_along_deadband_m` from the held value, and `final_along_interval_s` since its last change.
      - **Arrival**, on the first fresh tick whose fused along error is within `final_arrival_m` of the goal on the
        approach's side. The along set point is re-issued at the goal once: a set-point change that zeroes the
        approach's windup where the vehicle is. If the goal equals the held value, the re-issue is nudged by
        0.1 mm, so it is still a change.
      - **Hold.** The along axis follows the goal past `final_deadband_m`, like the lateral one.
      - **A new approach**, from whichever side, once the fused along error has been past
        `final_along_deadband_m` for `final_reapproach_s`. A stale tick neither restarts nor ends that clock.
      - The state record's `final_along`, and trial.json's `planner.final_along`, give the phase and each arrival
        and re-approach.
    - Under a controller that keeps the x/y integrals (M3-C, retired), no set-point change clears the windup.
    - v1.1 fixed the final goal at the stage start. On its M3-0 run the EKF frame drifted 13.2 cm across the
      approach during the 120 s final stage. The planner saw its lateral error grow to 12 cm but did not act.
  - The command walks to each stage's held goal (protocol v1.1). After every stage start, and after every
    final-stage re-target, the x/y set point moves from the last command at most `speed_cap_mps` and lands exactly.
    Yaw is the goal's from the stage start, and depth is kept. The refinement steps are not walked, and wait until
    the walk has landed. A stale tick pauses the walk.
    - While it walks, mvp_control zeroes the x/y integrals every tick: the anti-windup wanted there.
    - v1.0 walked only the final stage. On the M3-0 rerun, the 4.1 m handover step held the sway thruster at its
      ceiling from 42 to 252 s of mission time. The AUV turned 95° off the station and lost the tags for 197 s.
  The runtime checks only that they are well-formed. The values are the preregistration's (piccard-experiments #88).
- `apriltag_tag_size` defaults to `black_square_edge` here.

The poses are in the controller's dead-reckoned `race_auv/world_ned`, not the Stonefish world:

- The pinned EKF does not fuse absolute yaw, so `race_auv/odom` is base_link's start pose.
- `description.launch.py` puts `world_ned` at rpy (3.1415, 0, 1.571) from `race_auv/odom`.
- The AUV starts heading north with base_link at Stonefish world x −3.8.

As a result, the controller frame is the Stonefish world rotated +90° about z. The arm64 dev loop measured this:

```
x_C = -y_W      y_C = x_W + 3.8      z_C = depth      yaw_C = yaw_W + 1.571
```

The pose bounds are the tank interior mapped into that frame: x_C ±3.04 m, y_C −0.77 to 8.36 m, depth 0 to 4.30 m (#104).
- The tank mesh is a double shell, and the vehicle swims in the inner box: x_W ±4.57, y_W ±3.05, floor at about
  4.49 m (`campaigns/tank-floor-v1.json`).
- The depth bound keeps the vehicle's lowest point, 0.18 m below cg_link, above that floor.
- The earlier bounds (x_C ±3.66, depth to 5.10) were the outer shell's.
There is no stand-off floor. The collector records the measured alignment and the drift at every dwell end.
`mission.controller_from_ground_truth` converts a Stonefish-world pose.

## What a trial does

1. **Install and verify** (`run_campaign_trial.sh`). On a workspace built from `piccard-inc/race_auv` branch
   `piccard/docking-recipe` (race_auv#1), the docking setup is committed there as opt-in files:
   `config_sim_docking.yaml`, `helm_sim_docking.yaml`, `bhv_params_sim_docking.yaml`,
   `apriltag_black_square_edge.yaml` and the `*_docking_sim` launch includes.
   - `prepare_candidate.py` generates none of it. It checks each committed variant against what it would have
     generated from that variant's default, and refuses the run on any difference or on a partial set.
   - **What it still writes, and why:** only the request's flight and docking gains, into both copies of
     `config_sim_docking.yaml`, and only when they differ from the committed ones. The recipe lets a request vary
     P/I/D/V and the docking x/y limits; the committed file carries the planner study's p10 gains.
   - The record names `launch_variant: docking`, so `run_trial.sh` passes `variant:=docking` and the launch
     includes the committed controller and helm files.
   - **On the pinned image** (race_auv `da58963c`, no variants), it installs as before, byte for byte, until
     the image is re-pinned to the branch:
   - Writes `control_modes.flight` and a new `control_modes.docking` into both copies of `config_sim.yaml`.
   - Adds two entries to `helm_sim.yaml`:
     - the `direct_control` state: control mode `docking`, transitions to and from `start` and `kill`;
     - the behavior `bhv_direct_control`: `plugin: helm/DirectControl`, `priority: {direct_control: 1}`.
   - Adds `bhv_direct_control` to `bhv_params_sim.yaml`: `default_bhv_world_link: world_ned` and
     `default_bhv_child_link: cg_link`.
   - With `black_square_edge`, also writes `output/apriltag-black-square-edge.yaml` with tag sizes ×0.8, for
     the global list and each camera's list.
   - Verifies everything against the request.
2. **Launch** (`run_trial.sh`).
   - Starts graphical Stonefish at upstream's rate, window and quality. Production uses the host's Xorg `:99` on
     the NVIDIA GPU; development uses Xvfb.
   - Starts the RACE drivers, localization, description, controller, helm, the CPU AprilTag pipeline, the station
     bringup and upstream's ground-truth node.
   - The station bringup and the ground-truth node publish TF on `/piccard/ground_truth/{tf,tf_static}`. This keeps
     ground truth off the vehicle's TF tree and leaves the AprilTag fuser as the only parent of
     `race_station/dock_point`.
3. **Collect** (`collect_trial.py`).
   - **Readiness:** helm in `start`; controller values, both ground-truth odometries and the EKF odometry are
     flowing; services are up; the DirectControl subscriber is present; the `race_auv/world_ned → race_auv/cg_link`
     TF resolves.
   - **Perception (a planner mission):** within 30 s of readiness, both AprilTag detectors must be running. Each
     camera's `apriltag_detection/detections3d` topic needs a publisher and an array no older than 5 s. The detector
     publishes one on every tick once frames arrive, with tags in view or none. Otherwise the trial fails as
     `perception_not_running:<cameras>` before the mission, and the scorer excludes it. verify-planner-1 ran to its
     horizon with both detectors dead from startup (`provenance/verification-185/verify-planner-1/`).
   - **Audit:** records every node's subscriptions and the `/tf` publishers and edges. The trial fails if:
     - the helm or controller consume odometry other than `/race_auv/odometry/filtered`;
     - a vehicle node subscribes to ground truth;
     - ground-truth TF reaches the vehicle tree (`ground_truth_tf_on_vehicle_tree`): a ground-truth publisher on
       `/tf`, a station edge other than the fuser's dock point, or a frame with two parents;
     - a `/tf` frame is neither a vehicle/station frame nor the fuser's per-observation tag frame
       `race_auv/base_link → apriltag_<family>_<id>`, which is accepted while `/apriltag_fuser` publishes `/tf`
       (`unknown_tf_frame_on_vehicle_tree`);
     - any `alpha_rise` node runs;
     - for a planner mission (`planner_inputs`):
       - the planner node is missing, or runs without a planner mission;
       - it subscribes beyond `/tf`, `/tf_static` and `/race_auv/odometry/filtered`;
       - it reports a TF lookup beyond `world_ned → base_link`, `base_link → race_station/dock_point`,
         `base_link → auv_dock_point` and `base_link → cg_link` (`planner_reads_other_inputs`).

     The audit runs before the mission and again at the end. A problem at the end fails an otherwise completed
     trial. `/stonefish_simulator` always holds a `/tf` broadcaster, but it only sends
     `world_ned → <robot>/base_link` when a scenario enables it; that edge would fail the audit.
   - **Gain readback:** reads both cached modes and the controller parameters and checks that
     `odometry_source` is the EKF odometry.
   - **Start:** publishes the first setpoint, enables the controller, changes the helm to `direct_control`,
     waits for control mode `docking`, then reads back the active docking gains.
   - **Pose loop:** each pose is re-published at 1 Hz and must be echoed by the controller within 10 s.
   - **Planner mission (#109):**
     - The collector flies `fallback_pose` like a pose.
     - It then calls `/piccard_planner/start` and keeps re-publishing the fallback set point until the planner's
       first command. After that it stops publishing, and the planner's set points must be echoed within 10 s.
     - It records each planner stage as `pose_start`/`pose_end` events, labelled `planner_standoff_<s>m`, until the
       planner reports complete (the mission is then complete) or the horizon ends.
     - The trial fails if the planner's state goes stale for 2 s or its parameter hash differs from the mission's.
     - `run_trial.sh` starts the planner as its own process group.
   - **Failure conditions:** stale streams, `kill`, leaving `direct_control`, or the horizon. The horizon marks
     the trial `budget_censored`, not failed.

## Artifacts

- `telemetry.jsonl`: one row per message or event. Each row has
  `{kind, t, monotonic_ns, wall_time_ns, topic, sim, ...}`. Kinds:
  - `gt_auv` and `gt_station`: pose, quaternion, linear and angular velocity;
  - `dock`: the 10 Hz ground-truth dock-point relation;
  - `alignment`: controller-frame vs ground-truth `cg_link`, 2 Hz;
  - `odometry` (EKF), `value`, `set_point`, `controller_error`, `pid_*` and `thruster`;
  - `helm_state`, `setpoint_command`;
  - `planner`: the planner's state per tick (stage, stand-off, error, goal, held set point, command) for a planner
    mission;
  - `detections` (ids and camera-frame positions per camera), `fused_dock` and `fused_tf`;
  - `contact` (location and normal force);
  - `tf_static`, `gt_tf_static`, `gt_upstream`;
  - `event`.
- `trial.json` (`piccard.race-auv.trial/v1`):
  - status and stop reason;
  - the mission and its sha256;
  - expected, cached, parameter and active gains;
  - scenario provenance: the resolved `world_of_stonefish` share path and the sha256 of the wrapper, the
    included upstream scenario files and `sim_params.yaml`;
  - `simulator_seed`, always present (see "Simulator seed"):
    - `value`: the seed the simulator printed, or null;
    - `support`: `stonefish_seed_v1`, or `not_exposed_by_pinned_runner`;
    - `exported` and `origin` (`request` or `drawn`): what the runner set;
    - `applied`: whether the printed seed is the exported one;
    - with the patch, `source` and the seed of each generator;
  - the consumption audits before the mission and at the end;
  - `nonfinite_values`: where non-finite numbers occurred, as the first 50 rows of each class plus full counts.
    `expected` holds the stack's by-design infinities (`HelmState.max_duration = inf` in helm `change_state`
    responses); `unexpected` holds everything else, so a nonzero `unexpected.rows` is worth reading.
    `invalid_numeric_events` still counts rows of both classes;
  - observations: `direct_control` reached, ground-truth samples, tags detected, fused dock-point TF and pose
    samples, contact events, frame validation;
  - `planner`: null for a pose mission. For a planner mission:
    - `node`, `parameters`, and `parameters_sha256` (the parameters as JSON, sorted keys, no spaces), plus the hash
      the planner itself reported;
    - the stages it entered with their start times, `complete` and the handover time;
    - its state, command and stale row counts, and `stale_intervals` (`from_t`, `to_t`, `reason` age or frozen;
      `to_t` null if still stale at the end);
    - `tag_pivot`: the mission's value, the derived one (with the tags, the `apriltag.yaml` and the URDF it came
      from), the largest difference and `ok`;
    - `setpoint_steps`: each discrete step of the held set point as `[t, x, y, z, yaw]` (cg_link in
      `race_auv/world_ned`, m, rad) at the first state record whose `setpoint_updates` changed. From v1.1 the x/y
      command walks to each held value after a stage start, so a stage start's entry is where the walk lands. From
      v1.2 each final-stage re-target is an entry too;
    - `heading_rehold`: the final stage's heading re-hold, as the planner recorded it, with `received_t` in
      collector time. The planner's own `t` is on its clock. It is null until the final stage starts;
    - `approach_clearance_m` (v1.3): the mission's clearance, which every state record's `goal` also carries.
  - `tags`: every enabled camera's configured tags from the installed `apriltag.yaml` the fuser loads, each with its
    position in the station dock frame (from the station URDF it names). `size_m` gives the size in m under both
    conventions (`lab_configured` and `black_square_edge`), and `installed` names the one the fuser was given. An
    error is recorded rather than raised.
- `docking.json` (`piccard.race-auv.docking/v1`):
  - per pose: the dock-point distance, relative position (station dock frame), relative rpy, orientation error
    and AUV speed at dwell end; dwell statistics; controller-frame drift; the fused-vs-ground-truth dock point;
  - the trial minimum distance;
  - the controller-frame alignment (yaw and origin in the Stonefish world);
  - contacts;
  - perception first-seen times, tag ids and identity-pose counts per camera;
  - `frame_validation`: the constants vs the published URDF TF chain and vs upstream's ground-truth node.
- `candidate.json`, `candidate-install-verification.json`, `runner.json`, `launch.log`, `collector.log`,
  `glxinfo.txt`; with `--video`: `display.mp4`, `display-poster.png`, `media.json`, `follow-view.json`, `ffmpeg.log`;
  with `--onboard-camera-video`: `cam_front.mp4`, `cam_front-poster.png`, `cam_front-frames.csv`,
  `onboard-recorder.json`, `media.json`.
- The display and the wrapper (#122):
  - `display-watch.jsonl`: `runtime/display_watch.py` polls `xdpyinfo` at 1 Hz and records each change of the
    display's state: `up`, `lost` (with xdpyinfo's stderr), `unresponsive` or `no_xdpyinfo`. Each record carries
    `wall_time_ns` and `monotonic_ns`, like the telemetry. It only observes: a display loss still fails the trial
    closed through the streams it stops.
    - A server replaced between two polls answers both. In M3-0 v1.2 the host restarted its X server under the
      trial, and 1802 polls saw one state. Two checks of the server's identity catch that:
      - `server_changed`: each poll stats the server's socket, `/tmp/.X11-unix/X99` (the host directory is
        mounted into the container). A new server creates a new socket, so its inode and ctime change. A missing
        socket is a `socket` state, recorded once per change.
      - `connection_broken`: one long-lived `xprop -root -spy WM_NAME` holds a connection while the display is
        up. A server that goes away breaks it, even between polls; the exit code and xprop's stderr are kept.
        A new connection opens on the next poll that finds the display up.
    - What they cannot see:
      - a reset inside the same server process, which keeps its socket. Xorg regenerates when its last client
        disconnects, and the watchdog's own connection is a client throughout;
      - why a server went. The host keeps that side in `host-display/`.
    - The stop record adds `server_changes` and `connection_breaks`. At teardown the watchdog ends its xprop
      itself, and a break seen while stopping is not recorded.
  - `xvfb.log`: development only. In production the display is the host's Xorg, whose evidence the host keeps in
    `host-display/` beside `output/` (`tools/alpha-rise-campaign`).
  - `wrapper.log`: `run_campaign_trial.sh`'s own stdout and stderr. The host's `container.log` has the same text.
    Both are usually empty: the wrapper writes there only when it fails itself.
  - `collector.log` holds only the collector's uncaught errors and rclpy warnings. The collector records its own
    failures in `trial.json` (`stop_reason`, `error_type`).
  - `runner.json`'s inventory also lists `glxinfo.txt`, the watchdog's files and the helpers' logs. `wrapper.log` is
    still open when it is written, so it is not listed.

## Media (issue #86)

`--video` records the Stonefish window, turned on by the executor when a request sets `media.display_video`.
- **Recording.** ffmpeg x11grab at 1200×800 and 10 fps, libx264 ultrafast CRF 25, with the bitrate capped so a
  full-length clip stays within 60 MB. The cursor is hidden.
- **View.** Stonefish 1.5 has no scenario element for the GUI camera; the trackball centre can only be chosen in
  the window's VIEW panel. `runtime/follow_view.sh` therefore replays that interaction with xdotool on the pinned
  1200×800 GUI:
  - trackball centre set to `race_auv`. The click is confirmed from the combo box's text pixels and repeated
    only while the box still reads "Free", because a click before the scenario loads is lost;
  - a right-drag to look north (the approach direction), about 22° down;
  - zoom to an orbit of about 3.1 m, so the camera sits about 2.8 m behind and 1.1 m above the AUV.

  Each step is logged in `follow-view.json`. A failure leaves the default view, which shows the tank from above
  the water, and never affects the trial. For about the first 0.7 m of the approach, the camera's orbit position
  is still outside the tank's south wall, so the wall hides the AUV. `media.json` reports this as
  `first_centred_video_s`.
- **`media.json`** (`piccard.race-auv.media/v1`), written after the recorder stops. Per clip it has:
  - path, bytes, sha256 and an ffprobe summary;
  - a poster PNG taken at the middle of the hold pose, mapped to video time;
  - the GL renderer that drew the window, plus a software-renderer flag;
  - sampled-frame checks: black frames, and the fraction of samples with the yellow hull in the middle third;
  - the measured offset between the video and collector clocks. That is the x11grab first-frame wall time
    against the collector's first telemetry row; the mapping is `collector t = video time + offset`.

  `role` is `illustrative_media_only` and `publication_authorized` is `false`. Clips over the 100 MB budget are
  dropped and recorded as dropped, so the API never sees an over-cap report.
- **Presentation view (#93).** Once the follow view is confirmed, `follow_view.sh` sends one [H] press, which
  hides Stonefish's GUI panels and its status bar. There is no panel-only toggle, so the simulation-time readout
  goes too.
  - The result is confirmed from pixels: the status bar's static "Hit [K] for keymap" label stops correlating
    with its capture taken while shown.
  - The press is retried at most once, and only while the bar is still seen. [H] is the only key ever sent: ESC
    would quit the simulator.
  - A failure leaves the HUD shown and never affects the trial. `follow-view.json` records the press count and
    the correlation.
  - `media.json` states `source.view.gui` (`hidden` or `shown`) and `source.view.clock`. A hidden-GUI frame
    carries no burned-in clock, so the measured video-to-collector mapping is the only timestamp.
- **Isolation.** The collector and launch invocations are the same with or without `--video`. Recording adds
  no ROS node, so `trial.json` fields and audits are unchanged.
  - That isolation is structural. Compute isolation is **not established**: x11grab and libx264 share the host's
    CPUs with the simulator.
  - In the first two RISE media reruns on the L40S (2026-09-28), overshoot reproduced the scored trials within 2
    points and saturation matched. Settling inside the ±0.08 m band was slower on most steps, for example
    136 s vs 85 s. A single repeat pair cannot separate run-to-run variance from recording load.
  - **Until settled, scored trials do not record; media reruns only.** Before M2 records anything on scored trials:
    three paired no-video/with-video runs on the L40S. Compare settling times and the collector's timing
    (setpoint period jitter, telemetry gaps). If recording perturbs settling, scored trials never record.

`--onboard-camera-video` (`media.onboard_camera_video`) records the simulated front camera:
- **Recorder.** `runtime/onboard_recorder.py` runs one ROS node, `/piccard_media_recorder`, with a single
  subscription to `/race_auv/cam_front/stonefish/data/image_color` (1600×1200, declared at 10 Hz). It never
  publishes and never touches ground truth.
  - **QoS.** Reliable, keep-last 2. The publisher is reliable, and a best-effort Python reader of these 5.8 MB
    frames lost about 90% of them in the dev loop. A keep-last reader that falls behind overwrites its own
    history rather than holding the publisher back.
  - **Encoding.** Frames are halved to 800×600 and encoded with libx264 ultrafast CRF 25, with the bitrate
    capped at 40 MB for a full-length clip.
  - **Timeline.** Each frame is placed on a 10 fps timeline at its receive time and held until the next one
    arrives, so the clip plays in real time whatever rate the camera renders at. The camera stamps are wall
    time. The arm64 dev loop's software renderer publishes about 0.8 Hz.
  - **Back-pressure.** If the encoder falls behind, frames are dropped and counted rather than slowing ROS.
    `onboard-recorder.json` summarizes the frames received, written, dropped, merged and rejected.
- **`media.json`.** The `cam_front` clip gets the same fields as the display clip.
  - Its poster is taken at the middle of the hold, mapped to video time, so it shows the camera frame on
    screen at that moment.
  - Its synchronization is the same single offset as the display clip: `collector t = video time + offset`, to
    within one 0.1 s frame. The offset is measured from the first frame's receive time. It also reports the
    measured camera rate, and `cam_front-frames.csv` places each camera frame on the timeline.
  - The content checks skip the vehicle-centred test, because the camera rides on the AUV.
- **Audit.** The recorder is a real subscriber, so the consumption audits list it rather than hide it.
  - It appears under `media_nodes` (with `allowed_topics`), not under `subscriptions`.
  - The audit fails with `media_node_on_other_topics` if the node hears anything other than the front camera
    image.
  - **A recording run's `trial.json` differs from a non-recording run's in exactly that key.**
  - The recorder starts before the collector's first audit and stops after the collector.

## Frames

These are the pinned scenario and URDF facts `docking_metric.py` uses. Each trial checks them against the
published TF.

- `/race_auv/stonefish/odometry` is link `Base`: the nose tip, rolled by π. That is URDF `nose_tip_link`.
  - `base_link` is 0.7 m aft; `cg_link` is `base_link` rolled by π.
  - `auv_dock_point` is (−0.305, 0, −0.13) from the nose.
- `/race_station/stonefish/odometry` is `race_station/base_link`, at world (4.0, 0, 3.95) and not rolled.
  `dock_point` is (−0.410, 0.325, −0.04) rpy (π, 0, 0) from it.
- Upstream's `ground_truth_docking` node treats the AUV `Base` as `base_link`, so its pose is nose-referenced.
  The recipe uses it only as a cross-check.

## Known source findings

- **Tag size.** The tag textures' black square is 0.8 of the texture, and Stonefish stretches the whole texture
  over the 0.15 m and 0.05 m faces. The black edges are therefore 0.12 m and 0.04 m. With the lab's configured
  0.15/0.05, the CPU detector reports ranges 1.25× too long. M1 reproduces the lab configuration. The
  `black_square_edge` variant corrects it for the CPU backend only; the cuAprilTags path is never built.
- **AprilTag binding.** `race_auv_camera_pkg` needs `apriltag.estimate_tag_pose()`, which is only in AprilRobotics
  commits after v3.4.5. With v3.4.5, every detection silently falls back to an identity pose, and the fused dock
  point collapses toward the camera. The image pins `dc6316d` and solves a known pose at build time.
- **Build-only changes.** The top-level `race_station` package is empty and is COLCON_IGNOREd in this build only.
  `race_auv_bringup` declares `rviz2`, which is installed at build time and never launched.
- **Base workspace.** The base image's `/opt/ros2_ws` underlay also contains the RISE `alpha_rise_*` packages.
  They are on the package path; nothing in this recipe names or launches them.

## Simulator seed

Stonefish 7d52673 seeds its sensor-noise generators from `std::random_device` when the library loads. Each run
gets a new seed, which is not printed and cannot be set. In the RACE vehicle that noise is the pressure sensor's
(2.0 Pa) and the DVL's (velocity 0.01 m/s, altitude 0.03 m); the IMU's noise is set to zero.

- **No run before `runtime/simulator_seed.py` recorded a seed.** That includes both arms of the M3 report: their
  `trial.json` holds `simulator_seed: {"value": null, "support": "not_exposed_by_pinned_runner"}`.
- **The runner now chooses a seed before every launch** (`run_trial.sh`): the mission's `seed` if it is an
  integer, else a draw. It writes `simulator-seed.json` and exports the seed as `STONEFISH_SEED`.
- **`simulator-patches/stonefish_seed_v1.patch`** seeds the generators from `STONEFISH_SEED` and prints each seed
  used. Its hashes and how to apply it are in `simulator-patches/STONEFISH_SEED_V1.md`.
- **The docking image is not rebuilt with the patch yet.** That needs a separate release. Until then its
  simulator ignores `STONEFISH_SEED`, prints nothing, and `trial.json` records `value: null` with `support:
  not_exposed_by_pinned_runner`, next to the seed that was exported.
- **A seed fixes the sequence of sensor-noise draws, not the trajectory.** The simulator, the EKF, the controller,
  the AprilTag fuser and the renders run asynchronously. Which draw serves which sample, and so the run itself,
  depends on timing. Two runs with one seed share their noise sequence; they are not the same run.

## Build

Production (linux/amd64; run from the repository root; the default `BASE_IMAGE` is the pinned RISE base digest):

```bash
docker build --platform linux/amd64 -f packages/simulation/race-auv-docking/Dockerfile -t race-auv-docking .
```

The arm64 development loop builds the RISE Dockerfile at its pinned commit for arm64 and passes that image:

```bash
docker build --platform linux/arm64 --build-arg BASE_IMAGE=<local rise base> \
  -f packages/simulation/race-auv-docking/Dockerfile -t race-auv-docking:dev .
```

## Controller variant (M3-C, piccard-experiments #88)

`recipe-keepxyint-v1.json` (`race-auv-docking-keepxyint/v1`, `controller_variant` `keep_xy_integral`) is a second
recipe for one Piccard change to the lab's controller.

**What it changes.** mvp_control's `MvpControlROS::f_amend_set_point` (6cfea2d) zeroes an axis's integral whenever its
set point changes. The variant keeps the integral of x and y across set-point changes. z, roll, pitch, yaw and the
rates are unchanged. Gains, missions, the planner and everything else are `race-auv-docking/v1`'s.

**What it is not.** It is a Piccard modification of the lab's controller behaviour, not the lab's controller. It runs
only as its own arm (`context.arm` `planner_keepxyint` / `control_keepxyint`), and its results say nothing about the
lab's controller. The claim boundary of `race_m3_metrics` says the same for any trial whose `controller_variant` is not
`upstream`.

**Why.** On the M3-0 v1.3 run, the planner's final-stage re-targets zeroed the x/y integrals every 2.2 s (median), and
the vehicle stopped 4.7 cm short on P alone.

**The build.** `Dockerfile.controller-variant` builds a second image FROM the recipe image of the same commit, by
digest; the recipe `Dockerfile` is untouched:

```bash
docker build --platform linux/amd64 -f packages/simulation/race-auv-docking/Dockerfile.controller-variant \
  --build-arg BASE_IMAGE=<the recipe image of this commit, by digest> \
  --build-arg PICCARD_CONTROLLER_VARIANT=keep_xy_integral -t <repository>:recipe-<sha>-keepxyint .
```

The build runs these steps, in order:
1. checks the patch file's sha256 (`controller-variants/controller_variant.py`);
2. checks that `src/mvp_control/mvp_control_ros.cpp` is byte-identical to the pinned commit's;
3. applies `controller-variants/keep_xy_integral.patch`: one hunk, adding DOF_X and DOF_Y to the zeroing condition's
   exceptions;
4. checks the patched file's sha256;
5. rebuilds only mvp_control with the base's colcon flags;
6. writes `/opt/piccard-race-auv/controller-variant.json`.

The logic lives in `libmvp_control.so`, which the rebuild replaces. The node binary is unchanged. To check the patch
against the pinned source by hand (CI has no network), download the codeload archive of 6cfea2d and run
`python3 controller-variants/controller_variant.py apply-check keep_xy_integral <archive>`. It checks the archive's
sha256, then the pristine file, the patch and the patched file.

**No trace in the recipe image.** `Dockerfile.dockerignore` keeps `controller-variants/`, the variant Dockerfile and
the variant recipe out of the recipe image. So the recipe image has no variant record, and its trials say
`controller_variant` `upstream`.

**What a trial records.** The collector copies the record into `trial.json`:
- `controller_variant`;
- `source_pins.mvp_control_patch`: the patch and its sha256, and the pristine and patched file hashes;
- `unreadable` if the record is malformed.

**Recipe consistency.** `controller_variant.py recipe keep_xy_integral recipe-v1.json recipe-keepxyint-v1.json`
regenerates the variant recipe. `test_race_runtime.py` holds the committed file to `recipe-v1.json` with only the
declared differences:
- `recipe_id`;
- `controller_variant`;
- `controller_patch`;
- `images`;
- `notes.controller_variant`.

**The campaign builder.** `tools/campaigns/build_phase_r_campaign.py --controller-variant keep_xy_integral` emits the
same M3 jobs for the variant recipe, with these differences:
- the variant recipe id;
- `context.arm` suffixed `_keepxyint`;
- `context.controller_variant`;
- a `kxi` token in the trial ids.

**Licence.** The source lock records mvp_control as GPL-3.0. The variant image holds a modified GPL program and stays
internal to Piccard. If it or the patch ever leaves Piccard, the GPL's terms apply (the source and the changes). This
repository carries only the patch, with its context lines, and not upstream's source.

## Run locally (development)

```bash
docker run --rm -v "$PWD/input:/input:ro" -v "$PWD/output:/output" race-auv-docking:dev \
  --output /output --horizon-seconds 240 --wall-timeout-seconds 420 --ready-timeout-seconds 110 \
  --expected-gains-json /input/gains.json --context-json /input/context.json \
  --mission-profile-json /input/mission.json
```

## Run natively

`runtime/run_native_trial.sh` runs one trial from a request file to a scored result on a colcon workspace. The
workspace is built from `piccard-inc/race_auv` `piccard/docking-recipe` and its dependency manifest
(`piccard_docking_recipe/dependencies/`):

```bash
packages/simulation/race-auv-docking/runtime/run_native_trial.sh --workspace ~/race_ws \
  --request packages/simulation/race-auv-docking/examples/m3-planner-request.json --output ~/trials/m3-1
```

The planner arm's request is `examples/m3-planner-request.json`, the control arm's
`examples/m3-contact-request.json`.

1. **The request is split** into its gains, mission and context (`native_request.py split`). Numbers are written
   as the platform's submit path writes them: a float with an integral value as an integer. The collector and
   the planner package then hash the planner parameters alike.
2. **`prepare_candidate.py` installs and verifies** on the workspace. On a branch-built workspace it checks the
   committed docking variants and writes only gains that differ from them.
3. **`run_trial.sh` runs the lifecycle** with `PICCARD_NATIVE_WORKSPACE` set, so it uses the sourced workspace
   instead of the image's `/opt` paths.
   - The launch is `launch/docking_sim.launch.py` with the docking variant. The station bringup and the
     ground-truth pose node run with `/tf` and `/tf_static` remapped to `/piccard/ground_truth/*`, so the AprilTag
     fuser stays the only parent of `race_station/dock_point`.
   - A planner mission runs the planner from the `race_auv_docking_planner` package (`PICCARD_PLANNER_PARAMS`).
     Its parameter file comes from the mission: the planner parameters, and the fallback pose as
     `initial_setpoint`.
   - A pose mission (the control arm) runs no planner; the collector flies the poses.
4. **`tools/analysis/race_m3_metrics.py`** writes the scored result to `<output>/scored/`, scored as the report's
   trials were:
   - at clearance, at the mission's approach clearance (a pose mission: 0.02 m) within 0.01 m
     (`native_request.py scoring`);
   - docked by the request context's `protocol_version`, which for the M3 examples is v1.5, the first full hold.

The runner starts its own Xvfb display unless `PICCARD_EXTERNAL_DISPLAY=1`. ROS 2 Jazzy is sourced from
`ROS_SETUP` (default `/opt/ros/jazzy/setup.bash`). The exit status is `run_trial.sh`'s. The simulator seed is
chosen and recorded as in the image (see "Simulator seed"). A native Stonefish built with `stonefish_seed_v1`
uses the seed and prints it; an unpatched one ignores it.

## Tests

`runtime/test_race_runtime.py`, `runtime/test_planner.py`, `runtime/test_tag_pivot.py` and `test_conformance.py`
run offline in CI and need no ROS. `test_planner.py` covers:
- the planner mission;
- the planner's goals against the M2 pose derivation, stage settling, the set-point steps, stale and frozen
  estimates, the heading's median and hold, the re-anchored dock point, the walk in every stage, the waiting ticks,
  and completion;
- a replay of the station estimate on the M3-0 staging trace (`fixtures/m3-0-stage1-replay.jsonl`: the last 60 s
  at 3 m and the first 180 s at 1.5 m, against ground truth);
- a replay of the M3-0 rerun's handover (`fixtures/m3-0-rerun-handover-replay.jsonl`: 35 s of the dive and 5 s
  after, at 7.45 m with tag 146 alone). v1.0 reproduces the recorded first set point, 11° off; v1.1 takes the
  window's median, 2.45° off, and walks;
- a replay of the v1.1 run's final stage (`fixtures/m3-0-v1.1-final-replay.jsonl`: the last 32 s of the 0.3 m
  stage and the final stage's 101.6 s up to the stale onset), measuring where the command puts the AUV dock point
  against the true station dock point across the approach:
  - v1.1 reproduces the recorded final set point, and the EKF drift takes it to 13.9 cm;
  - v1.2 without the re-hold follows the drift: mean 1.3 cm, max 2.7 cm;
  - v1.2 re-holds the heading from 211 samples, 0.22° from truth (1.88° before): mean 0.49 cm, max 1.44 cm. The
    true heading gives mean 0.53 cm. v1.3 gives the same horizontal result;
- a replay of the v1.3 run's final stage (`fixtures/m3-0-v1.3-final-replay.jsonl`: the last 32 s of the 0.3 m
  stage and the final stage's 120 s), on the recorded EKF pose. The vehicle never came within 4.7 cm, so v1.5's
  along axis stays in its approach:
  - it changes once, at 60 s (v1.4 also changed it at 120 s). The lateral axis changes 21 times, as without the
    gate;
  - the cost of the approach's gate on this run: the EKF frame drifted about 7 cm along in the stage's first 40 s.
    Before the first along change, the along command sat +1.9 to +7.8 cm off the true dock point; after it, −3.3
    to +4.1 cm;
  - without the gate it follows every tick, 0.43 cm mean. The lateral command is 0.49 cm mean either way;
- a replay of the v1.4 run's final stage, upstream mvp_control (`fixtures/m3-0-v1.4-final-replay.jsonl`: the last
  32 s of the 0.3 m stage and the final stage's 300 s, the fuser's latest sample at each tick of the 5 Hz grid),
  under v1.5 and open loop on the recorded vehicle:
  - the first arrival comes 54.2 s into the stage, as the v1.4 read-out's scan found, with the vehicle 0.8 cm short
    in truth. There is no along change before it;
  - one new approach, at 176.4 s, when the recorded vehicle drifts back 2.8 cm short, then a second arrival;
  - in hold, the along command stays 0.55 cm mean and 2.05 cm max off the true dock point. v1.4's recorded command
    was 1.94 cm mean and 7.10 cm max;
  - the lateral axis follows as in v1.4: 67 changes (66 recorded), 0.50 / 1.78 cm mean / max;
- the final stage's axes:
  - a lateral-only drift never touches y, but for the arrival re-issue;
  - 4 mm on each axis re-targets neither;
  - the approach re-targets only a stale command (at 60 s and 120 s of 1 mm/s drift), and 1.5 cm of drift never;
  - arrival re-issues y once, at the goal, and a re-issue on the held value is nudged;
  - hold follows the goal;
  - a new approach comes from past the goal after `final_reapproach_s`, across stale ticks, and coming back inside
    restarts its clock;
  - the collector records each arrival and re-approach once, with its arrival time;
  - `mission.py` rejects an arrival at or past the along deadband;
- a replay of the v1.2 run's stage 0 (`fixtures/m3-0-v1.2-stage0-replay.jsonl`: from 30 s before the handover to
  stage 1's start, one fused sample per 0.8 s, with the EKF pose and the controller's set point). The vehicle is
  placed at the planner's own command plus the recorded tracking error.
  - On this model the v1.2 planner steps z 13 times, within 0.7 s of 13 of the run's 14 z steps, and advances at
    mission time 1728.6 (recorded 1729.35).
  - v1.3 steps nothing after the handover through the recorded 1689 s. The recorded tracking error is v1.2's own
    integral-reset transients and settles only in the stage's last seconds.
  - Run on for 60 s with the vehicle on its command (converged), v1.3 refines once after `settle_s` and advances
    after another. That is no z step at the 2 cm clearance, and exactly one with the target level;
- the clearance: every stage's vertical target is exactly `approach_clearance_m` above the level one. The v1.3
  refinement: a sub-millimetre goal move flagged by the controller's tail, and a 0.62° heading move flagged by the
  yaw swing, step nothing; a 1.5 cm depth tail keeps the arrival gate closed;
- its import boundary;
- the `planner_inputs` audit.

The other two cover:

- candidate install and verify on a synthetic workspace;
- the mission validator;
- the docking metric on exact geometry and on a recorded telemetry excerpt (`fixtures/`);
- the consumption audit;
- the alpha_rise grep;
- the launch package set;
- the sanitization;
- schema/runtime agreement;
- Dockerfile hashes against the source lock.
