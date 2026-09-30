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
| `recipe-v1.json` | Recipe descriptor. Covers the fixed source, fixed execution, image digests, overlay sources and notes: tag-size bias evidence, build dependencies, the controller frame and per-trial cost. |
| `request-v1.schema.json` | The executor claim request: `gains`, `mission`, `horizon_s`, `wall_timeout_s`, `context`. |
| `conformance-v1.json`, `test_conformance.py` | Accept/reject cases that the schema and the runtime validators must both agree on. |
| `examples/m1-smoke-request.json` | The M1 smoke request. |
| `campaigns/phase-r-a-20260928/` | The M2 Phase R-A pose missions and their derivation (#96). The jobs files come from `tools/campaigns/build_phase_r_campaign.py`; the metrics from `tools/analysis/race_m2_metrics.py`. |
| `campaigns/tank-floor-v1.json` | The tank's interior floor and the AUV's footprint, from the pinned world_of_stonefish (`tools/campaigns/extract_tank_floor.py`). The builder checks every pose for 0.3 m of floor clearance with it (#102). |
| `runtime/run_campaign_trial.sh` | Container entrypoint: install → verify → `run_trial.sh`. |
| `runtime/prepare_candidate.py` | Validates and installs the gains, helm state and tag-size variant. Verifies them before launch. |
| `runtime/mission.py` | `piccard.race-auv.pose-mission/v1` validator. |
| `runtime/launch/docking_sim.launch.py` | The trial's launch file. It mirrors upstream `bringup_simulation.launch.py` with no rviz, joystick or C2, and adds the station and the ground-truth node. |
| `runtime/scenario/race_auv_docking_trial.scn` | The upstream `race_auv_test.scn`, included unchanged, plus AUV↔station and AUV↔tank contact monitors. |
| `runtime/collect_trial.py` | Readiness, consumption audit, gain readback, `direct_control`, the pose loop, telemetry, `trial.json` and `docking.json`. |
| `runtime/docking_metric.py` | Pure-Python dock-point geometry, frame validation and the `docking.json` summary. |
| `runtime/check_image.py` | Build-time check against the real pinned files. |

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

`mission` follows `piccard.race-auv.pose-mission/v1`:

- `frame_id` is `world_ned` and `child_frame_id` is `cg_link`.
- `poses` holds 1–20 entries of `{label, x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad, dwell_s}`. Labels are unique.
- `seed` is `null`.
- `apriltag_tag_size` is `lab_configured` (the default) or `black_square_edge`.

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

1. **Install and verify** (`run_campaign_trial.sh`).
   - Writes `control_modes.flight` and a new `control_modes.docking` into both copies of `config_sim.yaml`.
   - Adds the `direct_control` state (control mode `docking`, transitions to and from `start` and `kill`) to
     `helm_sim.yaml`, and `bhv_direct_control` (`helm/DirectControl`, priority 1) to `bhv_params_sim.yaml`.
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
   - **Audit:** records every node's subscriptions and the `/tf` publishers and edges. The trial fails if:
     - the helm or controller consume odometry other than `/race_auv/odometry/filtered`;
     - a vehicle node subscribes to ground truth;
     - ground-truth TF reaches the vehicle tree (`ground_truth_tf_on_vehicle_tree`): a ground-truth publisher on
       `/tf`, a station edge other than the fuser's dock point, or a frame with two parents;
     - a `/tf` frame is neither a vehicle/station frame nor the fuser's per-observation tag frame
       `race_auv/base_link → apriltag_<family>_<id>`, which is accepted while `/apriltag_fuser` publishes `/tf`
       (`unknown_tf_frame_on_vehicle_tree`);
     - any `alpha_rise` node runs.

     The audit runs before the mission and again at the end. A problem at the end fails an otherwise completed
     trial. `/stonefish_simulator` always holds a `/tf` broadcaster, but it only sends
     `world_ned → <robot>/base_link` when a scenario enables it; that edge would fail the audit.
   - **Gain readback:** reads both cached modes and the controller parameters and checks that
     `odometry_source` is the EKF odometry.
   - **Start:** publishes the first setpoint, enables the controller, changes the helm to `direct_control`,
     waits for control mode `docking`, then reads back the active docking gains.
   - **Pose loop:** each pose is re-published at 1 Hz and must be echoed by the controller within 10 s.
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
  - the consumption audits before the mission and at the end;
  - `nonfinite_values`: where non-finite numbers occurred, as the first 50 rows of each class plus full counts.
    `expected` holds the stack's by-design infinities (`HelmState.max_duration = inf` in helm `change_state`
    responses); `unexpected` holds everything else, so a nonzero `unexpected.rows` is worth reading.
    `invalid_numeric_events` still counts rows of both classes;
  - observations: `direct_control` reached, ground-truth samples, tags detected, fused dock-point TF and pose
    samples, contact events, frame validation.
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

## Run locally (development)

```bash
docker run --rm -v "$PWD/input:/input:ro" -v "$PWD/output:/output" race-auv-docking:dev \
  --output /output --horizon-seconds 240 --wall-timeout-seconds 420 --ready-timeout-seconds 110 \
  --expected-gains-json /input/gains.json --context-json /input/context.json \
  --mission-profile-json /input/mission.json
```

## Tests

`runtime/test_race_runtime.py` and `test_conformance.py` run offline in CI and need no ROS. They cover:

- candidate install and verify on a synthetic workspace;
- the mission validator;
- the docking metric on exact geometry and on a recorded telemetry excerpt (`fixtures/`);
- the consumption audit;
- the alpha_rise grep;
- the launch package set;
- the sanitization;
- schema/runtime agreement;
- Dockerfile hashes against the source lock.
