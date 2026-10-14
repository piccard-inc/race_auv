# race_auv_docking_planner

The RACE docking planner, protocol v1.5 (piccard-experiments #88), as a ROS 2 package. It is the planner of the M3 report
(https://piccard.science/experiments/race-auv-docking-planner-2026-10). It brings the AUV's dock point onto the station
dock point that the AprilTag fuser estimates. It does this in stages, by commanding pose set points through the helm's
`direct_control` state, which the docking variant of the bringup adds. `race_auv_docking_planner/planner.py` documents
every decision.

## What ran, and what this is

- **Code.** `planner.py` holds the constants, the rigid transforms and `StagePlanner`. These are copied verbatim from
  Piccard's `runtime/planner_fused_dock.py` at commit `468d65f6` (piccard-physical-ai), the protocol v1.5 commit.
  `test/test_planner_source.py` pins the copied text by sha256.
- **Node.** `node.py` is that file's ROS node, with one difference: it reads ROS parameters where the original read the
  trial's mission file.
- **Parameters.** `config/docking_planner_v1_5.yaml` holds the v1.5 parameters. Every scored planner trial at the
  0.1 m/s speed cap used them. M3-B's two speed-cap trials changed only `speed_cap_mps`, to 0.05 and 0.2.
- **The parameters' hash.** The trial records state it as `planner.parameters_sha256`:
  `12c05989447431826f1e082169eb572512e0b654301b564138f0c18d5ba5cc4d`.
  - The hash is taken over JSON with sorted keys and no spaces.
  - The records write whole numbers as integers (`3`, not `3.0`). ROS holds every value here as a double, so
    `parameters.parameters_sha256` writes whole-number doubles as integers. The YAML then reproduces the trials' hash,
    and the node reports it in its start response and in every state record.
  - Piccard's example request, `examples/m3-planner-request.json`, writes some of these values with a decimal point.
    It holds the same values, but hashed as written it gives `95c3d8794d30161035f343815e6b52f0415b682b66f5753446195db282096fa7`.

## Interfaces

These are the inputs the trials audited (the collector's `planner_inputs` check), and nothing else:

| Kind | Name | Use |
|---|---|---|
| subscription | `/race_auv/odometry/filtered` (nav_msgs/Odometry) | the EKF odometry; its age gates every tick |
| TF (`/tf`, `/tf_static`) | `race_auv/base_link` → `race_station/dock_point` | the AprilTag fuser's station dock point |
| TF | `race_auv/world_ned` → `race_auv/base_link` | the vehicle in the controller's world, through the EKF's odom |
| TF (static) | `race_auv/base_link` → `race_auv/auv_dock_point`, → `race_auv/cg_link` | vehicle geometry |
| publication | `/race_auv/mvp_helm/bhv_direct_control/desired_setpoints` (mvp_msgs/ControlProcess) | the set points: cg_link in `race_auv/world_ned`, control mode `docking` |
| publication | `/piccard/planner/state` (std_msgs/String) | one JSON state record per tick |
| service | `/piccard_planner/start` (std_srvs/Trigger) | starts the approach; answers with the parameters' hash |

**Held set point.** The helm's `direct_control` set points are the planner's output, not an input. When the planner
starts, the controller already holds the fallback pose (M1's dive), which the trial publishes on the same topic before
the planner starts.
- The planner takes that pose as configuration: `initial_setpoint` in the YAML, equal to the trials' fallback pose. It
  starts from that pose, so the handover changes no axis that need not change.
- Reading the topic instead would add an input that the trials did not have.

**Ground truth.** Ground truth is not an input, an import or a dependency of this package.
`test/test_ground_truth_boundary.py` asserts:
- every import, file by file;
- the single subscription, the single TF listener and the single lookup, and the four edges;
- that no code or configuration names a ground-truth topic or frame;
- that the launch file remaps nothing;
- the exact `package.xml` dependencies.

**TF requirement.** The TF tree must carry the station dock point only from the fuser. The station's own
`robot_state_publisher` would otherwise give `race_station/dock_point` a second parent, `race_station/base_link`,
placed from the simulator. In the trials, the station bringup and the ground-truth pose node ran with `/tf` and
`/tf_static` remapped to `/piccard/ground_truth/tf` and `/piccard/ground_truth/tf_static`. The trial's TF audit checked
that only the fuser published into the station's frames. The documented trial path (to follow, #185 deliverable 4)
launches it that way.

**Clock.** The planner's clock is the host's monotonic clock. The station estimate's age is checked against the node's
clock, which is system time; the trials did not set `use_sim_time`.

## Build, run, test

In a colcon workspace with the pinned sources (`piccard_docking_recipe/dependencies/`), it needs `mvp_msgs` and the
standard ROS 2 Jazzy packages listed in `package.xml`:

    colcon build --packages-select race_auv_docking_planner
    ros2 launch race_auv_docking_planner docking_planner.launch.py   # params_file:=<file> for other parameters
    # once the controller holds the fallback pose (the trials dwelt 40 s there):
    ros2 service call /piccard_planner/start std_srvs/srv/Trigger

The tests need no ROS, only Python 3 and PyYAML. Run them with `python3 -m pytest race_auv_docking_planner/test` from
the repository root, or with `colcon test`.
- `test_stage_planner.py` is Piccard's `runtime/test_planner.py` unit tests of the planner's decisions, with this
  package's parameters.
- The replays of recorded trials are not here: their fixtures carry ground truth, which belongs with the scoring and
  analysis (#185 deliverable 5), not in the vehicle's package.

## Licence

Copyright Piccard Inc.; licence pending founder decision. The node imports `mvp_msgs` (GPL-3.0) at run time.
