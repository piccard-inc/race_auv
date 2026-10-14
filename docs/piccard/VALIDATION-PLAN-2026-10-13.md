Plan for the SOS Lab. Cut on 2026-10-14 to the one run Tony and Yuewei asked for in
[discussion #12](https://github.com/piccard-inc/race_auv/discussions/12). Nothing in it has been run.

# RACE tag-aided localization: pipeline check (one run)

**Purpose.** Check that the pipeline is correct before anything is evaluated: one simulated run in which the forward
camera sees tag 146, loses it, and finds it again; the EKF odometry and the tag detections recorded with ground truth;
a factor graph with the EKF odometry as BetweenFactors and the tag as a landmark; one bird's-eye figure. No metrics
and no pass criteria.

**Where it runs.** The RACE Stonefish simulation of the tank as it is: interior 6.1 m × 9.1 m, depth 0–4.3 m, the
station with tags 146, 541 and 558 at one end. Forward camera only. Tag 146 only; the forward camera detects it from
0.5 m to 7.85 m. The lab's controller gains, unchanged.

**The run.** All of it along the tank's long axis at 3.78 m depth. The stand-off is 2.3 m from the tags, facing them.

| Step | Pose | Hold | Tag 146 |
|---|---|---|---|
| 1 | Dive at the far end of the tank, facing the station | 40 s | in view, 7.1 m |
| 2 | Drive forward 4.8 m to the stand-off | 3 min | in view, 2.3 m |
| 3 | Turn 180° in place | 30 s | out of view |
| 4 | Drive back 4.0 m toward the far end, station behind the vehicle | 2 min | out of view |
| 5 | Turn 180° in place | 30 s | back in view, 6.3 m |
| 6 | Drive forward to the stand-off | 4 min | in view, closing 6.3 m → 2.3 m |

About 11 min; the tag out of view for roughly 2.5 of them. The vehicle moves between poses on the position
controller and settles at each pose before the next; on transits of this length it runs at 0.1–0.3 m/s in surge.
Visibility per pose was checked with the tank's tag-visibility tool.

**Recorded.** Ground-truth vehicle pose (`/race_auv/stonefish/odometry`); the EKF odometry
(`/race_auv/odometry/filtered`) at its native rate; tag 146's position from the forward camera
(`/cam_front/apriltag_detection/detections3d`, XYZ) at 7 Hz; the simulator seed.

**The graph.** EKF odometry between keyframes as BetweenFactors. Tag 146 as a landmark, measured as the detector's
XYZ translation only (a single tag's orientation from one frame can flip); its position known in the station frame.
A GTSAM projection factor on the tag corners is for later, after the pipeline check.

**Figure.** The lab's bird's-eye format: the vehicle's track from the EKF and from the GTSAM solution, the landmark,
and, since this is a simulation, the ground-truth track and the true tag position.

**What the lab decides.** Any change to the run before it is made; who writes the graph.
