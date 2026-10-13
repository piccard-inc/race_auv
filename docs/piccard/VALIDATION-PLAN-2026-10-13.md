Draft for the SOS Lab, 2026-10-13. Comments and push-back welcome, on this file or by reply. Nothing in it has been run.

# RACE tag-aided localization: validation plan (draft)

**Purpose.** Test, in simulation against ground truth, whether a tag-aided estimator corrects the vehicle's EKF drift
when the station tags go out of view and come back. The same suite scores our estimator and the lab's own
implementation on identical recordings.

**Where it runs.** The RACE Stonefish simulation of the tank: interior 6.1 m × 9.1 m, depth 0–4.3 m, tags 146, 541
and 558 on the station at one end, found reliably by the forward camera within 2.75–3 m. A 20 m-per-side box does not
fit, so the box keeps a 20 m path as a 4 m × 6 m rectangle. The vehicle moves at 0.06 m/s median, 0.11 m/s p90.

| | Scenario | Path | Tag in view | Duration | Tests |
|---|---|---|---|---|---|
| S1 | Box | 4 m × 6 m at 3.8 m depth, starting and ending at the stand-off facing the tags | 1/5 | 8 min | drift over one lap, one re-sighting |
| S2 | Out-and-back | 6 m down the tank, 180° turn, 6 m back | 1/3 | 6 min | one long loss and return |
| S3 | Turn away, tag out 1/3 | hold facing the tags, turn away, leg, return; twice | 2/3 | 15 min | re-sighting after short losses |
| S4 | Turn away, tag out 1/2 | S3 with longer legs | 1/2 | 15 min | re-sighting after long losses |
| S5 | Second depth | S1 at 3.0 m | 1/5 | 8 min | viewing geometry and drift at another depth |
| S6 | Long loop | S1 four times without stopping | 1/10 | 30 min | accumulated drift (EKF 1.4 cm/min median in earlier trials) |

Three seeded trials each. The visibility fractions are design targets, checked with the tank's tag-visibility tool
before registration.

**Metrics,** for each estimator and the EKF alike:
- ATE, RMSE and p90, against the simulator's vehicle pose (`/race_auv/stonefish/odometry`), aligned once at the first
  keyframe rather than by least squares, so drift stays visible.
- RPE: translation error over 10 m path segments, as a percentage.
- Tag reprojection error over time: each detection projected through the estimated pose, against the tag estimate and
  the true tag position.
- Reprojection spread: every accepted sighting's tag position reprojected from the pose at that moment, its distance
  to the median reprojected position per tag (σ and maximum), from the EKF and from the estimate.
- Acceptance rate per minute, and the final position error at the return to start.

The EKF baseline is the vehicle's `/race_auv/odometry/filtered`, recorded at its native rate beside the cameras'
`detections3d` topics.

**Pass criteria,** fixed before any run, per scenario, in all three trials:
1. ATE p90 not above the EKF's: tags must never make the estimate worse (none of 17 earlier simulated trials was).
2. Where the tag is out of view a third or more (S2, S3, S4, S6), ATE RMSE at most half the EKF's: the earlier revisit
   trials reached 6.1 cm against 11.5 cm and 5.0 cm against 14.1 cm medians.
3. Final error at the return to start at most 5 cm: twice the 2.6 cm median the estimator held with tags in view. The
   lab may set this number from the docking capture tolerance instead.
4. No minute after a re-sighting with 80 % or more of detections rejected: that is the lockout seen on the lab's
   tank recordings.
5. Median reprojection error on accepted detections at most 5 cm: earlier re-sightings gave 1.3–2.1 cm.
6. RPE over 10 m not above the EKF's: the graph must not add local jumps.
7. Reprojection spread σ smaller from the estimate than from the EKF, in every scenario: the tag must reproject to
   one place once the trajectory is corrected.

**Figures.** Per scenario, the lab's own bird's-eye format: two panels on the same East and North axes, the EKF on
the left and the estimate on the right; the path dotted with keyframe points; at every sighting the tag reprojected
from that pose, coloured per tag with a dashed line from the pose; per-run and global median markers; the true tag
position; σ and maximum of the reprojection spread per tag; a dashed box around the reprojected cluster. Beneath it,
reprojection error over time with each sighting's gate decision. One table gives the seven criteria per scenario.

**What the lab decides.** Scenarios to drop or add, and the thresholds. Whether a 20 m-per-side box in a new
open-water world is wanted instead. Whether the lab's estimator runs on the same recorded trials, scored on identical
data.

**Cost and time.** The simulator renders its cameras on a GPU host at 1.02× wall time; estimators and metrics run on
CPU in minutes. 18 trials take 4.1 h of simulated time, plus each trial's start-up.

**Sources.** J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers, "A benchmark for the evaluation of RGB-D SLAM
systems," IROS 2012, pp. 573–580, doi:10.1109/IROS.2012.6385773 (ATE and RPE). A. Geiger, P. Lenz, R. Urtasun, "Are we
ready for autonomous driving? The KITTI vision benchmark suite," CVPR 2012, pp. 3354–3361,
doi:10.1109/CVPR.2012.6248074 (segment-length odometry error). Z. Zhang, D. Scaramuzza, "A tutorial on quantitative
trajectory evaluation for visual(-inertial) odometry," IROS 2018, pp. 7244–7251, doi:10.1109/IROS.2018.8593941
(alignment choices).
