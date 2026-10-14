# Tag-146 out-and-back, pipeline-check run (2026-10-14)

One Stonefish run of the mission in `request.json` (SHA-256 `a28e7f59e1e353fb7f7095a76171c37cec73d94a8eaf576526534991bab7dbeb`), job `4ef8719e-4588-4402-87d5-4e8aea878ad8`, seed 910739171, mission completed in 640 s, rendered on an NVIDIA L40S.

- `bev.png`: one bird's-eye panel, EKF odometry (solid) against ground truth (dashed), the mission waypoints named, the true tag and the tag reprojected from the EKF pose at every sighting; below it the EKF error over time with the interval in which the tag was not detected. Errors are horizontal ATE RMSE and max against ground truth, aligned once at the first sample. Frame: Stonefish world, x north, y east, z down. `bev-v1-superseded.png` is the first version (two panels, sighting dots, medians), kept because the thread discussed it.
- `figure-numbers.json`: the numbers printed on the figure.
- `make_figure.py`: the script that drew it from the run's `telemetry.jsonl` and `docking.json` (the `alignment` rows pair the controller's EKF-derived cg pose with ground truth at 2 Hz; detections are in the front camera's optical frame).

The recorded run (telemetry, trial, docking and launch records, 54 files) is held in Piccard's evidence store and is available on request.
