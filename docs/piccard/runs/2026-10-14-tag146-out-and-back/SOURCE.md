# What ran, as source

The run used the public upstream `race_auv` at `da58963` plus Piccard's recipe delta: 117 files, manifest SHA-256 `ed1221b5d251af9287711daa9b3a619fc65d315e34fc60c174f60516a076a9d6` (`coordinator-source.json` in the run's evidence). That delta is now committed on this branch, verbatim, as the files under:

- `piccard_docking_recipe/` (the runtime: `run_trial.sh`, `launch/docking_sim.launch.py`, the collector, the scenario wrapper, the mission and request validators, tools and tests; `RUN.md` is the fresh-checkout procedure);
- `race_auv_bringup/launch/bringup_docking_simulation.launch.py`, `race_auv_bringup/launch/include/simulation/mvp_{control,mission}_docking_sim.launch.py`, `race_auv_bringup/config/simulation/apriltag_black_square_edge.yaml` and `sim_params.yaml`;
- `race_auv_config/mvp_control_config/config_sim_docking.yaml`, `race_auv_config/mvp_mission_config/{helm,bhv_params}_sim_docking.yaml`;
- `race_auv_docking_planner/` (not used by this run; the pose mission has no planner).

The one file of the delta not committed here is its `readme.md`, which would have replaced the upstream README; the recipe's own README is `piccard_docking_recipe/README.md`.

Nothing is modified at run time beyond what the request selects: the controller gains in `config_sim_docking.yaml` are the ones in `request.json` (this run: the lab's upstream gains, unchanged), and `apriltag_tag_size: black_square_edge` selects `apriltag_black_square_edge.yaml` (tag sizes × 0.8, the black square's edge). `RUN.md` names the `piccard/docking-recipe` branch for its `vcs` files; this branch carries the same files, so `BRANCH` may point here instead.
