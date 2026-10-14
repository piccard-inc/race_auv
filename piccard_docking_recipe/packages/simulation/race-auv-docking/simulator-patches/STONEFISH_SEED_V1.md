# stonefish_seed_v1: a recorded seed for Stonefish's sensor noise

Stonefish `7d52673791834caa743907dde83a1949134ef2f0` seeds its two noise generators when the library loads. In
`Library/src/sensors/Sensor.cpp` and `Library/src/comms/USBL.cpp` the line is
`std::mt19937 X::randomGenerator(randomDevice());`, with `randomDevice` a `std::random_device`. Every
`ScalarSensor` channel with a non-zero noise level draws from the `Sensor` generator; INS and GPS draw from it
too. The RACE vehicle (world_of_stonefish `d51d59e` `vehicles/race_auv.scn`, SHA-256 `1c5fa672…`, the file the
trials loaded) sets zero IMU noise. So in RACE the seed reaches two sensors: the pressure sensor (2.0 Pa) and the
DVL (velocity 0.01 m/s, altitude 0.03 m). Each simulator start gets a new seed, which is not printed and cannot
be set. No RACE trial before this patch recorded one: neither arm of the M3 report did, nor any earlier run. Their
`trial.json` holds `simulator_seed: {"value": null, "support": "not_exposed_by_pinned_runner"}`.

`stonefish_seed_v1.patch` changes only those two lines and adds a file-local function beside each.

- **Seed set.** When `STONEFISH_SEED` holds a decimal integer in [0, 4294967295], `Sensor` is seeded with it and
  `USBL` with it + 1 (mod 2^32). Any other non-empty value prints the reason and aborts the simulator.
- **Seed unset or empty.** Each generator draws from `std::random_device`, as upstream does.
- **Either way.** Each generator prints one line to stderr at load, for example
  `stonefish_seed_v1: Sensor noise seed 42 from STONEFISH_SEED`.

Under `ros2 launch` that line lands in `launch.log` with the `[stonefish_simulator-N]` prefix.
`runtime/simulator_seed.py` reads it back for `trial.json`.

| File | Pristine SHA-256 (7d52673) | Patched SHA-256 |
|---|---|---|
| `Library/src/sensors/Sensor.cpp` | `74ea906e4319061b0e50d5fe1e67ba4e6aecc700adc907f37a08c0906a5d6a22` | `b71b3f00ef4bfaf8bf9127a67f7e7f800a9a85dae6bbf9370d4ea02e2e837440` |
| `Library/src/comms/USBL.cpp` | `278a4d32cfef391fd74f7aac6b9705f5af407f00b37feb1fed78216bc343e60a` | `84394e87dd3c202bb55e96a2d35686576ffff29fa761df10164c4252088fe532` |

Patch SHA-256: `6068dbfbad5067aee90ccd6abc4e9875ff79d2866e76b0f2996bc8bd6595f16d`.

## Applying it

In the Stonefish checkout, before the build in Piccard's lineage record (and in the lab branch's
`piccard_docking_recipe/dependencies/DEPENDENCIES.md`):

    sha256sum Library/src/sensors/Sensor.cpp Library/src/comms/USBL.cpp   # the pristine hashes above
    patch -p1 < stonefish_seed_v1.patch
    sha256sum Library/src/sensors/Sensor.cpp Library/src/comms/USBL.cpp   # the patched hashes above

A patched build is no longer the recorded lineage. The unpatched `libStonefish.so` SHA-256 `7d43894d…`
(`backends/stonefish/race-openloop/evidence/stonefish-build-59dc69a`) does not apply to it.

**Docking image.** The image is not rebuilt by this change: the image recipe is unchanged. The image's Stonefish is
unpatched until it is rebuilt with this patch under a separate release. Until then, every trial records
`{"value": null, "support": "not_exposed_by_pinned_runner"}`, together with the seed the runner exported and that
the simulator ignored.

## What a seed does and does not fix

A seed fixes the sequence of sensor-noise draws. It does not fix the trajectory. The simulator steps, the EKF, the
controller, the AprilTag fuser and the renders run in separate processes and threads. Which draw serves which sensor
sample, and how late each message is, depend on timing. Two runs with the same seed therefore share a noise sequence,
but they are not the same run.

## Tests

- **In CI** (`runtime/test_simulator_seed.py`):
  - this file's hashes against the patch;
  - that the patch touches only those two files;
  - the runner exports the seed before the launch;
  - the record in each case: patched and applied, unpatched, a printed seed that differs, no seed file.
- **Locally, outside CI** (a clone of Stonefish at 7d52673; not repeatable in CI, which has no Stonefish checkout):
  - `git apply --check` and `patch -p1 --dry-run` on pristine 7d52673;
  - a clang++ build of the added function, run with `STONEFISH_SEED` set to 0, 42, 4294967295 (USBL wraps to 0),
    unset and empty, which seed as described;
  - `4294967296`, `-1`, `12x`, `" 7"` and `+5`, which abort;
  - the same seed twice, which gave the same first draws.
- **Not done:** a full Stonefish build.
