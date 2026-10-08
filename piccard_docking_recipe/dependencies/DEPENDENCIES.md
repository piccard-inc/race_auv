# Dependencies of the RACE docking simulation

`workspace.repos` and `libraries.repos` pin every source the Piccard docking image builds. This file records what is
done to them, so the build does not need that image. Each value below is copied from the image's build files and
their source locks, which pin each archive by SHA-256 as well as by commit.

## Target

- Ubuntu 24.04 (noble), ROS 2 Jazzy. The image starts from `ros:jazzy-ros-base-noble@sha256:c3706ef0a0aa45413c07803cf433602f543b22e45b4855f6fca955c2d8ecc4e8`.
- Apt packages: build-essential, ca-certificates, cmake, curl, git, ninja-build, pkg-config,
  python3-colcon-common-extensions, python3-rosdep, python3-venv, python3-vcstool, python3-yaml, libegl1-mesa-dev,
  libfreetype6-dev, libgl1-mesa-dev, libgl1-mesa-dri, ffmpeg, libglew-dev, libglm-dev, libsdl2-dev, libssl-dev,
  mesa-utils, x11-utils, xauth, xvfb, nlohmann-json3-dev, ros-jazzy-vision-msgs, python3-scipy, xdotool.

## Libraries (`libraries.repos`), installed to /usr/local first

**Stonefish `7d52673791834caa743907dde83a1949134ef2f0`** (GSO-soslab/stonefish; the same commit as
patrykcieslak/stonefish):

    cmake -S stonefish -B stonefish-build -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_TESTS=OFF -DEMBED_RESOURCES=OFF
    cmake --build stonefish-build --parallel 4 && sudo cmake --install stonefish-build && sudo ldconfig

The docking study applied no patch; to record the simulator seed, apply `stonefish_seed_v1` first (see "Patches").
Lineage of the unpatched build: a fresh build on linux/arm64 reproduced the docking image's `libStonefish.so`
byte for byte: SHA-256 `7d43894dd173b8d57eef0ef1ff12e085766594df10e01063ab078a30781408b5`, ELF build-id
`c403cb3e755e54657e01148d4fdd27ac83ea778a`. It used gcc/g++ 13.3.0-6ubuntu2~24.04.1, cmake 3.28.3 and ninja
1.11.1 on the base above. The record is Piccard physical-ai
`backends/stonefish/race-openloop/evidence/stonefish-build-59dc69a`. No amd64 reproduction is recorded.

**apriltag `dc6316dc37520e56819d64741b54a41b50c92866`** (AprilRobotics/apriltag, master after v3.4.5). It is the
first commit whose Python binding has `estimate_tag_pose`, which `race_auv_camera_pkg`'s CPU detector calls. With
v3.4.5, or Ubuntu's python3-apriltag, every detection falls back to an identity pose.

    cmake -S apriltag -B apriltag-build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF \
      -DPython3_EXECUTABLE=/usr/bin/python3
    sudo cmake --build apriltag-build --parallel 4 --target install && sudo ldconfig

The install puts the Python module in `/usr/local/lib/python3.12/site-packages`, which Ubuntu's `python3` does not
search (it reads `dist-packages`). The docking image's Dockerfile therefore makes the module importable in two
steps, then checks it:
1. it links `libapriltag.so.3` next to the module;
2. it names the module's directory in a `.pth` file in `dist-packages`.

On a host, the same commands with `sudo`:

    module_dir="$(dirname "$(find /usr/local/lib -name 'apriltag*.so' -path '*python3*' | head -1)")"
    sudo ln -sf /usr/local/lib/libapriltag.so.3 "$module_dir/libapriltag.so.3"
    sudo mkdir -p /usr/local/lib/python3.12/dist-packages
    test "$module_dir" = /usr/local/lib/python3.12/dist-packages || \
      echo "$module_dir" | sudo tee /usr/local/lib/python3.12/dist-packages/piccard-apriltag.pth
    python3 -c 'from apriltag import apriltag; assert callable(apriltag("tag36h11").estimate_tag_pose)'

Without these steps the import fails with "cannot import name 'apriltag' from 'apriltag'". The verification trial
on a fresh Ubuntu 24.04 host hit exactly that.

## Workspace (`workspace.repos`)

After `vcs import`, the image makes these changes before building:

- `touch race_auv/race_auv_perception/race_auv_apriltag_cuda/COLCON_IGNORE`, and remove
  `race_auv/race_auv_perception/third_party` (isaac_ros_nitros) and `race_auv/race_auv_perception/scripts`.
  The cuAprilTags path is never built; detection is the CPU `python` backend.
- `touch race_station/race_station/COLCON_IGNORE` (an empty top-level package).
- Delete the real-vehicle configurations `race_auv/race_auv_config/mvp_control_config/config.yaml` and
  `race_auv/race_auv_config/mvp_mission_config/helm.yaml`. Simulation uses the `*_sim.yaml` files and their docking
  variants, and the trial installer refuses a workspace that installs either real-vehicle file.

Then build:

    rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y \
      --skip-keys "dwe_camera_driver race_auv_apriltag_cuda"
    CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --merge-install --executor sequential \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

The image builds this as two workspaces. The base holds stonefish_ros2, the four mvp packages and acomms_msgs, with
a world_of_stonefish at `55139d46` that the overlay shadows. The overlay holds race_auv, race_auv_sim,
race_auv_perception, race_station and world_of_stonefish `d51d59e7`, the copy the simulation resolves. One workspace
from `workspace.repos` builds the same packages at the same commits.

## Patches

**The docking study applies none.** Its recipe runs `controller_variant: upstream`, the lab's mvp_control at
`6cfea2d` unmodified, and no other source is patched. Piccard's controller variants exist, but none was used in the
study. They are listed by SHA-256 of the patch file in Piccard physical-ai, and none is applied by these manifests:

| Variant | Patch SHA-256 | What it changes | Used for |
|---|---|---|---|
| `actuator_fix_v1` | `5ec7b835ddfd3006d70a46b183c43a3c6364dc2a765dcb9f15270577075aa26a` | mvp_control: bounded polynomial roots, pinned six-channel limits (heave_bow and sway_stern ±10 N, heave_stern ±13 N, surge ±20 N; 1 N per allocation), a stop and latch on any fault | Piccard's connected-loop (D1/D2) work, not the docking study |
| `keep_xy_integral` | `de08372ab728281be57ecf209e57c84cb9c6abc33bafab38e2e95119eb7877e7` | mvp_control `f_amend_set_point` keeps the x and y integrals across set-point changes | the M3-C arm, retired after protocol v1.4 |
| `keep_z_integral` | `d83927e041271af96b8de4c6d1ba05a1a193087ab7815c3b329a8e1863ee8aaf` | the same for the z integral | the ALPHA RISE depth campaign only; never in a RACE build |

A variant is built by checking the pristine file's hash, `patch -p1` in mvp_control, checking the patched file's
hash, then `colcon build --packages-select mvp_control --cmake-args -DCMAKE_BUILD_TYPE=Release`.

**Simulator patch: `stonefish_seed_v1`**, optional, SHA-256
`6068dbfbad5067aee90ccd6abc4e9875ff79d2866e76b0f2996bc8bd6595f16d`.

- **What it does.** Stonefish seeds its sensor-noise generators from `std::random_device` when it loads, so a run's
  seed can be neither set nor recorded. The patch changes only those two lines (`Library/src/sensors/Sensor.cpp`,
  `Library/src/comms/USBL.cpp`): the seed comes from `STONEFISH_SEED` when it is set, and each seed used is
  printed.
- **Where it is.** `piccard_docking_recipe/packages/simulation/race-auv-docking/simulator-patches/`, with
  `STONEFISH_SEED_V1.md` giving the pristine and patched file hashes. Apply it in the Stonefish checkout before
  the build: `patch -p1 < stonefish_seed_v1.patch`.
- **Effect on the build.** A patched build does not match the lineage hash above.
- **Effect in RACE.** The seed reaches only the pressure and DVL noise; the scene sets the IMU's noise to zero.
  The report's trials ran unpatched and recorded no seed. `piccard_docking_recipe/RUN.md` says what a seed does
  and does not fix.

## Licences

| Source | Licence as recorded |
|---|---|
| stonefish, stonefish_ros2, mvp_msgs, mvp_control, mvp_mission, mvp_utilities | **GPL-3.0** |
| acomms_msgs | package manifest states GPLv3; no recognized root licence |
| race_auv | one package manifest states GPLv3; three say "TODO: License declaration" |
| race_station | one package manifest states GPLv3; two say "TODO: License declaration" |
| race_auv_perception | package manifests state Apache-2.0 and MIT |
| apriltag | BSD-2-Clause |
| world_of_stonefish `d51d59e` | package manifest says "TODO: License declaration" |
| race_auv_sim | not reviewed |

The three variant patches and `stonefish_seed_v1` modify GPL-3.0 code and carry its terms. **Nothing from these sources, builds or patches
enters Mariana**, Piccard's physics engine. It talks to Piccard's bridge over a socket and contains none of these
files; its repository was checked by path on 2026-10-07. Piccard's records mark every source "review required before
redistribution or commercial packaging".
