# race-auv-docking/v1: the pinned RISE base (Stonefish 7d52673, stonefish_ros2 a7c9be9 and the MVP stack,
# built in /opt/ros2_ws) plus an /opt/race_ws overlay with the RACE vehicle, docking station, docking world
# and CPU AprilTag perception. world_of_stonefish d51d59e in the overlay shadows the base's 55139d4 copy;
# every trial records the resolved share path and scenario hash. Production builds use the default
# BASE_IMAGE; a local dev build may pass an equivalent image built from the same RISE Dockerfile.
ARG BASE_IMAGE=piccard/alpha-rise-gains-only-staging@sha256:f11a47bbe889d10338652488192977edc771112f648421e293bb991cf99aae2c
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# System Python only: nothing from the base image's RISE virtualenv is on the RACE path.
ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
# xdotool points the recorded Stonefish window at the AUV (runtime/follow_view.sh, --video only)
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-vision-msgs python3-scipy xdotool && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /opt/source
# AprilRobotics apriltag at dc6316d ("add python wrap for estimate_tag_pose", 2026-01-08; the first commit whose
# Python binding has the pose solve race_auv_camera_pkg calls; v3.4.5 and Ubuntu's python3-apriltag lack it and the
# detector then publishes identity poses). CPU backend only: cuAprilTags / isaac_ros_nitros are never fetched.
RUN curl --fail --location --max-time 300 \
      https://codeload.github.com/AprilRobotics/apriltag/tar.gz/dc6316dc37520e56819d64741b54a41b50c92866 \
      -o apriltag.tar.gz && \
    echo '34ac49fb753e85bd611c1cf9852622e8b4577c6ebdc7c167efe1bc353fe13b8b  apriltag.tar.gz' | sha256sum --check && \
    mkdir apriltag && tar -xzf apriltag.tar.gz -C apriltag --strip-components=1 && rm apriltag.tar.gz && \
    cmake -S apriltag -B apriltag-build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF \
      -DPython3_EXECUTABLE=/usr/bin/python3 && \
    cmake --build apriltag-build --parallel 4 --target install && ldconfig && \
    module_dir="$(dirname "$(find /usr/local/lib -name 'apriltag*.so' -path '*python3*' | head -1)")" && \
    ln -sf /usr/local/lib/libapriltag.so.3 "$module_dir/libapriltag.so.3" && \
    ( test "$module_dir" = /usr/local/lib/python3.12/dist-packages || \
      echo "$module_dir" > /usr/local/lib/python3.12/dist-packages/piccard-apriltag.pth ) && \
    /usr/bin/python3 -c 'from apriltag import apriltag; assert callable(apriltag("tag36h11").estimate_tag_pose)'
COPY packages/simulation/race-auv-docking/runtime/sanitize_seed.py /opt/piccard-race-auv/runtime/sanitize_seed.py
WORKDIR /opt/race_ws/src
RUN set -eux; \
    fetch() { \
      target="$1"; url="$2"; digest="$3"; \
      curl --fail --location --max-time 300 "$url" -o /tmp/source.tar.gz; \
      echo "${digest}  /tmp/source.tar.gz" | sha256sum --check; \
      mkdir -p "$target"; tar -xzf /tmp/source.tar.gz -C "$target" --strip-components=1; rm /tmp/source.tar.gz; \
    }; \
    fetch race_auv \
      https://codeload.github.com/piccard-inc/race_auv/tar.gz/da58963c048228856dfdd5917da21a14edfa1985 \
      59e80dfd1238314ec31e0b9979b6209883ed0b6590e3c5f541aba31cc31316fe; \
    fetch race_auv/race_auv_sim \
      https://codeload.github.com/GSO-soslab/race_auv_sim/tar.gz/3601a30f49c7b8ddac2845c41c99af8af92c0e65 \
      7d8091d3776198ef0d5b5b07cb0345e4713ee6950adbb37a8724bc7f21eaa177; \
    fetch race_auv/race_auv_perception \
      https://codeload.github.com/GSO-soslab/race_auv_perception/tar.gz/5944d5a6ebd44601bcc5579e91b82d5d1b3735cb \
      cfb07acd3ff3c288c46f15362993e6955d1f86e11cc0f26896371f91e701cbd4; \
    fetch race_station \
      https://codeload.github.com/GSO-soslab/race_station/tar.gz/100bc5a7ac253172b527c036d637da5482eaf0f3 \
      a29539a491725de026a70ba85e4c0aca0fe22c03938f62d6039c98bbfbe00ee2; \
    fetch world_of_stonefish \
      https://codeload.github.com/GSO-soslab/world_of_stonefish/tar.gz/d51d59e77211a436a0c3617c30a80c36337cdbbc \
      d7c10d04a39ab7bcc1aa6a58d2a6e59a27319dfe9957f589271f9e9cdbf055d2; \
    touch race_station/race_station/COLCON_IGNORE race_auv/race_auv_perception/race_auv_apriltag_cuda/COLCON_IGNORE; \
    rm -rf race_auv/race_auv_perception/third_party race_auv/race_auv_perception/scripts; \
    /usr/bin/python3 /opt/piccard-race-auv/runtime/sanitize_seed.py /opt/race_ws/src
RUN source /opt/ros/jazzy/setup.bash && source /opt/ros2_ws/install/setup.bash && rosdep update && \
    apt-get update && rosdep install --from-paths /opt/race_ws/src --ignore-src --rosdistro jazzy -r -y \
      --skip-keys "dwe_camera_driver race_auv_apriltag_cuda" && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/race_ws
RUN source /opt/ros/jazzy/setup.bash && source /opt/ros2_ws/install/setup.bash && \
    CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --merge-install --executor sequential \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /opt/race_ws/install/setup.bash && \
    test "$(ros2 pkg prefix world_of_stonefish)" = /opt/race_ws/install && \
    test -f "$(ros2 pkg prefix world_of_stonefish)/share/world_of_stonefish/world/race_auv_test.scn" && \
    test ! -e /opt/race_ws/install/share/race_auv_config/mvp_control_config/config.yaml && \
    test ! -e /opt/race_ws/install/share/race_auv_config/mvp_mission_config/helm.yaml && \
    test ! -e /opt/race_ws/install/share/race_auv_apriltag_cuda && \
    mkdir -p /output
COPY packages/simulation/race-auv-docking /opt/piccard-race-auv
RUN chmod 0555 /opt/piccard-race-auv/runtime/run_trial.sh /opt/piccard-race-auv/runtime/run_campaign_trial.sh \
      /opt/piccard-race-auv/runtime/follow_view.sh \
      /opt/piccard-race-auv/runtime/process_group_supervisor.py /opt/piccard-race-auv/runtime/test_process_group_linux.sh && \
    /usr/bin/python3 -m py_compile /opt/piccard-race-auv/runtime/*.py /opt/piccard-race-auv/runtime/launch/*.py && \
    /usr/bin/python3 /opt/piccard-race-auv/runtime/check_image.py /opt/race_ws && \
    command -v xdotool ffmpeg ffprobe && \
    bash /opt/piccard-race-auv/runtime/test_process_group_linux.sh
WORKDIR /opt/piccard-race-auv
ENTRYPOINT ["/opt/piccard-race-auv/runtime/run_campaign_trial.sh"]
