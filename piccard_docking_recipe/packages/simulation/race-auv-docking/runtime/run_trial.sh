#!/usr/bin/env bash
# Fresh RACE docking simulator/stack lifecycle; requires the candidate already installed and verified.
set -eo pipefail
if [[ -z "${PICCARD_NATIVE_WORKSPACE:-}" ]]; then  # the image's workspaces; run_native_trial.sh has sourced its own
  source /opt/ros/jazzy/setup.bash
  source /opt/ros2_ws/install/setup.bash
  source /opt/race_ws/install/setup.bash
fi
set -u

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
scenario="$script_dir/scenario/race_auv_docking_trial.scn"
output=/output
horizon=240
wall_timeout=360
ready_timeout=110
expected=""
context=""
mission=""
video=0
onboard=0
while (($#)); do
  case "$1" in
    --output) output="$2"; shift 2 ;;
    --horizon-seconds) horizon="$2"; shift 2 ;;
    --wall-timeout-seconds) wall_timeout="$2"; shift 2 ;;
    --ready-timeout-seconds) ready_timeout="$2"; shift 2 ;;
    --expected-gains-json) expected="$2"; shift 2 ;;
    --context-json) context="$2"; shift 2 ;;
    --mission-profile-json) mission="$2"; shift 2 ;;
    --video) video=1; shift ;;
    --onboard-camera-video) onboard=1; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 64 ;;
  esac
done
[[ -n "$expected" && -f "$expected" ]] || { echo 'Expected gain JSON is required' >&2; exit 64; }
[[ -n "$mission" && -f "$mission" ]] || { echo 'Mission JSON is required' >&2; exit 64; }
for value in "$horizon" "$wall_timeout" "$ready_timeout"; do
  [[ "$value" =~ ^[0-9]+$ ]] && ((value > 0 && value <= 3600)) || { echo 'Time bounds must be integers 1..3600' >&2; exit 64; }
done
mkdir -p "$output"
[[ ! -e "$output/launch.log" && ! -e "$output/telemetry.jsonl" && ! -e "$output/trial.json" ]] || { echo 'Output contains an existing trial; refusing overwrite' >&2; exit 64; }
[[ -f "$output/candidate-install-verification.json" ]] || { echo 'Candidate is not installed and verified' >&2; exit 64; }
apriltag_config="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["apriltag_config"] or "")' "$output/candidate.json")"
# A workspace built from piccard/docking-recipe runs the committed docking variants (prepare_candidate records it).
launch_variant="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("launch_variant") or "")' "$output/candidate.json")"
# M3 (#109): a planner mission runs the planner as its own node; the collector starts it after the dive. In the image
# that is planner_fused_dock.py; a native run (run_native_trial.sh, PICCARD_PLANNER_PARAMS) runs the same planner
# from the race_auv_docking_planner package with a parameter file made from the mission.
planner_mission="$(python3 -c 'import json,sys; print(int(json.load(open(sys.argv[1])).get("schema") == sys.argv[2]))' \
  "$mission" piccard.race-auv.planner-mission/v1)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((40 + RANDOM % 150))}"
export ROS_HOME="$output/ros-home"
export ROS_LOG_DIR="$output/ros-logs"
export DISPLAY="${PICCARD_DISPLAY:-${DISPLAY:-:99}}"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

launch_pid=""
xvfb_pid=""
watch_pid=""
video_pid=""
view_pid=""
onboard_pid=""
planner_pid=""
collector_pid=""
runner_exit=1
stop_group() {
  local target="$1"
  [[ -n "$target" ]] || return 0
  kill -INT -- "-$target" 2>/dev/null || true
  for _ in {1..20}; do kill -0 -- "-$target" 2>/dev/null || break; sleep .25; done
  kill -TERM -- "-$target" 2>/dev/null || true
  for _ in {1..20}; do kill -0 -- "-$target" 2>/dev/null || break; sleep .25; done
  kill -KILL -- "-$target" 2>/dev/null || true
  wait "$target" 2>/dev/null || true
}
cleanup() {
  trap - EXIT INT TERM
  set +e
  stop_group "$watch_pid"  # first: the teardown's own display stop is not a display death
  stop_group "$collector_pid"
  stop_group "$planner_pid"
  stop_group "$onboard_pid"
  stop_group "$view_pid"
  stop_group "$video_pid"  # while the simulator window is still drawn: no black tail in the clip
  stop_group "$launch_pid"
  stop_group "$xvfb_pid"
  if ((video || onboard)); then  # after the recorders have finalized their files; never changes the exit code
    timeout 600 python3 "$script_dir/media.py" --output "$output" --display-video "$video" \
      --onboard-video "$onboard" >"$output/media.log" 2>&1
  fi
  python3 "$script_dir/finalize_runner.py" --output "$output" --exit-code "$runner_exit" \
    --launch-pid "${launch_pid:-0}" --collector-pid "${collector_pid:-0}" \
    --video-pid "${video_pid:-0}" --xvfb-pid "${xvfb_pid:-0}" --view-pid "${view_pid:-0}" \
    --watch-pid "${watch_pid:-0}" \
    --onboard-pid "${onboard_pid:-0}" --planner-pid "${planner_pid:-0}" \
    --video-requested "$video" --onboard-requested "$onboard"
  finalizer_exit=$?
  if ((finalizer_exit != 0)); then exit "$finalizer_exit"; fi
}
trap cleanup EXIT
trap 'runner_exit=130; exit 130' INT
trap 'runner_exit=143; exit 143' TERM

if [[ "${PICCARD_EXTERNAL_DISPLAY:-0}" != 1 ]]; then
  setsid Xvfb "$DISPLAY" -screen 0 1200x800x24 +extension GLX >"$output/xvfb.log" 2>&1 &
  xvfb_pid=$!
  for _ in {1..40}; do
    xdpyinfo >/dev/null 2>&1 && break
    kill -0 "$xvfb_pid"
    sleep .25
  done
fi
xdpyinfo >/dev/null
glxinfo -B >"$output/glxinfo.txt" 2>&1
record_limit=$((wall_timeout + 15))
# #122: the display observed at 1 Hz (host Xorg :99 in production, Xvfb in development); observation only
setsid python3 "$script_dir/display_watch.py" --output "$output/display-watch.jsonl" --display "$DISPLAY" \
  --max-seconds "$record_limit" >"$output/display-watch.log" 2>&1 &
watch_pid=$!
if ((video)); then
  # info level keeps the x11grab stream start (wall clock of the first frame) that media.py uses for sync
  maxrate=$(python3 "$script_dir/media.py" --display-maxrate-kbps "$record_limit")
  setsid ffmpeg -nostdin -hide_banner -nostats -loglevel info -n -video_size 1200x800 \
    -framerate 10 -f x11grab -draw_mouse 0 -i "$DISPLAY" -t "$record_limit" \
    -c:v libx264 -preset ultrafast -crf 25 -maxrate "${maxrate}k" -bufsize "$((2 * maxrate))k" \
    -pix_fmt yuv420p "$output/display.mp4" >"$output/ffmpeg.log" 2>&1 &
  video_pid=$!
fi
# The simulator seed (simulator_seed.py): the mission's, else one drawn now, written to simulator-seed.json. A
# simulator built with simulator-patches/stonefish_seed_v1.patch seeds its sensor noise from it and prints it; the
# pinned simulator ignores it. The collector records which (trial.json simulator_seed).
STONEFISH_SEED="$(python3 "$script_dir/simulator_seed.py" --mission-profile-json "$mission" --output "$output")"
export STONEFISH_SEED
launch_args=("scenario:=$scenario")
[[ -z "$apriltag_config" ]] || launch_args+=("apriltag_config:=$apriltag_config")
[[ -z "$launch_variant" ]] || launch_args+=("variant:=$launch_variant")
setsid python3 "$script_dir/process_group_supervisor.py" \
  ros2 launch "$script_dir/launch/docking_sim.launch.py" "${launch_args[@]}" >"$output/launch.log" 2>&1 &
launch_pid=$!
if ((video)); then  # point the recorded window at the AUV; GUI view only, no effect on the simulation
  setsid bash "$script_dir/follow_view.sh" "$output/follow-view.json" 2 "$ready_timeout" >"$output/follow-view.log" 2>&1 &
  view_pid=$!
fi
if ((onboard)); then  # front camera clip: one ROS node hearing only the camera image, listed in the audit's media_nodes
  setsid python3 "$script_dir/onboard_recorder.py" --output "$output" --limit-seconds "$record_limit" \
    >"$output/onboard-recorder.log" 2>&1 &
  onboard_pid=$!
fi
if ((planner_mission)); then  # hears only the TF tree and the EKF odometry; the audit checks it (planner_inputs)
  if [[ -n "${PICCARD_PLANNER_PARAMS:-}" ]]; then
    setsid ros2 launch race_auv_docking_planner docking_planner.launch.py params_file:="$PICCARD_PLANNER_PARAMS" \
      >"$output/planner.log" 2>&1 &
  else
    setsid python3 "$script_dir/planner_fused_dock.py" --mission-profile-json "$mission" >"$output/planner.log" 2>&1 &
  fi
  planner_pid=$!
fi
extra=()
[[ -z "$context" ]] || extra+=(--context-json "$context")
setsid timeout --signal=INT --kill-after=10 "$((wall_timeout + 10))" \
  python3 "$script_dir/collect_trial.py" --output "$output" --launch-pid "$launch_pid" \
  --horizon-seconds "$horizon" --wall-timeout-seconds "$wall_timeout" \
  --ready-timeout-seconds "$ready_timeout" --expected-gains-json "$expected" \
  --mission-profile-json "$mission" --scenario "$scenario" \
  "${extra[@]}" >"$output/collector.log" 2>&1 &
collector_pid=$!
set +e
wait "$collector_pid"
runner_exit=$?
set -e
exit "$runner_exit"
