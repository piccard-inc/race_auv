#!/usr/bin/env bash
# One race-auv-docking/v1 trial without the image: on a colcon workspace built from piccard-inc/race_auv
# piccard/docking-recipe and its dependency manifest (piccard_docking_recipe/dependencies), from a request file to a
# scored result.
#
#   run_native_trial.sh --workspace WS --request REQUEST.json --output DIR [--ready-timeout-seconds N] [--video]
#
#   1. native_request.py split: the request's gains, mission and context, numbers written as the platform writes
#      them; the horizon and wall timeout.
#   2. prepare_candidate.py install, then verify, on WS (its src/ and install/): on a branch-built workspace this
#      checks the committed docking variants and writes only gains that differ from them.
#   3. A planner mission: native_request.py planner-params writes the race_auv_docking_planner parameter file from
#      the mission (its planner parameters, and its fallback pose as the initial set point).
#   4. run_trial.sh: the trial's lifecycle as in the image, on WS (PICCARD_NATIVE_WORKSPACE). The launch is
#      launch/docking_sim.launch.py with the docking variant; the station bringup and the ground-truth pose node run
#      with /tf and /tf_static remapped to /piccard/ground_truth/*. The planner comes from race_auv_docking_planner.
#      The collector flies the dive or the poses, starts the planner, and writes trial.json and docking.json. The
#      simulator seed is chosen and recorded as described in simulator_seed.py.
#   5. race_m3_metrics.py: DIR/scored/<DIR name>.m3.json, the scored result.
#
# The runner starts its own Xvfb display unless PICCARD_EXTERNAL_DISPLAY=1 (then DISPLAY or PICCARD_DISPLAY is used).
# ROS 2 Jazzy is sourced from ROS_SETUP (default /opt/ros/jazzy/setup.bash). The exit status is run_trial.sh's.
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
metrics="$(cd -- "$script_dir/../../../.." && pwd)/tools/analysis/race_m3_metrics.py"
workspace=""
request=""
output=""
ready_timeout=110
video=()
while (($#)); do
  case "$1" in
    --workspace) workspace="$2"; shift 2 ;;
    --request) request="$2"; shift 2 ;;
    --output) output="$2"; shift 2 ;;
    --ready-timeout-seconds) ready_timeout="$2"; shift 2 ;;
    --video) video=(--video); shift ;;
    *) echo "Unknown argument: $1" >&2; exit 64 ;;
  esac
done
[[ -n "$workspace" && -f "$workspace/install/setup.bash" ]] || { echo 'A built workspace (--workspace) is required' >&2; exit 64; }
[[ -n "$request" && -f "$request" ]] || { echo 'A request file (--request) is required' >&2; exit 64; }
[[ -n "$output" && ! -e "$output" ]] || { echo 'A new output directory (--output) is required' >&2; exit 64; }
[[ -f "$metrics" ]] || { echo "race_m3_metrics.py not found at $metrics" >&2; exit 64; }
workspace="$(cd -- "$workspace" && pwd)"
mkdir -p "$output"
output="$(cd -- "$output" && pwd)"

source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
source "$workspace/install/setup.bash"
export PICCARD_NATIVE_WORKSPACE="$workspace"

parts="$output/request"
python3 "$script_dir/native_request.py" split --request "$request" --output "$parts"
source "$parts/limits.env"
python3 "$script_dir/prepare_candidate.py" install --gains "$parts/expected-gains.json" \
  --mission "$parts/mission.json" --workspace "$workspace" --output "$output"
python3 "$script_dir/prepare_candidate.py" verify --gains "$parts/expected-gains.json" \
  --mission "$parts/mission.json" --workspace "$workspace" --output "$output"
if python3 -c 'import json,sys; sys.exit(json.load(open(sys.argv[1])).get("schema") != sys.argv[2])' \
    "$parts/mission.json" piccard.race-auv.planner-mission/v1; then
  python3 "$script_dir/native_request.py" planner-params --mission "$parts/mission.json" \
    --output "$parts/planner-params.yaml"
  export PICCARD_PLANNER_PARAMS="$parts/planner-params.yaml"
fi

set +e
"$script_dir/run_trial.sh" --output "$output" --horizon-seconds "$HORIZON_SECONDS" \
  --wall-timeout-seconds "$WALL_TIMEOUT_SECONDS" --ready-timeout-seconds "$ready_timeout" \
  --expected-gains-json "$parts/expected-gains.json" --mission-profile-json "$parts/mission.json" \
  --context-json "$parts/context.json" "${video[@]}"
status=$?
set -e
if [[ -f "$output/trial.json" ]]; then
  python3 "$metrics" "$output" --output-dir "$output/scored" || echo 'race_m3_metrics failed; trial.json is kept' >&2
fi
exit "$status"
