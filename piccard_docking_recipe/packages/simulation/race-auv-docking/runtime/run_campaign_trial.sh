#!/usr/bin/env bash
# Complete campaign path: validate/install/verify the candidate in this disposable container, then run natively.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
output=""
expected=""
mission=""
args=("$@")
while (($#)); do
  case "$1" in
    --output) output="$2"; shift 2 ;;
    --expected-gains-json) expected="$2"; shift 2 ;;
    --mission-profile-json) mission="$2"; shift 2 ;;
    *) shift ;;
  esac
done
[[ -n "$output" && -n "$expected" && -f "$expected" && -n "$mission" && -f "$mission" ]] || {
  echo "Campaign output, expected gains and mission are required" >&2
  exit 64
}
mkdir -p "$output"
# #122: the wrapper's own stdout and stderr also land in the output directory (the host keeps container.log)
exec > >(tee -a "$output/wrapper.log") 2>&1
python3 "$script_dir/prepare_candidate.py" install \
  --gains "$expected" --mission "$mission" --workspace /opt/race_ws --output "$output"
python3 "$script_dir/prepare_candidate.py" verify \
  --gains "$expected" --mission "$mission" --workspace /opt/race_ws --output "$output"
exec "$script_dir/run_trial.sh" "${args[@]}"
