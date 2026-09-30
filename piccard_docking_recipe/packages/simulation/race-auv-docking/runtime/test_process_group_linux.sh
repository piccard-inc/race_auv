#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
setsid python3 "$script_dir/process_group_supervisor.py" python3 -c '
import os
import signal
import time
signal.signal(signal.SIGINT, lambda *_: os._exit(0))
os.fork()
time.sleep(30)
os._exit(0)
' &
pid=$!
trap 'kill -KILL -- "-$pid" 2>/dev/null || true' EXIT
sleep .3
kill -INT -- "-$pid"
wait "$pid"
python3 -c '
import importlib.util
import sys
spec = importlib.util.spec_from_file_location("finalizer", sys.argv[2])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
members = module.process_group_members(int(sys.argv[1]))
print({"pgid": int(sys.argv[1]), "members": members})
assert members == []
' "$pid" "$script_dir/finalize_runner.py"
if kill -0 -- "-$pid" 2>/dev/null; then
  echo "process group remains signalable" >&2
  exit 75
fi
trap - EXIT
echo PROCESS_GROUP_REAP_TEST_PASS
