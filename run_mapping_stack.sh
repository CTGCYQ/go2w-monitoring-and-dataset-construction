#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VLA_ROOT="${GO2W_VLA_ROOT:-/home/dell/go2w_vla}"
# This hardware-specific profile must have a completed manual direction validation.
python3 - "$ROOT/display_alignment.json" <<'PY'
import json,sys
c=json.load(open(sys.argv[1]))
assert c.get('validated') is True and c.get('imu_acceleration_convention')=='go2_firmware_gravity_vector_negated_to_specific_force', 'Install the validated profile for this robot first.'
PY
if ! tmux has-session -t imuadapter 2>/dev/null; then
  tmux new-session -d -s imuadapter "bash '$ROOT/run_imu_adapter.sh' >> '$ROOT/imu_adapter.log' 2>&1"
fi
# Existing estimator must be stopped explicitly before switching its parameters.
PARAMS_FILE="$ROOT/config/point_lio_go2w_specific_force.yaml" bash "$VLA_ROOT/deploy/point_lio/start_point_lio.sh"
if tmux has-session -t mapworker 2>/dev/null; then
  tmux respawn-pane -k -t mapworker "bash '$ROOT/run_map_worker.sh' >> '$ROOT/map_worker.log' 2>&1"
else
  tmux new-session -d -s mapworker "bash '$ROOT/run_map_worker.sh' >> '$ROOT/map_worker.log' 2>&1"
fi
