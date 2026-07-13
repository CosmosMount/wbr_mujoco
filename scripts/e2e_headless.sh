#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_INSTALL="${MUJOCO_SIM_INSTALL_DIR:-$ROOT/../mujoco_sim_core/install}"
ECAL_ROOT="${ECAL_ROOT:-$ROOT/../mujoco_interface/third_party/ecal/usr}"
ECAL_CONFIG="${ECAL_CONFIG:-$ROOT/../mujoco_interface/third_party/ecal/etc/ecal}"
TICKS="${TICKS:-100}"
INSTANCE="wbr_e2e_${BASHPID}_$(date +%s%N)"

export PATH="$SIM_INSTALL/mujoco_sim_server/bin:$PATH"
# mujoco-sim-server embeds RUNPATH for MuJoCo, protobuf, and eCAL; keep this as a fallback.
export LD_LIBRARY_PATH="$SIM_INSTALL/mujoco_sim_core/lib:$ECAL_ROOT/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export ECAL_DATA="${ECAL_DATA:-$ECAL_CONFIG}"

if [[ ! -x "$SIM_INSTALL/mujoco_sim_server/bin/mujoco-sim-server" ]]; then
  echo "missing mujoco-sim-server at $SIM_INSTALL/mujoco_sim_server/bin" >&2
  exit 1
fi
if [[ ! -x "$ROOT/build/ctrl" ]]; then
  echo "missing $ROOT/build/ctrl; build wbr_mujoco first" >&2
  exit 1
fi

TMP_SERVER="$(mktemp)"
TMP_CTRL="$(mktemp)"
SERVER_LOG="$(mktemp)"
CTRL_LOG="$(mktemp)"
SERVER_PID=""
cleanup() {
  if [[ -n "$SERVER_PID" ]]; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
  rm -f "$TMP_SERVER" "$TMP_CTRL" "$SERVER_LOG" "$CTRL_LOG"
}
trap cleanup EXIT

cat >"$TMP_SERVER" <<EOF
server_name: $INSTANCE
model_path: $ROOT/mjcf/scene.xml
initial_keyframe: home
max_ticks: 0
tick_timeout_ms: 5000
activation_timeout_ms: 5000
allow_timed_out_adapter_replacement: true
status_period_ms: 100
headless: true
EOF

# Derive a short ctrl config from the e2e template with unique server name.
python3 - <<PY
import yaml
from pathlib import Path
cfg = yaml.safe_load(Path("$ROOT/config/robots/wbr_e2e.yaml").read_text())
cfg.setdefault("adapter", {})
cfg["adapter"]["server_name"] = "$INSTANCE"
cfg["adapter"]["controller_id"] = "wbr_${BASHPID}"
cfg["adapter"]["max_ticks"] = int("$TICKS")
Path("$TMP_CTRL").write_text(yaml.safe_dump(cfg))
PY

mujoco-sim-server --config "$TMP_SERVER" >"$SERVER_LOG" 2>&1 &
SERVER_PID=$!
sleep 0.2

set +e
"$ROOT/build/ctrl" -c "$TMP_CTRL" --server-name "$INSTANCE" >"$CTRL_LOG" 2>&1
CTRL_RC=$?
set -e

if [[ "$CTRL_RC" -ne 0 ]]; then
  echo "ctrl failed with $CTRL_RC" >&2
  sed -n '1,200p' "$CTRL_LOG" >&2 || true
  sed -n '1,200p' "$SERVER_LOG" >&2 || true
  exit 1
fi

if ! grep -Eq "ctrl summary ticks=${TICKS}" "$CTRL_LOG"; then
  echo "ctrl summary missing expected tick count ${TICKS}" >&2
  sed -n '1,200p' "$CTRL_LOG" >&2 || true
  sed -n '1,200p' "$SERVER_LOG" >&2 || true
  exit 1
fi

for event in server_starting model_loaded server_started controller_registered \
             controller_activated controller_unregistered; do
  if ! grep -Fq "event=${event}" "$SERVER_LOG"; then
    echo "server log missing event=${event}" >&2
    sed -n '1,240p' "$SERVER_LOG" >&2 || true
    exit 1
  fi
done

echo "e2e_headless ok ticks=${TICKS}"
cat "$CTRL_LOG"
