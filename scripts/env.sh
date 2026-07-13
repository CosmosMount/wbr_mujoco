#!/usr/bin/env bash
# Source before running mujoco-sim-server or ctrl manually:
#   source scripts/env.sh

_WBR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_SIM_INSTALL="${MUJOCO_SIM_INSTALL_DIR:-$_WBR_ROOT/../mujoco_sim_core/install}"
_ECAL_ROOT="${ECAL_ROOT:-$_WBR_ROOT/../mujoco_interface/third_party/ecal/usr}"

export PATH="$_SIM_INSTALL/mujoco_sim_server/bin:$PATH"
export LD_LIBRARY_PATH="$_SIM_INSTALL/mujoco_sim_core/lib:$_ECAL_ROOT/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export ECAL_DATA="${ECAL_DATA:-$_WBR_ROOT/../mujoco_interface/third_party/ecal/etc/ecal}"

unset _WBR_ROOT _SIM_INSTALL _ECAL_ROOT
