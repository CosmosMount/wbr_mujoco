# wbr_mujoco

WBR RoboMaster balance wheel-leg — controller adapter + robot assets.  
MuJoCo simulation server lives in the sibling **[mujoco_sim_core](https://github.com/CosmosMount/mujoco_sim_core)** workspace (`mujoco-sim-server`).

## Repositories

Clone side by side (sibling directories):

```bash
git clone git@github.com:CosmosMount/wbr_mujoco.git
git clone git@github.com:CosmosMount/mujoco_sim_core.git
# eCAL runtime for adapter SDK (fetch script in mujoco_interface if needed)
git clone git@github.com:CosmosMount/mujoco_interface.git
```

Expected layout:

```
code/
├── wbr_mujoco/          # this repo — controller adapter, MJCF, wbr.yaml
├── mujoco_sim_core/     # sim server + adapter SDK + lockstep core
└── mujoco_interface/    # vendored eCAL under third_party/ecal/usr
```

## Layout (wbr_mujoco)

```
controller/          control core + sim adapter I/O
config/robots/       wbr.yaml (adapter) + wbr_server.yaml (server)
mjcf/                scene + meshes
tests/               test_import (MJCF resource contract)
```

## Build

### One-time setup (mujoco_sim_core)

```bash
cd ../mujoco_sim_core
colcon build --packages-up-to mujoco_sim_server --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

### Controller + tests (this repo)

```bash
cd wbr_mujoco
cmake -B build -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_SIM_INSTALL_DIR=../mujoco_sim_core/install
cmake --build build
```

`cmake` looks for `../mujoco_sim_core/install` and `../mujoco_interface/third_party/ecal/usr` by default. Override if needed:

```bash
cmake -B build \
  -DMUJOCO_SIM_INSTALL_DIR=/path/to/mujoco_sim_core/install \
  -DECAL_ROOT=/path/to/ecal/usr
```

`ctrl` links `mujoco_sim::adapter_sdk` and runs as a Tick/Commit adapter. Keyboard input arrives on the `operator_input` topic published by the server GUI.

## Run

Two terminals. Build and source `mujoco_sim_core/install` in both. For eCAL config (`ecal.yaml`) and time-sync plugins, also:

```bash
source scripts/env.sh   # sets ECAL_DATA, LD_LIBRARY_PATH, PATH
```

```bash
# Terminal 1 — simulation server (from wbr_mujoco, or mujoco_sim_core install on PATH)
source scripts/env.sh
mujoco-sim-server --config config/robots/wbr_server.yaml

# Terminal 2 — controller adapter
source scripts/env.sh
./build/ctrl -c config/robots/wbr.yaml
```

Headless server (no viewer, use `input_script` / e2e config for scripted keys):

```bash
mujoco-sim-server --config config/robots/wbr_server.yaml --headless
./build/ctrl -c config/robots/wbr_e2e.yaml
```

Or run the bundled smoke script (requires built `mujoco-sim-server` and `ctrl`):

```bash
./scripts/e2e_headless.sh
```

Focus the sim window for live keyboard input. YAML `adapter.server_name` (legacy `ipc_prefix`) must match the server `server_name`.

## Architecture

- **Server** owns MuJoCo, publishes Tick observations and `operator_input` (GUI only).
- **ctrl** uses `SyncAdapterClient` for register/activate/Tick/Commit and `OperatorInputClient` for keyboard, with `input_script` fallback when operator input is stale.
