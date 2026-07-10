# mujoco_interface

Generic MuJoCo simulation server with eCAL transport.

Robot-specific MJCF and YAML (e.g. WBR) live in separate controller repos such as [wbr_mujoco](https://github.com/CosmosMount/wbr_mujoco).

## Build (standalone)

```bash
ln -sf /opt/mujoco-3.3.6 mujoco
./scripts/fetch_ecal.sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

## Targets

| target | role |
|--------|------|
| `mujoco_interface_core` | static lib: core + eCAL + viewer (linked by external controllers) |
| `mujoco_interface` | executable: simulation server → `build/bin/mujoco_interface` |
| `test_core` | unit tests |

## Run

```bash
# Generic example config shipped with this repo
./build/bin/mujoco_interface -c config/example.yaml --headless

# WBR robot — point at sibling wbr_mujoco checkout
./build/bin/mujoco_interface \
  -c ../wbr_mujoco/config/robots/wbr.yaml
```

Topic namespace defaults to YAML `ipc_prefix` when present (WBR uses `wbr`).

## With wbr_mujoco

Typical layout:

```
code/
├── mujoco_interface/    # build sim here
└── wbr_mujoco/          # build ctrl here; provides config + mjcf
```

**Terminal 1 — sim**

```bash
cd mujoco_interface
./build/bin/mujoco_interface -c ../wbr_mujoco/config/robots/wbr.yaml
```

**Terminal 2 — controller**

```bash
cd wbr_mujoco
./build/ctrl -c config/robots/wbr.yaml
```

`wbr_mujoco` can also add this repo as a sibling dependency (`-DMUJOCO_INTERFACE_DIR=../mujoco_interface`) and build `ctrl` + `mujoco_interface_core` in one CMake tree; the sim binary ends up at `wbr_mujoco/build/mujoco_interface/bin/mujoco_interface`.

## Git remote (standalone repo)

After splitting from `wbr_mujoco` submodule, fix `origin` / upstream tracking:

```bash
./scripts/fix_git_remote.sh
```

This sets `origin` → `git@github.com:CosmosMount/mujoco_interface.git`, fetches, checks out `architecture` (or `main`), and sets upstream. Override branch:

```bash
MUJOCO_INTERFACE_BRANCH=main ./scripts/fix_git_remote.sh
```

## Layout

```
include/mujoco_interface/
  types.hpp, robot_*.hpp     # robot binding
  core/                      # tick/barrier/simulation
  protocol/messages.hpp      # eCAL wire envelopes
  transport/ecal.hpp         # server + client
src/main.cpp                 # entry
src/viewer/sim_runner.cpp    # main loop
```
