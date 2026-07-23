# mujoco_interface

Generic MuJoCo simulation server with eCAL transport and a C++ client interface.

`mujoco_interface` owns the simulator runtime: MuJoCo model loading, optional
viewer rendering, headless stepping, eCAL topics, Tick/Commit synchronization,
and realtime-rate metrics. Robot-specific MJCF, YAML, and controllers should
live in separate downstream projects.

## Layout

```text
include/mujoco_interface/   public C++ API, protocol, transport, core types
src/                        simulator, transport, config loading, viewer loop
config/example.yaml         minimal example robot config
scripts/                    dependency/setup helpers
tests/                      core tests
```

## Build and install

Prepare MuJoCo and eCAL, then build:

```bash
ln -sf /opt/mujoco-3.3.6 mujoco
./scripts/fetch_ecal.sh

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
cmake --install build --prefix /opt/mujoco_interface
```

The build-tree executable is:

```bash
./build/mujoco_interface
```

The installed executable is:

```bash
/opt/mujoco_interface/bin/mujoco_interface
```

The install prefix is the release artifact. It contains:

- simulator executable;
- public headers;
- `mujoco_interface` CMake package files;
- `libmujoco_interface_core.a`;
- MuJoCo/eCAL runtime libraries required by the simulator and clients;
- eCAL runtime config.

The installed executable uses `$ORIGIN/../lib`, so the install prefix can be
moved as a unit without depending on this source checkout.

## Run

Use any compatible robot YAML:

```bash
./build/mujoco_interface -c /path/to/robot.yaml
```

Headless mode:

```bash
./build/mujoco_interface -c /path/to/robot.yaml --headless
```

Installed runtime:

```bash
/opt/mujoco_interface/bin/mujoco_interface -c /path/to/robot.yaml --headless
```

If `ipc_prefix` is present in YAML, it becomes the default eCAL topic namespace.
You can override the metrics cadence with `--metrics-period-ms`; use
`--metrics-period-ms 0` to disable metrics.

The simulator waits up to 5 ms for each controller commit by default. Once a controller has
registered, a missing commit no longer resets the model: the simulator holds the last command
for 5 ticks, then detaches the inactive controller and applies a zero command while continuing
normal physics stepping. The controller can register again after it recovers. Tune these
limits with `--commit-timeout-us` and `--command-hold-ticks`. Before any controller registers,
the simulator still holds the configured home pose.

Example metric line:

```text
sim metrics steps=1001 sim_time=1.001000 window_step_rate_hz=1000.6 window_real_time_rate=1.001 total_step_rate_hz=1000.6 total_real_time_rate=1.001
```

## CMake package for downstream controllers

Downstream projects should consume the installed package:

```cmake
find_package(mujoco_interface CONFIG REQUIRED)

target_link_libraries(my_controller PRIVATE
  mujoco_interface::mujoco_interface_core
)
```

Configure the downstream project with:

```bash
cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/mujoco_interface
```

## Public targets

| target | role |
| --- | --- |
| `mujoco_interface` | simulator executable |
| `mujoco_interface_core` | static library used by simulator and downstream clients |
| `test_core` | core test executable |

Installed package target:

```text
mujoco_interface::mujoco_interface_core
```
