# mujoco_interface

[English](README.md)

通用 MuJoCo 仿真服务端，提供 eCAL transport 和 C++ client interface。

`mujoco_interface` 负责 simulator runtime：MuJoCo 模型加载、可选 viewer
渲染、headless stepping、eCAL topic、Tick/Commit 同步以及 realtime-rate
指标。机器人自己的 MJCF、YAML 和控制器应放在下游项目中维护。

## 目录

```text
include/mujoco_interface/   public C++ API、protocol、transport、core types
src/                        simulator、transport、config loading、viewer loop
config/example.yaml         最小示例机器人配置
scripts/                    依赖/环境辅助脚本
tests/                      core tests
```

## 编译与安装

先准备 MuJoCo 和 eCAL，然后编译：

```bash
ln -sf /opt/mujoco-3.3.6 mujoco
./scripts/fetch_ecal.sh

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
cmake --install build --prefix /opt/mujoco_interface
```

构建目录中的可执行文件：

```bash
./build/mujoco_interface
```

安装后的可执行文件：

```bash
/opt/mujoco_interface/bin/mujoco_interface
```

安装前缀就是发布产物，包含：

- simulator 可执行文件；
- public headers；
- `mujoco_interface` CMake package；
- `libmujoco_interface_core.a`；
- simulator 和下游 client 所需的 MuJoCo/eCAL runtime libraries；
- eCAL runtime config。

安装后的可执行文件使用 `$ORIGIN/../lib`，所以整个安装前缀可以作为一个整体移动，
不会反向依赖本源码仓库。

## 运行

传入任意兼容的机器人 YAML：

```bash
./build/mujoco_interface -c /path/to/robot.yaml
```

Headless 模式：

```bash
./build/mujoco_interface -c /path/to/robot.yaml --headless
```

安装后运行：

```bash
/opt/mujoco_interface/bin/mujoco_interface -c /path/to/robot.yaml --headless
```

如果 YAML 中存在 `ipc_prefix`，它会作为默认 eCAL topic namespace。可以用
`--metrics-period-ms` 调整指标输出周期；传 `--metrics-period-ms 0` 可关闭指标。

simulator 默认等待每个 controller commit 最多 5 ms。controller 注册后，单次缺失 commit
不再复位模型：simulator 先保持上一条命令 5 个 tick，继续缺失则解除失联 controller 的注册、
切换为零命令并保持正常物理步进。controller 可在恢复后重新注册。
可使用 `--commit-timeout-us` 和 `--command-hold-ticks` 调整这两个限制。在任何 controller
注册之前，simulator 仍然保持配置中的 home 姿态。

指标示例：

```text
sim metrics steps=1001 sim_time=1.001000 window_step_rate_hz=1000.6 window_real_time_rate=1.001 total_step_rate_hz=1000.6 total_real_time_rate=1.001
```

## 下游控制器接入

下游项目应使用安装后的 CMake package：

```cmake
find_package(mujoco_interface CONFIG REQUIRED)

target_link_libraries(my_controller PRIVATE
  mujoco_interface::mujoco_interface_core
)
```

配置下游项目：

```bash
cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/mujoco_interface
```

## Public targets

| target | 作用 |
| --- | --- |
| `mujoco_interface` | simulator 可执行文件 |
| `mujoco_interface_core` | simulator 和下游 client 使用的 static library |
| `test_core` | core test executable |

安装后的 package target：

```text
mujoco_interface::mujoco_interface_core
```
