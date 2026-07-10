# mujoco_interface

[English](README.md)

通用 MuJoCo 仿真服务端，eCAL 传输。  
机器人 MJCF / YAML（如 WBR）放在 [wbr_mujoco](https://github.com/CosmosMount/wbr_mujoco) 等独立仓库。

## 独立编译

```bash
ln -sf /opt/mujoco-3.3.6 mujoco
./scripts/fetch_ecal.sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

## 运行

```bash
./build/bin/mujoco_interface -c config/example.yaml --headless

# WBR — 指向同级 wbr_mujoco
./build/bin/mujoco_interface -c ../wbr_mujoco/config/robots/wbr.yaml
```

未指定 `--topic-ns` 时使用 YAML 中的 `ipc_prefix`。

## 与 wbr_mujoco 联调

```
code/
├── mujoco_interface/    # 编译 sim
└── wbr_mujoco/          # 编译 ctrl；提供 config + mjcf
```

```bash
# 终端 1
cd mujoco_interface
./build/bin/mujoco_interface -c ../wbr_mujoco/config/robots/wbr.yaml

# 终端 2
cd wbr_mujoco
./build/ctrl -c config/robots/wbr.yaml
```

`wbr_mujoco` 也可通过 `-DMUJOCO_INTERFACE_DIR=../mujoco_interface` 一次 CMake 编出 `ctrl` 和 `mujoco_interface_core`；sim 可执行文件在 `build/mujoco_interface/bin/mujoco_interface`。

## Git remote（独立仓库）

从 `wbr_mujoco` 子模块拆出后，修复 remote / upstream：

```bash
./scripts/fix_git_remote.sh
```

会将 `origin` 设为 `git@github.com:CosmosMount/mujoco_interface.git`，fetch 并 checkout `architecture`（或 `main`）且设置 upstream。指定分支：

```bash
MUJOCO_INTERFACE_BRANCH=main ./scripts/fix_git_remote.sh
```

## 布局

```
include/mujoco_interface/   头文件 + core + protocol + transport
src/main.cpp                入口
src/viewer/sim_runner.cpp   主循环
```

`mujoco_interface_core` 供外部控制器链接；`mujoco_interface` 为仿真服务端可执行文件。
