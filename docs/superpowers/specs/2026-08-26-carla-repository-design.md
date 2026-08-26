# CARLA 研究仓库设计

## 目标

在 `D:\Work\carla` 建立一个独立的 Git 仓库，用于 CARLA 0.10.0（UE5.5）研究。
本机负责代码、离线测试和版本管理，用户手动将仓库同步到 Linux 服务器；CARLA
发行包、仿真运行和真实采集均留在服务器。

仓库服务于以下渐进路线：

1. 验证 CARLA Python API 连接、地图与 Traffic Manager。
2. 生成目标车辆和背景交通。
3. 用空中传感器跟随目标车并采集 RGB、深度和实例分割。
4. 与 CarlaAir 做同距离、同 FOV、同分辨率的 A/B 对照。
5. 构造单车—建筑遮挡场景并比较固定观测轨迹。
6. 只有固定轨迹实验表明最优路径随场景变化后，才进入世界模型阶段。

## 与现有 `wm` 仓库的关系

新仓库沿用 `D:\Work\wm` 已形成的顶层布局和工具风格：

- 使用 `cfg/` 保存 YAML 配置。
- 使用 `data/` 保存原始采集数据。
- 使用 `out/` 保存指标、报告和图表。
- 使用 `docs/` 保存设计、计划和服务器操作说明。
- 使用 `src/` 保存 Python 入口与模块。
- 使用 `tests/` 保存无需连接仿真器的测试。
- 保持 Python 3.10、Ruff、pytest、`requirements.txt` 和
  `requirements-dev.txt` 的组织方式。

两个仓库保持独立。`wm` 继续保留 CarlaAir 资格测试和后续世界模型代码；
`carla` 只负责原生 CARLA 0.10.0 实验。A/B 对照的数据格式可以兼容，但第一阶段
不复制 `wm/src/qualify`，避免带入 AirSim RPC、坐标系和兼容逻辑。

## 初始目录

```text
carla/
├── cfg/
│   ├── simulator.yaml
│   ├── traffic.yaml
│   └── experiments/
├── data/
│   ├── smoke/
│   ├── qualify/
│   └── scenarios/
├── docs/
│   ├── setup-server.md
│   ├── research-roadmap.md
│   └── superpowers/
│       ├── plans/
│       └── specs/
├── out/
│   ├── smoke/
│   ├── qualify/
│   └── scenarios/
├── src/
│   ├── __init__.py
│   ├── smoke.py
│   ├── traffic.py
│   ├── follow.py
│   └── carla_experiments/
│       ├── __init__.py
│       ├── client.py
│       ├── config.py
│       ├── progress.py
│       ├── actors.py
│       ├── sensors.py
│       ├── storage.py
│       ├── metrics.py
│       ├── scenarios/
│       └── trajectories/
├── tests/
├── .gitignore
├── pyproject.toml
├── README.md
├── requirements.txt
└── requirements-dev.txt
```

Git 不保存空目录，因此 `data/` 和 `out/` 的实际运行目录由程序按需创建，不放
`.gitkeep`。初始提交只创建代码、配置、文档与测试骨架。

## 模块边界

- `src/smoke.py`：验证客户端连接、打印地图、端口和 Actor 摘要。
- `src/traffic.py`：按配置生成并清理 Traffic Manager 车辆。
- `src/follow.py`：选择目标车，创建空中相机并持续跟随，保存少量验证帧。
- `client.py`：建立 CARLA 客户端、设置超时并管理 world settings。
- `actors.py`：生成、选择和销毁目标车辆与背景车辆。
- `sensors.py`：创建 RGB、深度和实例分割传感器，并执行同帧采集。
- `storage.py`：建立 Run ID 目录并原子化写入图像和元数据。
- `metrics.py`：计算目标投影尺寸、可见像素、FPS和同步状态。
- `scenarios/`：以后加入建筑遮挡等可重复场景。
- `trajectories/`：以后加入尾随、抄近路、前方等待、原地等待和上升越障。

脚本层只编排流程，算法和仿真接口放在小模块中，便于用假对象离线测试。

## 配置和数据流

配置使用 YAML，默认连接 `localhost:2000`，客户端超时 10 秒。服务器运行流程为：

```text
读取配置
→ 连接 CARLA 0.10.0
→ 选择地图和同步模式
→ 生成目标与背景交通
→ 创建空中传感器
→ 等待传感器预热并取得稳定同帧数据
→ 写入 data/<experiment>/<run_id>/
→ 计算指标并写入 out/<experiment>/<run_id>/
→ 销毁本次创建的 Actor
→ 恢复 world settings
```

每个运行目录包含 `manifest.json`，记录 CARLA 版本、地图、天气、随机种子、配置和
Git 提交。运行失败也应留下状态和错误原因。

## Git 与大文件边界

`.gitignore` 至少排除：

- Python 缓存、pytest/Ruff 缓存和虚拟环境。
- `data/`、`out/` 中的运行产物。
- 日志、模型权重和检查点。
- CARLA 发行目录、压缩包、wheel、egg 和核心转储。
- 本地环境文件和编辑器临时文件。

CARLA 发行包继续位于服务器现有目录，例如
`/mnt/fast18/sunbo/CARLA-0.10.0/Carla-0.10.0-Linux-Shipping`，不进入本仓库。
仓库建议同步到 `/mnt/fast18/sunbo/carla`。

## 错误处理

- 连接、加载地图、生成 Actor 和等待传感器均设置有限超时。
- 程序只销毁自己创建的 Actor，不清理其他客户端对象。
- 使用 `try/finally` 恢复同步模式和 Traffic Manager 设置。
- 每个阶段输出立即刷新的进度日志，远程终端不应长时间无反馈。
- 相机创建或换位后丢弃预热帧；只有 RGB、深度和实例分割连续稳定后才正式记录。

## 测试和验收

初始脚手架的验收条件：

- 目录与 `wm` 顶层结构一致。
- Git 默认分支为 `main`，存在初始提交，工作区干净。
- `python -m pytest` 能发现并运行骨架测试。
- `python -m ruff check .` 通过。
- 配置加载、路径构造和资源清理可在没有 CARLA 的本机使用假对象测试。
- README 清楚区分本机开发与服务器仿真，并给出手动同步后的启动顺序。

真实 CARLA 冒烟测试不属于本机脚手架验收；完成同步后在服务器执行。

## 初始范围之外

本阶段不实现世界模型、目标检测器训练、AirSim 无人机动力学、大规模数据采集、
CARLA 源码编译或自动部署。它们只有在连接、交通和空中相机最小链路验证后才逐步加入。
