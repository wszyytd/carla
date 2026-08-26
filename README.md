# CARLA 主动观测实验

本仓库用于 CARLA 0.10.0（Unreal Engine 5.5）上的主动目标观测研究。本机负责
编写代码、离线测试和 Git 管理；Linux 服务器负责运行 CARLA、生成交通和采集数据。

它与 `D:\Work\wm` 分开维护：`wm` 保留 CarlaAir 资格测试与后续世界模型工作，
本仓库只放原生 CARLA 实验。两边可以使用兼容的实验条件和指标做 A/B 对照，但不共享
AirSim 运行时逻辑。

## 当前研究顺序

1. 验证 CARLA 客户端连接、地图和 Traffic Manager。
2. 生成目标车辆与背景交通。
3. 用空中 RGB、深度和实例分割相机跟随目标车。
4. 与 CarlaAir 做同距离、同 FOV、同分辨率的 A/B 测试。
5. 构造单车—建筑遮挡场景，比较多条固定观测轨迹。
6. 只有固定轨迹实验表明最优策略随场景变化后，才训练世界模型。

## 目录

```text
cfg/                         YAML 配置
data/                        原始采集数据，不进入 Git
docs/                        设计、计划和服务器说明
out/                         指标、报告和图表，不进入 Git
src/                         入口脚本与实验模块
tests/                       无需启动 CARLA 的离线测试
```

计划中的前三个入口是：

- `src/smoke.py`：连接 CARLA 并打印地图与 Actor 摘要。
- `src/traffic.py`：生成并安全清理 Traffic Manager 车辆。
- `src/follow.py`：选择目标车，让空中相机跟随并保存验证帧。

当前脚手架阶段只建立清晰边界和离线可测试的基础模块，不伪造仿真成功结果。

## 本机开发

建议在 Python 3.10 环境中执行：

```powershell
cd D:\Work\carla
python -m pip install -r requirements-dev.txt
python -m pytest -v
python -m ruff check .
```

本机不需要安装完整 CARLA 发行包，也不运行真实仿真。

## 服务器同步

建议把仓库手动同步到：

```text
/mnt/fast18/sunbo/carla
```

CARLA 0.10.0 发行包继续独立放在服务器现有目录：

```text
/mnt/fast18/sunbo/CARLA-0.10.0/Carla-0.10.0-Linux-Shipping
```

不要把发行包、wheel、采集图片、报告或模型权重提交到 Git。详细步骤见
[`docs/setup-server.md`](docs/setup-server.md)。

## 当前完成状态

仓库结构设计与实施计划已经提交。配置加载、进度输出、CARLA 连接、交通、传感器和
轨迹模块将在各自的测试与服务器验证边界内逐步实现。
