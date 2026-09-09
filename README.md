# CARLA 主动观测实验

本仓库用于 CARLA 0.10.0（Unreal Engine 5.5）上的主动目标观测研究。本机负责
编写代码、离线测试和 Git 管理；Linux 服务器负责运行 CARLA、生成交通和采集数据。

它与 `D:\Work\wm` 分开维护：`wm` 保留 CarlaAir 资格测试与后续世界模型工作，
本仓库只放原生 CARLA 实验。两边可以使用兼容的实验条件和指标做 A/B 对照，但不共享
AirSim 运行时逻辑。

## 当前研究顺序

1. 验证 CARLA 客户端连接、地图和 Traffic Manager。
2. 生成目标车辆与背景交通。
3. 在 Town10 的连续、非路口 120 m 弯道（总转角至少 60°）上进行工程验收，比较 Hover 与
   Vertical Follow 的观测有效性和路径长度。
4. 扩展 Reactive、有限时域前瞻和全路线 Oracle 策略。
5. 与 CarlaAir 做同距离、同 FOV、同分辨率的 A/B 测试。
6. 构造单车—建筑遮挡场景，比较多条固定观测轨迹。
7. 只有固定轨迹实验表明最优策略随场景变化后，才训练世界模型。

## 目录

```text
cfg/                         YAML 配置
data/                        原始采集数据，不进入 Git
docs/                        设计、计划和服务器说明
out/                         指标、报告和图表，不进入 Git
src/                         入口脚本与实验模块
tests/                       无需启动 CARLA 的离线测试
```

入口状态：

- `src/smoke.py`：已实现，只读连接 CARLA 并打印地图与 Actor 摘要。
- `src/path_cost.py`：已实现，自动选择连续、非路口的单车道路窗口并运行路径代价 Pilot。
- `src/traffic.py`：后续生成并安全清理 Traffic Manager 车辆。
- `src/follow.py`：后续选择目标车，让空中相机跟随并保存验证帧。

## 本机开发

建议在 Python 3.10 环境中执行：

```powershell
cd D:\Work\carla
python -m pip install -r requirements-dev.txt
python -m pytest -v
python -m ruff check .
```

本机不需要安装完整 CARLA 发行包，也不运行真实仿真。

## 连接冒烟测试

同步到服务器并启动 CARLA 后执行：

```bash
python -m src.smoke --config cfg/simulator.yaml
```

该命令只读取客户端/服务器版本、地图、同步模式以及车辆、行人和全部 Actor 数量。它不
加载地图、不应用世界设置、不推进 tick，也不生成或销毁 Actor。

退出码：

- `0`：连接和摘要读取成功。
- `2`：配置文件无效。
- `3`：CARLA 包导入、RPC 读取或版本解析失败。
- `4`：客户端与服务器主次版本不匹配。

## 连续弯道路径代价 Pilot

当前 Town10 工程验收从驾驶车道中选择一条连续、非路口的 120 m 弯道，要求累计转角至少
60°，再选择配置中的候选名次。目标车辆通过 Traffic Manager `set_path` 沿真实车道行驶；
空中 RGB 与实例分割相机按同一 CARLA 帧配对。控制台会打印所选 road、section、lane、起始
`s`、目标执行质量、有效观测比例以及无人机/目标路径长度。这个单一弯道只用于工程验收，
不足以支持正式研究结论；后续正式实验必须覆盖多个路线形状和曲率等级，并在地图可用时包含
S/U 形路线。

当前 Pilot 只有一辆动态目标车且不生成背景交通。实例像素数在目标投影框内，从语义标签为
车辆的像素中选择出现最多的 `(G,B)` 实例颜色来计算；CARLA 的实例颜色不能与 Python
`actor.id` 对应。该方法只适用于这个单目标、无背景交通的工程验收，不能作为多车场景中的
身份关联方法。

服务器运行命令：

```bash
cd /mnt/fast18/sunbo/carla
conda activate carla10
python -m src.path_cost --config cfg/experiments/path_cost_pilot.yaml --policy hover
python -m src.path_cost --config cfg/experiments/path_cost_pilot.yaml --policy vertical_follow
```

运行期间该进程是唯一的同步 tick 主控，不要同时运行其他会调用 `world.tick()` 的客户端。
退出码 `5` 表示仿真正常完成但实验阈值未通过，是有效的实验拒绝结果，不是软件崩溃。
每次运行的解析配置、逐帧 CSV、摘要和抽样 PNG 写入 `out/path_cost/<experiment-id>/`。

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

配置验证、即时进度输出、只读连接冒烟测试，以及 Hover/Vertical Follow 的单场景路径
代价 Pilot 已完成本机离线测试。真实地图候选、车辆路径跟随、传感器吞吐和阈值仍需在
服务器的 CARLA 0.10.0 环境完成首次验收；Reactive、前瞻规划和 Oracle 尚未实现。
