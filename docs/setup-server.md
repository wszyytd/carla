# CARLA 0.10.0 服务器环境

## 路径边界

- CARLA 发行包：`/mnt/fast18/sunbo/CARLA-0.10.0/Carla-0.10.0-Linux-Shipping`
- 研究仓库：`/mnt/fast18/sunbo/carla`
- Conda 环境：`carla10`
- Python：3.10

发行包和研究代码必须分开。Git 仓库不保存 CARLA 二进制、引擎资源或 Python wheel。

## 安装版本匹配的 Python API

```bash
conda activate carla10
cd /mnt/fast18/sunbo/CARLA-0.10.0/Carla-0.10.0-Linux-Shipping

python -m pip install \
  ./PythonAPI/carla/dist/carla-0.10.0-cp310-cp310-linux_x86_64.whl
```

验证：

```bash
python -c "import carla; print(carla.__file__)"
```

## 启动仿真器

通过图形桌面观察时：

```bash
cd /mnt/fast18/sunbo/CARLA-0.10.0/Carla-0.10.0-Linux-Shipping
LANG=C.UTF-8 LC_ALL=C.UTF-8 ./CarlaUnreal.sh
```

仅在远程终端运行时：

```bash
LANG=C.UTF-8 LC_ALL=C.UTF-8 ./CarlaUnreal.sh -RenderOffScreen -nosound
```

启动前关闭其他占用 CARLA 2000 端口的仿真程序。

## 同步研究仓库后

```bash
cd /mnt/fast18/sunbo/carla
conda activate carla10
python -m pip install -r requirements.txt
python -m pytest -v
```

保持 CARLA 服务器运行，然后执行只读冒烟测试：

```bash
python -m src.smoke --config cfg/simulator.yaml
echo "退出码：$?"
```

正常输出包括客户端/服务器版本、当前地图、同步模式、车辆、行人、Actor 总数和请求耗时，
退出码为 `0`。该命令不会切换地图、修改同步模式、推进世界或创建/销毁 Actor。

其他退出码：配置无效为 `2`，CARLA 导入/RPC/版本解析失败为 `3`，客户端与服务器主次
版本不匹配为 `4`。

## 运行连续弯道路径代价 Pilot

确认冒烟测试成功后，分别运行两个诊断基线：

```bash
cd /mnt/fast18/sunbo/carla
conda activate carla10
python -m src.path_cost --config cfg/experiments/path_cost_pilot.yaml --policy hover
python -m src.path_cost --config cfg/experiments/path_cost_pilot.yaml --policy vertical_follow
```

当前 Town10 工程验收会自动选择一条连续、非路口的 120 m 单车道路弯道（累计转角至少
60°），并在控制台打印所选 road、section、lane 和起始 `s`。运行期间不要启动其他同步
客户端；本进程必须是唯一调用 `world.tick()` 的主控。程序退出时会恢复原始世界设置、关闭
Traffic Manager 同步模式，并只销毁本回合创建的车辆和相机。

这个单一弯道只用于工程验收，不能支持正式研究结论。后续正式实验必须使用多个路线形状和
曲率等级，并在地图可用时加入 S/U 形路线。

路径代价 Pilot 退出码：

- `0`：回合完成，目标执行和观测阈值均通过；
- `2`：配置文件无效；
- `3`：CARLA 导入、RPC、路线选择、演员、传感器或存储运行失败；
- `5`：回合正常完成，但目标执行或观测阈值未通过。该状态是有效实验拒绝，不是崩溃。

先运行 Hover，并保留控制台输出及对应 `out/path_cost/<experiment-id>/summary.json`。
确认路线和目标执行有效后再运行 Vertical Follow；不要通过针对某个策略调参来制造优势。
