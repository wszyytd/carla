# CARLA 0.10.0 视点库服务器操作

本机完成代码及不依赖 CARLA 的验证。首次 v1 实拍记录 17/18 个节点，但 RGB 几乎全黑；修复版需按下面步骤在新 v2 目录重采。
不要对旧 v1 黑帧批次使用 --resume。见 [首次实拍诊断](first-capture-diagnosis.md)。
仓库 `/mnt/fast18/sunbo/carla`，环境 `carla10`。只使用专用空闲 CARLA 实例，停掉其他 tick 主控、交通及环境编辑客户端。

## 1. 同步实现分支

先查看并保存服务器已有修改，不使用 reset/clean/强制 checkout。
首次创建跟踪分支：

```bash
cd /mnt/fast18/sunbo/carla
git status --short
git fetch origin codex/magician-viewbank-implementation
git switch --track -c codex/magician-viewbank-implementation origin/codex/magician-viewbank-implementation
conda activate carla10
python -m pip install -r requirements.txt
```

若该本地分支已经存在，用以下两条替代 `git switch --track -c ...`：

```bash
git switch codex/magician-viewbank-implementation
git pull --ff-only origin codex/magician-viewbank-implementation
```

## 2. 启动 CARLA 并准备地图

在单独终端启动发行包（不要在仓库里解压/提交 CARLA）：

```bash
cd /mnt/fast18/sunbo/CARLA-0.10.0/Carla-0.10.0-Linux-Shipping
LANG=C.UTF-8 LC_ALL=C.UTF-8 ./CarlaUnreal.sh -RenderOffScreen -nosound
```

等待服务器就绪。下列命令**重载专用世界**以获得没有外部交通的 Town10HD_Opt；只在确认实例无他人任务时运行。
采集器自身不会重载地图或销毁外部 Actor。

```bash
cd /mnt/fast18/sunbo/carla
conda activate carla10
python -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(120); print(c.load_world('Town10HD_Opt').get_map().name)"
python -m src.smoke --config cfg/simulator.yaml
```

Python API wheel 必须与 CARLA 0.10.0 对应；安装位置见 [setup-server.md](../setup-server.md)。
不要使用 no-rendering 模式替代 RenderOffScreen。

## 2a. 先检查当前地图的天气能力（只读）

```bash
python -m src.viewbank doctor --config cfg/viewbank/town10_aod_smoke.yaml
```

此命令只读，不切图、不推进世界、不设置天气、不生成传感器或数据目录。
`passed=true` 只表示地图/天气能力/动态 Actor 预检通过，不代表 RGB 或天气设置已经验收。

- `weather_enabled=false`：当前地图没有 CARLA 天气 Actor。不要反复 capture、调曝光或关闭天气检查。
- `weather_enabled=null`：Python API 没有 `is_weather_enabled`；检查输出的 wheel 路径和客户端/服务器版本，安装发行包自带 0.10.0 wheel。
- `weather_enabled=true` 但 capture 仍报 mismatch：采集器最多推进 20 帧，并受客户端超时限制等待异步设置；仍不一致则保留 requested/actual，排查其他天气控制脚本及服务器日志。

CARLA 0.10.0 的无天气 Actor 路径会返回除 Rayleigh=0.0331 外全零的默认天气，
与 2026-09-25 服务器返回值一致；仅凭这一数值模式仍不能代替 `is_weather_enabled()` 的确认。
`set_weather()` 是异步 RPC，因此没有 Python 异常不能证明设置成功。
官方实现见 [服务端天气接口](https://github.com/carla-simulator/carla/blob/0.10.0/Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Server/CarlaServer.cpp)、
[客户端异步设置](https://github.com/carla-simulator/carla/blob/0.10.0/LibCarla/source/carla/client/detail/Client.cpp)。

如果返回 false，在 CARLA 发行包目录查日志（不改文件）：

```bash
cd /mnt/fast18/sunbo/CARLA-0.10.0/Carla-0.10.0-Linux-Shipping
find . -type f -name '*.log' -path '*/Saved/Logs/*' -exec grep -nE 'Missing weather class|weather is disabled|weather actor|Failed to load|Failed to find' {} +
```

先保留 doctor 输出和相关日志。确认专用实例没有其他任务后，可以按第 2 节重新加载同一 Town10 地图，
再跑 doctor；重新加载不保证修复缺失资源。如果仍 false，需要检查该地图使用的 CARLA GameMode 是否配置了
WeatherClass，或关卡中是否存在 CARLA AWeather 派生蓝图，以及对应资源是否被正确打包。
源码 [CarlaGameModeBase.cpp](https://github.com/carla-simulator/carla/blob/0.10.0/Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Game/CarlaGameModeBase.cpp)
会寻找已有天气 Actor，否则尝试通过 WeatherClass 创建。Python 没有一个通用开关可以修补缺失的天气蓝图。
不要为了让检查变绿而未经确认切到不同地图，旧 ROI 和 AABB 必须继续对应真实地图。

本次 v2 在传感器生成前失败，captured=0；修复天气能力后可对**这个空 v2 批次**使用：

```bash
cd /mnt/fast18/sunbo/carla
python -m src.viewbank capture --config cfg/viewbank/town10_aod_smoke.yaml --output data/viewbank/town10_aod_smoke_v2 --resume
python -m src.viewbank check --input data/viewbank/town10_aod_smoke_v2
```

这不适用于旧 v1 黑帧批次。若修改配置则改 scene_id/输出目录，不能混合续采。
早期失败的 `scene.json` 保留版本、preflight 和 weather_application（若已尝试设置）；
`check` 同时展示原始错误。`missing CARLA environment provenance` 是未完成环境初始化的后果，不是另一项独立根因。

## 3. 离线生成小型计划

```bash
python -m src.viewbank plan --config cfg/viewbank/town10_aod_smoke.yaml --output out/viewbank_smoke_plan_v2
cat out/viewbank_smoke_plan_v2/summary.json
```

预期 `requested_nodes=18`、`theoretical_edges=66`、`status=plan_only`，图起点有效且非孤立。
此命令不连接 CARLA；它不能确认实际几何和可见内容。目录存在时换新计划目录，不覆盖。
完整 Pilot 计划可另运行：

```bash
python -m src.viewbank plan --config cfg/viewbank/town10_aod_probe.yaml --output out/viewbank_pilot_plan_v2
```

预期 72 节点、408 条理论边；72 不是业务代码常量。

## 4. 采集 18 节点冒烟批次

```bash
python -m src.viewbank capture --config cfg/viewbank/town10_aod_smoke.yaml --output data/viewbank/town10_aod_smoke_v2
```

退出码 0 才表示采集端机器验收通过。5 是质量失败，3 是运行/清理失败，2 是配置或文件错误，130 是中断。
默认天气 ClearNoon、零风、手动曝光、无动态车辆/行人；交通灯固定红灯。
没有生成 AOD 三类目标车，也不计算发现率。固定高度是世界 z，不能当成实测离地高度。

## 5. 独立 check

```bash
python -m src.viewbank check --input data/viewbank/town10_aod_smoke_v2
cat data/viewbank/town10_aod_smoke_v2/summary.json
cat data/viewbank/town10_aod_smoke_v2/quality.json
cat data/viewbank/town10_aod_smoke_v2/scene.json
```

成功条件：check 返回 0；`passed=true`、`status=complete`、errors 为空；所有几何有效节点采完；
拒绝节点有几何理由；起点有效且非孤立；有效子图连通；清理失败为空；未出现部分帧。
如果几何拒绝起点或使图断开，修改配置并使用新 scene_id/新目录，不通过放宽检查掩盖问题。

## 6. 人工查看什么

用 VS Code Remote SSH 打开 `frames/n_0000/rgb.png`、不同位置/高度的若干 RGB、mask、instance 和 camera.json。
核查黑帧/过曝、上一视点残留、建筑内部视点、图像尺寸、frame/位姿、固定相机参数；
检查 `capture.log` 的失败节点及清理记录，`region_geometry.json` 的查询类别/缺失类别。
`quality.json` 的 `node_quality` 列出每点位姿误差、有限/有效深度比例，camera 中有曝光稳定性诊断。

`depth.npy` 不是预览图，必须读取 float32 米制数值。可在服务器执行：

```bash
python - <<'PY'
from pathlib import Path
import json
import numpy as np
root = Path('data/viewbank/town10_aod_smoke_v2')
node = next(json.loads(line) for line in (root/'nodes.jsonl').read_text().splitlines()
            if json.loads(line)['status'] == 'captured')
d = np.load(root/node['observation']['depth'], allow_pickle=False)
print(node['node_id'], d.shape, d.dtype, 'finite=', np.isfinite(d).mean(),
      'metres=', float(np.nanmin(d)), float(np.nanmax(d)))
PY
```

首次实拍还要检查反投影点云方向、尺度以及平移/转向后的几何关系，确认射线距离→z-depth 的接口假设。
曝光/分辨率不适合时，修改整批配置并换新目录；所有后续策略必须共享最终相机参数。
机器 complete 不会代替人工验收，`server_visual_review` 保留 pending。

## 7. 失败后安全重试

先读 `capture.log`、`scene.json` 的 error/cleanup_failures、quality errors。确认 CARLA 连接恢复，
残留 Actor 已清理，其他客户端均停止。Ctrl+C 会尝试停止/销毁本任务传感器并恢复天气、灯和世界设置；
SIGKILL、机器断电或原生崩溃无法保证服务器清理，必要时重新启动专用 CARLA/重载地图。

同配置、同环境的续采：

```bash
python -m src.viewbank capture --config cfg/viewbank/town10_aod_smoke.yaml --output data/viewbank/town10_aod_smoke_v2 --resume
python -m src.viewbank check --input data/viewbank/town10_aod_smoke_v2
```

已通过 receipt/哈希/语义检查的节点保留，部分/损坏目录隔离到 diagnostic/recovery 后重新采集。
如果出现配置或环境 mismatch，不编辑哈希或强行拼接数据；保留原目录，换新的 scene_id/配置和输出目录。
OS 输出锁随进程退出自动释放；父目录残留 `.town10_aod_smoke_v2.capture.lock` 文件可保留。
出现锁冲突先检查是否有另一个采集进程；不要删除持锁文件来绕过锁。

## 8. 冒烟验收后采完整 72 节点

```bash
python -m src.viewbank capture --config cfg/viewbank/town10_aod_probe.yaml --output data/viewbank/town10_aod_probe_v2
python -m src.viewbank check --input data/viewbank/town10_aod_probe_v2
```

该配置包含 yaw ±90° 边。采集遍历顺序不等于飞行路线，静态 AABB 只能近似排除障碍；
不是完整碰撞网格，不保证真实无人机可飞，可能漏掉未查询障碍物。

## 9. 整库复制到 MAGICIAN 服务器

MAGICIAN 的实际 SSH 地址和仓库路径尚未提供，以下命令交互输入真实目的地，避免写入猜测路径。
先 check 通过；目标根目录应是 MAGICIAN 项目实际使用的 `data/viewbank` 或指定外部数据目录。
不要只复制 frames，必须保留所有清单和 SHA256。下列传输不使用 --delete。

```bash
read -r -p 'MAGICIAN SSH 地址（如 user@host）: ' MAGICIAN_HOST
read -r -p 'MAGICIAN 视点库绝对根目录（不含 scene_id）: ' MAGICIAN_VIEWBANK_ROOT
rsync -av --progress data/viewbank/town10_aod_probe_v2/ "${MAGICIAN_HOST}:${MAGICIAN_VIEWBANK_ROOT}/town10_aod_probe_v2/"
```

目标根目录需事先存在。接收端在带本实现分支的 CARLA 工具仓库、安装本工具离线依赖后，对实际接收路径执行：

```bash
read -r -p '接收后的视点库绝对路径: ' RECEIVED_VIEWBANK
python -m src.viewbank check --input "$RECEIVED_VIEWBANK"
```

如果两个项目就在同一台服务器，可改用本地 `rsync -av` 到实际 MAGICIAN 目录。
数据不纳入 Git；check 通过后再交给未来 ViewBankBackend。不要让普通 planner 直接遍历全库图像。

## 10. 结果能说明什么

这批数据只能支持采集、几何与离线接口的工程验收。尚未实现 MAGICIAN 后端、读取审计、
Random/Greedy/Beam Search 公平对照、Oracle 覆盖评测或多场景泛化；没有训练模型。
因此不能声称 MAGICIAN 更优秀，也不能用 AABB 合法性冒充真实飞行能力。


## 首次实拍修复后的额外质量门

天气名称现在映射到版本 2 的项目固定配置，不把 CARLA 原生预设名称当成已生效的证据。
ClearNoon/CloudyNoon 的太阳高度为 75°，ClearSunset 为 15°；风/降水为零，散射强度 1、
Mie 0.03、Rayleigh 0.0331；云量为 5/60。实际天气必须与请求回读匹配，否则在生成相机前失败。
若服务器天气接口未生效或不支持这些值，不应跳过校验；先排查服务器构建和天气支持。
这些是本项目配置，不声称与任意 CARLA 版本内置预设逐项一致。

`quality.rgb_mean_min=5.0`、`rgb_mean_max=250.0` 检查 RGB 三通道全图平均值（0–255）。
稳定但全黑/全白的图像不再成为成功节点。历史 v1 配置缺少这两个键时使用同样的保守默认值，
解析时不插入键，保留旧配置哈希；新配置显式记录阈值。这只是退化图像质量门，不能替代人工判断曝光。
若 v2 天气读回正常但 RGB 仍过暗，整批标定手动曝光并另建新目录，不降低阈值掩盖黑帧。

短暂的传感器 frame/timestamp 错配会丢弃该 bundle、重建稳定窗口并继续等待，
仍受原超时和最大 tick 数约束，不放宽同帧/时间戳阈值。拒绝次数和最后一组原始帧号/时间戳写入
camera 稳定性诊断；持续错配仍记失败。
局部旋转矩阵的离线重算使用绝对 1e-10 容差以容纳 Linux/Windows 三角函数末位差异，
实际请求/拍摄位姿误差门没有放宽。
