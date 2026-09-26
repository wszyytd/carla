# CARLA 0.10.0 视点库服务器操作

本机完成代码及不依赖 CARLA 的验证。首次 v1 实拍记录 17/18 个节点，但 RGB 几乎全黑；修复版需按下面步骤在新 v3 固定日间目录重采。
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

## 2a. CARLA 0.10.0 固定日间模式与只读预检

**2026-09-26 更正：天气 API 不可用是 0.10.0 发布版的已知限制，不能据此判定安装损坏。**
[官方发布说明](https://carla.org/2024/12/19/release-0.10.0/) 明确说明天气固定为日间，云、雨、雾和太阳位置不能修改。
此前以“缺少天气 Actor”要求修补地图资产过于武断，已由下面的显式模式替代。

当前两份配置使用 `scene.weather: MapDefaultDaylight`，scene_id 为 v3。
该模式只接受客户端/服务器均为 0.10.0、Town10HD_Opt 且 `is_weather_enabled()=false` 的环境。
这不是自动忽略天气错误：ClearNoon 等原有 API 预设仍要求天气能力存在且数值回读匹配。
固定日间模式不调用 set_weather，也不在清理时伪装恢复天气；它沿用同一个服务器/地图构建内置的照明。

```bash
python -m src.viewbank doctor --config cfg/viewbank/town10_aod_smoke.yaml
```

预期 `passed=true`、`weather_enabled=false`、`lighting_mode=map_default_daylight`。
命令只读，不切图、不推进世界、不创建 Actor。首次 Connection refused 表示当时服务器未监听或尚未就绪，
待 CARLA 完成启动再重试 doctor。若版本或地图不同，先核对实际环境，不自动放行。

元数据中的 `actual_weather`、`weather_request` 和 `environment.weather` 为 null；
`preflight.diagnostic_weather_api_return` 保留原始占位回读，仅作诊断。
全零加 Rayleigh=0.0331 的返回值不能解释为实际太阳高度 0°，也不能用于解释黑帧。
`lighting_parameters_observable=false` 明确表示无法由此 API 证明数值照明条件，
`lighting_review=pending` 要求人为查看同一批的画面。环境指纹仍记录版本、地图、相机和几何，
但它不是完整地图资源哈希，不能检测任意编辑器改灯；不要混用不同地图构建或外部照明控制脚本。

使用新 v3 目录；旧 v1 黑帧批次和旧 v2 ClearNoon 配置均保留作诊断。新旧配置哈希不同，禁止续采混合。
**固定日间适配没有解决或掩盖曝光问题**：RGB 全图均值仍须在 5–250，稳定黑帧仍会失败。
如果 v3 报 dark/overexposed frame，下一步应在相同地图、相同视点下标定整批共享的相机曝光/渲染设置，
保留失败日志并换新配置/目录，不应重装地图天气资源或降低黑帧阈值。

## 2b. v3 黑帧：先做一次曝光对照，再决定完整批次参数

本机只读检查用户的 v3：18/18 因 RGB 退化失败，均值约 0.00126–0.00606，98 个文件哈希全匹配。
天气预检已通过；起点无效、图断开和部分帧是全部节点采集失败的后果，不能用修改图消除。
`frames/.n_XXXX.tmp/` 保留质量检查前的原始文件，没有被原子提交成成功节点。

源码 [0.10.0 ActorBlueprintFunctionLibrary.cpp](https://github.com/carla-simulator/carla/blob/0.10.0/Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Actor/ActorBlueprintFunctionLibrary.cpp)
定义的曝光数值默认值为 ISO 300000、shutter_speed 15（倒数秒）、fstop 9.8、exposure_compensation 1.5。
当前 ISO 100/快门 200/f2.8/补偿 0 没有经过 UE5 实拍标定，不能照搬真实日间摄影经验。
原生默认模式是 histogram；本对照所有候选继续明确使用 manual，以便最终整批固定同一曝光。
源码依据不等于某个候选已通过实拍，生产 YAML 暂不盲目改值。

在唯一 tick 主控、无外部车辆/行人的 CARLA 实例运行：

```bash
cd /mnt/fast18/sunbo/carla
conda activate carla10
python -m src.viewbank calibrate --config cfg/viewbank/town10_aod_smoke.yaml --output out/viewbank_exposure_v1
```

无需先重采 18/72 点。此命令从原配置起点选择一个空间邻居，在同样两个位置顺序测试四组手动曝光：

| case | ISO | shutter_speed | fstop | compensation |
|---|---:|---:|---:|---:|
| baseline | 原配置 | 原配置 | 原配置 | 原配置 |
| ue5_low | 300000 | 15 | 9.8 | -0.5 |
| ue5_default | 300000 | 15 | 9.8 | 1.5 |
| ue5_high | 300000 | 15 | 9.8 | 3.5 |

分辨率/FOV/姿态/天气模式/随机种子/质量阈值相同；候选不自动替换正式配置。
它直接复用 capture，保留每组同帧 RGB-D、原始实例、相机参数、有效/失败状态、哈希和日志。
`cases/` 中每组只是两节点诊断数据，不能作为完整 MAGICIAN 库。

查看或整体复制回本机：

- `report.html`：原始 RGB 并排显示，不做增亮或后期校正；用浏览器打开，须保留 cases 相对目录。
- `report.json`：RGB 均值、像素亮度百分位、近黑比例、接近饱和的通道比例、实际位姿和各组采集结果。
- `cases/<case>/capture.log`、`scene.json`、`frames/`：采集证据；`.tmp` 图像明确 rejected，不能当成功帧。
- `configs/<case>.yaml`：恢复完整输入网格的候选配置，可供人工选定后使用，scene_id 已与 v3 分开。

输出目录存在则拒绝覆盖；对照中断不续跑整个对照目录，换新输出目录。
普通图像质量失败（退出 5）仍继续后续候选；连接/清理错误或中断停止并保存已有报告。
命令返回 0 仅代表至少一组通过两点机器质量门；不代表完成曝光标定或整库验收，selection 始终为 null。
两点均无严重欠曝/过曝且纹理清晰后，人工选定一组，在新的数据目录运行完整网格 capture/check。
所有后续策略使用同一份选定配置。如果四组仍黑或都过曝，把整个对照目录交回分析，
不要关闭质量门，也不要再重复旧 v3 全批。后续 UE5 开发版的 post_process_profile 接口与 0.10.0 tag 不同，
当前采集已确认 manual/iso 等属性存在，不以其他版本的论坛参数替换运行时证据。

## 3. 离线生成小型计划

```bash
python -m src.viewbank plan --config cfg/viewbank/town10_aod_smoke.yaml --output out/viewbank_smoke_plan_v3
cat out/viewbank_smoke_plan_v3/summary.json
```

预期 `requested_nodes=18`、`theoretical_edges=66`、`status=plan_only`，图起点有效且非孤立。
此命令不连接 CARLA；它不能确认实际几何和可见内容。目录存在时换新计划目录，不覆盖。
完整 Pilot 计划可另运行：

```bash
python -m src.viewbank plan --config cfg/viewbank/town10_aod_probe.yaml --output out/viewbank_pilot_plan_v3
```

预期 72 节点、408 条理论边；72 不是业务代码常量。

## 4. 采集 18 节点冒烟批次

```bash
python -m src.viewbank capture --config cfg/viewbank/town10_aod_smoke.yaml --output data/viewbank/town10_aod_smoke_v3
```

退出码 0 才表示采集端机器验收通过。5 是质量失败，3 是运行/清理失败，2 是配置或文件错误，130 是中断。
默认使用地图内置日间照明、手动曝光、无动态车辆/行人；交通灯固定红灯。天气数值和风不可由 API 观测。
没有生成 AOD 三类目标车，也不计算发现率。固定高度是世界 z，不能当成实测离地高度。

## 5. 独立 check

```bash
python -m src.viewbank check --input data/viewbank/town10_aod_smoke_v3
cat data/viewbank/town10_aod_smoke_v3/summary.json
cat data/viewbank/town10_aod_smoke_v3/quality.json
cat data/viewbank/town10_aod_smoke_v3/scene.json
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
root = Path('data/viewbank/town10_aod_smoke_v3')
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
python -m src.viewbank capture --config cfg/viewbank/town10_aod_smoke.yaml --output data/viewbank/town10_aod_smoke_v3 --resume
python -m src.viewbank check --input data/viewbank/town10_aod_smoke_v3
```

已通过 receipt/哈希/语义检查的节点保留，部分/损坏目录隔离到 diagnostic/recovery 后重新采集。
如果出现配置或环境 mismatch，不编辑哈希或强行拼接数据；保留原目录，换新的 scene_id/配置和输出目录。
OS 输出锁随进程退出自动释放；父目录残留 `.town10_aod_smoke_v3.capture.lock` 文件可保留。
出现锁冲突先检查是否有另一个采集进程；不要删除持锁文件来绕过锁。

## 8. 冒烟验收后采完整 72 节点

```bash
python -m src.viewbank capture --config cfg/viewbank/town10_aod_probe.yaml --output data/viewbank/town10_aod_probe_v3
python -m src.viewbank check --input data/viewbank/town10_aod_probe_v3
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
rsync -av --progress data/viewbank/town10_aod_probe_v3/ "${MAGICIAN_HOST}:${MAGICIAN_VIEWBANK_ROOT}/town10_aod_probe_v3/"
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

以下 API 预设仅供天气可控的构建；当前 0.10.0 默认配置使用第 2a 节的固定日间模式。
API 天气名称映射到版本 2 的项目固定配置，不把 CARLA 原生预设名称当成已生效的证据。
ClearNoon/CloudyNoon 的太阳高度为 75°，ClearSunset 为 15°；风/降水为零，散射强度 1、
Mie 0.03、Rayleigh 0.0331；云量为 5/60。实际天气必须与请求回读匹配，否则在生成相机前失败。
若服务器天气接口未生效或不支持这些值，不应跳过校验；先排查服务器构建和天气支持。
这些是本项目配置，不声称与任意 CARLA 版本内置预设逐项一致。

`quality.rgb_mean_min=5.0`、`rgb_mean_max=250.0` 检查 RGB 三通道全图平均值（0–255）。
稳定但全黑/全白的图像不再成为成功节点。历史 v1 配置缺少这两个键时使用同样的保守默认值，
解析时不插入键，保留旧配置哈希；新配置显式记录阈值。这只是退化图像质量门，不能替代人工判断曝光。
若固定日间模式下 RGB 仍过暗，整批标定手动曝光并另建新目录，不降低阈值掩盖黑帧。

短暂的传感器 frame/timestamp 错配会丢弃该 bundle、重建稳定窗口并继续等待，
仍受原超时和最大 tick 数约束，不放宽同帧/时间戳阈值。拒绝次数和最后一组原始帧号/时间戳写入
camera 稳定性诊断；持续错配仍记失败。
局部旋转矩阵的离线重算使用绝对 1e-10 容差以容纳 Linux/Windows 三角函数末位差异，
实际请求/拍摄位姿误差门没有放宽。
