# AOD 多视角预览：第一次服务器采集

本轮交付是静态目标的预览采集器。先确认相机、车辆外观、裁剪和记录正确，再扩展完整视点图并训练 AOD。当前还没有训练环境、Q 网络、世界模型或连续飞行控制。

## 1. 同步代码与检查连接

本机代码在分支 `codex/aod-preview`。首次同步到服务器：

```bash
cd /mnt/fast18/sunbo/carla
git status --short
git fetch origin
git switch --track origin/codex/aod-preview
conda activate carla10
python -m pip install -r requirements.txt
python -m src.smoke --config cfg/simulator.yaml
```

如果这个分支已经存在，使用 `git switch codex/aod-preview` 和 `git pull --ff-only origin codex/aod-preview`。若切换提示本地文件会被覆盖，先保存自己的修改，不使用强制覆盖。

保持 CARLA 运行，启动方式见 [服务器环境](setup-server.md)。采集时只运行这一个 tick 主控，不同时运行动态交通、路径代价 Pilot 或其他改变世界的客户端。需要一个没有车辆和行人的空闲世界；采集器不会删除别人创建的对象，也不会自动加载地图。

## 2. 导出真实资产，生成第一份配置

```bash
python -m src.aod inventory --config cfg/simulator.yaml --output out/aod_inventory_v1.json
python -m src.aod prepare --inventory out/aod_inventory_v1.json --spawn-index 0 --output out/aod_preview_v1.yaml
python -m src.aod validate --config out/aod_preview_v1.yaml
```

- inventory 只读服务器，保存当前地图、版本、四轮车辆蓝图和出生点坐标，不推进仿真。
- prepare 和 validate 完全离线，不需要导入 CARLA。
- 默认选清单中按 ID 排序的前三个四轮蓝图；不足三种就使用现有种类。它们只是管线调试对象，不能认为已选定正式辨识类别。
- `--spawn-index 0` 是先尝试清单中的第 0 个真实出生点，并非已经确认适合实验的区域。需要通过预览判断，生成失败或视点被建筑挡住时换清单中的其他出生点。
- 可用 `--blueprints` 后接清单中实际存在的多个 ID 指定车辆。先看 validate 打印的选中 ID。
- 默认把出生点 z 当成临时高度基准，标记为 `spawn_point_unverified`；这不是地面测量。知道实际基准高度后，用 prepare 的 `--ground-z` 明确指定。相机高度是基准加 20、30、40 m。
- 所有输出都拒绝覆盖；改配置或重试时换 v2 文件名和新的采集目录。

配置中 `observation_center` 是所有类别共用的注视中心，`target_pose` 是车辆出生位姿。先制动、等待落地稳定，再冻结车辆。记录冻结后的实际位姿和落地位移；水平漂移超过 1 m 或仍未静止会停止采集。

## 3. 采集与检查

```bash
python -m src.aod capture --config out/aod_preview_v1.yaml --output data/aod_preview_v1
python -m src.aod check --input data/aod_preview_v1
```

预览每类 24 个候选位置：8 个水平位置 × 3 个高度。三类共 72 个候选观测，每个成功观测保存四张派生图片。几何拒绝的节点也写入记录，不强求 72 个成功节点。全被拒绝会返回失败。

水平位置依次为相对中心的 `(10,0), (-10,0), (0,10), (0,-10), (10,10), (10,-10), (-10,10), (-10,-10)` 米。对角点更远，这不是只改变方位角的等距离实验。默认 RGB 为 1920×1080、水平 FOV 60°，仿真步长 0.05 s，相机周期 0.1 s。

采集时直接设置相机位置；此遍历顺序不是无人机动作轨迹。每个位置预热后，只接受帧号、时间戳、拍摄位姿均匹配的 RGB/实例图像对。短暂相机延迟时继续推进 tick，超过帧数或时间上限才失败。CARLA 相机可能延迟返回，帧上携带的 transform 表示拍摄时位姿，依据见 [CARLA API](https://carla.readthedocs.io/en/latest/python_api/) 和 [同步模式说明](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/)。

默认 `warmup_ticks=6`、`settle_ticks=40`、`max_frame_ticks=120`、`timeout_seconds=30`。首次测试可能需要根据 GPU 吞吐调整等待参数。单次等待上限不包含车辆落地、文件写入及整个任务耗时。

退出码：0 成功；2 配置或文件错误；3 依赖、CARLA 或运行错误；130 用户中断。运行失败时先查看日志与 summary；若出现清理失败，先处理服务器残留对象/连接问题，再重新采集。进程被强制终止或 CARLA 原生崩溃时无法保证 Python 清理完成。

## 4. 去哪里看结果

为便于整批复制，图片和诊断文件统一放在一个输出目录：

```text
data/aod_preview_v1/
  inventory.json                 本次连接的资产与出生点清单
  config.json                    解析后配置，其 SHA256 写入 metadata
  capture_config.resolved.yaml   同一份配置的可读 YAML
  metadata.json                  代码提交、天气、类别、最终目标位姿、相机属性
  region_geometry.json           静态 AABB 与查询覆盖情况；人工检查 pending
  nodes.jsonl                    每点状态、拍摄位姿、帧号、目标框与图片路径
  images/c00_h20_p1_rgb.png       原 RGB
  images/c00_h20_p1_instance.png  无损实例颜色；未转换为可视化调色板
  images/c00_h20_p1_overlay.png   原图加目标投影框
  images/c00_h20_p1_crop.png      固定 300×300 裁剪
  contact_sheet_00.png            类别 0 的 3×8 预览拼图
  summary.json                   成功/拒绝/未记录数量、误差、清理与错误状态
  capture.log                    进度及错误，UTC 时间
  checksums.json                 输出文件 SHA256 校验和
```

类别编号按配置 `blueprint_ids` 顺序排列。每类一张拼图，行是高度，列是水平位置。拼图用于定位视角，细节请打开原图和 crop。

check 检查完成状态、节点计数、重复节点、文件校验和、PNG 可读性及尺寸；不验证实际渲染是否合理，也不代表模型实验已经成功。缺失或损坏文件、未完成的采集返回非零退出码。

投影框来自三维框，不能当可见目标轮廓。300×300 窗口以投影中心裁剪，超出原图的区域补黑，不把目标缩放到统一大小；框无有效图像交集时保存黑色 crop 并标记。遮挡、小目标、低亮度都保留，不按“看得清”筛图。实例像素计数是单目标场景的诊断启发式，不是 Python actor.id 对应的精确目标掩码。

几何检查仅用查询到的建筑等静态包围盒膨胀 1 m 排除相机节点，未验证点间路径，也不保证覆盖所有障碍。包围盒 API 的定义见 [CARLA API](https://carla.readthedocs.io/en/latest/python_api/)。同轮沿用当前天气与固定相机属性，但自动曝光、引擎状态等仍需实拍核对；不承诺逐像素复现。

目标蓝图、类别、三维位姿、实例图与几何信息是监督/诊断数据，后续普通策略输入不可直接包含这些答案信息。

## 5. 第一批如何验收

先看三张 contact_sheet，再看几个对应的 rgb、overlay 和 crop：

1. 三类车辆是否各自正确生成，放置位置和注视中心是否一致？
2. 20/30/40 m 与各方位的外观变化是否合理？
3. 框是否对准车辆，crop 是否保留足够辨识信息，是否经常截断目标？
4. 是否存在黑帧、明显曝光变化、上一视点残留画面或进入建筑的相机？
5. summary 是否 complete、清理失败是否为空？换新目录重采后是否有明显差异？

第一批只做工程验收，不计算“我们比 AOD 更好”。默认三类一个区域也不能支持泛化结论。通过后，再扩展 245 点网格、合法移动边、区域划分、单帧分类器和原 AOD 基线。

## 6. 本机开发检查

```bash
python -m pytest -q -p no:cacheprovider
python -m ruff check --no-cache .
python -m src.aod --help
```

本机测试用模拟 CARLA 边界和合成图片，覆盖真实采集逻辑、图片落盘、延迟/错位帧、三类切换、失败清理及文件损坏。真实 CARLA 0.10.0、GPU 渲染与 Python 3.10 运行仍需服务器验收。

实现沿用 AOD 论文的“预采集多视点、离线交互”方向；24 点、CARLA 场景和本轮相机参数是我们自己的工程设置，不是原论文数据集的精确复刻。参考论文：Toward Active Object Detection for UAVs in the Wild: A Large-Scale Dataset, Benchmark and Method；作者入口见 [LUDO_dataset](https://github.com/Leo000ooo/LUDO_dataset)。
