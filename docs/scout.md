# Terminal camera scout

Start CARLA first, then run from the repository root:

```bash
conda activate carla10
python -m src.scout
```

Each movement saves an RGB PNG and a JSON record containing actual camera pose,
frame, simulation timestamp, map, image dimensions, FOV, and the command.
Files go to `out/scout/<timestamp>/`. Open images with VS Code Remote SSH.

Commands (press Enter after each):

| Command | Action |
| --- | --- |
| `w 10` / `s 10` | Horizontal forward / backward 10 m |
| `a 10` / `d 10` | Horizontal left / right 10 m |
| `up 5` / `down 5` | Vertical movement 5 m |
| `yaw 30` | Relative right turn 30 degrees |
| `pitch -45` | Absolute downward pitch 45 degrees |
| `home` | Return to initial spectator pose |
| `spawns` | List road spawn locations |
| `spawn 0 40` | Move 40 m above road spawn 0 |
| `load out/scout/<timestamp>/0001.json` | Restore a captured pose |
| `save` | Capture current pose again |
| `pose` / `help` / `quit` | Inspect / help / exit |

Distances default to 5 m when omitted. Movement is relative to camera yaw,
with horizontal movement independent of pitch. This is a camera scouting tool:
it does not simulate a drone or check collisions. World settings are unchanged.
Do not run alongside another controller moving the same spectator.

If the world is already synchronous, stop other ticking clients and run
`python -m src.scout --tick`. Do not use `--tick` in an asynchronous world.
The sensor is destroyed and spectator restored on normal exit or Ctrl+C.
No-rendering mode must be disabled for RGB capture.

Local unit tests cover displacement and pose matching. Actual CARLA rendering
must be verified on the server.

## Automatic batch scouting

```bash
python -m src.scout --auto
```

Default: up to 12 spatially spread road spawn positions, 40 m above each spawn,
pitch -45 degrees, world yaw 0/90/180/270. This produces up to 48 images then exits.
It reuses the interactive capture checks for fresh frames at the requested pose.

```bash
python -m src.scout --auto --locations 20 --heights 30 60 --width 1280 --height 720
```

This produces up to 160 images. Add `--tick` only for an already synchronous
world with no other tick client. The output folder contains numbered PNG/JSON
pairs, `plan.json`, `summary.json`, and `overview_01.jpg` etc. (24 images per page).
Use the image number to locate its JSON, then use interactive `load PATH.json`
to return to that position. Heights are relative to road spawn elevation, not
measured terrain clearance. Sampling covers road positions; it does not guarantee
park coverage, collision-free poses, or interesting occlusion. Images from roofs,
bridges, or streaming delays need visual screening. This batch is for selecting
regions, not a physically flown trajectory or a finalized training dataset.

Timeouts are recorded and skipped. Three initial failures abort the batch; any
failed view makes the completed batch exit nonzero. Ctrl+C preserves already
saved images and generates an overview of them during cleanup.

## 第七张院落：同一起点的路线对照

这是场景筛选工具，不是世界模型策略、目标检测器或可飞行路径规划器。
起点已收录到 `cfg/scenes/yard_scout_0007.json`，服务器无需复制原截图目录。
先同步代码到服务器，保持 Town10HD_Opt 地图和静态场景；停止会移动车辆的交通脚本。
本工具不冻结其他 Actor，不重置天气、车辆、物理状态，也不修改同步设置。

先检查计划（本机也能运行，不连接 CARLA）：

```bash
python -m src.scout --compare-from cfg/scenes/yard_scout_0007.json --dry-run --output out/yard_compare_plan
```

在运行 CARLA 的服务器仓库根目录执行：

```bash
conda activate carla10
python -m src.scout --compare-from cfg/scenes/yard_scout_0007.json --output out/yard_compare
```

默认每步 10 m、每条路线两步，10 条路线、30 张图。每条路线都重新采集起点图作为停止对照，
然后采集第一步和第二步；每条完整路线的请求位移都是 20 m。
路线包括上升、下降、左移、右移、前进、后退各两步，以及左→上、上→左、右→上、上→右。
左右/前后相对相机 yaw，水平方向与 pitch 无关；整个比较保持朝向不变。
想缩小邻域可加 `--step-m 5`，每条路线总位移随之变为 10 m。

默认继承源 JSON 的分辨率和 FOV；显式指定 `--width`、`--height`、`--fov` 可以覆盖，
但不要把不同相机参数的批次当成同条件对照。俯仰角来自源 JSON，`--pitch` 仅用于 `--auto`。
地图不一致时会报错，不自动切换地图。`--auto` 与 `--compare-from` 不能同时使用。
已有同步世界且没有其他 tick 客户端时才加 `--tick`；异步世界不要加。

输出位置为 `out/yard_compare/<时间戳>/`：

- `comparison.html`：离线浏览器打开，每行一个路线的起点、第一步、第二步；点击图片放大。
- `comparison.csv`：路线、动作、请求位姿、累计请求位移、状态、图片名；留有人工记录列。
- `0001.png` / `0001.json`：图像及该图真实传感器位姿、frame、时间戳和路线信息。
- `plan.json`：所有计划位姿、地图和相机参数。
- `summary.json`：成功、失败、未执行数量和准确的图片对应关系；中断也保留报告。
- `overview_*.jpg`：便于快速浏览的拼图。

复制结果时保留整个输出文件夹，HTML 与图片需放在一起。打开 HTML 可使用浏览器；
VS Code 普通文本编辑器打开 HTML 只会显示源码。
失败视点不会用后续图片冒充；超时会在报告留空并标明原因，采集不完整时返回非零退出码。

### 这批结果怎么用

1. 看每条路线是否确实露出不同地面，记录新看到的院落、通道或广场。
2. 人工确认是不是同一辆车，填写 `new_target_count` 和 `notes`；不知道就留空。
3. 没有地面投影/遮挡测量时，不填写 `new_visible_ground_m2`，不能从 RGB 像素面积推算平方米。
4. 上→左和左→上的终点相同，在静态场景中终点图应近似相同；差别在途中见过的区域。
5. 只有筛出有意义的观察差异后，才定义搜索 ROI、配置带身份的目标车辆、检查碰撞并收集训练轨迹。

**限制：** 相机直接设置到各采样位姿，不检查端点或两点之间的碰撞，不保存沿途视频，
路径长度只是请求位姿之间的几何折线长度，不包含路线重置，不代表可执行飞行或真实耗能。
世界 z 坐标不是测得的离地高度；不要把穿过屋顶后得到的图片当成可执行路线。
这批图可以验证相机和观察关系，不能直接作为世界模型效果对比、覆盖率或发现率结论。

## 稳定采集与同帧测量（v2）

之前只等待新帧与位姿匹配，不能保证曝光已收敛。新版所有 scout 模式都使用以下默认设置：

- 手动曝光：`exposure_mode=manual`、ISO 100、快门 1/200 s、f/2.8、曝光补偿 0；关闭运动模糊。
- 每个视点至少预热 2 个仿真秒；约每 0.1 个仿真秒取一次稳定性样本。
- 最近 8 个样本覆盖至少 0.5 个仿真秒。RGB 通道均值范围和空间采样误差中位数均不超过 1/255。
- 空间中位数用于容忍少量海面动画，不代表每个像素都静止。亮度稳定也不能保证资产全部加载。
- 等不到稳定图像就记录超时，不把仍在变化的画面当成功。`--timeout` 是墙钟秒，必须大于预热时间加 1 秒。
- 批次结束比较相同请求位姿的平均 RGB，任一通道变化超过 3/255 时在 `quality.json` 标记失败并返回非零退出码。
  没有重复位姿时结果为 null（未评估），不是通过。重复性门槛是工程验收参数，不是科研结论。

路线对照模式默认同时采集 RGB、深度和实例分割。三者必须具有完全相同的 frame、相符的时间戳、
尺寸和相机位姿。每种传感器的原始帧号保存在对应编号 JSON 的 `sensor_frames`。
普通交互/自动找景模式仍只采集 RGB；路线对照也可加 `--rgb-only` 排查渲染问题。

服务器推荐先重新采一批，不覆盖旧数据：

```bash
python -m src.scout --compare-from cfg/scenes/yard_scout_0007.json --output out/scout/yard_compare_v2 --timeout 60
```

仍然沿用旧命令也可以，会启用新版默认设置。以下参数可用于全批次统一调节：

- `--warmup-seconds 4`：延长每个视点的仿真预热时间。
- `--stability-threshold 2`：放宽单视点稳定门槛；先查日志中的误差原因，不要为追求通过盲目放宽。
- `--exposure-compensation 1`：统一增加曝光补偿；不要对不同路线分别调参。
- `--exposure-mode histogram`：仅供诊断自动曝光；同样执行预热、稳定检查和跨路线重复性检查。

手动曝光的默认值需要服务器实拍验证，不能保证所有天气都曝光合适。
如果图像过暗或过曝，先固定天气并标定整批使用的曝光；不要拿 v1 自动曝光图与 v2 直接比较检测成绩。
若当前 CARLA 蓝图缺少必需属性，脚本会明确报属性名，不会静默退回自动曝光。

新增输出：

| 文件 | 内容 |
| --- | --- |
| `capture_settings.json` | 实际请求的曝光/稳定参数、当前 CARLA 车辆语义标签、传感器列表 |
| `quality.json` | 重复位姿亮度范围及验收结果 |
| `depth/0001.npy` | float32 深度，单位米，保留 0–1000 m 编码范围 |
| `depth/0001.png` | 原始 RGB 编码深度，不能直接当灰度距离读 |
| `depth/0001_preview.png` | 0–150 m 映射的灰度预览，超过范围截断；仅用于查看 |
| `instance/0001.png` | 原始实例编码图；不要做调色、JPEG 压缩后再解码 |
| `0001.json` 的 `measurements` | 可见车辆类实例、像素数、图像包围框及沿本路线的新增/累计实例统计 |

深度按官方公式 `(R + 256*G + 65536*B) / (256^3 - 1) * 1000` 解码。
原始 `raw_data` 是 BGRA 顺序。实例键使用 `语义标签:绿色字节:蓝色字节`，
只在当前运行内用于核查重复出现；不能当作 Python Actor ID，也不能直接跨地图重载关联。
车辆标签从当前客户端 `CityObjectLabel` 枚举获取，不硬编码旧版标签 10。

可见实例至少有 20 个像素。初始帧计入累计实例，“移动新增实例”从第一步开始算；
每条路线独立重置历史。此前任一步漏采，新增/累计统计为 null，避免把“没记录过”误当“首次发现”。
这些是仿真分割标签的诊断量，不是训练好的检测器输出；不能直接作为车辆发现率或 RL 奖励。
地图装饰车辆可能未标成车辆，一辆车也可能含多个网格实例，所以还需核验与真实目标的一一对应关系。

当前没有自动划定搜索 ROI、生成目标清单或计算地面覆盖面积；也未实现“此前被遮挡/此前在视野外”的
自动归因。新增测量数据为后续世界坐标投影和目标身份核验提供输入。
碰撞检查、连续飞行控制与世界模型训练仍不在这条采集命令内。

传感器配置与解码依据：[CARLA UE5 官方传感器文档](https://carla-ue5.readthedocs.io/en/latest/ref_sensors/)。
