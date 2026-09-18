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
