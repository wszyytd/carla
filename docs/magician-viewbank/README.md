# MAGICIAN 离线 RGB-D 视点库

当前实现范围是 CARLA 数据采集端：`plan`、`capture`、`check`。本机离线测试使用合成图像和
模拟 CARLA 边界；**服务器 CARLA 0.10.0 实拍待运行**。未来的 `ViewBankBackend`、访问审计、
覆盖评测和 MAGICIAN 策略不在本轮实现范围。[需求分析](requirements.md) 中跨仓库的后续验收门仍然保留。

## 命令

从仓库根目录执行：

```bash
python -m src.viewbank plan --config cfg/viewbank/town10_aod_probe.yaml --output out/viewbank_plan
python -m src.viewbank capture --config cfg/viewbank/town10_aod_probe.yaml --output data/viewbank/town10_aod_probe_v1
python -m src.viewbank check --input data/viewbank/town10_aod_probe_v1
```

`plan` 和 `check` 不导入 CARLA、不连接服务器。`plan` 输出 `config.resolved.json`、
`nodes.jsonl`、`edges.jsonl`、`summary.json`，明确标记 `plan_only`、`geometry_checked=false`。
理论节点/边的 `valid=true` 不是物理碰撞验收。输出目录已存在时拒绝覆盖。

`capture` 需要专用、无背景交通的 CARLA 实例和唯一 tick 主控；不自动换地图或删除外部 Actor。
启动前停掉交通、scout、AOD、path_cost 和其他改动世界的客户端。
同目录重试必须显式添加 `--resume`，配置内容及已记录环境指纹必须一致。

退出码：`0` 成功；`2` 配置/文件/输出锁错误；`3` CARLA 导入、连接、运行或清理失败；
`5` 采集质量门未通过或离线检查失败；`130` Ctrl+C 中断。
单节点超时记为 `capture_failed`，继续处理其他节点，整批返回 `5`。
运行中 RPC 等异常终止本轮、恢复设置并保留已完成帧，返回 `3`。

## 配置和图

所有配置字段必须显式给出，未知字段、重复 YAML key、非有限数字、非法类型、重复格点和越界起点均拒绝。
配置哈希是规范化 JSON 的 SHA256（排序 key、紧凑 UTF-8），不是 YAML 字节哈希；注释和数值等价的浮点写法不影响续采。
顶层组为 `client/scene/grid/camera/graph/capture/quality`，示例配置是完整字段参考。

- `town10_aod_probe.yaml`：3×3×2×4 = 72 节点，408 条理论有向边。
- `town10_aod_smoke.yaml`：3×3×2×1 = 18 节点，66 条理论边；仅做 RGB-D 接口冒烟，不覆盖转向动作。
- 节点按 x、y、z、yaw 索引的嵌套顺序排列，yaw 最快变化。ID 从 `n_0000` 递增。
- 平移只连接 x/y/z 的相邻索引，动作是世界轴 `x+/x-/y+/y-/up/down`；
  转向只连接同位置且 yaw 相差 ±90° 的已有节点。边 ID 从 `e_00000` 递增。
- 成本 = 米制平移距离 × `translation_cost_per_m` + 绝对转角 × `rotation_cost_per_deg`。
- ROI 边界相对 `origin_m`，局部轴与 CARLA 世界轴平行。`z_world_m` 是绝对世界 z，**不是离地高度**。
- 默认 origin 来自真实 `aod_preview_v2/config.json` 的 observation_center：
  `[-64.64484405517578, 24.471010208129883, 1.5999999642372131]`。
  高度从旧数据的未验证地面基准 `0.599999964` 加 40/60 m 得到，仍需实拍筛选。
- 旧预览是 1920×1080、60° FOV、histogram 曝光；新库统一为 456×256、91.6°、手动曝光。
  不能混合两种数据，也不能让不同策略使用不同相机设置。

几何使用 AOD 的静态包围盒查询，膨胀 `clearance_m` 后做相机中心排除和闭线段 slab 相交。
先按格点范围过滤远处包围盒，再缓存相同空间线段的结果；转向不重复做平移查询。
`edge_step_m` 保留为显式配置/元数据，当前精确线段—AABB 检查不依赖采样步长，不会漏掉采样点之间的薄障碍。

节点拒绝区分 `inside_obstacle` 与 `outside_roi`；边穿越障碍为 `segment_collision`；
采集失败为 `capture_failed`。有限格点外的动作由 `action_target` 返回 `out_of_graph`，
不创建指向虚构节点的边，也不把数据集边界记成物理碰撞。

**AABB 不是完整碰撞网格，不能证明真实无人机可飞，可能遗漏不在查询结果中的障碍。**
采集器直接设置相机位姿，不模拟连续飞行。旧 AABB 初筛通过也不能替代本次服务器查询。

## 数据格式 v1

```text
<scene>/
  scene.json
  config.resolved.json
  nodes.jsonl
  edges.jsonl
  region_geometry.json               # diagnostic；完整查询清单
  frames/n_0000/
    rgb.png                         # RGB，无损 PNG
    depth.npy                       # float32 米制射线距离
    mask.png                        # uint8: 0 或 255
    camera.json
    instance.png                    # 可选，原始 RGB 编码，仅评测
    receipt.json                    # 节点事务记录及本目录文件哈希
  quality.json
  summary.json
  checksums.json
  capture.log
  diagnostic/recovery/               # 续采隔离的损坏/部分写入，若存在
```

文本为 UTF-8，路径相对数据集根目录；目录整体移动后仍可检查。拒绝绝对路径、`..`、反斜杠和符号链接。
`checksums.json` 覆盖全部文件（自身除外），包含相机、清单、配置、质量报告和恢复诊断。
它用于完整性检测，不是对恶意篡改的加密签名。

`scene.json` 包含版本、配置哈希、Git 提交、CARLA 客户端/服务器版本、地图、实际天气、种子、
局部坐标定义、固定相机模型、深度语义、起点及 planner 可读字段说明。
环境指纹包括地图全名、客户端/服务器版本、天气、全部查询 AABB 的哈希和计数、固定相机请求属性。
`actual_sensor_attributes` 保存已生成传感器的实际属性。视点库不承诺纹理、水面、光照逐像素复现。

节点保存请求/实际 `[x,y,z,pitch,yaw,roll]`、局部位置/旋转、K、尺寸、FOV、frame、timestamp、状态和路径。
成功节点的 `observation` 只列 RGB、深度、mask 和 camera；实例路径仅在 `diagnostic`。
`camera.json` 保存每个传感器拍摄时位姿、实际 FOV、frame、timestamp、内参、
传感器→世界/世界→传感器矩阵及曝光稳定性诊断。镜头畸变参数固定为零以匹配针孔模型。

深度保留原始米制射线距离，不生成会被误当模型深度的 PNG。
mask 由有限值且 `depth_min_m <= d < depth_max_m` 定义；原始距离不会因 mask 而被替换。
实例图保留 CARLA BGRA 中 B/G/R 字节对应的原始 RGB 编码，无调色和 JPEG；alpha 不是标签通道。
供后续适配使用的 z-depth 转换是：

```text
z = d / sqrt(1 + ((u-cx)/fx)^2 + ((v-cy)/fy)^2)
```

`ray_to_z` 已提供纯离线函数及投影往返测试；当前不额外落盘 z-depth，`depth.npy` 始终保持射线距离。
CARLA 传感器轴为 x 前/y 右/z 上，光学轴为 x 右/y 下/z 前；相机记录明确光学轴映射；K 由实际图像 FOV/尺寸计算，允许 CARLA FOV 的微小浮点舍入。
PyTorch3D 约定转换及 MAGICIAN 后端仍待实现，不能仅凭图像外观判断转换正确。
参考 [UE5 传感器文档](https://carla-ue5.readthedocs.io/en/latest/ref_sensors/) 和
[UE5 Python API](https://carla-ue5.readthedocs.io/en/latest/python_api/)。

## 同步、恢复和完成条件

采集复用 scout 的 `FrameInbox`、`StabilityGate`、`decode_depth`，复用 AOD 的
版本连接、位姿比较、RGB 解码、静态 AABB 查询、JSON 写入和 SHA256；
共享 `runtime.apply_camera_capture_settings`、`OwnedActors` 与传感器弱引用回调。
现有 AOD、scout、path_cost 的接口保持不变。

固定天气预设、零风、手动曝光、零运动模糊和随机种子；不生成目标车或背景交通。
外部车辆、行人、控制器或可动物理 props 会导致拒绝；交通灯在同轮固定红灯并冻结，结束恢复。
逐 tick 检查外部动态 Actor；每节点重新检查天气。必须保证没有其他客户端修改环境。
这些检查无法发现所有引擎动画或人为改动，故仍需人工查看图像与点云。

每个节点：写临时目录→检查数据→生成文件哈希与 receipt→fsync→同文件系统原子改名→原子更新 JSONL。
目录已有且 receipt/哈希/语义通过时绝不覆盖；清单尚未更新的已提交目录可从 receipt 恢复。
部分或损坏目录被移动到 `diagnostic/recovery/` 后重新采集；失败节点不记成功。
配置哈希不同拒绝续采；环境指纹不同也拒绝混合，需新目录。
输出父目录的 `.场景名.capture.lock` 使用 OS 锁防并发，进程退出自动释放；残留的锁文件本身不代表持锁。
不要在持锁采集期间运行 `check`，清单仍在变化时检查应当失败。

`check` 重新构造配置图及几何状态，检查 schema/配置哈希/ID/引用、尺寸、深度 dtype 与范围、mask、
同帧和时间戳、请求/实际位姿、内外参、全部 SHA256、起点、连通性、报告一致性、完成状态和部分文件。
只有全部几何有效节点采集成功、质量阈值通过、起点非孤立、连通性门通过且清理无误，批次才 complete。
几何拒绝节点保留清单，不强求 72 张成功帧；单节点图不能通过非孤立起点门。
`server_visual_review=pending` 不会因机器校验通过而自动变为人工验收通过。

## 信息隔离边界

本库在磁盘上拥有全部节点数据；它不是安全沙箱。未来后端必须只允许已执行到的当前节点 RGB-D 和位姿，
仅公开合法邻接边与动作成本。`diagnostic`、实例标签、未访问 RGB-D、真实增益、目标标签和 Oracle 结果
禁止进入默认 planner observation。本阶段不生成覆盖增益/Oracle，不实现后端读取审计。

服务器完整步骤见 [server-operations.md](server-operations.md)。
