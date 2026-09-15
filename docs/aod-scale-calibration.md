# AOD v2：目标尺度校准

本轮只改变观察位置，保持当前车辆、目标放置位置、1920×1080、FOV 60° 和 300×300 固定窗口裁剪。尚未加入新的遮挡场景，也未训练模型。

## 服务器执行

已有 v1 资产清单时，可以直接复用，不必重新运行 inventory。先更新主分支：

```bash
cd /mnt/fast18/sunbo/carla
git pull --ff-only origin main
conda activate carla10
```

生成并核对新的 v2 配置，前一步成功才继续：

```bash
python -m src.aod prepare --inventory out/aod_inventory_v1.json --spawn-index 0 --heights 60 100 140 --horizontal-offset 60 --output out/aod_preview_v2.yaml &&
python -m src.aod validate --config out/aod_preview_v2.yaml
```

上述命令适用于此前使用默认三类车辆、出生点 0 的 v1。如果改过蓝图、地面高度或其他相机/场景参数，需要把 v1 的这些设置原样保留到 v2；本轮只改 view_grid。validate 应显示 60/100/140 m、水平轴向偏移 60 m。确认后采集：

```bash
python -m src.aod capture --config out/aod_preview_v2.yaml --output data/aod_preview_v2 &&
python -m src.aod check --input data/aod_preview_v2
```

继续保持同一地图、天气与单目标环境，采集期间不运行其他 tick 主控。每类仍是 24 个候选视点；拒绝点和异常会记录，原数据不覆盖。

## 参数与兼容性

新增配置：

```yaml
view_grid:
  heights_m: [60, 100, 140]
  horizontal_offset_m: 60
```

高度是相对配置 ground_z 的高度。水平位置为 (±60,0)、(0,±60)、(±60,±60)，对角位置的水平距离约 84.9 m；不是等半径环。所有位置仍注视同一个 observation_center。

高度必须是三个严格递增的正数，最大 1000 m；轴向偏移须大于零且不超过 1000 m。这些边界用于配置检查，不代表所设位置已通过飞行/地图安全检查。

新 prepare 默认生成这套 v2 参数。**旧配置缺少 view_grid 时，仍使用 20/30/40 m 和 ±10 m**，不会因软件更新而悄悄改变历史设置；实际采用的参数会写入解析配置。若要复现旧版新生成配置，可指定 --heights 20 30 40 --horizontal-offset 10。

60/100/140 m、±60 m 是待实拍验证的起始值，不是原 AOD 的精确复刻，也不保证每个车型都进入参考像素范围。

## 先看什么

- contact_sheet_00.png 等：缩略原图，检查相机、环境和位置。
- **crop_sheet_00.png 等**：3 个高度 × 8 个方位；每张 crop 原样以 300×300 像素粘贴，不缩放目标。请在看图软件中用 100% 缩放查看。
- **scale_report.json**：全批及每车型/高度的目标尺度统计。
- nodes.jsonl 中的 scale：每个已采集视点的详细统计。
- summary.json 中的 scale_overall：尺度汇总。

拼图标注 L 为投影框长边像素数，crop_cut 表示固定窗口是否截断投影框。整张拼图被看图软件缩小显示时，屏幕上每辆车会更小；这不改变实际输入网络的 crop。

| 字段 | 含义 |
|---|---|
| width_px / height_px / long_side_px | 完整三维投影框在原图坐标下的像素尺寸 |
| box_area_over_crop_area | 投影框面积除以 300×300，可能大于 1；不是可见目标占比 |
| fraction_retained | 投影框落在原图与裁剪窗口交集内的面积比例 |
| crop_truncated | 投影框超过固定裁剪窗口 |
| image_truncated | 投影框超过原图边界 |
| band | 长边低于 40、处于 40～120、超过 120 px，或投影未知 |

40～120 px 只是普通汽车的预览参照，大货车可以更大。所有尺度、遮挡、亮度条件的图像都保留，统计不会筛掉“不合适”的图片。无有效投影时报告 unknown/null，不把它当成零像素小目标。

这些量测的是投影框，不能衡量树木/建筑造成的实际可见比例。fraction_retained=1 也可能是完全遮挡目标。

## 与 v1 比较

服务器保留完整 v1/v2 采集目录时执行：

```bash
python -m src.aod compare --before data/aod_preview_v1 --after data/aod_preview_v2 --output out/aod_scale_v1_vs_v2
```

输出 compare_00.png 等新旧对照拼图，以及 comparison.json。上方 BEFORE、下方 AFTER 都保持 crop 的原始像素尺度，高度在每个单元格标明。类别按蓝图 ID 配对，避免类别编号顺序变化导致错配。comparison.json 会补算旧版没有保存的尺度指标，原始目录不会修改。

程序会先检查两批文件完整性，并要求配置的地图、相机、目标出生位姿、观察中心、地面基准、类别集合及记录的天气一致。引擎渲染、曝光、车辆落地等细微差异仍需检查，这只是尺度对照，不是严格的策略性能实验。

**只复制 images/ 不够做可靠的新旧比较。** 回传本机时请复制整个 data/aod_preview_v2；做 compare 还需要完整 v1，包括 config.json、metadata.json、nodes.jsonl、summary.json、checksums.json 和图片。比较也可直接在服务器执行，再把 out/aod_scale_v1_vs_v2 拿回本机看。

## 验收后做什么

先确认：
1. 新 crop 是否能容纳完整车辆，普通汽车是否接近参考尺度。
2. 不同高度/方向是否保留自然的尺度与外观变化。
3. 是否出现过小、被原图裁断、完全遮挡、几何拒绝等情况；区分原因。

尺度合适后再选树木或建筑附近场景，增加视角相关遮挡，并筛选更接近的车型。最终用固定分类器验证“初始辨识困难、移动后能改善”；本轮图片统计不能证明这一点。
