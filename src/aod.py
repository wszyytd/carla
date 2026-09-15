"""Inventory, prepare, collect and check the first AOD multiview preview."""

from __future__ import annotations

import argparse
import importlib
import json
from collections.abc import Sequence
from pathlib import Path

import yaml

from src.carla_experiments.aod.artifacts import check_artifacts
from src.carla_experiments.aod.config import parse_preview_config, prepare_config
from src.carla_experiments.config import load_config, parse_client_config
from src.carla_experiments.progress import ProgressReporter


def build_parser():
    parser = argparse.ArgumentParser(description="AOD 静态多视角预览采集")
    sub = parser.add_subparsers(dest="command", required=True)
    inventory = sub.add_parser("inventory", help="只读导出服务器资产和出生点")
    inventory.add_argument("--config", type=Path, default=Path("cfg/simulator.yaml"))
    inventory.add_argument("--output", type=Path, required=True)
    prepare = sub.add_parser("prepare", help="离线生成预览配置")
    prepare.add_argument("--inventory", type=Path, required=True)
    prepare.add_argument("--spawn-index", type=int, required=True)
    prepare.add_argument("--blueprints", nargs="+")
    prepare.add_argument("--ground-z", type=float)
    prepare.add_argument("--output", type=Path, required=True)
    prepare.add_argument(
        "--heights",
        type=float,
        nargs=3,
        default=[60, 100, 140],
        help="Three increasing heights above the ground reference, in metres",
    )
    prepare.add_argument(
        "--horizontal-offset",
        type=float,
        default=60,
        help="Axis offset in metres (diagonal nodes have a greater range)",
    )
    compare = sub.add_parser("compare", help="Compare fixed-scale crops of two completed captures")
    compare.add_argument("--before", type=Path, required=True)
    compare.add_argument("--after", type=Path, required=True)
    compare.add_argument("--output", type=Path, required=True)
    for command in ("validate", "capture"):
        item = sub.add_parser(
            command, help="离线校验配置" if command == "validate" else "服务器采集"
        )
        item.add_argument("--config", type=Path, required=True)
        if command == "capture":
            item.add_argument("--output", type=Path, required=True)
    check = sub.add_parser("check", help="离线检查输出完整性和校验和")
    check.add_argument("--input", type=Path, required=True)
    return parser


def _write_new(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(text)


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    report = ProgressReporter()
    try:
        if args.command == "prepare":
            inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
            raw = prepare_config(
                inventory,
                spawn_index=args.spawn_index,
                blueprint_ids=args.blueprints,
                ground_z=args.ground_z,
                heights_m=args.heights,
                horizontal_offset_m=args.horizontal_offset,
            )
            _write_new(args.output, yaml.safe_dump(raw, allow_unicode=True, sort_keys=False))
            report(f"已生成 {args.output}；高度基准：{raw['scene']['ground_reference']}")
        elif args.command == "compare":
            from src.carla_experiments.aod.scale import compare_runs

            compare_runs(args.before, args.after, args.output)
            report(f"Comparison saved: {args.output}")
        elif args.command == "check":
            summary = check_artifacts(args.input)
            report(f"完整性检查通过：{summary['captured']} 张观测；仍需人工检查视角和类别")
            if "scale_overall" in summary:
                report(f"Scale diagnostics: {summary['scale_overall']}")
        elif args.command == "validate":
            config = parse_preview_config(load_config(args.config))
            report(
                f"配置通过：{len(config.blueprint_ids)} 类 × 24 视点；"
                f"总计 {24 * len(config.blueprint_ids)} 个计划视点"
            )
            report(
                f"蓝图：{', '.join(config.blueprint_ids)}；"
                f"地面参考 z={config.ground_z}，请通过预览核对"
            )
            report(f"Heights: {config.heights_m} m; axis offset: {config.horizontal_offset_m} m")
        else:
            config = (
                parse_client_config(load_config(args.config))
                if args.command == "inventory"
                else parse_preview_config(load_config(args.config))
            )
            if args.output.exists():
                raise FileExistsError(f"output already exists: {args.output}")
            carla = importlib.import_module("carla")
            from src.carla_experiments.aod.capture import capture, inventory

            if args.command == "inventory":
                data = inventory(carla, config)
                _write_new(args.output, json.dumps(data, indent=2, ensure_ascii=False) + "\n")
                report(f"资产清单已保存：{args.output}；四轮蓝图 {len(data['vehicles'])} 个")
            else:
                result = capture(carla, config, args.output, report)
                report(
                    f"预览采集完成：{result['captured']} 个已采集，"
                    f"{result['rejected']} 个几何拒绝；输出 {args.output}"
                )
        return 0
    except ModuleNotFoundError as error:
        report(f"缺少依赖：{error}；inventory/capture 需要服务器 CARLA Python 环境")
        return 3
    except TimeoutError as error:
        report(f"CARLA timeout: {error}")
        return 3
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        report(f"配置或文件错误：{error}")
        return 2
    except KeyboardInterrupt:
        report("采集已中断；检查输出目录中的 summary.json 和清理记录")
        return 130
    except Exception as error:
        report(f"运行失败：{type(error).__name__}: {error}")
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
