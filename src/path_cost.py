from __future__ import annotations

import argparse
import importlib
from collections.abc import Sequence
from pathlib import Path

import yaml

from src.carla_experiments.config import load_config, parse_path_cost_config
from src.carla_experiments.path_cost_runner import EpisodeSummary, run_path_cost_episode
from src.carla_experiments.progress import ProgressReporter


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="运行 Town10 连续、非路口的 120 m 弯道（总转角至少 60°）路径代价 Pilot"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("cfg/experiments/path_cost_pilot.yaml"),
        help="Pilot YAML 配置路径（默认：cfg/experiments/path_cost_pilot.yaml）",
    )
    parser.add_argument(
        "--policy",
        choices=("hover", "vertical_follow"),
        required=True,
        help="空中相机基线策略",
    )
    return parser


def _pass_fail(value: bool) -> str:
    return "通过" if value else "未通过"


def _print_summary(summary: EpisodeSummary, report: ProgressReporter) -> None:
    report("实验完成")
    report(f"实验 ID：{summary.experiment_id}")
    report(f"策略：{summary.policy}")
    report(
        f"路线：road={summary.road_id}, section={summary.section_id}, "
        f"lane={summary.lane_id}, start_s={summary.start_s:.3f}"
    )
    report(f"目标路线执行：{_pass_fail(summary.completed_route)}")
    report(f"目标执行质量：{_pass_fail(summary.target_execution_valid)}")
    report(f"有效观测：{summary.observation_valid_fraction:.3%}")
    report(f"最长连续无效：{summary.longest_invalid_seconds:.3f} s")
    report(f"无人机路径长度：{summary.uav_path_length_m:.3f} m")
    report(f"目标路径长度：{summary.target_path_length_m:.3f} m")
    report(f"路径长度比：{summary.path_length_ratio:.6f}")
    report(f"世界帧/传感器帧：{summary.measured_world_frames}/{summary.measured_sensor_frames}")
    report(f"实验判定：{_pass_fail(summary.episode_success)}")


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    report = ProgressReporter()

    try:
        config = parse_path_cost_config(load_config(args.config))
    except (OSError, ValueError, yaml.YAMLError) as error:
        report(f"错误：配置无效：{error}")
        return 2

    try:
        carla_module = importlib.import_module("carla")
    except ModuleNotFoundError:
        report("错误：无法导入 carla；请安装 CARLA 0.10.0 发行包自带的 cp310 wheel")
        return 3

    try:
        summary = run_path_cost_episode(carla_module, config, args.policy)
    except Exception as error:
        report(f"错误：实验运行失败：{type(error).__name__}: {error}")
        return 3

    _print_summary(summary, report)
    return 0 if summary.episode_success else 5


if __name__ == "__main__":
    raise SystemExit(main())
