from __future__ import annotations

import argparse
import importlib
from collections.abc import Sequence
from pathlib import Path

import yaml

from src.carla_experiments.client import inspect_world, versions_compatible
from src.carla_experiments.config import load_config, parse_client_config
from src.carla_experiments.progress import ProgressReporter


def build_parser() -> argparse.ArgumentParser:
    """Build the read-only CARLA smoke-test command parser."""

    parser = argparse.ArgumentParser(description="只读检查 CARLA 服务器连接和世界摘要")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("cfg/simulator.yaml"),
        help="模拟器 YAML 配置路径（默认：cfg/simulator.yaml）",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the CARLA connection smoke test and return a documented exit code."""

    args = build_parser().parse_args(argv)
    report = ProgressReporter()

    try:
        client_config = parse_client_config(load_config(args.config))
    except (OSError, ValueError, yaml.YAMLError) as error:
        report(f"错误：配置无效：{error}")
        return 2

    report(
        f"连接 CARLA：{client_config.host}:{client_config.port}，"
        f"超时 {client_config.timeout_seconds:.1f}s"
    )

    try:
        carla_module = importlib.import_module("carla")
    except ModuleNotFoundError:
        report("错误：无法导入 carla；请安装 CARLA 0.10.0 发行包自带的 cp310 wheel")
        return 3

    try:
        summary = inspect_world(
            carla_module,
            client_config.host,
            client_config.port,
            client_config.timeout_seconds,
        )
        compatible = versions_compatible(summary.client_version, summary.server_version)
    except Exception as error:
        report(f"错误：CARLA 读取失败：{type(error).__name__}: {error}")
        return 3

    if not compatible:
        report(
            "错误：CARLA 主次版本不匹配："
            f"客户端 {summary.client_version}，服务器 {summary.server_version}"
        )
        return 4

    report("连接成功")
    report(f"客户端版本：{summary.client_version}")
    report(f"服务器版本：{summary.server_version}")
    report(f"地图：{summary.map_name}")
    report(f"同步模式：{str(summary.synchronous_mode).lower()}")
    report(f"车辆：{summary.vehicle_count}")
    report(f"行人：{summary.walker_count}")
    report(f"Actor 总数：{summary.actor_count}")
    report(f"请求耗时：{summary.elapsed_seconds:.3f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
