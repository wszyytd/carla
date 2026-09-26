"""Versioned CARLA RGB-D viewbank producer: offline plan/check and server capture."""

import argparse
import importlib
import json
from pathlib import Path

import yaml

from .carla_experiments.viewbank.artifacts import write_json, write_jsonl
from .carla_experiments.viewbank.config import config_hash, load_config
from .carla_experiments.viewbank.geometry import GEOMETRY_SCOPE, graph_report
from .carla_experiments.viewbank.grid import build_grid, start_node_id
from .carla_experiments.viewbank.validate import check_dataset


def plan(cfg, output):
    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    nodes, edges = build_grid(cfg)
    summary = {
        "schema_version": 1,
        "scene_id": cfg["scene_id"],
        "status": "plan_only",
        "config_sha256": config_hash(cfg),
        "requested_nodes": len(nodes),
        "theoretical_edges": len(edges),
        "geometry_checked": False,
        "geometry_scope": GEOMETRY_SCOPE,
        "graph": graph_report(nodes, edges, start_node_id(cfg, nodes)),
    }
    write_json(root / "config.resolved.json", cfg)
    write_jsonl(root / "nodes.jsonl", nodes)
    write_jsonl(root / "edges.jsonl", edges)
    write_json(root / "summary.json", summary)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("plan", "capture", "calibrate"):
        sub = commands.add_parser(command)
        sub.add_argument("--config", type=Path, required=True)
        sub.add_argument("--output", type=Path, required=True)
        if command == "capture":
            sub.add_argument(
                "--resume", action="store_true", help="verify and resume identical configuration"
            )
    sub = commands.add_parser("doctor", help="read-only server/weather capability diagnosis")
    sub.add_argument("--config", type=Path, required=True)
    sub = commands.add_parser("check")
    sub.add_argument("--input", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "check":
            result = check_dataset(args.input)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0 if result["passed"] else 5
        cfg = load_config(args.config)
        if args.command == "plan":
            result = plan(cfg, args.output)
        elif args.command == "calibrate":
            from .carla_experiments.viewbank.calibrate import calibrate

            result = calibrate(importlib.import_module("carla"), cfg, args.output)
        elif args.command == "doctor":
            from .carla_experiments.viewbank.capture import doctor

            result = doctor(importlib.import_module("carla"), cfg)
        else:
            from .carla_experiments.viewbank.capture import capture

            result = capture(importlib.import_module("carla"), cfg, args.output, resume=args.resume)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if result.get("interrupted"):
            return 130
        return result.get("exit_code", 0 if result.get("passed", True) else 5)
    except (ValueError, OSError, yaml.YAMLError) as error:
        print(f"configuration/file error: {error}")
        return 2
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f"CARLA/runtime error: {error}")
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
