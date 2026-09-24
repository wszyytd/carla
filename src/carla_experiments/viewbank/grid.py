"""Deterministic directed world-axis lattice; no implicit boundary collisions."""

import itertools
import math

from ..metrics import build_projection_matrix
from .geometry import local_pose

ACTIONS = (
    (0, 1, "x+"),
    (0, -1, "x-"),
    (1, 1, "y+"),
    (1, -1, "y-"),
    (2, 1, "up"),
    (2, -1, "down"),
    (3, 90, "yaw+90"),
    (3, -90, "yaw-90"),
)


def build_grid(cfg):
    grid, camera = cfg["grid"], cfg["camera"]
    axes = [grid[k] for k in ("x_offsets_m", "y_offsets_m", "z_world_m", "yaw_deg")]
    intrinsics = build_projection_matrix(
        width=camera["width"], height=camera["height"], fov_deg=camera["fov_deg"]
    ).tolist()
    nodes = []
    for index in itertools.product(*(range(len(a)) for a in axes)):
        x, y, z, yaw = (axes[i][j] for i, j in enumerate(index))
        pose = [grid["origin_m"][0] + x, grid["origin_m"][1] + y, z, grid["pitch_deg"], yaw, 0.0]
        node_id = f"n_{len(nodes):04d}"
        nodes.append(
            {
                "node_id": node_id,
                "grid_index": list(index),
                "requested_transform": pose,
                "actual_transform": None,
                "local_pose": local_pose(pose, grid["origin_m"]),
                "valid": True,
                "invalid_reason": None,
                "status": "pending",
                "frame_dir": f"frames/{node_id}",
                "frame": None,
                "timestamp": None,
                "intrinsics": intrinsics,
                "image_size": [camera["width"], camera["height"]],
                "fov_deg": camera["fov_deg"],
            }
        )
    lookup = {tuple(n["grid_index"]): n for n in nodes}
    edges = []
    for node in nodes:
        for axis, delta, action in ACTIONS:
            target = node["grid_index"][:]
            if axis == 3:
                yaw = (axes[3][target[3]] + delta) % 360
                if yaw not in axes[3]:
                    continue
                target[3] = axes[3].index(yaw)
            else:
                target[axis] += delta
            other = lookup.get(tuple(target))
            if other is None:
                continue
            distance = math.dist(node["requested_transform"][:3], other["requested_transform"][:3])
            angle = abs(delta) if axis == 3 else 0
            edges.append(
                {
                    "edge_id": f"e_{len(edges):05d}",
                    "source": node["node_id"],
                    "target": other["node_id"],
                    "action": action,
                    "translation_m": distance,
                    "rotation_deg": angle,
                    "cost": distance * cfg["graph"]["translation_cost_per_m"]
                    + angle * cfg["graph"]["rotation_cost_per_deg"],
                    "valid": True,
                    "invalid_reason": None,
                }
            )
    return nodes, edges


def action_target(nodes, source, action, *, edges):
    """Lookup public action outcome, preserving geometric versus dataset-boundary reasons."""
    if any(n["node_id"] == source for n in nodes):
        for edge in edges:
            if edge["source"] == source and edge["action"] == action:
                return {key: edge[key] for key in ("target", "valid", "invalid_reason")}
    return {"valid": False, "invalid_reason": "out_of_graph", "target": None}


def start_node_id(cfg, nodes):
    return next(n["node_id"] for n in nodes if n["grid_index"] == cfg["graph"]["start_index"])
