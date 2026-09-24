"""Pure CARLA-coordinate geometry. AABBs are not a flight safety certificate."""

import math

import numpy as np

from ..aod.preview import blocked_by

GEOMETRY_SCOPE = (
    "inflated queried static AABBs only, not a full collision mesh; "
    "does not prove drone flyability; unqueried obstacles may be missing"
)


def segment_aabb(start, end, box, clearance=0):
    """Closed-segment slab intersection, including tangent and zero-length segments."""
    lower, upper = box
    if (
        len(start) != 3
        or len(end) != 3
        or len(lower) != 3
        or len(upper) != 3
        or not all(math.isfinite(v) for v in (*start, *end, *lower, *upper, clearance))
        or clearance < 0
        or any(a > b for a, b in zip(lower, upper, strict=True))
    ):
        raise ValueError("invalid segment/AABB")
    enter, leave = 0.0, 1.0
    for i in range(3):
        lo, hi = lower[i] - clearance, upper[i] + clearance
        delta = end[i] - start[i]
        if delta == 0:
            if not lo <= start[i] <= hi:
                return False
        else:
            a, b = sorted(((lo - start[i]) / delta, (hi - start[i]) / delta))
            enter, leave = max(enter, a), min(leave, b)
            if enter > leave:
                return False
    return True


def pose_matrix(pose):
    """CARLA local sensor axes (+x forward, +y right, +z up) to world; degrees."""
    x, y, z, pitch, yaw, roll = pose
    p, a, r = map(math.radians, (pitch, yaw, roll))
    cp, sp, cy, sy, cr, sr = (
        math.cos(p),
        math.sin(p),
        math.cos(a),
        math.sin(a),
        math.cos(r),
        math.sin(r),
    )
    return [
        [cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr, x],
        [cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr, y],
        [sp, -cp * sr, cp * cr, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def local_pose(pose, origin):
    return {
        "position_m": [pose[i] - origin[i] for i in range(3)],
        "rotation_deg_pitch_yaw_roll": list(pose[3:]),
        "rotation_matrix": [r[:3] for r in pose_matrix(pose)[:3]],
    }


def ray_to_z(depth, intrinsics):
    y, x = np.indices(depth.shape)
    k = np.asarray(intrinsics)
    return (
        depth / np.sqrt(1 + ((x - k[0, 2]) / k[0, 0]) ** 2 + ((y - k[1, 2]) / k[1, 1]) ** 2)
    ).astype(np.float32)


def point_reason(point, cfg, boxes):
    lo, hi = cfg["scene"]["roi_min_m"], cfg["scene"]["roi_max_m"]
    local = [point[i] - cfg["grid"]["origin_m"][i] for i in range(3)]
    if not all(lo[i] <= local[i] <= hi[i] for i in range(3)):
        return "outside_roi"
    if blocked_by(point, boxes, clearance_m=cfg["graph"]["clearance_m"]):
        return "inside_obstacle"
    return None


def validate_geometry(nodes, edges, cfg, boxes):
    margin = cfg["graph"]["clearance_m"]
    # Broad phase: no segment between grid points can leave their enclosing box.
    # Keep the complete native inventory on disk, but test only overlapping boxes.
    if boxes:
        values = np.asarray(boxes, dtype=float)
        if (
            values.ndim != 3
            or values.shape[1:] != (2, 3)
            or not np.isfinite(values).all()
            or (values[:, 0] > values[:, 1]).any()
        ):
            raise ValueError("invalid static AABB inventory")
        positions = np.asarray([node["requested_transform"][:3] for node in nodes])
        candidates = (values[:, 1] + margin >= positions.min(axis=0)).all(axis=1) & (
            values[:, 0] - margin <= positions.max(axis=0)
        ).all(axis=1)
        boxes = values[candidates].tolist()
    for node in nodes:
        reason = point_reason(node["requested_transform"][:3], cfg, boxes)
        node.update(
            valid=reason is None,
            invalid_reason=reason,
            status="pending" if reason is None else "rejected",
        )
    lookup = {n["node_id"]: n for n in nodes}
    intersections = {}
    for edge in edges:
        a, b = lookup[edge["source"]], lookup[edge["target"]]
        reason = a["invalid_reason"] or b["invalid_reason"]
        if reason is None:
            segment = tuple(
                sorted((tuple(a["requested_transform"][:3]), tuple(b["requested_transform"][:3])))
            )
            if segment not in intersections:
                intersections[segment] = any(segment_aabb(*segment, box, margin) for box in boxes)
            if intersections[segment]:
                reason = "segment_collision"
        edge.update(valid=reason is None, invalid_reason=reason)
    return nodes, edges


def graph_report(nodes, edges, start):
    adjacency = {n["node_id"]: set() for n in nodes if n["valid"]}
    for e in edges:
        if e["valid"] and e["source"] in adjacency and e["target"] in adjacency:
            adjacency[e["source"]].add(e["target"])
    seen, todo = set(), [start] if start in adjacency else []
    while todo:
        current = todo.pop()
        if current not in seen:
            seen.add(current)
            todo.extend(adjacency[current] - seen)
    return {
        "valid_nodes": len(adjacency),
        "reachable_from_start": len(seen),
        "connected": bool(adjacency) and len(seen) == len(adjacency),
        "start_valid": start in adjacency,
        "start_nonisolated": bool(adjacency.get(start)),
        "isolated_nodes": sorted(n for n, neighbors in adjacency.items() if not neighbors),
    }
