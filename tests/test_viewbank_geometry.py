import importlib

import numpy as np
import pytest

from tests.test_viewbank_config import raw_config


def api():
    return importlib.import_module("src.carla_experiments.viewbank.geometry")


@pytest.mark.parametrize(
    "a,b,hit",
    [
        ((-2, 0, 0), (2, 0, 0), True),
        ((-2, 2, 0), (2, 2, 0), False),
        ((0, 0, 0), (0, 0, 0), True),
        ((2, 0, 0), (2, 0, 0), False),
        ((-2, 1, 0), (2, 1, 0), True),
        ((-2, 0, 0), (-1.1, 0, 0), False),
    ],
)
def test_segment_slab_intersection(a, b, hit):
    assert api().segment_aabb(a, b, ([-1] * 3, [1] * 3), 0) is hit


def test_geometry_reasons_and_disconnected_start():
    from src.carla_experiments.viewbank.grid import build_grid

    cfg = raw_config()
    nodes, edges = build_grid(cfg)
    # Blocks a thin slab between grid endpoints: endpoint-only tests would miss it.
    api().validate_geometry(nodes, edges, cfg, [([9, -30, 0], [11, 30, 50])])
    assert all(n["valid"] for n in nodes)
    assert any(e["invalid_reason"] == "segment_collision" for e in edges)
    assert not api().graph_report(nodes, edges, "n_0032")["connected"]
    nodes, edges = build_grid(cfg)
    cfg["scene"]["roi_max_m"][0] = 19
    api().validate_geometry(nodes, edges, cfg, [([-1, -1, 19], [1, 1, 21])])
    assert {n["invalid_reason"] for n in nodes} >= {"inside_obstacle", "outside_roi"}


def test_carla_matrix_and_ray_depth_projection_roundtrip():
    pose = [10, 20, 30, -45, 90, 0]
    matrix = np.array(api().pose_matrix(pose))
    assert np.allclose(matrix[:3, 0], [0, 2**-0.5, -(2**-0.5)])
    assert np.allclose(matrix @ np.linalg.inv(matrix), np.eye(4))
    k = [[2, 0, 1], [0, 2, 1], [0, 0, 1]]
    depth = np.full((3, 3), 10, np.float32)
    z = api().ray_to_z(depth, k)
    assert z[1, 1] == 10
    assert z[0, 0] == pytest.approx(10 / (1.5**0.5))
    y, x = np.indices(depth.shape)
    points = np.stack(((x - 1) * z / 2, (y - 1) * z / 2, z), axis=-1)
    assert np.allclose(np.linalg.norm(points, axis=-1), depth)
    assert np.allclose(points[..., 0] / points[..., 2] * 2 + 1, x)


def test_geometry_broadphase_omits_far_boxes_and_reuses_spatial_segments(monkeypatch):
    from src.carla_experiments.viewbank.grid import build_grid

    module = api()
    nodes, edges = build_grid(raw_config())
    calls = []
    original = module.segment_aabb

    def counted(*args):
        calls.append(args)
        return original(*args)

    monkeypatch.setattr(module, "segment_aabb", counted)
    boxes = [([9, -30, 0], [11, 30, 50])] + [([1000] * 3, [1001] * 3)] * 100
    module.validate_geometry(nodes, edges, raw_config(), boxes)
    assert any(e["invalid_reason"] == "segment_collision" for e in edges)
    assert len(calls) < 100
