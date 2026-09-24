import importlib

from tests.test_viewbank_config import raw_config


def test_deterministic_count_order_neighbors_and_costs():
    module = importlib.import_module("src.carla_experiments.viewbank.grid")
    cfg = raw_config()
    nodes, edges = module.build_grid(cfg)
    assert (nodes, edges) == module.build_grid(cfg)
    assert len(nodes) == 72
    assert nodes[0]["grid_index"] == [0, 0, 0, 0]
    assert nodes[-1]["node_id"] == "n_0071"
    assert len(edges) == 408
    by_id = {n["node_id"]: n for n in nodes}
    assert {e["action"] for e in edges} == {
        "x+",
        "x-",
        "y+",
        "y-",
        "up",
        "down",
        "yaw+90",
        "yaw-90",
    }
    for e in edges:
        a, b = by_id[e["source"]], by_id[e["target"]]
        changed = [abs(x - y) for x, y in zip(a["grid_index"], b["grid_index"], strict=True)]
        assert sum(v != 0 for v in changed) == 1
        assert e["cost"] == e["translation_m"] + 0.01 * e["rotation_deg"]
    assert (
        module.action_target(nodes, "n_0000", "x-", edges=edges)["invalid_reason"] == "out_of_graph"
    )


def test_action_lookup_preserves_edge_collision_reason():
    from src.carla_experiments.viewbank.grid import action_target, build_grid

    nodes, edges = build_grid(raw_config())
    edges[0].update(valid=False, invalid_reason="segment_collision")
    result = action_target(nodes, edges[0]["source"], edges[0]["action"], edges=edges)
    assert not result["valid"] and result["invalid_reason"] == "segment_collision"
