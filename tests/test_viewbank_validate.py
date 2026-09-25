import importlib
import json

import numpy as np
import pytest

from tests.test_viewbank_artifacts import api, bundle_for, setup_bank


def complete_bank(tmp_path):
    root, cfg, scene, nodes, edges = setup_bank(tmp_path, small=True)
    # Empty queried geometry is an explicit fixture, not an implicit missing file.
    api().write_json(root / "region_geometry.json", {"aabbs": [], "counts": {"Buildings": 0}})
    from src.carla_experiments.viewbank.config import config_hash

    scene["environment"] = {"versions": ["0.10.0", "0.10.0"], "geometry_sha256": config_hash([])}
    for i, node in enumerate(nodes):
        nodes[i] = api().write_frame(root, node, bundle_for(node, cfg, i + 10), cfg, {})
    api().finalize(root, cfg, scene, nodes, edges, [], None)
    return root, cfg, scene, nodes, edges


def check(root):
    return importlib.import_module("src.carla_experiments.viewbank.validate").check_dataset(root)


def test_complete_bank_passes_and_hashes_cover_manifests(tmp_path):
    root, _, _, _, _ = complete_bank(tmp_path)
    report = check(root)
    assert report["passed"] and report["captured"] == 2
    assert report["graph"]["connected"]
    checksums = json.loads((root / "checksums.json").read_text())
    assert {"scene.json", "quality.json", "nodes.jsonl", "edges.jsonl"} <= set(checksums)


@pytest.mark.parametrize(
    "damage",
    [
        "missing",
        "corrupt",
        "float64",
        "dimensions",
        "nonfinite",
        "partial",
        "duplicate",
        "reference",
        "incomplete",
        "config",
    ],
)
def test_checker_rejects_damage(tmp_path, damage):
    root, _, _, _, _ = complete_bank(tmp_path)
    depth_path = root / "frames/n_0000/depth.npy"
    if damage == "missing":
        depth_path.unlink()
    elif damage == "corrupt":
        depth_path.write_bytes(b"bad")
    elif damage in ("float64", "dimensions", "nonfinite"):
        depth = np.load(depth_path)
        depth = depth.astype(np.float64) if damage == "float64" else depth
        depth = depth[:2] if damage == "dimensions" else depth
        if damage == "nonfinite":
            depth[:] = np.nan
        np.save(depth_path, depth)
    elif damage == "partial":
        (root / "frames/.n_0001.tmp").mkdir()
    elif damage == "duplicate":
        path = root / "nodes.jsonl"
        path.write_text(path.read_text() + path.read_text().splitlines()[0] + "\n")
    elif damage == "reference":
        edges = api().read_jsonl(root / "edges.jsonl")
        edges[0]["target"] = "unknown"
        api().write_jsonl(root / "edges.jsonl", edges)
    elif damage == "incomplete":
        path = root / "scene.json"
        value = json.loads(path.read_text())
        value["status"] = "incomplete"
        api().write_json(path, value)
    else:
        path = root / "config.resolved.json"
        value = json.loads(path.read_text())
        value["scene"]["seed"] += 1
        api().write_json(path, value)
    assert not check(root)["passed"]


def test_checker_catches_semantic_depth_error_even_after_rehash(tmp_path):
    root, _, _, _, _ = complete_bank(tmp_path)
    folder = root / "frames/n_0000"
    np.save(folder / "depth.npy", np.ones((6, 8), np.float64))
    receipt = json.loads((folder / "receipt.json").read_text())
    receipt["sha256"]["depth.npy"] = api().digest(folder / "depth.npy")
    api().write_json(folder / "receipt.json", receipt)
    api().write_checksums(root)
    assert any("float32" in e for e in check(root)["errors"])


@pytest.mark.parametrize(
    "field", ["schema", "intrinsics", "pose", "frame", "timestamp", "path", "geometry_hash"]
)
def test_semantic_tampering_rejected_after_file_rehash(tmp_path, field):
    root, _, _, _, _ = complete_bank(tmp_path)
    path = root / "frames/n_0000/camera.json"
    value = api().read_json(path)
    if field == "schema":
        value["schema_version"] = 999
    elif field == "intrinsics":
        value["intrinsics"][0][0] += 1
    elif field == "pose":
        value["actual_transform"][0] += 1
    elif field == "frame":
        value["sensors"]["depth"]["frame"] += 1
    elif field == "timestamp":
        value["sensors"]["depth"]["timestamp"] += 0.1
    elif field == "path":
        nodes = api().read_jsonl(root / "nodes.jsonl")
        nodes[0]["observation"]["rgb"] = "../secret.png"
        api().write_jsonl(root / "nodes.jsonl", nodes)
    else:
        scene = api().read_json(root / "scene.json")
        scene["environment"]["geometry_sha256"] = "0" * 64
        api().write_json(root / "scene.json", scene)
    api().write_json(path, value)
    receipt = api().read_json(path.parent / "receipt.json")
    receipt["sha256"]["camera.json"] = api().digest(path)
    api().write_json(path.parent / "receipt.json", receipt)
    api().write_checksums(root)
    assert not check(root)["passed"]


@pytest.mark.parametrize(
    "path,value",
    [
        ("scene.json", None),
        ("checksums.json", []),
        ("frames/n_0000/camera.json", None),
        ("frames/n_0000/receipt.json", {"sha256": []}),
    ],
)
def test_malformed_json_reports_failure_without_crashing(tmp_path, path, value):
    root, _, _, _, _ = complete_bank(tmp_path)
    api().write_json(root / path, value)
    assert not check(root)["passed"]


@pytest.mark.parametrize("delta,accepted", [(1.1e-16, True), (1e-3, False)])
def test_cross_platform_local_rotation_roundoff(tmp_path, delta, accepted):
    root, cfg, _, nodes, _ = complete_bank(tmp_path)
    folder = root / "frames/n_0000"
    camera = api().read_json(folder / "camera.json")
    camera["local_pose"]["rotation_matrix"][0][2] += delta
    nodes[0]["local_pose"] = camera["local_pose"]
    api().write_json(folder / "camera.json", camera)
    module = importlib.import_module("src.carla_experiments.viewbank.validate")
    if accepted:
        module.inspect_camera_files(folder, nodes[0], cfg)
    else:
        with pytest.raises(ValueError, match="local pose"):
            module.inspect_camera_files(folder, nodes[0], cfg)


@pytest.mark.parametrize("value", [0, 255])
def test_offline_check_rejects_black_and_white_rgb(tmp_path, value):
    from PIL import Image

    root, cfg, _, nodes, _ = complete_bank(tmp_path)
    folder = root / "frames/n_0000"
    Image.fromarray(np.full((6, 8, 3), value, np.uint8)).save(folder / "rgb.png")
    module = importlib.import_module("src.carla_experiments.viewbank.validate")
    with pytest.raises(ValueError, match="RGB"):
        module.inspect_camera_files(folder, nodes[0], cfg)


@pytest.mark.parametrize("delta,accepted", [(1e-16, True), (1e-3, False)])
def test_quality_report_recomputation_allows_only_roundoff(tmp_path, delta, accepted):
    root, _, _, _, _ = complete_bank(tmp_path)
    quality = api().read_json(root / "quality.json")
    quality["node_quality"]["n_0000"]["position_error_m"] += delta
    api().write_json(root / "quality.json", quality)
    api().write_checksums(root)
    assert check(root)["passed"] is accepted
