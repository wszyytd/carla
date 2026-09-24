import importlib
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from tests.test_aod_capture import Transform
from tests.test_viewbank_config import raw_config


def api():
    return importlib.import_module("src.carla_experiments.viewbank.artifacts")


def setup_bank(tmp_path, small=False):
    from src.carla_experiments.viewbank.config import parse_config

    raw = raw_config()
    if small:
        raw["grid"].update(x_offsets_m=[0, 20], y_offsets_m=[0], z_world_m=[20], yaw_deg=[0])
        raw["graph"]["start_index"] = [0, 0, 0, 0]
    cfg = parse_config(raw)
    root = tmp_path / "bank"
    scene, nodes, edges = api().initialize(root, cfg)
    return root, cfg, scene, nodes, edges


def bundle_for(node, cfg, frame=10):
    from src.carla_experiments.aod.capture import transform
    from tests.test_aod_capture import World, fake_carla

    pose = transform(fake_carla(World()), node["requested_transform"])
    h, w = cfg["camera"]["height"], cfg["camera"]["width"]
    raw = np.full((h, w, 4), 70, np.uint8).tobytes()
    return {
        name: NS(
            frame=frame,
            timestamp=frame * 0.05,
            width=w,
            height=h,
            raw_data=raw,
            fov=cfg["camera"]["fov_deg"],
            transform=pose,
        )
        for name in ("rgb", "depth", "instance")
    }


def test_jsonl_relative_paths_and_hash(tmp_path):
    module = api()
    module.write_jsonl(tmp_path / "x.jsonl", [{"字": "值"}, {"id": 2}])
    assert module.read_jsonl(tmp_path / "x.jsonl") == [{"字": "值"}, {"id": 2}]
    assert len(module.digest(tmp_path / "x.jsonl")) == 64
    for path in ("/absolute", "../escape", "C:/escape", "frames\\bad", "a/../b"):
        with pytest.raises(ValueError):
            module.relative_file(tmp_path, path)


def test_atomic_frame_receipt_and_orphan_resume(tmp_path):
    root, cfg, _, nodes, _ = setup_bank(tmp_path)
    node = nodes[0]
    saved = api().write_frame(root, node, bundle_for(node, cfg), cfg, {})
    assert saved["status"] == "captured"
    folder = root / saved["frame_dir"]
    depth = np.load(folder / "depth.npy", allow_pickle=False)
    assert depth.dtype == np.float32 and depth.shape == (6, 8)
    assert "instance" not in saved["observation"]
    assert saved["diagnostic"]["instance"] == "frames/n_0000/instance.png"
    original = {p.name: p.read_bytes() for p in folder.iterdir()}
    # Simulate death before the node JSONL was updated.
    recovered = api().initialize(root, cfg, resume=True)[1][0]
    assert recovered == saved
    assert {p.name: p.read_bytes() for p in folder.iterdir()} == original
    assert api().write_frame(root, saved, bundle_for(node, cfg, 99), cfg, {}) == saved


def test_partial_and_corrupt_capture_recovery_and_config_mismatch(tmp_path):
    root, cfg, _, nodes, _ = setup_bank(tmp_path)
    temp = root / "frames" / ".n_0000.tmp"
    temp.mkdir()
    (temp / "rgb.png").write_bytes(b"partial")
    api().initialize(root, cfg, resume=True)
    assert not temp.exists()
    assert list((root / "diagnostic" / "recovery").iterdir())
    saved = api().write_frame(root, nodes[0], bundle_for(nodes[0], cfg), cfg, {})
    (root / saved["frame_dir"] / "depth.npy").write_bytes(b"corrupt")
    assert api().initialize(root, cfg, resume=True)[1][0]["status"] == "pending"
    cfg["scene"]["seed"] += 1
    with pytest.raises(ValueError, match="config"):
        api().initialize(root, cfg, resume=True)


@pytest.mark.parametrize("kind", ["frame", "timestamp", "pose", "size", "nan", "fov"])
def test_mismatched_sensor_bundle_never_committed(tmp_path, kind):
    root, cfg, _, nodes, _ = setup_bank(tmp_path)
    bundle = bundle_for(nodes[0], cfg)
    if kind == "frame":
        bundle["depth"].frame += 1
    elif kind == "timestamp":
        bundle["depth"].timestamp += 0.1
    elif kind == "pose":
        bundle["depth"].transform = Transform()
    elif kind == "size":
        bundle["depth"].width += 1
    elif kind == "fov":
        bundle["depth"].fov = 60
    else:
        bundle["depth"].timestamp = float("nan")
    with pytest.raises(ValueError):
        api().write_frame(root, nodes[0], bundle, cfg, {})
    assert not (root / "frames/n_0000").exists()


def test_receipt_cannot_reassign_node_id(tmp_path):
    root, cfg, _, nodes, _ = setup_bank(tmp_path)
    api().write_frame(root, nodes[0], bundle_for(nodes[0], cfg), cfg, {})
    path = root / "frames/n_0000/receipt.json"
    receipt = json.loads(path.read_text())
    receipt["node"]["node_id"] = "../escape"
    api().write_json(path, receipt)
    assert api().initialize(root, cfg, resume=True)[1][0]["status"] == "pending"


def test_actual_intrinsics_follow_native_fov_roundoff(tmp_path):
    root, cfg, _, nodes, _ = setup_bank(tmp_path)
    bundle = bundle_for(nodes[0], cfg)
    for image in bundle.values():
        image.fov = 91.5999984741211
    saved = api().write_frame(root, nodes[0], bundle, cfg, {})
    camera = api().read_json(root / saved["frame_dir"] / "camera.json")
    assert camera["fov_deg"] == bundle["rgb"].fov
    from src.carla_experiments.metrics import build_projection_matrix

    k = build_projection_matrix(width=8, height=6, fov_deg=bundle["rgb"].fov)
    assert np.array_equal(camera["intrinsics"], k)


@pytest.mark.parametrize("value", [None, [], "corrupted", 7])
def test_nonobject_receipt_recovers_as_pending(tmp_path, value):
    root, cfg, _, nodes, _ = setup_bank(tmp_path)
    api().write_frame(root, nodes[0], bundle_for(nodes[0], cfg), cfg, {})
    api().write_json(root / "frames/n_0000/receipt.json", value)
    assert api().initialize(root, cfg, resume=True)[1][0]["status"] == "pending"
