import json
import subprocess
import sys

import pytest

from src import scout_batch


def record():
    return {
        "map": "Carla/Maps/Town10HD_Opt",
        "pose": [0, 0, 50, -45, 90, 0],
        "width": 1280,
        "height": 720,
        "fov": 90,
    }


def test_comparison_resets_origin_and_compares_equal_distance_routes():
    assert hasattr(scout_batch, "build_comparison"), "comparison planner is missing"
    views = scout_batch.build_comparison(record(), 10)
    routes = {}
    for view in views:
        routes.setdefault(view["route"], []).append(view)
    for steps in routes.values():
        assert steps[0]["pose"] == [0, 0, 50, -45, 90, 0]
        assert [v["path_length_m"] for v in steps] == [0, 10, 20]
        assert [v["step"] for v in steps] == [0, 1, 2]
    assert routes["left_up"][1]["pose"][:3] == pytest.approx([10, 0, 50])
    assert routes["up_left"][1]["pose"][:3] == pytest.approx([0, 0, 60])
    assert routes["left_up"][-1]["pose"] == pytest.approx(routes["up_left"][-1]["pose"])
    assert routes["up"][-1]["pose"][:3] == [0, 0, 70]
    assert record()["pose"] == [0, 0, 50, -45, 90, 0]


@pytest.mark.parametrize("step", [0, -1, float("nan"), float("inf")])
def test_comparison_rejects_bad_step(step):
    assert hasattr(scout_batch, "build_comparison"), "comparison planner is missing"
    with pytest.raises(ValueError):
        scout_batch.build_comparison(record(), step)


@pytest.mark.parametrize(
    "field,value",
    [("pose", [1, 2]), ("pose", [0, 0, 50, -100, 0, 0]), ("width", 0), ("map", ""), ("fov", 180)],
)
def test_comparison_rejects_invalid_source(field, value):
    assert hasattr(scout_batch, "build_comparison"), "comparison planner is missing"
    source = record()
    source[field] = value
    with pytest.raises(ValueError):
        scout_batch.build_comparison(source, 10)


def test_comparison_dry_run_needs_no_carla_and_preserves_source_camera(tmp_path):
    source = tmp_path / "source.json"
    source.write_text(json.dumps(record()), encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "src.scout",
            "--compare-from",
            str(source),
            "--dry-run",
            "--output",
            str(tmp_path / "result"),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    plans = list((tmp_path / "result").glob("*/plan.json"))
    assert len(plans) == 1
    plan = json.loads(plans[0].read_text(encoding="utf-8"))
    assert plan["camera"] == {"width": 1280, "height": 720, "fov": 90}
    assert plan["camera_only_no_collision_check"] is True
    assert plan["views"][0]["pose"] == record()["pose"]
    assert not list(plans[0].parent.glob("*.png"))


def test_report_keeps_failed_steps_and_does_not_invent_metrics(tmp_path):
    assert hasattr(scout_batch, "write_comparison_report"), "comparison report is missing"
    views = [
        {"route": "left_up", "step": 0, "action": "start", "pose": [0] * 6, "path_length_m": 0},
        {"route": "left_up", "step": 1, "action": "a", "pose": [1] * 6, "path_length_m": 10},
    ]
    results = [
        {"view": 1, "image": "0001.png", "status": "ok"},
        {"view": 2, "status": "failed", "error": "timeout <test>"},
    ]
    scout_batch.write_comparison_report(tmp_path, views, results)
    import csv

    with (tmp_path / "comparison.csv").open(encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2
    assert rows[0]["image"] == "0001.png"
    assert rows[1]["image"] == ""
    assert rows[1]["status"] == "failed"
    assert rows[0]["new_target_count"] == ""
    html = (tmp_path / "comparison.html").read_text(encoding="utf-8")
    assert 'src="0001.png"' in html
    assert 'src="0002.png"' not in html
    assert "timeout &lt;test&gt;" in html


@pytest.mark.parametrize("mode", ["ok", "timeout", "interrupt"])
def test_batch_capture_outputs_real_file_mapping_and_restores_camera(tmp_path, monkeypatch, mode):
    from types import SimpleNamespace as NS

    from PIL import Image

    from src import scout

    def transform(location=None, rotation=None):
        return NS(
            location=location or NS(x=1, y=2, z=3), rotation=rotation or NS(pitch=0, yaw=0, roll=0)
        )

    class Actor:
        def __init__(self):
            self.pose = transform()
            self.callback = None
            self.stopped = False
            self.destroyed = False

        def get_transform(self):
            return self.pose

        def set_transform(self, pose):
            self.pose = pose

        def listen(self, callback):
            self.callback = callback

        def stop(self):
            self.stopped = True

        def destroy(self):
            self.destroyed = True

    spectator, sensor = Actor(), Actor()
    initial = spectator.pose
    frame = 0

    def advance(_):
        nonlocal frame
        if mode == "interrupt":
            raise KeyboardInterrupt
        frame += 10
        sensor.callback(
            NS(
                frame=frame,
                timestamp=frame * 0.05,
                transform=initial if mode == "timeout" else sensor.pose,
                width=1280,
                height=720,
                save_to_disk=lambda path: Image.new("RGB", (16, 9)).save(path),
            )
        )

    world = NS(
        get_settings=lambda: NS(no_rendering_mode=False, synchronous_mode=False),
        get_map=lambda: NS(name=record()["map"], get_spawn_points=lambda: []),
        get_spectator=lambda: spectator,
        get_blueprint_library=lambda: NS(find=lambda _: NS(set_attribute=lambda k, v: None)),
        get_snapshot=lambda: NS(frame=frame),
        spawn_actor=lambda bp, pose: sensor,
    )
    client = NS(set_timeout=lambda _: None, get_world=lambda: world)
    monkeypatch.setitem(
        sys.modules,
        "carla",
        NS(Client=lambda host, port: client, Transform=transform, Location=NS, Rotation=NS),
    )
    monkeypatch.setattr(scout.time, "sleep", advance)
    source = tmp_path / "source.json"
    source.write_text(json.dumps(record()), encoding="utf-8")
    monkeypatch.setattr(
        sys, "argv", ["scout", "--compare-from", str(source), "--output", str(tmp_path / "output")]
    )
    if mode == "timeout":
        monkeypatch.setattr(scout.time, "monotonic", iter(range(10000)).__next__)
        with pytest.raises(RuntimeError, match="Three captures failed"):
            scout.main()
    elif mode == "interrupt":
        with pytest.raises(SystemExit) as stopped:
            scout.main()
        assert stopped.value.code == 130
    else:
        scout.main()
    (summary_path,) = (tmp_path / "output").glob("*/summary.json")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    assert spectator.pose is initial
    assert sensor.stopped and sensor.destroyed
    assert (summary_path.parent / "comparison.html").exists()
    if mode != "ok":
        assert summary["complete"] is False
        assert summary["completed"] == 0
        assert summary["not_run"] == (27 if mode == "timeout" else 30)
        assert len(summary["failures"]) == (3 if mode == "timeout" else 0)
        return
    assert summary["complete"] is True
    assert summary["completed"] == summary["planned"] == 30
    for result in summary["results"]:
        assert (summary_path.parent / result["image"]).exists()
    metadata = json.loads((summary_path.parent / "0006.json").read_text(encoding="utf-8"))
    assert metadata["pose"][:3] == [0, 0, 30]
    assert metadata["comparison"]["route"] == "down"
    assert metadata["comparison"]["path_length_m"] == 20
    assert spectator.pose is initial
    assert sensor.stopped and sensor.destroyed
    assert (summary_path.parent / "comparison.html").exists()
