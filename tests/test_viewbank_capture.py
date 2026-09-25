"""Real capture pipeline exercised through the existing small CARLA fake boundary."""

import copy
import importlib
from types import SimpleNamespace as NS

import pytest

from tests.test_aod_capture import Actor, Blueprint, World, fake_carla
from tests.test_viewbank_config import raw_config


class CaptureActor(Actor):
    def listen(self, callback):
        def add_fov(image):
            image.fov = float(self.attributes["fov"])
            callback(image)

        super().listen(add_fov)


class CaptureWorld(World):
    def __init__(self):
        super().__init__()
        self.weather = NS(cloudiness=10.0, sun_altitude_angle=60.0, wind_intensity=0.0)

    def spawn_actor(self, blueprint, transform):
        actor = CaptureActor(blueprint, transform)
        self.actors.append(actor)
        return actor

    def is_weather_enabled(self):
        return True

    def get_weather(self):
        return copy.deepcopy(self.weather)

    def set_weather(self, value):
        self.weather = copy.deepcopy(value)

    def get_snapshot(self):
        return NS(frame=self.frame)

    def get_blueprint_library(self):
        def find(name):
            bp = Blueprint(name)
            for key in (
                "image_size_x",
                "image_size_y",
                "fov",
                "sensor_tick",
                "lens_k",
                "lens_kcube",
                "lens_circle_multiplier",
                "exposure_mode",
                "iso",
                "shutter_speed",
                "fstop",
                "exposure_compensation",
                "motion_blur_intensity",
            ):
                bp.attributes[key] = "0"
            return bp

        return NS(find=find)


def simulator(world):
    carla = fake_carla(world)
    carla.WeatherParameters = NS(
        ClearNoon=NS(cloudiness=0.0, sun_altitude_angle=75.0, wind_intensity=0.0)
    )
    carla.TrafficLightState = NS(Red="Red")
    return carla


def small_config():
    cfg = raw_config()
    cfg["grid"].update(x_offsets_m=[0, 20], y_offsets_m=[0], z_world_m=[20], yaw_deg=[0])
    cfg["graph"]["start_index"] = [0, 0, 0, 0]
    return cfg


def api():
    return importlib.import_module("src.carla_experiments.viewbank.capture")


def test_fake_capture_same_frame_cleanup_and_verified_resume(tmp_path):
    world = CaptureWorld()
    before = world.get_weather()
    root = tmp_path / "capture"
    result = api().capture(simulator(world), small_config(), root)
    assert result["passed"] and result["captured"] == 2
    assert all(a.destroyed and a.callback is None for a in world.actors)
    assert not world.settings.synchronous_mode and world.weather == before
    rgb = root / "frames/n_0000/rgb.png"
    stamp = rgb.stat().st_mtime_ns
    frames_before = world.frame
    assert api().capture(simulator(world), small_config(), root, resume=True)["passed"]
    assert rgb.stat().st_mtime_ns == stamp
    assert world.frame == frames_before
    from src.carla_experiments.viewbank.validate import check_dataset

    assert check_dataset(root)["passed"]


@pytest.mark.parametrize("failure", ["tick", "interrupt", "destroy", "weather_restore"])
def test_failure_and_cleanup_reports_never_complete(tmp_path, monkeypatch, failure):
    world = CaptureWorld()
    if failure == "tick":
        world.fail_at = 26
    elif failure == "interrupt":
        tick = world.tick

        def interrupted(*args):
            if world.frame >= 26:
                raise KeyboardInterrupt()
            return tick(*args)

        world.tick = interrupted
    elif failure == "destroy":
        monkeypatch.setattr(Actor, "destroy", lambda self: False)
    else:
        set_weather = world.set_weather

        def restore(value):
            if value.cloudiness == 10:
                raise RuntimeError("restore failed")
            set_weather(value)

        world.set_weather = restore
    root = tmp_path / failure
    result = api().capture(simulator(world), small_config(), root)
    assert not result["passed"]
    assert result["status"] == "incomplete"
    from src.carla_experiments.viewbank.artifacts import read_json

    scene = read_json(root / "scene.json")
    assert scene["error"] or scene["cleanup_failures"]
    assert not world.settings.synchronous_mode
    if failure == "interrupt":
        assert result["interrupted"]


def test_resume_after_mid_capture_failure_preserves_first_node(tmp_path):
    world = CaptureWorld()
    world.fail_at = 26
    root = tmp_path / "resume"
    result = api().capture(simulator(world), small_config(), root)
    assert result["captured"] == 1
    first = (root / "frames/n_0000/receipt.json").read_bytes()
    world.fail_at = None
    result = api().capture(simulator(world), small_config(), root, resume=True)
    assert result["passed"] and result["captured"] == 2
    assert (root / "frames/n_0000/receipt.json").read_bytes() == first


@pytest.mark.parametrize("change", ["map", "traffic", "geometry"])
def test_changed_world_refused(tmp_path, change):
    world = CaptureWorld()
    root = tmp_path / "guard"
    assert api().capture(simulator(world), small_config(), root)["passed"]
    if change == "map":
        world.get_map = lambda: NS(name="WrongMap")
    elif change == "traffic":
        world.actors.append(Actor(Blueprint("vehicle.other"), NS()))
    else:
        from tests.test_aod_capture import Box

        world.get_level_bbs = lambda tag: [Box()]
    result = api().capture(simulator(world), small_config(), root, resume=True)
    assert not result["passed"]
    assert "mismatch" in " ".join(result["errors"]) or "occupied" in " ".join(result["errors"])


def test_frame_inbox_rejects_stale_and_mismatched_sensor_data():
    from src.carla_experiments.viewbank.grid import build_grid
    from src.scout_quality import FrameInbox
    from tests.test_viewbank_artifacts import bundle_for

    cfg = small_config()
    node = build_grid(cfg)[0][0]
    inbox = FrameInbox(("rgb", "depth", "instance"))
    images = bundle_for(node, cfg)
    for name in ("instance", "rgb"):
        inbox.put(name, images[name])
    assert not inbox.ready(9, lambda t: True)
    inbox.put("depth", images["depth"])
    assert len(inbox.ready(9, lambda t: True)) == 1
    for name, image in images.items():
        inbox.put(name, image)
    assert not inbox.ready(11, lambda t: True)


def test_plan_and_check_do_not_import_carla(tmp_path, monkeypatch):
    import builtins

    import yaml

    original = builtins.__import__

    def guarded(name, *args, **kwargs):
        if name == "carla":
            raise AssertionError("offline command imported CARLA")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded)
    cli = importlib.import_module("src.viewbank")
    config = tmp_path / "cfg.yaml"
    config.write_text(yaml.safe_dump(small_config()), encoding="utf-8")
    output = tmp_path / "plan"
    assert cli.main(["plan", "--config", str(config), "--output", str(output)]) == 0
    assert cli.main(["plan", "--config", str(config), "--output", str(output)]) == 2
    assert cli.main(["check", "--input", str(output)]) == 5


def test_weather_native_object_need_not_support_copy(tmp_path):
    class NativeWeather:
        cloudiness = 0.0
        sun_altitude_angle = 75.0
        wind_intensity = 0.0

        def __copy__(self):
            raise TypeError("native object cannot be pickled")

    world = CaptureWorld()
    carla = simulator(world)
    carla.WeatherParameters.ClearNoon = NativeWeather()
    assert api().capture(carla, small_config(), tmp_path / "native")["passed"]


def test_output_lock_is_exclusive_and_released(tmp_path):
    root = tmp_path / "locked"
    with api().output_lock(root):
        with pytest.raises(ValueError, match="lock"):
            with api().output_lock(root):
                pass
    with api().output_lock(root):
        pass


def test_optional_instance_camera(tmp_path):
    cfg = small_config()
    cfg["camera"]["instance"] = False
    world = CaptureWorld()
    result = api().capture(simulator(world), cfg, tmp_path / "rgbd")
    assert result["passed"]
    assert not list((tmp_path / "rgbd").rglob("instance.png"))


def test_sensor_spawn_failure_and_external_reporter_do_not_skip_cleanup(tmp_path):
    world = CaptureWorld()
    spawn = world.spawn_actor

    def fail_second(bp, pose):
        if len(world.actors) == 1:
            raise RuntimeError("spawn failure")
        return spawn(bp, pose)

    world.spawn_actor = fail_second

    def broken_report(message):
        raise RuntimeError("broken progress UI")

    result = api().capture(
        simulator(world), small_config(), tmp_path / "spawn", report=broken_report
    )
    assert not result["passed"]
    assert all(a.destroyed for a in world.actors)
    assert not world.settings.synchronous_mode


def test_delayed_frames_survive_one_tick_callback_lag(tmp_path):
    world = CaptureWorld()
    world.frame = 1000000
    original_spawn = world.spawn_actor

    def spawn(bp, pose):
        sensor = original_spawn(bp, pose)
        original_listen = sensor.listen
        pending = []

        def listen(callback):
            def delayed(image):
                if pending:
                    callback(pending.pop())
                pending.append(image)

            original_listen(delayed)

        sensor.listen = listen
        return sensor

    world.spawn_actor = spawn
    result = api().capture(simulator(world), small_config(), tmp_path / "lag")
    assert result["passed"]


def test_actual_pose_crossing_inflated_obstacle_is_rejected(tmp_path):
    world = CaptureWorld()

    class ThinWall:
        def get_world_vertices(self, transform):
            return [NS(x=x, y=y, z=z) for x in (-1.02, -1.01) for y in (-2, 2) for z in (18, 22)]

    world.get_level_bbs = lambda tag: [ThinWall()]
    spawn = world.spawn_actor

    def shifted_spawn(bp, pose):
        sensor = spawn(bp, pose)
        move = sensor.set_transform

        def shifted(request):
            request = copy.deepcopy(request)
            request.location.x -= 0.05
            move(request)

        sensor.set_transform = shifted
        return sensor

    world.spawn_actor = shifted_spawn
    root = tmp_path / "inside"
    result = api().capture(simulator(world), small_config(), root)
    assert not result["passed"]
    from src.carla_experiments.viewbank.artifacts import read_jsonl

    assert read_jsonl(root / "nodes.jsonl")[0]["invalid_reason"] == "capture_failed"


def test_resume_hashes_quarantined_partial_manifest_files(tmp_path):
    world = CaptureWorld()
    world.fail_at = 26
    root = tmp_path / "recovery-files"
    assert not api().capture(simulator(world), small_config(), root)["passed"]
    folder = root / "frames/.n_0001.tmp"
    folder.mkdir()
    (folder / "camera.json.tmp").write_bytes(b"partial manifest")
    world.fail_at = None
    assert api().capture(simulator(world), small_config(), root, resume=True)["passed"]
    from src.carla_experiments.viewbank.validate import check_dataset

    assert check_dataset(root)["passed"]


def test_zero_native_noon_preset_does_not_create_zero_sun_scene(tmp_path):
    world = CaptureWorld()
    carla = simulator(world)
    carla.WeatherParameters.ClearNoon.sun_altitude_angle = 0
    root = tmp_path / "weather-profile"
    assert api().capture(carla, small_config(), root)["passed"]
    from src.carla_experiments.viewbank.artifacts import read_json

    assert read_json(root / "scene.json")["actual_weather"]["sun_altitude_angle"] == 75


def test_unapplied_weather_fails_before_spawning_sensors(tmp_path):
    world = CaptureWorld()
    world.set_weather = lambda value: None
    result = api().capture(simulator(world), small_config(), tmp_path / "weather-failure")
    assert not result["passed"] and not world.actors
    assert "weather" in " ".join(result["errors"])


@pytest.mark.parametrize("persistent", [False, True])
def test_timestamp_mismatch_retries_without_relaxing_tolerance(tmp_path, persistent):
    world = CaptureWorld()
    spawn = world.spawn_actor

    def mismatching_spawn(bp, pose):
        sensor = spawn(bp, pose)
        if "depth" in bp.id:
            listen = sensor.listen

            def wrapped(callback):
                count = 0

                def shifted(image):
                    nonlocal count
                    count += 1
                    if persistent or count == 1:
                        image.timestamp += 0.01
                    callback(image)

                listen(shifted)

            sensor.listen = wrapped
        return sensor

    world.spawn_actor = mismatching_spawn
    cfg = small_config()
    cfg["capture"]["max_frame_ticks"] = 40
    result = api().capture(simulator(world), cfg, tmp_path / "timestamp")
    assert result["passed"] is not persistent
    if persistent:
        assert "timestamp" in (tmp_path / "timestamp/capture.log").read_text()


@pytest.mark.parametrize("enabled", [False, None])
def test_missing_weather_capability_fails_without_world_mutation(tmp_path, enabled):
    from src.carla_experiments.viewbank.artifacts import read_json

    world = CaptureWorld()
    world.is_weather_enabled = (lambda: enabled) if enabled is not None else None

    def forbidden(*args):
        pytest.fail("preflight must not mutate a world without weather support")

    world.set_weather = world.apply_settings = world.tick = world.spawn_actor = forbidden
    root = tmp_path / "disabled-weather"
    result = api().capture(simulator(world), small_config(), root)
    assert result["exit_code"] == 3
    scene = read_json(root / "scene.json")
    assert scene["preflight"]["weather_enabled"] is enabled
    assert scene["carla_server_version"]
    assert "weather" in scene["error"]
    assert not scene["cleanup_failures"]
    from src.carla_experiments.viewbank.validate import check_dataset

    assert scene["error"] in check_dataset(root)["errors"]


def test_async_weather_update_waits_for_readback_before_spawning(tmp_path):
    from src.carla_experiments.viewbank.artifacts import read_json

    world = CaptureWorld()
    tick, spawn = world.tick, world.spawn_actor
    pending = []
    world.set_weather = lambda value: pending.append(copy.deepcopy(value))

    def delayed_tick(*args):
        if pending:
            world.weather = pending.pop(0)
        return tick(*args)

    def verified_spawn(*args):
        assert world.weather.sun_altitude_angle == 75
        return spawn(*args)

    world.tick, world.spawn_actor = delayed_tick, verified_spawn
    root = tmp_path / "async-weather"
    assert api().capture(simulator(world), small_config(), root)["passed"]
    assert read_json(root / "scene.json")["weather_application"]["ticks"] == 1


@pytest.mark.parametrize("enabled", [True, False, None])
def test_doctor_is_read_only_and_reports_weather_support(enabled):
    world = CaptureWorld()
    world.is_weather_enabled = (lambda: enabled) if enabled is not None else None

    def forbidden(*args):
        pytest.fail("doctor must not mutate or tick the world")

    world.set_weather = world.apply_settings = world.tick = world.spawn_actor = forbidden
    carla = simulator(world)
    result = api().doctor(carla, small_config())
    assert result["weather_enabled"] is enabled
    assert result["passed"] is (enabled is True)
    assert result["read_only"] is True
    assert result["actual_weather"]["sun_altitude_angle"] == 60


def test_resume_zero_frame_weather_failure_after_server_repair(tmp_path):
    world = CaptureWorld()
    world.is_weather_enabled = lambda: False
    root = tmp_path / "weather-repair"
    failed = api().capture(simulator(world), small_config(), root)
    assert failed["captured"] == 0 and failed["exit_code"] == 3
    world.is_weather_enabled = lambda: True
    assert api().capture(simulator(world), small_config(), root, resume=True)["passed"]


def test_doctor_cli_outputs_json_without_output_directory(monkeypatch, capsys):
    import json

    from src import viewbank

    world = CaptureWorld()
    world.is_weather_enabled = lambda: False
    monkeypatch.setitem(__import__("sys").modules, "carla", simulator(world))
    assert viewbank.main(["doctor", "--config", "cfg/viewbank/town10_aod_smoke.yaml"]) == 3
    assert json.loads(capsys.readouterr().out)["weather_enabled"] is False
