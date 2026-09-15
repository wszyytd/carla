"""Exercise the real collector with an in-memory CARLA boundary."""

from __future__ import annotations

import copy
import fnmatch
import importlib
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from src.carla_experiments.aod.config import parse_preview_config, prepare_config


class Transform:
    def __init__(self, location=None, rotation=None):
        self.location = location or NS(x=0, y=0, z=0)
        self.rotation = rotation or NS(pitch=0, yaw=0, roll=0)

    def get_inverse_matrix(self):
        p, y = map(math.radians, (self.rotation.pitch, self.rotation.yaw))
        rotation = np.array(
            [
                [math.cos(p) * math.cos(y), -math.sin(y), -math.sin(p) * math.cos(y)],
                [math.cos(p) * math.sin(y), math.cos(y), -math.sin(p) * math.sin(y)],
                [math.sin(p), 0, math.cos(p)],
            ]
        )
        matrix = np.eye(4)
        matrix[:3, :3] = rotation.T
        matrix[:3, 3] = -rotation.T @ [self.location.x, self.location.y, self.location.z]
        return matrix.tolist()


class Blueprint:
    def __init__(self, name):
        self.id = name
        self.attributes = {"number_of_wheels": "4"}

    def has_attribute(self, name):
        return name in self.attributes

    def get_attribute(self, name):
        return self.attributes[name]

    def set_attribute(self, name, value):
        self.attributes[name] = value


class Library:
    def filter(self, pattern):
        return [Blueprint("vehicle.a"), Blueprint("vehicle.b"), Blueprint("vehicle.c")]

    def find(self, name):
        return Blueprint(name)


class ActorList(list):
    def filter(self, pattern):
        return ActorList(a for a in self if fnmatch.fnmatch(a.type_id, pattern))


class Box:
    def get_world_vertices(self, transform):
        p = transform.location
        return [
            NS(x=p.x + x, y=p.y + y, z=p.z + z) for x in (-2, 2) for y in (-1, 1) for z in (0, 1)
        ]


class Actor:
    def __init__(self, blueprint, transform):
        self.type_id = blueprint.id
        self.id = id(self)
        self.attributes = dict(blueprint.attributes)
        self.transform = copy.deepcopy(transform)
        self.bounding_box = Box()
        self.semantic_tags = [14]
        self.callback = None
        self.destroyed = False

    def apply_control(self, control):
        pass

    def set_simulate_physics(self, enabled):
        pass

    def get_velocity(self):
        return NS(x=0, y=0, z=0)

    def get_transform(self):
        return copy.deepcopy(self.transform)

    def set_transform(self, transform):
        self.transform = copy.deepcopy(transform)

    def listen(self, callback):
        self.callback = callback

    def stop(self):
        self.callback = None

    def destroy(self):
        self.destroyed = True
        return True

    def emit(self, frame, dt):
        if self.callback:
            width = int(self.attributes["image_size_x"])
            height = int(self.attributes["image_size_y"])
            value = 0 if "instance" in self.type_id else 70
            raw = np.full((height, width, 4), value, np.uint8).tobytes()
            self.callback(
                NS(
                    frame=frame,
                    timestamp=frame * dt,
                    transform=copy.deepcopy(self.transform),
                    width=width,
                    height=height,
                    raw_data=raw,
                )
            )


class World:
    def __init__(self):
        self.settings = NS(
            synchronous_mode=False,
            fixed_delta_seconds=None,
            no_rendering_mode=False,
            substepping=True,
            max_substep_delta_time=0.01,
            max_substeps=10,
        )
        self.actors = ActorList()
        self.frame = 0
        self.applied = []
        self.fail_at = None

    def get_settings(self):
        return copy.deepcopy(self.settings)

    def apply_settings(self, settings):
        self.applied.append(copy.deepcopy(settings))
        self.settings = copy.deepcopy(settings)

    def get_map(self):
        return NS(name="/Game/Town10HD_Opt", get_spawn_points=lambda: [Transform()])

    def get_blueprint_library(self):
        return Library()

    def get_actors(self):
        return ActorList(a for a in self.actors if not a.destroyed)

    def get_weather(self):
        return NS(cloudiness=10.0, sun_altitude_angle=60.0)

    def get_level_bbs(self, tag):
        return []

    def try_spawn_actor(self, blueprint, transform):
        actor = Actor(blueprint, transform)
        self.actors.append(actor)
        return actor

    spawn_actor = try_spawn_actor

    def tick(self, *args):
        self.frame += 1
        if self.fail_at is not None and self.frame >= self.fail_at:
            raise RuntimeError("injected tick failure")
        for actor in self.get_actors():
            actor.emit(self.frame, self.settings.fixed_delta_seconds)
        return self.frame


def fake_carla(world):
    client = NS(
        get_world=lambda: world,
        set_timeout=lambda value: None,
        get_client_version=lambda: "0.10.0",
        get_server_version=lambda: "0.10.0",
    )
    return NS(
        Client=lambda host, port: client,
        Transform=Transform,
        Location=lambda **kw: NS(**kw),
        Rotation=lambda **kw: NS(**kw),
        VehicleControl=lambda **kw: NS(**kw),
        CityObjectLabel=NS(Buildings=1),
    )


def config():
    inv = {
        "schema_version": 1,
        "map": "/Game/Town10HD_Opt",
        "client": {"host": "localhost", "port": 2000, "timeout_seconds": 1},
        "spawn_points": [{"index": 0, "pose": [0, 0, 0, 0, 0, 0]}],
        "vehicles": [{"id": "vehicle.a", "number_of_wheels": 4}],
    }
    raw = prepare_config(inv, spawn_index=0)
    raw["camera"].update(width=32, height=24)
    raw["capture"].update(warmup_ticks=1, settle_ticks=2)
    return parse_preview_config(raw)


def api():
    return importlib.import_module("src.carla_experiments.aod.capture")


def test_inventory_has_no_world_mutations():
    world = World()
    inventory = api().inventory(fake_carla(world), config().client)
    assert inventory["spawn_points"][0]["pose"] == [0, 0, 0, 0, 0, 0]
    assert len(inventory["vehicles"]) == 3
    assert world.frame == 0 and not world.applied and not world.actors


def test_capture_retains_occluded_nodes_and_validates_all_artifacts(tmp_path):
    world = World()
    destination = tmp_path / "preview"
    summary = api().capture(fake_carla(world), config(), destination)
    assert summary["status"] == "complete"
    assert summary["captured"] == 24
    assert summary["rejected"] == 0
    assert len(list(destination.rglob("*_rgb.png"))) == 24
    assert len(list(destination.glob("contact_sheet_*.png"))) == 1
    assert all(actor.destroyed for actor in world.actors)
    assert not world.settings.synchronous_mode
    artifacts = importlib.import_module("src.carla_experiments.aod.artifacts")
    assert artifacts.check_artifacts(destination)["captured"] == 24
    (next(destination.rglob("*_rgb.png"))).write_bytes(b"broken")
    with pytest.raises(ValueError, match="checksum"):
        artifacts.check_artifacts(destination)


def test_capture_failure_keeps_partial_data_and_restores_world(tmp_path):
    world = World()
    world.fail_at = 9
    destination = tmp_path / "partial"
    with pytest.raises(RuntimeError, match="injected"):
        api().capture(fake_carla(world), config(), destination)
    import json

    summary = json.loads((destination / "summary.json").read_text())
    assert summary["status"] == "incomplete"
    assert 0 < summary["captured"] < 24
    assert all(actor.destroyed for actor in world.actors)
    assert not world.settings.synchronous_mode
    assert (destination / "nodes.jsonl").exists()


def test_capture_does_not_overwrite_or_touch_occupied_world(tmp_path):
    world = World()
    api_module = api()
    destination = tmp_path / "existing"
    destination.mkdir()
    with pytest.raises(FileExistsError):
        api_module.capture(fake_carla(world), config(), destination)
    assert world.frame == 0 and not world.applied
    world.actors.append(Actor(Blueprint("vehicle.other"), Transform()))
    with pytest.raises(RuntimeError, match="vehicle|traffic|occupied"):
        api_module.capture(fake_carla(world), config(), tmp_path / "occupied")
    assert world.frame == 0 and not world.applied


def test_wait_continues_ticking_after_short_sensor_timeout():
    module = api()
    world = World()
    desired = Transform()
    calls = []

    def receive(max_frame, timeout_seconds):
        calls.append(max_frame)
        if len(calls) == 1:
            raise TimeoutError("GPU frame delayed")
        image = NS(frame=max_frame, timestamp=0.1, transform=desired)
        return [NS(frame=max_frame, rgb=image, instance=image)]

    pair = module.await_view(
        world, NS(wait_for_ready=receive), desired, min_frame=0, config=config()
    )
    assert pair.frame == 2 and calls == [1, 2]


def test_three_classes_share_view_grid_and_cleanup_between_classes(tmp_path):
    world = World()
    raw = config().resolved
    raw["scene"]["blueprint_ids"] = ["vehicle.a", "vehicle.b", "vehicle.c"]
    result = api().capture(fake_carla(world), parse_preview_config(raw), tmp_path / "three")
    assert result["captured"] == 72
    assert len(world.actors) == 9
    assert all(a.destroyed for a in world.actors)


def test_failed_actor_destruction_is_not_reported_as_complete(tmp_path, monkeypatch):
    world = World()
    monkeypatch.setattr(Actor, "destroy", lambda self: False)
    with pytest.raises(RuntimeError, match="cleanup"):
        api().capture(fake_carla(world), config(), tmp_path / "failed-cleanup")
    assert not world.settings.synchronous_mode


def test_all_geometry_blocked_records_rejections(tmp_path):
    world = World()

    class HugeBox:
        def get_world_vertices(self, transform):
            return [
                NS(x=x, y=y, z=z) for x in (-100, 100) for y in (-100, 100) for z in (-100, 100)
            ]

    world.get_level_bbs = lambda tag: [HugeBox()]
    with pytest.raises(RuntimeError, match="blocked"):
        api().capture(fake_carla(world), config(), tmp_path / "blocked")
    import json

    summary = json.loads((tmp_path / "blocked" / "summary.json").read_text())
    assert summary["captured"] == 0 and summary["rejected"] == 24


def test_frame_wait_has_a_bound_when_camera_never_matches():
    module = api()
    world = World()
    raw = config().resolved
    raw["capture"]["max_frame_ticks"] = 3
    wrong = Transform(NS(x=100, y=0, z=0))

    def receive(max_frame, timeout_seconds):
        image = NS(frame=max_frame, timestamp=0.1, transform=wrong)
        return [NS(frame=max_frame, rgb=image, instance=image)]

    with pytest.raises(TimeoutError):
        module.await_view(
            world,
            NS(wait_for_ready=receive),
            Transform(),
            min_frame=0,
            config=parse_preview_config(raw),
        )
    assert world.frame == 3


def test_pair_timestamp_tolerance_does_not_expand_with_simulation_age():
    from src.carla_experiments.aod.preview import matches_view

    pose = Transform()
    rgb = NS(frame=10, timestamp=1000000.0, transform=pose)
    instance = NS(frame=10, timestamp=1000000.0001, transform=pose)
    assert not matches_view(NS(frame=10, rgb=rgb, instance=instance), pose, min_frame=9)
