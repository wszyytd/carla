import gc
import weakref
from types import SimpleNamespace

import pytest

from src.carla_experiments.config import CameraConfig
from src.carla_experiments.runtime import OwnedActors
from src.carla_experiments.sensors import AerialSensorRig, spawn_paired_camera_rig


class Blueprint:
    def __init__(self, blueprint_id: str) -> None:
        self.id = blueprint_id
        self.attributes: dict[str, str] = {}

    def set_attribute(self, name: str, value: str) -> None:
        self.attributes[name] = value


class Sensor:
    def __init__(self, sensor_id: str) -> None:
        self.id = sensor_id
        self.callback = None
        self.transforms: list[object] = []
        self.destroyed = False
        self.stopped = False

    def listen(self, callback: object) -> None:
        self.callback = callback

    def emit(self, frame: int) -> object:
        image = SimpleNamespace(frame=frame, raw_data=b"")
        assert callable(self.callback)
        self.callback(image)
        return image

    def set_transform(self, transform: object) -> None:
        self.transforms.append(transform)

    def stop(self) -> None:
        self.stopped = True

    def destroy(self) -> None:
        self.destroyed = True


class World:
    def __init__(self) -> None:
        self.blueprints = {
            name: Blueprint(name)
            for name in ("sensor.camera.rgb", "sensor.camera.instance_segmentation")
        }
        self.sensors = [Sensor("rgb"), Sensor("instance")]
        self.spawn_calls: list[tuple[Blueprint, object]] = []

    def get_blueprint_library(self) -> object:
        return SimpleNamespace(find=lambda name: self.blueprints[name])

    def spawn_actor(self, blueprint: Blueprint, transform: object) -> Sensor:
        self.spawn_calls.append((blueprint, transform))
        return self.sensors[len(self.spawn_calls) - 1]


def camera_config() -> CameraConfig:
    return CameraConfig(1920, 1080, 60.0, 0.1)


def test_spawn_paired_camera_configures_identical_unattached_sensors() -> None:
    world = World()
    transform = object()
    actors = OwnedActors()

    rig = spawn_paired_camera_rig(world, camera_config(), transform, actors)

    expected = {
        "image_size_x": "1920",
        "image_size_y": "1080",
        "fov": "60.0",
        "sensor_tick": "0.1",
    }
    assert world.blueprints["sensor.camera.rgb"].attributes == expected
    assert world.blueprints["sensor.camera.instance_segmentation"].attributes == expected
    assert world.spawn_calls == [
        (world.blueprints["sensor.camera.rgb"], transform),
        (world.blueprints["sensor.camera.instance_segmentation"], transform),
    ]
    assert callable(world.sensors[0].callback)
    assert callable(world.sensors[1].callback)
    assert isinstance(rig, AerialSensorRig)
    assert actors.destroy_all() == ()
    assert all(sensor.stopped and sensor.destroyed for sensor in world.sensors)


def test_rig_pairs_only_equal_frames_and_accepts_gpu_delayed_pair() -> None:
    rgb = Sensor("rgb")
    instance = Sensor("instance")
    rig = AerialSensorRig(rgb, instance)
    rgb_10 = rgb.emit(10)
    instance.emit(11)
    instance_10 = instance.emit(10)

    pairs = rig.drain_ready(11)

    assert len(pairs) == 1
    assert pairs[0].frame == 10
    assert pairs[0].rgb is rgb_10
    assert pairs[0].instance is instance_10

    rgb_12 = rgb.emit(12)
    instance_12 = instance.emit(12)
    delayed = rig.drain_ready(14)
    assert [(item.frame, item.rgb, item.instance) for item in delayed] == [
        (12, rgb_12, instance_12)
    ]


def test_rig_mirrors_transform_and_times_out_for_missing_modality() -> None:
    rgb = Sensor("rgb")
    instance = Sensor("instance")
    rig = AerialSensorRig(rgb, instance)
    transform = object()

    rig.set_transform(transform)
    rgb.emit(123)

    assert rgb.transforms == [transform]
    assert instance.transforms == [transform]
    with pytest.raises(
        TimeoutError,
        match=r"paired camera frames through 123 timed out",
    ):
        rig.wait_for_ready(123, timeout_seconds=0.01)


def test_rig_caps_each_unmatched_modality_buffer_at_32_frames() -> None:
    rgb = Sensor("rgb")
    instance = Sensor("instance")
    rig = AerialSensorRig(rgb, instance)
    for frame in range(40):
        rgb.emit(frame)

    instance.emit(0)
    assert rig.drain_ready(40) == ()

    instance_39 = instance.emit(39)
    pairs = rig.drain_ready(40)
    assert len(pairs) == 1
    assert pairs[0].frame == 39
    assert pairs[0].instance is instance_39


def test_sensor_callbacks_do_not_keep_rig_alive_after_episode() -> None:
    rgb = Sensor("rgb")
    instance = Sensor("instance")
    rig = AerialSensorRig(rgb, instance)
    rig_reference = weakref.ref(rig)

    del rig
    gc.collect()

    assert rig_reference() is None
    rgb.emit(1)
    instance.emit(1)
