"""Synchronized RGB, depth, and instance-segmentation sensor capture."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any

from .config import CameraConfig
from .runtime import OwnedActors


@dataclass(frozen=True)
class FramePair:
    frame: int
    rgb: Any
    instance: Any


class AerialSensorRig:
    """Buffer RGB and instance images until matching CARLA frames are available."""

    _MAX_BUFFER_SIZE = 32

    def __init__(self, rgb_sensor: Any, instance_sensor: Any) -> None:
        self.rgb_sensor = rgb_sensor
        self.instance_sensor = instance_sensor
        self._condition = threading.Condition()
        self._rgb_frames: dict[int, Any] = {}
        self._instance_frames: dict[int, Any] = {}
        rgb_sensor.listen(self._receive_rgb)
        instance_sensor.listen(self._receive_instance)

    def _store(self, buffer: dict[int, Any], image: Any) -> None:
        with self._condition:
            buffer[int(image.frame)] = image
            while len(buffer) > self._MAX_BUFFER_SIZE:
                del buffer[min(buffer)]
            self._condition.notify_all()

    def _receive_rgb(self, image: Any) -> None:
        self._store(self._rgb_frames, image)

    def _receive_instance(self, image: Any) -> None:
        self._store(self._instance_frames, image)

    def set_transform(self, transform: Any) -> None:
        self.rgb_sensor.set_transform(transform)
        self.instance_sensor.set_transform(transform)

    def _drain_locked(self, max_frame: int) -> tuple[FramePair, ...]:
        ready_frames = sorted(
            frame
            for frame in self._rgb_frames.keys() & self._instance_frames.keys()
            if frame <= max_frame
        )
        pairs = tuple(
            FramePair(
                frame=frame,
                rgb=self._rgb_frames.pop(frame),
                instance=self._instance_frames.pop(frame),
            )
            for frame in ready_frames
        )
        if ready_frames:
            newest = ready_frames[-1]
            for buffer in (self._rgb_frames, self._instance_frames):
                for frame in tuple(buffer):
                    if frame <= newest:
                        del buffer[frame]
        return pairs

    def drain_ready(self, max_frame: int) -> tuple[FramePair, ...]:
        with self._condition:
            return self._drain_locked(max_frame)

    def wait_for_ready(
        self, max_frame: int, timeout_seconds: float
    ) -> tuple[FramePair, ...]:
        deadline = time.monotonic() + timeout_seconds
        with self._condition:
            while True:
                pairs = self._drain_locked(max_frame)
                if pairs:
                    return pairs
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        f"paired camera frames through {max_frame} timed out"
                    )
                self._condition.wait(remaining)


def _configure_camera_blueprint(blueprint: Any, config: CameraConfig) -> None:
    attributes = {
        "image_size_x": str(config.width),
        "image_size_y": str(config.height),
        "fov": str(config.fov_deg),
        "sensor_tick": str(config.sensor_tick_seconds),
    }
    for name, value in attributes.items():
        blueprint.set_attribute(name, value)


def spawn_paired_camera_rig(
    world: Any,
    config: CameraConfig,
    initial_transform: Any,
    actors: OwnedActors,
) -> AerialSensorRig:
    """Spawn unattached RGB and instance cameras with identical optics."""

    library = world.get_blueprint_library()
    rgb_blueprint = library.find("sensor.camera.rgb")
    instance_blueprint = library.find("sensor.camera.instance_segmentation")
    _configure_camera_blueprint(rgb_blueprint, config)
    _configure_camera_blueprint(instance_blueprint, config)

    rgb_sensor = actors.add(world.spawn_actor(rgb_blueprint, initial_transform))
    instance_sensor = actors.add(
        world.spawn_actor(instance_blueprint, initial_transform)
    )
    return AerialSensorRig(rgb_sensor, instance_sensor)
