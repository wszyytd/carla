from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from .config import PathCostConfig


@dataclass(frozen=True)
class CleanupFailure:
    actor_id: Any
    operation: str
    message: str


class OwnedActors:
    """Registry that cleans up only actors created by this experiment."""

    def __init__(self) -> None:
        self._actors: list[Any] = []

    def add(self, actor: Any) -> Any:
        if actor is not None:
            self._actors.append(actor)
        return actor

    def destroy_all(
        self, progress: Callable[[str], None] | None = None
    ) -> tuple[CleanupFailure, ...]:
        actors = tuple(self._actors)
        self._actors.clear()
        failures: list[CleanupFailure] = []

        for index in reversed(range(len(actors))):
            actor = actors[index]
            stop = getattr(actor, "stop", None)
            if callable(stop):
                if progress is not None:
                    progress(f"清理：停止 actor[{index}] 前")
                try:
                    stop()
                except Exception as exc:  # cleanup must continue
                    failures.append(
                        CleanupFailure(getattr(actor, "id", None), "stop", str(exc))
                    )
                finally:
                    if progress is not None:
                        progress(f"清理：停止 actor[{index}] 后")

        for index in reversed(range(len(actors))):
            actor = actors[index]
            destroy = getattr(actor, "destroy", None)
            if callable(destroy):
                if progress is not None:
                    progress(f"清理：销毁 actor[{index}] 前")
                try:
                    destroy()
                except Exception as exc:  # cleanup must continue
                    failures.append(
                        CleanupFailure(getattr(actor, "id", None), "destroy", str(exc))
                    )
                finally:
                    if progress is not None:
                        progress(f"清理：销毁 actor[{index}] 后")

        return tuple(failures)


class SynchronousSession:
    """Own synchronous CARLA settings, ticks, Traffic Manager, and actors."""

    def __init__(self, client: Any, config: PathCostConfig) -> None:
        self.client = client
        self.config = config
        self.world: Any = None
        self.traffic_manager: Any = None
        self.actors = OwnedActors()
        self.cleanup_failures: tuple[CleanupFailure, ...] = ()
        self._original_settings: Any = None
        self._progress: Callable[[str], None] | None = None

    def set_progress_reporter(
        self, progress: Callable[[str], None] | None
    ) -> None:
        self._progress = progress

    def __enter__(self) -> SynchronousSession:
        self.world = self.client.get_world()
        self._original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.config.world.fixed_delta_seconds
        self.world.apply_settings(settings)

        self.traffic_manager = self.client.get_trafficmanager(
            self.config.traffic_manager.port
        )
        self.traffic_manager.set_random_device_seed(
            self.config.traffic_manager.random_seed
        )
        self.traffic_manager.set_synchronous_mode(True)
        return self

    def tick(self) -> int:
        return int(self.world.tick())

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> bool:
        if self._progress is not None:
            self._progress("会话清理开始")
        failures = list(self.actors.destroy_all(progress=self._progress))
        if self.traffic_manager is not None:
            if self._progress is not None:
                self._progress("清理：恢复 Traffic Manager 异步模式 前")
            try:
                self.traffic_manager.set_synchronous_mode(False)
            except Exception as cleanup_error:  # cleanup must continue
                failures.append(
                    CleanupFailure("traffic_manager", "disable_sync", str(cleanup_error))
                )
            finally:
                if self._progress is not None:
                    self._progress("清理：恢复 Traffic Manager 异步模式 后")
        if self.world is not None and self._original_settings is not None:
            if self._progress is not None:
                self._progress("清理：恢复世界设置 前")
            try:
                self.world.apply_settings(self._original_settings)
            except Exception as cleanup_error:  # cleanup must continue
                failures.append(
                    CleanupFailure("world", "restore_settings", str(cleanup_error))
                )
            finally:
                if self._progress is not None:
                    self._progress("清理：恢复世界设置 后")
        self.cleanup_failures = tuple(failures)
        if self._progress is not None:
            self._progress("会话清理完成")
        return False
