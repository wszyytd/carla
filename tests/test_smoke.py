from pathlib import Path
from types import SimpleNamespace

import pytest

from src import smoke


def write_config(path: Path, text: str | None = None) -> Path:
    path.write_text(
        text or "client:\n  host: localhost\n  port: 2000\n  timeout_seconds: 10\n",
        encoding="utf-8",
    )
    return path


def make_carla_module(
    client_version: str = "0.10.0",
    server_version: str = "0.10.1",
    error: Exception | None = None,
) -> SimpleNamespace:
    actors = [object() for _ in range(5)]

    class ActorCollection(list[object]):
        def filter(self, pattern: str) -> list[object]:
            count = {"vehicle.*": 2, "walker.pedestrian.*": 1}[pattern]
            return [object() for _ in range(count)]

    class World:
        def get_map(self) -> SimpleNamespace:
            return SimpleNamespace(name="Carla/Maps/Town10HD_Opt")

        def get_settings(self) -> SimpleNamespace:
            return SimpleNamespace(synchronous_mode=False)

        def get_actors(self) -> ActorCollection:
            return ActorCollection(actors)

    class Client:
        def __init__(self, host: str, port: int) -> None:
            if error is not None:
                raise error
            self.host = host
            self.port = port

        def set_timeout(self, timeout: float) -> None:
            self.timeout = timeout

        def get_client_version(self) -> str:
            return client_version

        def get_server_version(self) -> str:
            return server_version

        def get_world(self) -> World:
            return World()

    return SimpleNamespace(Client=Client)


def test_main_reports_world_summary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    path = write_config(tmp_path / "simulator.yaml")
    monkeypatch.setattr(smoke.importlib, "import_module", lambda name: make_carla_module())

    exit_code = smoke.main(["--config", str(path)])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "连接 CARLA：localhost:2000，超时 10.0s" in output
    assert "连接成功" in output
    assert "客户端版本：0.10.0" in output
    assert "服务器版本：0.10.1" in output
    assert "地图：Carla/Maps/Town10HD_Opt" in output
    assert "同步模式：false" in output
    assert "车辆：2" in output
    assert "行人：1" in output
    assert "Actor 总数：5" in output
    assert "请求耗时：" in output


def test_invalid_config_returns_two_before_importing_carla(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    path = write_config(tmp_path / "simulator.yaml", "client:\n  host: ''\n")

    def fail_import(name: str) -> object:
        raise AssertionError(f"unexpected import: {name}")

    monkeypatch.setattr(smoke.importlib, "import_module", fail_import)

    exit_code = smoke.main(["--config", str(path)])

    output = capsys.readouterr().out
    assert exit_code == 2
    assert "错误：配置无效：" in output
    assert "client.host" in output
    assert "Traceback" not in output


def test_missing_carla_returns_three_with_wheel_hint(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    path = write_config(tmp_path / "simulator.yaml")

    def missing_carla(name: str) -> object:
        raise ModuleNotFoundError(name)

    monkeypatch.setattr(smoke.importlib, "import_module", missing_carla)

    exit_code = smoke.main(["--config", str(path)])

    output = capsys.readouterr().out
    assert exit_code == 3
    assert "cp310" in output
    assert "wheel" in output
    assert "Traceback" not in output


def test_rpc_error_returns_three_without_traceback(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    path = write_config(tmp_path / "simulator.yaml")
    module = make_carla_module(error=RuntimeError("time-out of 10000ms"))
    monkeypatch.setattr(smoke.importlib, "import_module", lambda name: module)

    exit_code = smoke.main(["--config", str(path)])

    output = capsys.readouterr().out
    assert exit_code == 3
    assert "RuntimeError: time-out of 10000ms" in output
    assert "Traceback" not in output


def test_malformed_version_returns_three(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    path = write_config(tmp_path / "simulator.yaml")
    module = make_carla_module(client_version="development")
    monkeypatch.setattr(smoke.importlib, "import_module", lambda name: module)

    exit_code = smoke.main(["--config", str(path)])

    output = capsys.readouterr().out
    assert exit_code == 3
    assert "invalid CARLA version: development" in output
    assert "Traceback" not in output


def test_incompatible_version_returns_four(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    path = write_config(tmp_path / "simulator.yaml")
    module = make_carla_module(client_version="0.9.16", server_version="0.10.0")
    monkeypatch.setattr(smoke.importlib, "import_module", lambda name: module)

    exit_code = smoke.main(["--config", str(path)])

    output = capsys.readouterr().out
    assert exit_code == 4
    assert "客户端 0.9.16" in output
    assert "服务器 0.10.0" in output
    assert "Traceback" not in output
