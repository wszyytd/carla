from pathlib import Path

import pytest

from src import path_cost
from src.carla_experiments.path_cost_runner import EpisodeSummary


def summary(*, success: bool = True) -> EpisodeSummary:
    return EpisodeSummary(
        experiment_id="20260826T120000Z-hover-seed20260826",
        policy="hover",
        road_id=7,
        section_id=2,
        lane_id=-1,
        start_s=14.0,
        completed_route=success,
        target_execution_valid=success,
        observation_valid_fraction=0.995 if success else 0.5,
        longest_invalid_seconds=0.2 if success else 0.3,
        uav_path_length_m=0.0,
        target_path_length_m=120.0,
        path_length_ratio=0.0,
        episode_success=success,
        measured_world_frames=100,
        measured_sensor_frames=50,
    )


def test_parser_accepts_only_supported_policies() -> None:
    parser = path_cost.build_parser()

    assert parser.parse_args(["--policy", "hover"]).policy == "hover"
    assert parser.parse_args(["--policy", "vertical_follow"]).policy == "vertical_follow"
    with pytest.raises(SystemExit):
        parser.parse_args(["--policy", "orbit"])


def test_parser_help_describes_the_current_continuous_curve_pilot() -> None:
    help_text = path_cost.build_parser().format_help()

    assert "连续、非路口的 120 m 弯道（总转角至少 60°）" in help_text


def test_main_prints_complete_success_summary(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(path_cost.importlib, "import_module", lambda name: object())
    monkeypatch.setattr(path_cost, "run_path_cost_episode", lambda *args, **kwargs: summary())

    exit_code = path_cost.main(
        ["--config", "cfg/experiments/path_cost_pilot.yaml", "--policy", "hover"]
    )

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "实验完成" in output
    assert "策略：hover" in output
    assert "路线：road=7, section=2, lane=-1, start_s=14.000" in output
    assert "目标路线执行：通过" in output
    assert "有效观测：99.500%" in output
    assert "无人机路径长度：0.000 m" in output
    assert "目标路径长度：120.000 m" in output
    assert "路径长度比：0.000000" in output
    assert "实验判定：通过" in output


def test_invalid_config_returns_two_before_importing_carla(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text("client:\n  host: ''\n", encoding="utf-8")

    def fail_import(name: str) -> object:
        raise AssertionError(f"unexpected import: {name}")

    monkeypatch.setattr(path_cost.importlib, "import_module", fail_import)

    exit_code = path_cost.main(["--config", str(config_path), "--policy", "hover"])

    output = capsys.readouterr().out
    assert exit_code == 2
    assert "错误：配置无效：" in output
    assert "Traceback" not in output


def test_missing_carla_returns_three_with_cp310_wheel_guidance(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(
        path_cost.importlib,
        "import_module",
        lambda name: (_ for _ in ()).throw(ModuleNotFoundError(name)),
    )

    exit_code = path_cost.main(
        ["--config", "cfg/experiments/path_cost_pilot.yaml", "--policy", "hover"]
    )

    output = capsys.readouterr().out
    assert exit_code == 3
    assert "cp310" in output
    assert "wheel" in output
    assert "Traceback" not in output


def test_runner_timeout_returns_three_with_type_and_message(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(path_cost.importlib, "import_module", lambda name: object())
    monkeypatch.setattr(
        path_cost,
        "run_path_cost_episode",
        lambda *args, **kwargs: (_ for _ in ()).throw(TimeoutError("sensor stalled")),
    )

    exit_code = path_cost.main(
        ["--config", "cfg/experiments/path_cost_pilot.yaml", "--policy", "hover"]
    )

    output = capsys.readouterr().out
    assert exit_code == 3
    assert "错误：实验运行失败：TimeoutError: sensor stalled" in output
    assert "Traceback" not in output


def test_completed_but_rejected_experiment_returns_five_without_error(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(path_cost.importlib, "import_module", lambda name: object())
    monkeypatch.setattr(
        path_cost, "run_path_cost_episode", lambda *args, **kwargs: summary(success=False)
    )

    exit_code = path_cost.main(
        ["--config", "cfg/experiments/path_cost_pilot.yaml", "--policy", "hover"]
    )

    output = capsys.readouterr().out
    assert exit_code == 5
    assert "实验判定：未通过" in output
    assert "错误" not in output


def test_main_passes_its_progress_reporter_to_the_runner(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(path_cost.importlib, "import_module", lambda name: object())
    received = {}

    def runner(*args, **kwargs):
        received.update(kwargs)
        return summary()

    monkeypatch.setattr(path_cost, "run_path_cost_episode", runner)

    assert path_cost.main(["--policy", "hover"]) == 0
    assert isinstance(received["progress"], path_cost.ProgressReporter)
