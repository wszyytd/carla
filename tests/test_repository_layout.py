from pathlib import Path

REQUIRED_DIRECTORIES = (
    "cfg",
    "data",
    "data/smoke",
    "data/qualify",
    "data/scenarios",
    "docs",
    "out",
    "out/smoke",
    "out/qualify",
    "out/scenarios",
    "src",
    "tests",
    "cfg/experiments",
    "src/carla_experiments/scenarios",
    "src/carla_experiments/trajectories",
)

REQUIRED_FILES = (
    ".gitignore",
    "README.md",
    "pyproject.toml",
    "requirements.txt",
    "requirements-dev.txt",
    "cfg/simulator.yaml",
    "cfg/traffic.yaml",
    "cfg/experiments/qualification.yaml",
    "src/smoke.py",
    "src/traffic.py",
    "src/follow.py",
)


def test_required_repository_layout_exists() -> None:
    root = Path(__file__).resolve().parents[1]

    missing_directories = [path for path in REQUIRED_DIRECTORIES if not (root / path).is_dir()]
    missing_files = [path for path in REQUIRED_FILES if not (root / path).is_file()]

    assert missing_directories == []
    assert missing_files == []


def test_generated_data_directories_are_ignored() -> None:
    root = Path(__file__).resolve().parents[1]
    ignore_lines = {
        line.strip()
        for line in (root / ".gitignore").read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    }

    assert "/data/" in ignore_lines
    assert "/out/" in ignore_lines
