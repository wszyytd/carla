# CARLA Repository Scaffold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a Git-tracked, testable CARLA 0.10.0 research repository scaffold at `D:\Work\carla` whose top-level organization matches `D:\Work\wm`.

**Architecture:** Keep the existing `cfg/data/docs/out/src/tests` convention and separate thin experiment entry points from reusable modules under `src/carla_experiments`. Track code, configuration, tests, and documentation while generating large run directories at runtime and excluding them from Git.

**Tech Stack:** Python 3.10, pytest, Ruff, PyYAML, Git, CARLA 0.10.0 Python API on the Linux server.

**Spec:** `docs/superpowers/specs/2026-08-26-carla-repository-design.md`

## Global Constraints

- The repository path is exactly `D:\Work\carla`; the recommended server path is `/mnt/fast18/sunbo/carla`.
- Top-level project directories use the existing names `cfg`, `data`, `docs`, `out`, `src`, and `tests`.
- The local development and test baseline is Python 3.10.
- CARLA distributions, archives, wheels, eggs, run data, reports, model weights, and core dumps must not be tracked.
- CARLA runtime verification occurs only after the user manually synchronizes the repository to the Linux server.
- Initial scaffolding does not implement a world model, detector training, AirSim dynamics, automatic deployment, or CARLA source compilation.

---

### Task 1: Repository Policy and Metadata

**Files:**
- Create: `.gitignore`
- Create: `pyproject.toml`
- Create: `requirements.txt`
- Create: `requirements-dev.txt`
- Create: `README.md`
- Create: `docs/setup-server.md`
- Create: `docs/research-roadmap.md`

**Interfaces:**
- Consumes: the approved repository design specification.
- Produces: repository policy, Python tooling configuration, dependency boundaries, and human-readable local/server workflow.

- [ ] **Step 1: Write the repository policy files**

Create `.gitignore` with explicit rules for Python caches, environments, `data/`, `out/`, model files, CARLA packages, logs, core dumps, and local environment files. Create `pyproject.toml` with Ruff line length 100, Python 3.10 target, lint rules `E`, `F`, `I`, `UP`, and `B`, plus pytest discovery under `tests`.

- [ ] **Step 2: Define dependencies**

Set `requirements.txt` to runtime packages available on both local and server environments:

```text
numpy>=1.24,<3
Pillow>=10,<13
PyYAML>=6,<7
```

Set `requirements-dev.txt` to:

```text
-r requirements.txt
pytest>=8,<10
ruff>=0.9,<1
```

Do not add `carla` because the server installs the version-matched wheel from its CARLA 0.10.0 distribution.

- [ ] **Step 3: Document the workflow and scope**

Create `README.md` with the research stages, directory map, local setup, server synchronization boundary, and the first three future entry points (`src/smoke.py`, `src/traffic.py`, `src/follow.py`). Create `docs/setup-server.md` with the existing server distribution path, Python 3.10 wheel installation pattern, and the intended repository path. Create `docs/research-roadmap.md` with the six approved stages and explicit criteria for advancing from fixed paths to a world model.

- [ ] **Step 4: Validate the metadata text**

Run:

```powershell
rg -n "PLACEHOLDER|XXX|D:\\Work\\wm|CarlaAir RPC" README.md docs .gitignore pyproject.toml requirements*.txt
```

Expected: no placeholders; `D:\Work\wm` may appear only as the comparison repository, and no CarlaAir runtime dependency is introduced.

- [ ] **Step 5: Commit the repository policy**

```powershell
git add .gitignore pyproject.toml requirements.txt requirements-dev.txt README.md docs/setup-server.md docs/research-roadmap.md
git commit -m "chore: scaffold CARLA research repository"
```

### Task 2: Configuration and Python Package Skeleton

**Files:**
- Create: `cfg/simulator.yaml`
- Create: `cfg/traffic.yaml`
- Create: `cfg/experiments/qualification.yaml`
- Create: `src/__init__.py`
- Create: `src/smoke.py`
- Create: `src/traffic.py`
- Create: `src/follow.py`
- Create: `src/carla_experiments/__init__.py`
- Create: `src/carla_experiments/client.py`
- Create: `src/carla_experiments/config.py`
- Create: `src/carla_experiments/progress.py`
- Create: `src/carla_experiments/actors.py`
- Create: `src/carla_experiments/sensors.py`
- Create: `src/carla_experiments/storage.py`
- Create: `src/carla_experiments/metrics.py`
- Create: `src/carla_experiments/scenarios/__init__.py`
- Create: `src/carla_experiments/trajectories/__init__.py`
- Test: `tests/test_config.py`
- Test: `tests/test_progress.py`

**Interfaces:**
- Consumes: YAML files from `cfg/` and a `TextIO` output stream.
- Produces: `load_config(path: str | Path) -> dict[str, Any]` and `ProgressReporter(stream: TextIO | None = None)` whose call immediately writes and flushes one line.

- [ ] **Step 1: Write failing configuration tests**

Create `tests/test_config.py` that writes a temporary valid mapping, asserts `load_config` returns it, writes a YAML list, and asserts `load_config` raises `ValueError("configuration root must be a mapping")`.

- [ ] **Step 2: Run the configuration tests to verify failure**

Run:

```powershell
python -m pytest tests/test_config.py -v
```

Expected: collection fails because `src.carla_experiments.config` does not exist.

- [ ] **Step 3: Implement the minimal configuration loader**

Create `src/carla_experiments/config.py` using `yaml.safe_load`. Treat an empty document as `{}` and reject every non-dictionary root with the exact error text asserted by the test.

- [ ] **Step 4: Write and implement progress reporting with a test-first cycle**

Create `tests/test_progress.py` with a recording stream that counts `flush()` calls. Assert that `ProgressReporter(stream)("连接 CARLA")` writes exactly `连接 CARLA\n` and flushes once. Confirm the import fails, then implement `src/carla_experiments/progress.py` using `print(message, file=self.stream, flush=True)`.

- [ ] **Step 5: Add configuration files and focused module boundaries**

Create YAML defaults with `host: localhost`, `port: 2000`, `timeout_seconds: 10`, map `Town10HD_Opt`, Traffic Manager port `8000`, 20 background vehicles, zero walkers, fixed random seed `20260826`, and a qualification matrix of resolutions `640x360`, `1920x1080`, `3840x2160`, FOV values `90` and `30`, and distances `30`, `60`, `100`, `150` metres.

Create package marker files and focused modules. Non-implemented experiment modules contain module-level docstrings describing only their approved responsibility; they must not expose fake success paths or raise at import time. The three entry files contain module docstrings and no simulated CARLA behavior.

- [ ] **Step 6: Run focused tests and Ruff**

Run:

```powershell
python -m pytest tests/test_config.py tests/test_progress.py -v
python -m ruff check src tests
```

Expected: all tests pass and Ruff reports no errors.

- [ ] **Step 7: Commit the package skeleton**

```powershell
git add cfg src tests/test_config.py tests/test_progress.py
git commit -m "feat: add CARLA experiment package skeleton"
```

### Task 3: Layout Verification and Final Repository Check

**Files:**
- Create: `tests/test_repository_layout.py`
- Modify: `README.md`

**Interfaces:**
- Consumes: the repository root and all files produced by Tasks 1 and 2.
- Produces: an automated assertion that the agreed scaffold is present and generated-data directories are ignored.

- [ ] **Step 1: Write the failing layout test**

Create `tests/test_repository_layout.py` that resolves the repository root from `__file__`, asserts every required top-level file and directory exists, and parses `.gitignore` to assert it contains anchored `/data/` and `/out/` rules.

- [ ] **Step 2: Run the layout test and correct only actual gaps**

Run:

```powershell
python -m pytest tests/test_repository_layout.py -v
```

Expected: PASS if Tasks 1 and 2 match the approved design. If it fails, create only the missing agreed path or correct the exact ignore rule named in the failure.

- [ ] **Step 3: Run complete offline verification**

Run:

```powershell
python -m pytest -v
python -m ruff check .
python -m compileall -q src tests
git status --short
```

Expected: all tests pass, Ruff and compileall return zero, and Git status lists only the new layout test or final README correction before commit.

- [ ] **Step 4: Record the actual scaffold status in README**

State that configuration loading and immediate progress reporting are implemented and offline-tested. State separately that CARLA connection, traffic generation, sensors, and trajectories remain subsequent features requiring their own design and server verification.

- [ ] **Step 5: Commit final verification artifacts**

```powershell
git add README.md tests/test_repository_layout.py
git commit -m "test: verify CARLA repository layout"
```

- [ ] **Step 6: Verify Git completion state**

Run:

```powershell
git log --oneline -4
git status --short --branch
```

Expected: branch `main`, design plus three scaffold commits, and an empty working tree.
