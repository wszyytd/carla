# CARLA Connection Smoke Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a read-only `python -m src.smoke` command that validates configuration, connects to CARLA 0.10.0, reports a deterministic world summary, and returns documented exit codes.

**Architecture:** Extend the existing configuration module with a typed client section, keep CARLA API reads in `client.py`, and let the CLI coordinate validation, delayed CARLA import, reporting, version checking, and error-to-exit-code conversion. All CARLA objects are supplied by fake modules in offline tests so the local Python 3.10 environment needs no CARLA wheel.

**Tech Stack:** Python 3.10, dataclasses, argparse, PyYAML, pytest, Ruff, CARLA 0.10.0 Python API on the Linux server.

**Spec:** `docs/superpowers/specs/2026-08-26-carla-smoke-test-design.md`

## Global Constraints

- The command is exactly `python -m src.smoke --config cfg/simulator.yaml`.
- It never calls map loading, world setting mutation, Actor creation/destruction, or `world.tick()`.
- Configuration is validated before importing `carla`.
- Successful exit is `0`; invalid configuration is `2`; CARLA import/RPC/version-parse failure is `3`; incompatible major/minor versions are `4`.
- Client/server versions are compatible when their first two numeric dot-separated components match.
- Local verification uses Python 3.10 and fake CARLA objects; real runtime verification happens only after manual server synchronization.

---

### Task 1: Typed Client Configuration

**Files:**
- Modify: `src/carla_experiments/config.py`
- Modify: `src/carla_experiments/__init__.py`
- Modify: `tests/test_config.py`

**Interfaces:**
- Consumes: the mapping returned by `load_config(path)`.
- Produces: immutable `ClientConfig(host: str, port: int, timeout_seconds: float)` and `parse_client_config(config: Mapping[str, Any]) -> ClientConfig`.

- [ ] **Step 1: Write failing validation tests**

Add parametrized tests for an absent/non-mapping `client`, blank/non-string `host`, boolean/zero/65536/non-integer `port`, and boolean/zero/infinite/non-numeric `timeout_seconds`. Assert each `ValueError` contains the full field name such as `client.port`.

Add a valid test:

```python
assert parse_client_config(
    {"client": {"host": "localhost", "port": 2000, "timeout_seconds": 10}}
) == ClientConfig("localhost", 2000, 10.0)
```

- [ ] **Step 2: Run tests to verify failure**

Run: `python -m pytest tests/test_config.py -v`

Expected: collection fails because `ClientConfig` and `parse_client_config` are absent.

- [ ] **Step 3: Implement exact validation**

Use a frozen dataclass. Reject booleans explicitly because `bool` subclasses `int`; accept integer or float timeouts only when `math.isfinite(value)` and `value > 0`. Return a normalized float timeout.

- [ ] **Step 4: Run focused verification**

Run:

```bash
python -m pytest tests/test_config.py -v
python -m ruff check src/carla_experiments/config.py tests/test_config.py
```

Expected: all configuration tests and Ruff pass.

- [ ] **Step 5: Commit**

```bash
git add src/carla_experiments/config.py src/carla_experiments/__init__.py tests/test_config.py
git commit -m "feat: validate CARLA client configuration"
```

### Task 2: Read-Only World Inspection

**Files:**
- Modify: `src/carla_experiments/client.py`
- Modify: `src/carla_experiments/__init__.py`
- Create: `tests/test_client.py`

**Interfaces:**
- Consumes: a CARLA-compatible module object, host, port, timeout, and callable monotonic clock.
- Produces: frozen `WorldSummary`, `inspect_world(...) -> WorldSummary`, and `versions_compatible(client_version: str, server_version: str) -> bool`.

- [ ] **Step 1: Write fake CARLA objects and failing summary test**

Create fakes that record `Client(host, port)`, `set_timeout(timeout)`, and actor filter patterns. Return client version `0.10.0`, server version `0.10.1`, map name `Carla/Maps/Town10HD_Opt`, synchronous mode `False`, two vehicles, one walker, and five total actors. Inject `iter([10.0, 10.25]).__next__` as the clock and assert the exact `WorldSummary`, including `elapsed_seconds == 0.25`.

- [ ] **Step 2: Verify the world test fails**

Run: `python -m pytest tests/test_client.py::test_inspect_world_returns_read_only_summary -v`

Expected: import fails because `WorldSummary` and `inspect_world` are absent.

- [ ] **Step 3: Implement read-only inspection**

Call only:

```python
client = carla_module.Client(host, port)
client.set_timeout(timeout_seconds)
client.get_client_version()
client.get_server_version()
world = client.get_world()
world.get_map().name
world.get_settings().synchronous_mode
actors = world.get_actors()
actors.filter("vehicle.*")
actors.filter("walker.pedestrian.*")
len(actors)
```

Start the clock immediately before constructing the client and stop it after all reads. Do not catch CARLA errors in this layer.

- [ ] **Step 4: Test version parsing**

Assert `0.10.0`/`0.10.1` and `0.10.0-dirty`/`0.10.9` are compatible, `0.9.16`/`0.10.0` is incompatible, and malformed versions raise `ValueError("invalid CARLA version: ...")`.

- [ ] **Step 5: Run focused verification**

Run:

```bash
python -m pytest tests/test_client.py -v
python -m ruff check src/carla_experiments/client.py tests/test_client.py
```

Expected: all client tests and Ruff pass.

- [ ] **Step 6: Commit**

```bash
git add src/carla_experiments/client.py src/carla_experiments/__init__.py tests/test_client.py
git commit -m "feat: inspect CARLA world without mutation"
```

### Task 3: Smoke-Test CLI and Exit Codes

**Files:**
- Modify: `src/smoke.py`
- Create: `tests/test_smoke.py`

**Interfaces:**
- Consumes: `load_config`, `parse_client_config`, delayed `importlib.import_module("carla")`, `inspect_world`, `versions_compatible`, and `ProgressReporter`.
- Produces: `build_parser() -> argparse.ArgumentParser` and `main(argv: Sequence[str] | None = None) -> int`.

- [ ] **Step 1: Write failing successful-CLI test**

Monkeypatch `src.smoke.importlib.import_module` to return a fake CARLA module and call `main(["--config", str(path)])`. Assert exit `0` and output contains all labels from the design: connection target, success, both versions, map, lowercase synchronous status, vehicle/walker/total counts, and elapsed seconds.

- [ ] **Step 2: Run the successful-CLI test to verify failure**

Run: `python -m pytest tests/test_smoke.py::test_main_reports_world_summary -v`

Expected: failure because `src.smoke.main` is absent.

- [ ] **Step 3: Implement parser, delayed import, and formatting**

Validate and parse YAML before `importlib.import_module("carla")`. Use `ProgressReporter` for every line. Format timeout with one decimal, synchronous mode as `true`/`false`, and elapsed time with three decimals. The module footer is:

```python
if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Write error-path tests**

Assert:

- invalid client configuration returns `2`, does not call the import function, and prints `错误：配置无效：...`;
- CARLA import failure returns `3` and mentions the version-matched `cp310` wheel;
- fake `RuntimeError("time-out of 10000ms")` returns `3` with its type and message;
- malformed version returns `3`;
- `0.9.16` versus `0.10.0` returns `4` and reports both versions;
- failure output contains no `Traceback`.

- [ ] **Step 5: Run focused verification**

Run:

```bash
python -m pytest tests/test_smoke.py -v
python -m ruff check src/smoke.py tests/test_smoke.py
```

Expected: all CLI tests and Ruff pass.

- [ ] **Step 6: Commit**

```bash
git add src/smoke.py tests/test_smoke.py
git commit -m "feat: add CARLA connection smoke test"
```

### Task 4: Documentation and Full Offline Verification

**Files:**
- Modify: `README.md`
- Modify: `docs/setup-server.md`

**Interfaces:**
- Consumes: the implemented smoke command and documented server paths.
- Produces: exact local dry verification and real server execution instructions.

- [ ] **Step 1: Update documentation**

Mark the connection smoke test implemented. Document its read-only guarantee, output fields, exit codes `0/2/3/4`, and exact server command. Keep traffic and sensor entry points marked as subsequent work.

- [ ] **Step 2: Run the complete offline gate**

Run:

```bash
python -m pytest -v
python -m ruff check .
python -m compileall -q src tests
```

Expected: all tests pass and both static commands return zero.

- [ ] **Step 3: Verify command help without CARLA installed**

Run: `python -m src.smoke --help`

Expected: exit `0`, displays `--config`, and does not attempt to import CARLA.

- [ ] **Step 4: Commit documentation**

```bash
git add README.md docs/setup-server.md
git commit -m "docs: explain CARLA smoke-test workflow"
```

- [ ] **Step 5: Verify final repository state**

Run:

```bash
git status --short --branch
git log --oneline -5
```

Expected: branch `main`, an empty working tree, and four smoke-test implementation commits after the design and plan commits.
