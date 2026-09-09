# Path-Cost Pilot Runtime Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct target speed and target visibility measurements in the CARLA 0.10 pilot, and make long-running/cleanup stages observable enough to localize the remaining native shutdown abort.

**Architecture:** Convert configured SI speed to Traffic Manager km/h at the API boundary. Derive target-visible pixels from the dominant vehicle instance color inside the projected target box, avoiding the invalid assumption that CARLA instance colors equal Python actor IDs. Thread an optional progress callback through the runner and cleanup registry without changing cleanup order.

**Tech Stack:** Python 3.10, CARLA 0.10 Python API, NumPy, pytest, Ruff

**Spec:** `docs/superpowers/specs/2026-08-26-s-curve-anticipatory-observation-design.md`

## Global Constraints

- `target_speed_mps` remains SI everywhere except the `TrafficManager.set_desired_speed` call, which receives `m/s × 3.6` km/h.
- The current pilot has exactly one dynamic vehicle and no background traffic.
- Count the most frequent `(G,B)` instance-color pair among semantic vehicle pixels inside the clipped projected target box; do not compare encoded instance colors with `target.id`.
- Preserve the `instance_pixels` artifact field and observation threshold for schema compatibility.
- Do not change camera resolution, observation thresholds, route selection, policies, cleanup ordering, or experiment acceptance rules.
- Emit progress immediately at major stages, every 100 measured world frames, and before/after every cleanup operation that calls CARLA native code.
- The cleanup logging is diagnostic instrumentation, not a claim that the native abort is fixed.

---

### Task 1: Correct Traffic Manager Speed Units

**Files:**
- Modify: `src/carla_experiments/actors.py`
- Modify: `tests/test_actors.py`

**Interfaces:**
- `configure_target_path(..., target_speed_mps: float)` continues accepting m/s.
- `traffic_manager.set_desired_speed(vehicle, speed_kmh)` receives `target_speed_mps * 3.6`.

- [x] Change the existing exact-order test to expect `28.8` for an `8.0 m/s` input and verify RED.
- [x] Implement the conversion only at the CARLA API boundary.
- [x] Run `python -m pytest tests/test_actors.py -q -p no:cacheprovider` and commit.

---

### Task 2: Count the Target Instance Inside Its Projection

**Files:**
- Modify: `src/carla_experiments/metrics.py`
- Modify: `src/carla_experiments/path_cost_runner.py`
- Modify: `tests/test_metrics.py`
- Modify: relevant runner tests
- Modify: `README.md`
- Modify: `docs/superpowers/specs/2026-08-26-s-curve-anticipatory-observation-design.md`

**Interfaces:**
- Produces `count_dominant_vehicle_instance_pixels(raw, *, width, height, projected_box, vehicle_semantic_tag=10) -> int`.
- Consumes CARLA instance-segmentation BGRA bytes and the already computed `ProjectedBox`.

- [x] Add failing tests for clipping, out-of-frame/behind-camera zero, semantic tag filtering, and selecting the dominant `(G,B)` pair rather than summing different vehicles.
- [x] Implement exact raw-length validation and clipped integer crop bounds.
- [x] Switch the runner evaluator to the new function and remove its actor-ID dependency.
- [x] Document the single-target/no-background assumption and that this is not identity matching under multi-vehicle traffic.
- [x] Run focused metrics/runner tests and Ruff, then commit.

---

### Task 3: Add Runtime and Cleanup Progress

**Files:**
- Modify: `src/path_cost.py`
- Modify: `src/carla_experiments/path_cost_runner.py`
- Modify: `src/carla_experiments/runtime.py`
- Modify: `tests/test_path_cost.py`
- Modify: `tests/test_path_cost_runner.py`
- Modify: `tests/test_runtime.py`

**Interfaces:**
- `run_path_cost_episode(..., progress: Callable[[str], None] | None = None)`.
- `OwnedActors.destroy_all(progress: Callable[[str], None] | None = None)`.
- `SynchronousSession.set_progress_reporter(progress)` optional hook used by the runner when present.

- [x] Add tests for major-stage/100-frame messages and exact cleanup before/after messages.
- [x] Pass the existing `ProgressReporter` from CLI to the runner.
- [x] Report route selection, actor/sensor readiness, warmup, every 100 frames, artifact finalization, and route completion/timeout.
- [x] Report immediately before and after sensor stop, actor destroy, Traffic Manager async restore, and world-settings restore.
- [x] Preserve injected session factories that do not implement the optional progress hook.
- [x] Run focused path-cost/runtime tests and commit.

---

### Task 4: Full Verification and Review

**Files:**
- Verify all branch changes.

- [x] Run `python -m pytest -q -p no:cacheprovider`.
- [x] Run `python -m ruff check --no-cache .`.
- [x] Run `python -m compileall -q src tests` with a writable temporary bytecode prefix.
- [x] Run `git diff --check main..HEAD` and verify branch scope contains no SDD scratch reports.
- [ ] Commit this plan document, request whole-branch review, and resolve Critical/Important findings.
