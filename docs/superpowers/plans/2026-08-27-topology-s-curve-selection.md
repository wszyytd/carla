# Topology-Continuous S-Curve Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Select S-shaped CARLA driving routes that remain continuous across OpenDRIVE road and section boundaries.

**Architecture:** Keep the existing pure window scorer, but replace the selector's exact `(road_id, section_id, lane_id)` grouping with bounded forward traversal through `Waypoint.next(distance)`. Reject junction waypoints, prevent loops with stable waypoint identities, sort branches deterministically, and score each path as soon as it reaches the configured metric length.

**Tech Stack:** Python 3.10, CARLA 0.10 Python API, pytest, Ruff

**Spec:** `docs/superpowers/specs/2026-08-26-s-curve-anticipatory-observation-design.md`

## Global Constraints

- Preserve the existing CLI and `RouteCandidate` interface used by actor spawning and artifact storage.
- Keep junctions excluded from this pilot so intersections and traffic lights do not confound the path-length comparison.
- Preserve original CARLA waypoint objects for target spawning and Traffic Manager path configuration.
- Use deterministic branch ordering and bounded expansion.
- Do not change camera, vehicle-control, metric, or output behavior.

---

### Task 1: Reproduce Cross-Boundary Selection Failure

**Files:**
- Modify: `tests/test_s_curve.py`

**Interfaces:**
- Consumes: `select_s_curve_route(map_obj, config)` and CARLA-compatible fake waypoints exposing `next(distance)`.
- Produces: A regression test whose route changes road and section metadata while remaining topologically continuous.

- [ ] **Step 1: Add topology-aware fake waypoints**

Add a mutable `TopologyWaypoint` test double with the existing waypoint metadata, an `id`, a `successors` list, and `next(distance)` returning its successors.

- [ ] **Step 2: Write the failing cross-boundary regression test**

Build the chain `0° -> 10° -> 20° -> 10° -> 0°` with 2 m spacing and metadata changes between nodes. Make `generate_waypoints()` return the chain's first node, select an 8 m route with an 8° bidirectional-turn threshold, and assert all five original waypoint objects are selected.

- [ ] **Step 3: Run the focused test and verify RED**

Run: `python -m pytest tests/test_s_curve.py::test_select_s_curve_route_follows_topology_across_road_boundaries -q`

Expected: FAIL because the current selector only scores exact road/section/lane groups and cannot reconstruct the chain.

---

### Task 2: Traverse CARLA Waypoint Topology

**Files:**
- Modify: `src/carla_experiments/trajectories/s_curve.py`
- Modify: `src/carla_experiments/trajectories/__init__.py`
- Modify: `tests/test_s_curve.py`

**Interfaces:**
- Consumes: seed waypoints; each waypoint's `next(step_distance_m)`, `id` or metadata/location fallback identity, `is_junction`, and transform.
- Produces: `score_topology_paths(seeds, *, step_distance_m, window_length_m, min_turn_each_direction_deg) -> list[RouteCandidate]`.

- [ ] **Step 1: Implement stable waypoint and branch keys**

Prefer CARLA's waypoint `id`; otherwise derive an identity from road, section, lane, `s`, and rounded XYZ coordinates. Sort successors by road, section, lane, `s`, coordinates, and yaw so CARLA list ordering cannot change candidate order.

- [ ] **Step 2: Implement bounded forward expansion**

For each non-junction seed, repeatedly call `next(step_distance_m)`. Reject junction successors, reject nodes already present in the current path, stop a path at the first point where polyline length reaches `window_length_m`, and cap the number of completed/active paths retained per seed at 64.

- [ ] **Step 3: Score and deterministically rank completed paths**

Pass each completed path to `score_waypoint_window`, deduplicate identical waypoint-identity sequences, and retain the existing descending-score and starting-metadata tie-break.

- [ ] **Step 4: Switch the production selector**

Call `map_obj.generate_waypoints(config.waypoint_spacing_m)` once, pass those seeds into `score_topology_paths`, and leave `score_lane_windows` available for its existing pure same-lane tests and callers.

- [ ] **Step 5: Export the topology scorer and verify GREEN**

Run: `python -m pytest tests/test_s_curve.py -q`

Expected: all S-curve tests pass, including the cross-boundary regression.

---

### Task 3: Cover Determinism, Junction Rejection, and Loop Safety

**Files:**
- Modify: `tests/test_s_curve.py`
- Modify: `src/carla_experiments/trajectories/s_curve.py`

**Interfaces:**
- Consumes: `score_topology_paths(...)`.
- Produces: Regression coverage for branch ordering, junction filtering, and cyclic graphs.

- [ ] **Step 1: Add a deterministic branch test**

Create two eligible successor branches returned in reverse metadata order and assert candidates retain deterministic score/metadata ordering.

- [ ] **Step 2: Add junction and loop tests**

Assert a chain containing a junction yields no candidate. Assert a cyclic fake topology terminates and yields no incomplete candidate.

- [ ] **Step 3: Run focused tests**

Run: `python -m pytest tests/test_s_curve.py -q`

Expected: all focused tests pass without hangs.

---

### Task 4: Improve Empty-Search Diagnostics

**Files:**
- Modify: `src/carla_experiments/trajectories/s_curve.py`
- Modify: `tests/test_s_curve.py`

**Interfaces:**
- Consumes: topology expansion counts.
- Produces: An empty-search `ValueError` that preserves existing spacing/length/turn text and adds seed and completed-path counts.

- [ ] **Step 1: Add diagnostic assertions**

Extend the empty-map test to require `seeds=0` and `completed_paths=0` while retaining the existing error prefix.

- [ ] **Step 2: Return internal traversal statistics**

Track seed count and paths reaching the configured length independently from paths passing S-turn scoring, then include the counts in the exception.

- [ ] **Step 3: Run focused tests**

Run: `python -m pytest tests/test_s_curve.py -q`

Expected: all focused tests pass and the error remains actionable when a server map has no candidate.

---

### Task 5: Full Verification and Integration Commit

**Files:**
- Verify: all changed files

**Interfaces:**
- Consumes: completed implementation and regression suite.
- Produces: a reviewable Git commit on `codex/topology-s-curve`.

- [ ] **Step 1: Run the complete test suite**

Run: `python -m pytest -q`

Expected: zero failures.

- [ ] **Step 2: Run static and bytecode checks**

Run: `python -m ruff check .`

Run: `python -m compileall -q src tests`

Expected: both commands exit 0.

- [ ] **Step 3: Review the diff and repository status**

Run: `git diff --check`

Run: `git diff --stat`

Run: `git status --short`

Expected: only the plan, topology selector, exports, and S-curve tests changed; no whitespace errors.

- [ ] **Step 4: Commit the fix**

Run: `git add docs/superpowers/plans/2026-08-27-topology-s-curve-selection.md src/carla_experiments/trajectories/s_curve.py src/carla_experiments/trajectories/__init__.py tests/test_s_curve.py`

Run: `git commit -m "fix: follow topology for S-curve selection"`

