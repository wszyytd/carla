# Curved Route Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the path-cost pilot select a continuous non-junction road curve in Town10HD_Opt without requiring an S-shaped left/right turn sequence.

**Architecture:** Preserve topology traversal and original CARLA waypoint objects. Replace the bidirectional-turn acceptance rule with total absolute heading change, rename the route threshold to `min_total_turn_deg`, and rank eligible windows by that total turn while retaining positive/negative diagnostics.

**Tech Stack:** Python 3.10, CARLA 0.10 Python API, PyYAML, pytest, Ruff

**Spec:** `docs/superpowers/specs/2026-08-26-s-curve-anticipatory-observation-design.md` as amended by the approved pilot decision: a 120 m continuous curve with at least 60° total turn is sufficient for engineering validation; S/U/multiple shapes remain required for later research conclusions.

## Global Constraints

- Use `window_length_m: 120.0` and `min_total_turn_deg: 60.0` for the Town10 pilot.
- Keep junctions excluded and retain deterministic topology traversal, loop prevention, and the 64-path bound.
- Preserve the `RouteCandidate` fields and original CARLA waypoint objects used by actor spawning and storage.
- Define route score as `positive_turn_deg + negative_turn_deg` and rank descending by that score.
- Do not change vehicle control, cameras, observation constraints, metrics, or artifact formats.
- Update user-facing wording from “S 形道路” to “连续弯道” where it describes the current pilot; historical implementation plans remain unchanged.

---

### Task 1: Migrate Route Configuration

**Files:**
- Modify: `src/carla_experiments/config.py`
- Modify: `cfg/experiments/path_cost_pilot.yaml`
- Modify: `tests/test_path_cost_config.py`

**Interfaces:**
- Produces: `RouteConfig.min_total_turn_deg: float` parsed from `route.min_total_turn_deg`.
- Removes: current runtime dependence on `min_turn_each_direction_deg`.

- [ ] Add a failing parser test requiring `min_total_turn_deg` and rejecting the old missing key.
- [ ] Rename the dataclass field and parser key, preserving positive-finite validation.
- [ ] Set the pilot YAML threshold to `60.0`.
- [ ] Run `python -m pytest tests/test_path_cost_config.py -q -p no:cacheprovider` and commit.

---

### Task 2: Generalize Route Scoring

**Files:**
- Modify: `src/carla_experiments/trajectories/s_curve.py`
- Modify: `tests/test_s_curve.py`

**Interfaces:**
- `score_waypoint_window(..., min_total_turn_deg: float) -> RouteCandidate | None`
- `score_lane_windows(..., min_total_turn_deg: float) -> list[RouteCandidate]`
- `score_topology_paths(..., min_total_turn_deg: float) -> list[RouteCandidate]`
- `select_s_curve_route(map_obj, config) -> RouteCandidate` remains as a compatibility name for existing imports.

- [ ] Add a failing test proving a one-direction `0°→20°→40°→60°→80°` curve is accepted at 60° and a 40° curve is rejected.
- [ ] Change score to `positive_turn_deg + negative_turn_deg`; retain both component fields for diagnostics.
- [ ] Propagate `min_total_turn_deg` through lane and topology scorers and selector.
- [ ] Change the empty-search message to “no curved driving-lane window found” and report `min_total_turn`.
- [ ] Update existing score/order/error tests without weakening topology safety coverage.
- [ ] Run `python -m pytest tests/test_s_curve.py -q -p no:cacheprovider` and commit.

---

### Task 3: Update Pilot Documentation and CLI Wording

**Files:**
- Modify: `README.md`
- Modify: `docs/research-roadmap.md`
- Modify: `docs/setup-server.md`
- Modify: `docs/superpowers/specs/2026-08-26-s-curve-anticipatory-observation-design.md`
- Modify: `src/path_cost.py`
- Modify: relevant CLI/documentation tests if wording is asserted

**Interfaces:**
- Produces: user-facing descriptions that call the current engineering pilot a continuous-curve test and explicitly reserve multiple route shapes for formal conclusions.

- [ ] Replace current-pilot S-only claims with continuous-curve wording.
- [ ] Record that the first Town10 pilot uses a single curve; formal experiments require multiple shapes/curvatures.
- [ ] Update CLI help text without changing flags.
- [ ] Run focused CLI/documentation tests and commit.

---

### Task 4: Full Verification

**Files:**
- Verify all branch changes.

**Interfaces:**
- Produces: a clean, reviewable branch ready for integration.

- [ ] Run `python -m pytest -q -p no:cacheprovider`.
- [ ] Run `python -m ruff check --no-cache .`.
- [ ] Run `python -m compileall -q src tests` with bytecode directed to a writable temporary prefix if needed.
- [ ] Run `git diff --check main..HEAD`, inspect `git diff --stat main..HEAD`, and confirm clean status.
- [ ] Request whole-branch code review and resolve Critical/Important findings.
