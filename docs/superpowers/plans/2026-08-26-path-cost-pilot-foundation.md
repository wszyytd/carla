# Path-Cost Pilot Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a server-runnable `python -m src.path_cost` pilot that automatically selects one S-like CARLA road segment, drives one target vehicle along it, runs either Hover or Vertical Follow aerial-camera policy, measures observation validity and motion, and writes reproducible artifacts.

**Architecture:** Keep CARLA-facing mutation behind a synchronous session and owned-actor registry, while route scoring, camera motion, projection, quality checks, and metric aggregation remain pure Python and are tested without a CARLA wheel. The pilot intentionally implements only the two diagnostic baselines; Reactive, finite-horizon, and full-route Oracle planners are a second implementation plan built on these interfaces.

**Tech Stack:** Python 3.10, dataclasses, argparse, NumPy, Pillow, PyYAML, pytest, Ruff, CARLA 0.10.0 UE5 Python API, Traffic Manager `set_path`/`set_desired_speed` on the Linux server.

**Spec:** `docs/superpowers/specs/2026-08-26-s-curve-anticipatory-observation-design.md`

## Global Constraints

- The pilot command is `python -m src.path_cost --config cfg/experiments/path_cost_pilot.yaml --policy <hover|vertical_follow>`.
- CARLA runs synchronously with `fixed_delta_seconds: 0.05`; this client is the only world-tick master during a run.
- Traffic Manager uses port `8000`, synchronous mode, and random seed `20260826`.
- The target follows a real driving lane through Traffic Manager `set_path`; no per-frame vehicle Transform teleportation is allowed.
- The initial camera is at 40 m altitude with 1920×1080 RGB and instance-segmentation sensors, horizontal FOV 60°, and `sensor_tick: 0.1`.
- Pilot target speed is 8 m/s; provisional UAV limits are 12 m/s speed, 4 m/s² acceleration, 8 m/s³ jerk, and 90°/s gimbal rate.
- An observation is valid only when all configured projection, pixel-count, distance, and motion limits pass.
- Pilot success requires at least 99.5% valid measured frames and no continuous invalid interval longer than 0.2 s.
- Path length is reported as flight distance, not battery energy.
- Local tests use fake CARLA objects. Real CARLA execution occurs only after manual synchronization to `/mnt/fast18/sunbo/carla`.
- The run restores the exact original `WorldSettings` and disables Traffic Manager synchronous mode in `finally`; it destroys only actors it created.
- `preview.mp4`, planning baselines, parameter sweeps, confidence intervals, and Pareto plots are outside this first plan; the pilot saves sampled PNG frames and machine-readable per-frame artifacts needed by those later stages.

Official API references used by this plan:

- `TrafficManager.set_path` and `set_desired_speed`: <https://carla-ue5.readthedocs.io/en/latest/python_api/>
- synchronous world and sensor queues: <https://carla-ue5.readthedocs.io/en/latest/adv_synchrony_timestep/>
- bounding-box projection and UE-to-camera axis conversion: <https://carla-ue5.readthedocs.io/en/latest/tuto_G_bounding_boxes/>
- instance segmentation ID encoding: <https://carla-ue5.readthedocs.io/en/latest/tuto_G_instance_segmentation_sensor/>

---

## File Map

- `cfg/experiments/path_cost_pilot.yaml`: all provisional pilot values and thresholds.
- `src/carla_experiments/config.py`: typed validation for the pilot configuration.
- `src/carla_experiments/trajectories/s_curve.py`: S-like lane-window scoring and selection.
- `src/carla_experiments/trajectories/baselines.py`: Hover and jerk-limited Vertical Follow commands.
- `src/carla_experiments/runtime.py`: synchronous CARLA session and owned-actor cleanup.
- `src/carla_experiments/actors.py`: deterministic target-vehicle spawn and Traffic Manager path setup.
- `src/carla_experiments/sensors.py`: paired RGB/instance camera creation and delayed exact-frame buffering.
- `src/carla_experiments/metrics.py`: projection, instance-pixel decoding, observation validity, and trajectory metrics.
- `src/carla_experiments/storage.py`: resolved config, CSV, JSON, and sampled PNG artifacts.
- `src/carla_experiments/path_cost_runner.py`: one pilot episode orchestration.
- `src/path_cost.py`: CLI, delayed CARLA import, progress output, and exit codes.
- `tests/test_path_cost_config.py`: pilot configuration validation.
- `tests/test_s_curve.py`: route ordering, turn scoring, and candidate selection.
- `tests/test_baselines.py`: look-at angles and motion limits.
- `tests/test_runtime.py`: world/TM restoration and ownership cleanup.
- `tests/test_actors.py`: deterministic vehicle spawn and TM calls.
- `tests/test_sensors.py`: paired frame collection and timeout behavior.
- `tests/test_metrics.py`: camera projection, instance ID pixels, validity, and path length.
- `tests/test_storage.py`: stable headers and artifact serialization.
- `tests/test_path_cost_runner.py`: successful and failed episode orchestration with fakes.
- `tests/test_path_cost.py`: CLI output and exit-code mapping.

### Task 1: Typed Pilot Configuration

**Files:**
- Create: `cfg/experiments/path_cost_pilot.yaml`
- Modify: `src/carla_experiments/config.py`
- Modify: `src/carla_experiments/__init__.py`
- Create: `tests/test_path_cost_config.py`

**Interfaces:**
- Consumes: `Mapping[str, Any]` from `load_config(path)`.
- Produces: `PathCostConfig` and `parse_path_cost_config(config: Mapping[str, Any]) -> PathCostConfig`.

- [ ] **Step 1: Write a failing valid-configuration test**

Use one helper that returns the complete mapping and assert exact normalized dataclasses:

```python
def valid_mapping() -> dict[str, object]:
    return {
        "random_seed": 20260826,
        "client": {"host": "localhost", "port": 2000, "timeout_seconds": 10},
        "world": {"map": "Town10HD_Opt", "fixed_delta_seconds": 0.05},
        "traffic_manager": {
            "port": 8000,
            "random_seed": 20260826,
            "synchronous_mode": True,
        },
        "route": {
            "waypoint_spacing_m": 2.0,
            "window_length_m": 120.0,
            "min_turn_each_direction_deg": 8.0,
            "candidate_rank": 0,
            "target_speed_mps": 8.0,
            "completion_tolerance_m": 3.0,
            "max_cross_track_error_m": 2.0,
            "speed_tolerance_fraction": 0.15,
            "max_duration_seconds": 40.0,
        },
        "target": {"blueprint_filter": "vehicle.*"},
        "camera": {
            "width": 1920,
            "height": 1080,
            "fov_deg": 60.0,
            "sensor_tick_seconds": 0.1,
        },
        "uav": {
            "altitude_m": 40.0,
            "initial_offset_x_m": 0.0,
            "initial_offset_y_m": 0.0,
            "max_speed_mps": 12.0,
            "max_acceleration_mps2": 4.0,
            "max_jerk_mps3": 8.0,
            "max_gimbal_rate_deg_s": 90.0,
        },
        "observation": {
            "edge_margin_fraction": 0.05,
            "min_bbox_short_side_px": 32.0,
            "max_center_error_fraction": 0.2,
            "max_distance_m": 120.0,
            "min_instance_pixels": 500,
            "min_valid_fraction": 0.995,
            "max_continuous_invalid_seconds": 0.2,
        },
        "output": {
            "root": "out/path_cost",
            "warmup_frames": 4,
            "sample_every_frames": 10,
        },
    }


def test_parse_path_cost_config_returns_typed_values() -> None:
    parsed = parse_path_cost_config(valid_mapping())
    assert parsed.world.fixed_delta_seconds == 0.05
    assert parsed.route.target_speed_mps == 8.0
    assert parsed.camera.resolution == (1920, 1080)
    assert parsed.observation.min_valid_fraction == 0.995
    assert parsed.output.root == Path("out/path_cost")
```

- [ ] **Step 2: Run the focused test to prove it fails**

Run: `python -m pytest tests/test_path_cost_config.py::test_parse_path_cost_config_returns_typed_values -v`

Expected: import failure because `PathCostConfig` and `parse_path_cost_config` do not exist.

- [ ] **Step 3: Implement frozen nested configuration dataclasses**

Add exact dataclasses `WorldConfig`, `TrafficManagerConfig`, `RouteConfig`, `TargetConfig`, `CameraConfig`, `UavConfig`, `ObservationConfig`, `OutputConfig`, and:

```python
@dataclass(frozen=True)
class PathCostConfig:
    client: ClientConfig
    world: WorldConfig
    traffic_manager: TrafficManagerConfig
    route: RouteConfig
    target: TargetConfig
    camera: CameraConfig
    uav: UavConfig
    observation: ObservationConfig
    output: OutputConfig
    random_seed: int
```

`CameraConfig.resolution` is a property returning `(width, height)`. Validation helpers must reject booleans, non-finite numbers, empty strings, ports outside `1..65535`, fractions outside `0..1`, non-positive physical limits, negative counts, `candidate_rank < 0`, `sensor_tick_seconds < fixed_delta_seconds`, and a sensor tick that is not an integer multiple of the fixed delta within `1e-9` tolerance.

- [ ] **Step 4: Add invalid-boundary tests**

Parameterize the full field path and invalid value. Cover at least:

```python
("world.fixed_delta_seconds", 0)
('traffic_manager.port', True)
('route.candidate_rank', -1)
('route.target_speed_mps', float('inf'))
('route.max_cross_track_error_m', 0)
('route.speed_tolerance_fraction', 1.1)
('camera.width', 0)
('camera.sensor_tick_seconds', 0.075)
('uav.max_jerk_mps3', 0)
('observation.edge_margin_fraction', 0.5)
('observation.min_valid_fraction', 1.1)
('output.warmup_frames', -1)
```

For each case assert the `ValueError` message contains the exact field path.

- [ ] **Step 5: Add the pilot YAML using the values from Global Constraints**

The YAML contains `random_seed: 20260826` and every section used by `valid_mapping()`. Do not duplicate client/world values through YAML anchors; the resolved config artifact must be plain data.

- [ ] **Step 6: Run configuration verification**

Run:

```bash
python -m pytest tests/test_config.py tests/test_path_cost_config.py -v
python -m ruff check src/carla_experiments/config.py tests/test_path_cost_config.py
```

Expected: all tests pass and Ruff returns zero.

- [ ] **Step 7: Commit**

```bash
git add cfg/experiments/path_cost_pilot.yaml src/carla_experiments/config.py \
  src/carla_experiments/__init__.py tests/test_path_cost_config.py
git commit -m "feat: define path-cost pilot configuration"
```

### Task 2: S-Like Road Candidate Selection

**Files:**
- Create: `src/carla_experiments/trajectories/s_curve.py`
- Modify: `src/carla_experiments/trajectories/__init__.py`
- Create: `tests/test_s_curve.py`

**Interfaces:**
- Consumes: CARLA-compatible waypoints with `road_id`, `section_id`, `lane_id`, `s`, and `transform`.
- Produces: `RouteCandidate`, `score_lane_windows(...) -> list[RouteCandidate]`, and `select_s_curve_route(map_obj: Any, config: RouteConfig) -> RouteCandidate`.

- [ ] **Step 1: Write failing tests for angle wrapping and S scoring**

Build fake waypoints whose yaw sequence is `0, 10, 20, 10, 0` degrees and whose positions are 2 m apart. Assert:

```python
candidate = score_waypoint_window(tuple(waypoints), min_turn_each_direction_deg=8.0)
assert candidate is not None
assert candidate.positive_turn_deg == pytest.approx(20.0)
assert candidate.negative_turn_deg == pytest.approx(20.0)
assert candidate.score == pytest.approx(20.0)
```

Also assert yaw transition `179 -> -179` is a `+2°` change, not `-358°`, and a one-direction-only curve is rejected.

- [ ] **Step 2: Run the scoring test to prove it fails**

Run: `python -m pytest tests/test_s_curve.py -v`

Expected: import failure because `s_curve.py` does not exist.

- [ ] **Step 3: Implement immutable route data and pure scoring**

Define:

```python
@dataclass(frozen=True)
class RouteCandidate:
    waypoints: tuple[Any, ...]
    road_id: int
    section_id: int
    lane_id: int
    start_s: float
    end_s: float
    length_m: float
    positive_turn_deg: float
    negative_turn_deg: float
    score: float
```

Group generated waypoints by `(road_id, section_id, lane_id)`, sort them by `s`, orient the group so its point-to-point direction agrees with the first waypoint forward yaw, and extract consecutive windows whose polyline length first reaches `window_length_m`. Sum positive and negative wrapped yaw deltas separately. The score is `min(positive_turn_deg, negative_turn_deg)`; reject windows below the configured minimum in either direction.

- [ ] **Step 4: Test deterministic selection and error details**

Given candidates with scores `12`, `20`, and `20`, assert ordering uses descending score followed by `(road_id, section_id, lane_id, start_s)` as a stable tie-break. Assert `candidate_rank=1` selects the second entry. When no candidate exists, raise:

```text
no S-like driving-lane window found: spacing=2.0m, length=120.0m, min_turn=8.0deg
```

- [ ] **Step 5: Implement map integration**

`select_s_curve_route` calls `map_obj.generate_waypoints(config.waypoint_spacing_m)` once, passes the result to the pure scorer, and returns the configured rank. CARLA documents `Map.generate_waypoints()` as generating waypoints on driving lanes; preserve the original waypoint objects so later code can use the first Transform and all waypoint Locations. Reject junction-crossing windows in the scorer so one pilot candidate remains a single unambiguous lane segment.

- [ ] **Step 6: Run route verification**

Run:

```bash
python -m pytest tests/test_s_curve.py -v
python -m ruff check src/carla_experiments/trajectories tests/test_s_curve.py
```

Expected: all route tests pass and Ruff returns zero.

- [ ] **Step 7: Commit**

```bash
git add src/carla_experiments/trajectories/s_curve.py \
  src/carla_experiments/trajectories/__init__.py tests/test_s_curve.py
git commit -m "feat: select S-like CARLA road segments"
```

### Task 3: Pure Camera Baseline Motion

**Files:**
- Create: `src/carla_experiments/trajectories/baselines.py`
- Modify: `src/carla_experiments/trajectories/__init__.py`
- Create: `tests/test_baselines.py`

**Interfaces:**
- Consumes: target `Vec3`, prior `MotionState`, `UavLimits`, fixed step, altitude, and policy name.
- Produces: `Vec3`, `Euler`, `MotionState`, `CameraCommand`, `look_at`, `advance_jerk_limited`, and `command_baseline`.

- [ ] **Step 1: Write failing look-at and angle-rate tests**

Use exact expectations:

```python
assert look_at(Vec3(0, 0, 40), Vec3(0, 0, 0)) == Euler(pitch=-90, yaw=0, roll=0)
assert look_at(Vec3(0, 0, 40), Vec3(10, 0, 40)) == Euler(pitch=0, yaw=0, roll=0)
assert look_at(Vec3(0, 0, 40), Vec3(0, 10, 40)) == Euler(pitch=0, yaw=90, roll=0)
assert step_angle(179, -179, max_delta_deg=1) == pytest.approx(180)
```

- [ ] **Step 2: Run the baseline test to prove it fails**

Run: `python -m pytest tests/test_baselines.py -v`

Expected: import failure because `baselines.py` does not exist.

- [ ] **Step 3: Implement focused immutable types**

```python
@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Euler:
    pitch: float
    yaw: float
    roll: float = 0.0


@dataclass(frozen=True)
class MotionState:
    position: Vec3
    velocity: Vec3
    acceleration: Vec3
    gimbal: Euler


@dataclass(frozen=True)
class UavLimits:
    max_speed_mps: float
    max_acceleration_mps2: float
    max_jerk_mps3: float
    max_gimbal_rate_deg_s: float


@dataclass(frozen=True)
class CameraCommand:
    state: MotionState
    policy: str
```

Use Euclidean-norm clipping for velocity, acceleration, and jerk. `advance_jerk_limited` computes desired velocity toward the desired position, desired acceleration toward that velocity, clips the acceleration change to `max_jerk * dt`, integrates velocity, clips speed, then integrates position.

- [ ] **Step 4: Test every hard motion limit**

For a 100 m desired step with `dt=0.05`, assert the returned norms satisfy:

```python
norm(next_state.velocity) <= limits.max_speed_mps
norm(next_state.acceleration) <= limits.max_acceleration_mps2
norm(next_state.acceleration - state.acceleration) / dt <= limits.max_jerk_mps3
angle_distance(next_state.gimbal.yaw, state.gimbal.yaw) / dt \
    <= limits.max_gimbal_rate_deg_s
```

Test `hover` keeps the original desired position. Test `vertical_follow` sets desired horizontal coordinates to the target and desired `z` to `target.z + altitude_m` before applying the limiter.

- [ ] **Step 5: Run motion verification**

Run:

```bash
python -m pytest tests/test_baselines.py -v
python -m ruff check src/carla_experiments/trajectories tests/test_baselines.py
```

Expected: all baseline tests pass and Ruff returns zero.

- [ ] **Step 6: Commit**

```bash
git add src/carla_experiments/trajectories/baselines.py \
  src/carla_experiments/trajectories/__init__.py tests/test_baselines.py
git commit -m "feat: add constrained aerial camera baselines"
```

### Task 4: Observation and Motion Metrics

**Files:**
- Modify: `src/carla_experiments/metrics.py`
- Create: `tests/test_metrics.py`

**Interfaces:**
- Consumes: world-space target box vertices, camera inverse matrix, camera intrinsics, raw instance image bytes, actor ID, trajectory samples, and `ObservationConfig`.
- Produces: `ProjectedBox`, `ObservationMetrics`, `MotionMetrics`, `build_projection_matrix`, `project_bounding_box`, `count_instance_pixels`, `evaluate_observation`, `path_length`, and `derive_motion_metrics`.

- [ ] **Step 1: Write failing projection tests from the official CARLA convention**

Use identity world-to-camera and a point whose UE coordinate is `(10, 0, 0)`. After the required `(x, y, z) -> (y, -z, x)` conversion it projects to image center. Assert the matrix focal length is:

```python
focal = width / (2.0 * np.tan(np.deg2rad(fov_deg) / 2.0))
```

Assert a box with any vertex at camera depth `<= 0` is marked `in_front=False`, not divided by zero.

- [ ] **Step 2: Run the projection tests to prove they fail**

Run: `python -m pytest tests/test_metrics.py -v`

Expected: imports fail because the metric interfaces do not exist.

- [ ] **Step 3: Implement projection data and functions**

```python
@dataclass(frozen=True)
class ProjectedBox:
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    in_front: bool

    @property
    def short_side_px(self) -> float: ...

    @property
    def center(self) -> tuple[float, float]: ...
```

`project_bounding_box` accepts an `(N, 3)` float array and a `(4, 4)` inverse matrix, returns `in_front=False` if any transformed depth is non-positive, and otherwise computes extrema without clipping them to the image. Clipping would hide edge violations.

- [ ] **Step 4: Test and implement instance-ID decoding**

Construct one `2×2` BGRA raw image. For actor ID `0x1437`, mark two pixels with RGB `[10, 0x14, 0x37]`, which means raw BGRA bytes `[0x37, 0x14, 10, 255]`. Assert `count_instance_pixels(raw, width=2, height=2, actor_id=0x1437) == 2`.

Decode actor ID as `green * 256 + blue` after converting the raw BGRA channel order. Validate byte length equals `width * height * 4`.

- [ ] **Step 5: Test and implement observation validity**

Define `ObservationMetrics` with the projected box, `instance_pixels`, `center_error_fraction`, `distance_m`, individual boolean checks, and final `valid`. The normalized center error is Euclidean distance from image center divided by the image half-diagonal. Edge margin applies independently on all four edges.

Test that changing each of these inputs alone makes an otherwise valid observation invalid: behind camera, edge margin, short side, center error, distance, instance pixels, speed, acceleration, jerk, and gimbal rate.

- [ ] **Step 6: Test and implement trajectory derivation**

For positions `(0,0,0)`, `(3,0,0)`, `(3,4,0)`, assert `path_length == 7`. With constant `dt`, compute velocity by first difference, acceleration by second difference, and jerk by third difference; the first unavailable derivatives are zero and every output list has the same length as the input.

- [ ] **Step 7: Run metric verification**

Run:

```bash
python -m pytest tests/test_metrics.py -v
python -m ruff check src/carla_experiments/metrics.py tests/test_metrics.py
```

Expected: all metric tests pass and Ruff returns zero.

- [ ] **Step 8: Commit**

```bash
git add src/carla_experiments/metrics.py tests/test_metrics.py
git commit -m "feat: measure path cost and observation validity"
```

### Task 5: Synchronous Runtime and Owned Actor Cleanup

**Files:**
- Create: `src/carla_experiments/runtime.py`
- Modify: `src/carla_experiments/__init__.py`
- Create: `tests/test_runtime.py`

**Interfaces:**
- Consumes: CARLA client, `WorldConfig`, and `TrafficManagerConfig`.
- Produces: `OwnedActors` and `SynchronousSession` context managers with `.world`, `.traffic_manager`, `.tick()`, and `.actors`.

- [ ] **Step 1: Write a failing restoration test**

Create fake settings with asynchronous mode and `fixed_delta_seconds=None`. Inside the context assert applied settings are synchronous with `0.05`. After a raised `RuntimeError("boom")`, assert:

```python
assert world.applied_settings[-1] is original_settings
assert traffic_manager.synchronous_calls == [True, False]
assert traffic_manager.seed_calls == [20260826]
```

- [ ] **Step 2: Run the runtime test to prove it fails**

Run: `python -m pytest tests/test_runtime.py -v`

Expected: import failure because `runtime.py` does not exist.

- [ ] **Step 3: Implement `OwnedActors`**

`add(actor: Any) -> Any` stores only non-`None` actors and returns the actor for convenient assignment. `destroy_all()` first calls `stop()` on actors that expose it, then calls `destroy()` in reverse creation order. It catches individual cleanup exceptions, returns a tuple of `CleanupFailure(actor_id, operation, message)`, and always clears the registry.

- [ ] **Step 4: Implement `SynchronousSession`**

On entry:

```python
self.world = client.get_world()
self._original_settings = self.world.get_settings()
settings = self.world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = config.world.fixed_delta_seconds
self.world.apply_settings(settings)
self.traffic_manager = client.get_trafficmanager(config.traffic_manager.port)
self.traffic_manager.set_random_device_seed(config.traffic_manager.random_seed)
self.traffic_manager.set_synchronous_mode(True)
```

On exit, destroy owned actors, call `traffic_manager.set_synchronous_mode(False)`, and apply the saved original settings even when body or cleanup fails. Preserve the original body exception; attach cleanup failures to `session.cleanup_failures` for reporting.

- [ ] **Step 5: Test tick ownership and reverse cleanup**

Assert each `.tick()` invokes `world.tick()` exactly once. Register camera A, camera B, and vehicle C; assert stop order is B then A for sensors exposing `stop`, and destroy order is C, B, A.

- [ ] **Step 6: Run runtime verification**

Run:

```bash
python -m pytest tests/test_runtime.py -v
python -m ruff check src/carla_experiments/runtime.py tests/test_runtime.py
```

Expected: all runtime tests pass and Ruff returns zero.

- [ ] **Step 7: Commit**

```bash
git add src/carla_experiments/runtime.py src/carla_experiments/__init__.py \
  tests/test_runtime.py
git commit -m "feat: manage synchronous CARLA experiment sessions"
```

### Task 6: Target Vehicle and Traffic Manager Path

**Files:**
- Modify: `src/carla_experiments/actors.py`
- Create: `tests/test_actors.py`

**Interfaces:**
- Consumes: world, selected `RouteCandidate`, `TargetConfig`, Traffic Manager, TM port, target speed, and `OwnedActors`.
- Produces: `spawn_target_vehicle(...) -> Any` and `configure_target_path(...) -> None`.

- [ ] **Step 1: Write failing deterministic-blueprint tests**

Create fake blueprints in unsorted order with IDs `vehicle.z`, `vehicle.a`, and `vehicle.two_wheel`; give the last one `number_of_wheels=2`. Assert selection chooses `vehicle.a` after sorting eligible four-wheel blueprints by ID. Assert an empty eligible set raises `RuntimeError` mentioning `target.blueprint_filter`.

- [ ] **Step 2: Run actor tests to prove they fail**

Run: `python -m pytest tests/test_actors.py -v`

Expected: imports fail because actor functions do not exist.

- [ ] **Step 3: Implement safe target spawn**

Set `role_name=path_cost_target` when supported. Spawn at the first route waypoint Transform with `world.try_spawn_actor`. If it returns `None`, raise a targeted error containing road, section, lane, and start `s`. Register the vehicle immediately in `OwnedActors` before any Traffic Manager call.

- [ ] **Step 4: Test and implement Traffic Manager setup**

Call exactly:

```python
vehicle.set_autopilot(True, traffic_manager_port)
traffic_manager.auto_lane_change(vehicle, False)
traffic_manager.random_left_lanechange_percentage(vehicle, 0.0)
traffic_manager.random_right_lanechange_percentage(vehicle, 0.0)
traffic_manager.set_desired_speed(vehicle, target_speed_mps)
traffic_manager.set_path(vehicle, [wp.transform.location for wp in route.waypoints[1:]])
```

Reject a route with fewer than two waypoints before enabling autopilot.

- [ ] **Step 5: Run actor verification**

Run:

```bash
python -m pytest tests/test_actors.py -v
python -m ruff check src/carla_experiments/actors.py tests/test_actors.py
```

Expected: all actor tests pass and Ruff returns zero.

- [ ] **Step 6: Commit**

```bash
git add src/carla_experiments/actors.py tests/test_actors.py
git commit -m "feat: drive a target along a selected CARLA path"
```

### Task 7: Paired Aerial Sensors

**Files:**
- Modify: `src/carla_experiments/sensors.py`
- Create: `tests/test_sensors.py`

**Interfaces:**
- Consumes: world, CARLA module, `CameraConfig`, initial camera Transform, `OwnedActors`, current world frame, and timeout.
- Produces: `FramePair` and `AerialSensorRig` with `.set_transform(transform)`, `.drain_ready(max_frame)`, and `.wait_for_ready(max_frame, timeout_seconds)`.

- [ ] **Step 1: Write failing blueprint and spawn tests**

Assert both `sensor.camera.rgb` and `sensor.camera.instance_segmentation` receive identical string attributes:

```python
{
    "image_size_x": "1920",
    "image_size_y": "1080",
    "fov": "60.0",
    "sensor_tick": "0.1",
}
```

Assert both sensors are spawned unattached at the same Transform, registered for cleanup, and started with separate queue callbacks.

- [ ] **Step 2: Run sensor tests to prove they fail**

Run: `python -m pytest tests/test_sensors.py -v`

Expected: imports fail because the sensor interfaces do not exist.

- [ ] **Step 3: Implement delayed exact-frame buffering**

```python
@dataclass(frozen=True)
class FramePair:
    frame: int
    rgb: Any
    instance: Any
```

Callbacks store images in two dictionaries keyed by `image.frame` under a `threading.Condition`. `drain_ready(max_frame)` returns every exact RGB/instance pair whose frame is `<= max_frame`, sorted by frame, removes the returned entries, and never pairs different frame numbers. `wait_for_ready(max_frame, timeout_seconds)` waits only for callback progress already triggered by prior world ticks, then returns available pairs or raises `TimeoutError("paired camera frames through {max_frame} timed out")` on deadline. Both methods discard unmatched entries older than the newest returned pair and cap each modality buffer at 32 entries.

- [ ] **Step 4: Handle camera cadence and GPU delivery delay correctly**

Because `sensor_tick=0.1` and world step is `0.05`, not every world frame has an image, and CARLA GPU sensors can deliver callbacks several world frames late. Do not predict frame IDs or block before advancing the world. The runner retains a bounded `FrameContext` keyed by each world frame and calls `drain_ready(current_world_frame)` after every tick; each returned pair is evaluated against the target/camera Transform captured under that pair's exact frame ID. Prune a context only after its paired image was processed or after the final drain reports it absent.

- [ ] **Step 5: Test transform mirroring and timeout**

Assert `.set_transform` sends the same Transform to both sensors. Feed RGB frame 10, instance frame 11, then instance frame 10 and assert `drain_ready(11)` returns only pair 10. Feed RGB frame 12 and instance frame 12 after the world has reached frame 14 and assert the delayed pair is still returned as frame 12. Assert a permanently missing modality times out with the exact message and buffers never exceed 32 frames.

- [ ] **Step 6: Run sensor verification**

Run:

```bash
python -m pytest tests/test_sensors.py -v
python -m ruff check src/carla_experiments/sensors.py tests/test_sensors.py
```

Expected: all sensor tests pass and Ruff returns zero.

- [ ] **Step 7: Commit**

```bash
git add src/carla_experiments/sensors.py tests/test_sensors.py
git commit -m "feat: synchronize aerial RGB and instance sensors"
```

### Task 8: Reproducible Pilot Artifacts

**Files:**
- Modify: `src/carla_experiments/storage.py`
- Create: `tests/test_storage.py`

**Interfaces:**
- Consumes: output root, resolved configuration, episode metadata, trajectory rows, frame-metric rows, summary mapping, and sampled RGB images.
- Produces: `PilotArtifacts` with append methods and atomic `.finalize(summary)`.

- [ ] **Step 1: Write failing layout and header tests**

With a fixed ID `20260826T120000Z-hover-seed20260826`, assert creation of:

```text
config.resolved.yaml
episode.json
trajectory.csv
frame_metrics.csv
samples/
```

Assert `trajectory.csv` header tuple is exactly:

```python
TRAJECTORY_FIELDS = (
    "frame", "sim_time_s", "target_x", "target_y", "target_z",
    "target_speed_mps", "uav_x", "uav_y", "uav_z", "uav_vx", "uav_vy",
    "uav_vz", "uav_ax", "uav_ay", "uav_az", "gimbal_pitch",
    "gimbal_yaw", "policy",
)
```

and `frame_metrics.csv` header tuple is exactly:

```python
FRAME_METRIC_FIELDS = (
    "frame", "sim_time_s", "sensor_frame", "bbox_x_min", "bbox_y_min",
    "bbox_x_max", "bbox_y_max", "bbox_short_side_px", "instance_pixels",
    "center_error_fraction", "distance_m", "speed_mps", "acceleration_mps2",
    "jerk_mps3", "gimbal_rate_deg_s", "valid", "invalid_reasons",
)
```

Pass these tuples directly to `csv.DictWriter(fieldnames=...)` so header order is stable.

- [ ] **Step 2: Run storage tests to prove they fail**

Run: `python -m pytest tests/test_storage.py -v`

Expected: imports fail because `PilotArtifacts` does not exist.

- [ ] **Step 3: Implement deterministic serialization**

Write YAML with `sort_keys=False`, JSON with UTF-8, `ensure_ascii=False`, `indent=2`, and `sort_keys=True`; write CSV with `newline=""`. `finalize` writes `summary.json.tmp`, flushes and closes both CSV streams, then replaces `summary.json` atomically using `Path.replace`.

- [ ] **Step 4: Implement sampled PNG output**

Convert CARLA BGRA raw bytes to RGB NumPy data and use `PIL.Image.fromarray(rgb).save(samples / f"{frame:08d}.png")`. Test a known one-pixel BGRA value becomes the expected RGB pixel. Do not add OpenCV or a video dependency in this plan.

- [ ] **Step 5: Run storage verification**

Run:

```bash
python -m pytest tests/test_storage.py -v
python -m ruff check src/carla_experiments/storage.py tests/test_storage.py
```

Expected: all storage tests pass and Ruff returns zero.

- [ ] **Step 6: Commit**

```bash
git add src/carla_experiments/storage.py tests/test_storage.py
git commit -m "feat: write path-cost pilot artifacts"
```

### Task 9: One-Episode Runner

**Files:**
- Create: `src/carla_experiments/path_cost_runner.py`
- Create: `tests/test_path_cost_runner.py`

**Interfaces:**
- Consumes: CARLA module, `PathCostConfig`, policy name, clock, and previously defined runtime/route/actor/sensor/metric/storage interfaces.
- Produces: `EpisodeSummary` and `run_path_cost_episode(...) -> EpisodeSummary`.

- [ ] **Step 1: Write a failing orchestration test with injected factories**

Define:

```python
@dataclass(frozen=True)
class EpisodeSummary:
    experiment_id: str
    policy: str
    completed_route: bool
    target_execution_valid: bool
    observation_valid_fraction: float
    longest_invalid_seconds: float
    uav_path_length_m: float
    target_path_length_m: float
    path_length_ratio: float
    episode_success: bool
    measured_world_frames: int
    measured_sensor_frames: int
```

Inject session, route selector, actor spawner, sensor-rig, and artifact factories into keyword-only parameters. The fake session advances five frames and delivers the pair captured at frame 2 only after frame 4; assert the runner creates the route and target once, moves both sensors once per world step, writes five trajectory rows, attributes the delayed observation to frame 2, writes metrics only for exact paired frames, and finalizes exactly once.

- [ ] **Step 2: Run the runner test to prove it fails**

Run: `python -m pytest tests/test_path_cost_runner.py -v`

Expected: import failure because `path_cost_runner.py` does not exist.

- [ ] **Step 3: Implement the episode lifecycle**

The runner performs this exact order:

```text
create Client and set timeout
enter SynchronousSession
verify current map name ends with configured world.map
select S-like route
spawn and configure target
create initial MotionState above target start
spawn paired sensors at the initial camera transform
run configured warmup ticks without metric rows
for each measured world tick:
    read target state
    compute hover or vertical_follow command
    mirror command Transform to both sensors
    tick world exactly once
    store a FrameContext and append its trajectory row
    drain every exact sensor pair delivered so far
    evaluate each pair against the FrameContext with the same world frame ID
    stop on route completion or max duration
wait briefly for callbacks already triggered by the last tick and drain again
finalize artifacts
exit session and restore world/TM
```

`FrameContext` stores target Transform, target speed, UAV `MotionState`, camera Transform, and simulation timestamp. For a delivered pair, use `target.bounding_box.get_world_vertices(context.target_transform)` and `context.camera_transform.get_inverse_matrix()` rather than the actors' current transforms. Derive the target actor ID from `target.id` for instance-pixel counting. Warmup pairs are drained and discarded; they never enter measured artifacts.

- [ ] **Step 4: Implement target-execution validity**

At each world frame, compare target distance to the nearest remaining selected route waypoint and record target speed. `target_execution_valid` requires route completion within `completion_tolerance_m`, no backward waypoint-index movement, maximum cross-track error `<= max_cross_track_error_m`, and median measured speed within `speed_tolerance_fraction` of `target_speed_mps` after warmup. If this fails, set `episode_success=False` regardless of camera metrics.

- [ ] **Step 5: Implement observation success aggregation**

`observation_valid_fraction` is valid sensor frames divided by measured sensor frames. `longest_invalid_seconds` is the maximum consecutive invalid sensor frames multiplied by `sensor_tick_seconds`. Require both configured thresholds. If zero sensor frames are measured, return a failed summary instead of dividing by zero.

- [ ] **Step 6: Test failure and cleanup paths**

Cover route timeout, sensor timeout, zero sensor frames, target deviation, observation fraction below `0.995`, and one continuous invalid interval over `0.2 s`. Assert artifacts retain `episode.json` and partial CSV files, `summary.json` records failure when the error is recoverable, and session cleanup runs for every case.

- [ ] **Step 7: Run runner verification**

Run:

```bash
python -m pytest tests/test_path_cost_runner.py -v
python -m ruff check src/carla_experiments/path_cost_runner.py tests/test_path_cost_runner.py
```

Expected: all runner tests pass and Ruff returns zero.

- [ ] **Step 8: Commit**

```bash
git add src/carla_experiments/path_cost_runner.py tests/test_path_cost_runner.py
git commit -m "feat: orchestrate path-cost pilot episodes"
```

### Task 10: Pilot CLI, Documentation, and Full Offline Gate

**Files:**
- Create: `src/path_cost.py`
- Create: `tests/test_path_cost.py`
- Modify: `README.md`
- Modify: `docs/research-roadmap.md`
- Modify: `docs/setup-server.md`
- Modify: `tests/test_repository_layout.py`

**Interfaces:**
- Consumes: configuration loading/parsing, delayed `carla` import, `run_path_cost_episode`, and `ProgressReporter`.
- Produces: `build_parser() -> argparse.ArgumentParser` and `main(argv: Sequence[str] | None = None) -> int`.

- [ ] **Step 1: Write failing parser and successful-output tests**

Assert the parser accepts only `hover` and `vertical_follow`. Monkeypatch the delayed CARLA import and runner, then assert exit `0` and output contains:

```text
实验完成
策略：hover
目标路线执行：通过
有效观测：99.500%
无人机路径长度：0.000 m
目标路径长度：120.000 m
路径长度比：0.000000
实验判定：通过
```

- [ ] **Step 2: Run the CLI test to prove it fails**

Run: `python -m pytest tests/test_path_cost.py -v`

Expected: import failure because `src.path_cost` does not exist.

- [ ] **Step 3: Implement CLI and exit codes**

Use default config `cfg/experiments/path_cost_pilot.yaml`. Validate YAML before importing CARLA. Return:

- `0`: episode ran and passed all pilot thresholds;
- `2`: invalid/unreadable configuration;
- `3`: CARLA import, RPC, route selection, actor, sensor, storage, or unexpected runtime failure;
- `5`: episode ran to a summary but failed target or observation acceptance.

Every failure prints one `错误：...` line without a traceback. A threshold failure prints the complete summary before returning `5`.

- [ ] **Step 4: Add CLI error tests**

Assert invalid config returns `2` before CARLA import, missing CARLA returns `3` with cp310 wheel guidance, runner `TimeoutError` returns `3` with its type/message, and failed `EpisodeSummary` returns `5` without the word `错误` because the experiment completed normally.

- [ ] **Step 5: Update layout and operator documentation**

Add `cfg/experiments/path_cost_pilot.yaml` and `src/path_cost.py` to `REQUIRED_FILES`. Document that the pilot automatically ranks S-like single-lane windows and prints selected road/section/lane metadata. Add exact server commands:

```bash
cd /mnt/fast18/sunbo/carla
conda activate carla10
python -m src.path_cost --config cfg/experiments/path_cost_pilot.yaml --policy hover
python -m src.path_cost --config cfg/experiments/path_cost_pilot.yaml --policy vertical_follow
```

Document that no other client may tick the same world during these runs and that exit `5` is a valid experimental rejection, not a software crash.

- [ ] **Step 6: Run the complete offline gate**

Run:

```bash
python -m pytest -v
python -m ruff check .
python -m compileall -q src tests
python -m src.path_cost --help
```

Expected: all tests pass, both static checks return zero, help lists `--config` and `--policy`, and help does not import CARLA.

- [ ] **Step 7: Commit**

```bash
git add src/path_cost.py tests/test_path_cost.py tests/test_repository_layout.py \
  README.md docs/research-roadmap.md docs/setup-server.md
git commit -m "feat: expose path-cost pilot workflow"
```

- [ ] **Step 8: Verify final local state**

Run:

```bash
git status --short --branch
git log --oneline -12
```

Expected after all ten tasks: clean `main`, ahead of `origin/main` by 12 commits (the existing design, this plan, and ten implementation commits), with no generated `data/` or `out/` artifacts tracked.

## Server Acceptance Checkpoint

After manual synchronization, run Hover first and send back its console output plus `summary.json`. Interpret results before running Vertical Follow:

- If Hover passes, do not tune code to make Vertical Follow win; tighten or spatially enlarge the pilot based on recorded pixel, center, and distance margins.
- If route selection reports no candidate, use the printed lane diagnostics to adjust only `window_length_m` or `min_turn_each_direction_deg`, preserving all camera constraints.
- If the target execution is invalid, fix route following or target speed before comparing camera policies.
- If paired sensor frames time out, verify this process is the only synchronous tick master and inspect recorded CARLA frame numbers before changing timeouts.

Only after Hover is a meaningful diagnostic and Vertical Follow produces valid artifacts should the next plan add Reactive, finite-horizon, and full-route Oracle planners.
