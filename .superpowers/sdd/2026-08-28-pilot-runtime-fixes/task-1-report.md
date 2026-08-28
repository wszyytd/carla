# Task 1 Report: Traffic Manager Speed Units

## Change

- Updated the exact-order test to expect `28.8` km/h for an `8.0` m/s input.
- Converted units only at the `TrafficManager.set_desired_speed` boundary with `target_speed_mps * 3.6`.
- The `configure_target_path` input and all other route/runtime speed handling remain in m/s.

## TDD Evidence

RED, after changing the test expectation and before production code:

```text
1 failed, 4 passed in 0.30s
At index 4 diff: ('desired_speed', 8.0) != ('desired_speed', 28.8)
```

GREEN, after the boundary conversion:

```text
5 passed in 0.10s
```

Command used for both runs:

```text
python -m pytest tests/test_actors.py -q -p no:cacheprovider
```

## Self-review

- `git diff --check` passed.
- Diff is limited to the requested production line, its exact-order expectation, and this report.
- No conversion was added outside the CARLA API call.

## Commit

See the commit recorded after this report was created.
