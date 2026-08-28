# Task 1 Report: Migrate Route Configuration

## Red evidence

Updated `tests/test_path_cost_config.py` first: the valid fixture now supplies
`route.min_total_turn_deg`, and a regression test requires that key. The exact
focused command then failed as expected because the parser still requested the
old `route.min_turn_each_direction_deg` key (`12 failed, 5 passed`).

Command:

```text
python -m pytest tests/test_path_cost_config.py -q -p no:cacheprovider
```

## Green evidence

After the minimal production/config migration, the same exact command passed:

```text
17 passed in 0.07s
```

## Files changed

- `src/carla_experiments/config.py`: renamed `RouteConfig` field and parser key
  to `min_total_turn_deg`, retaining positive-finite validation.
- `tests/test_path_cost_config.py`: migrated the fixture and added the missing
  new-key rejection test.
- `cfg/experiments/path_cost_pilot.yaml`: set `min_total_turn_deg: 60.0`.

## Compatibility concerns

The route-scoring implementation and its existing tests still reference
`min_turn_each_direction_deg`; those references are intentionally left for the
following scoring-propagation task. Until then, callers that execute the
scoring path through `PathCostConfig` will need the Task 2 migration.

## Self-review

- Scope is limited to the three brief-listed files.
- No route scoring behavior was modified.
- Validation remains strict for finite values greater than zero.
- `git diff --check` reports no whitespace errors.

## Commit

`Migrate route config turn threshold` (final commit hash supplied in handoff)
