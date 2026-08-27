# Task 3 Report: Topology Traversal Edge Coverage

## Files changed

- `tests/test_s_curve.py`
  - Imports `score_topology_paths` and adds five direct topology-scorer regressions.
- `.superpowers/sdd/2026-08-27-topology-s-curve-selection/task-3-report.md`
  - This implementation and verification report.

`src/carla_experiments/trajectories/s_curve.py` was inspected but not changed: each
new test passed against its existing topology traversal safeguards.

## Test designs

1. **Deterministic reverse successor ordering with heterogeneous IDs**
   - A seed returns road 2 before road 1, while their first successor IDs are an
     integer and a string.  Both complete the same S-shaped path.  The scorer must
     return the two equal-score paths in road metadata order (`1`, then `2`) without
     comparing unlike ID types.
   - This catches removal or corruption of deterministic successor sorting, or a
     sort key that compares raw heterogeneous IDs.

2. **Junction rejection**
   - An otherwise valid five-waypoint S chain marks its middle waypoint as a
     junction.  The scorer must return no candidates.
   - This catches accepting a junction successor during forward expansion.

3. **Cycle termination**
   - A three-waypoint graph links back to its seed and is shorter than the requested
     window.  The scorer returns no incomplete candidate and completes normally.
   - This catches removal of per-path identity loop detection.

4. **64 retained-path ceiling**
   - One seed exposes 65 unique successors that immediately complete a two-metre
     window.  With a zero turn threshold each is scoreable; exactly 64 candidates
     may be retained.
   - This catches removal or off-by-one errors in the active/completed-path bound.

5. **Duplicate path identity elimination**
   - Two distinct fake chains use the same waypoint identity sequence.  They both
     form valid S paths, but only one candidate may be returned.
   - This catches removal of post-score path-identity deduplication.

## Red/green evidence

The five tests were added before any production edit.  While tightening the first
fixture to use actual heterogeneous IDs, its first successor was accidentally placed
at the seed location; the targeted assertion was red (`[]` rather than the two
expected paths).  Moving that fixture's first successor to 2 m made the intended
8 m S path valid.  This was a test-fixture correction, not a production defect.

The completed direct coverage then passed against the existing scorer, so no
production defect was exposed:

```
python -m pytest tests/test_s_curve.py -q
11 passed, 1 warning in 0.09s
```

A final focused rerun also passed:

```
python -m pytest tests/test_s_curve.py -q
11 passed, 1 warning in 0.09s
```

The warning is pytest's cache provider being denied permission to create a temporary
cache directory in this worktree; it does not affect collection or test execution.

## Production changes

None.  The existing implementation already:

- sorts successors by metadata and a representation-safe identity tie-breaker;
- skips junction seeds and successors;
- rejects identities already in the current path;
- retains at most 64 active or completed paths for a seed; and
- suppresses duplicate completed path identity sequences.

## Self-review

- The assertions are over `score_topology_paths` results, not implementation
  internals or mocked calls.
- Expected ordering and counts are literal, hand-constructed outcomes.
- The deterministic test includes genuinely heterogeneous successor IDs (`"road-one"`
  and `2`) and reverse input ordering.
- `git diff --check` completed without whitespace errors.

## Commit

Commit message: `test: cover topology path traversal edge cases`.
