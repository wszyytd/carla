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

## Review-fix round 1

The review identified that three original fixtures did not exercise the intended
traversal behavior.  No production change was needed; `score_topology_paths` already
has the required guards.

- **Determinism and heterogeneous IDs:** both successor branches now have identical
  road, section, lane, `s`, coordinate, and yaw metadata.  Their first IDs are the
  integer `2` and string `"branch-a"`, supplied in the reverse of the expected
  representation-safe sort order.  The assertion reads the returned second-waypoint
  IDs directly: `["branch-a", 2]`.  It fails if sorting is removed and raises if a
  sort key directly compares the unlike raw IDs.
- **Junction expansion:** the junction node's `next()` raises `AssertionError`.
  The path is deliberately shorter than the window, so accepting it would force a
  subsequent topology expansion and raise.  The outcome still asserts no candidate.
- **Combined retained-path cap:** the seed exposes one immediately completed path and
  64 one-metre active paths.  Only 63 active paths can coexist with the completed
  one; each retained active path then reaches two metres and completes.  The observed
  result is exactly 64 candidates, while removing the combined active-plus-completed
  cap would yield 65.

Focused verification command and output:

```
python -m pytest \
  tests/test_s_curve.py::test_score_topology_paths_sorts_reverse_successors_with_heterogeneous_ids \
  tests/test_s_curve.py::test_score_topology_paths_rejects_chain_that_enters_junction \
  tests/test_s_curve.py::test_score_topology_paths_retains_at_most_64_paths_per_seed -q
3 passed, 1 warning in 0.08s
```

The warning remains pytest cache-provider permission denial in the worktree and does
not affect test execution.  Required full focused-suite verification:

```
python -m pytest tests/test_s_curve.py -q
11 passed, 1 warning in 0.08s
```

Fix commit message: `test: strengthen topology traversal edge coverage`.
