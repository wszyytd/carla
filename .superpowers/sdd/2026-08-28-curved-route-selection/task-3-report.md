# Task 3 Report: Curved-route Pilot Documentation and CLI Wording

## Files changed

- `README.md`
- `docs/research-roadmap.md`
- `docs/setup-server.md`
- `docs/superpowers/specs/2026-08-26-s-curve-anticipatory-observation-design.md`
- `src/path_cost.py`
- `tests/test_path_cost.py`

## Wording boundaries

- The current Town10 engineering validation selects one continuous, non-junction 120 m curve with at least 60° cumulative turn.
- This single curve is explicitly limited to engineering validation and is not presented as evidence for formal research conclusions.
- Formal experiments are explicitly reserved for multiple route shapes and curvature levels, including S/U shapes where the map provides them.
- No CLI flags or choices changed. The parser description alone now states the current-pilot boundary.
- Historical implementation-plan files were left unchanged. In particular, the pre-existing untracked `docs/superpowers/plans/2026-08-28-curved-route-selection.md` was not modified or staged.

## Tests

- Red: `python -m pytest tests/test_path_cost.py -q` failed as expected after adding the parser-help assertion and before updating the CLI description.
- Green: `python -m pytest tests/test_path_cost.py -q -p no:cacheprovider` — 7 passed.
- `python -m ruff check . --no-cache` — all checks passed.
- `git diff --check` — no whitespace errors.
- `python -m pytest tests/test_path_cost.py tests/test_repository_layout.py -q` had one unrelated failure: this worktree lacks eight directories expected by `test_repository_layout.py`. It is not caused by these changes. The same invocation also emitted cache write warnings because the worktree cache directory is denied.

## Self-review

- Confirmed every current-pilot statement in the requested README, roadmap, setup guide, research spec, and CLI refers to the same Town10 single-curve constraints.
- Confirmed every formal-research disclaimer requires route-shape and curvature variation, with S/U only when available.
- Confirmed the CLI help wording is covered by a focused test and the command-line interface remains unchanged.

## Commit

`docs: clarify curved route pilot boundary`
