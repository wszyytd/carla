# Viewbank implementation record

Spec: requirements.md and high-level-design.md; scope narrowed by the implementation request to the CARLA producer only.
Execution: inline, no subagents; commit after all local checks, push implementation branch, no PR.

- [x] Strict configuration, deterministic grid and pure geometry: test invalid fields, 72/18 counts, adjacency, analytic slab intersection, reasons and connectivity before implementation.
- [x] Transactional artifacts and offline validator: test JSONL, path containment, depth and image shapes, hashes, orphan commits, partial directories and incompatible resume before implementation.
- [x] CARLA adapter: reuse scout FrameInbox/StabilityGate/decode_depth, AOD geometry/pose/JSON/hash helpers and runtime cleanup; test fake CARLA success, delayed frames, failure, interruption, cleanup and resume.
- [x] CLI, docs, full pytest, Ruff, diff check, self-review, commit and push.

Interfaces: configuration is a normalized JSON-compatible dictionary; grid returns node/edge dictionaries; geometry annotates them; atomic frame directories contain their own commit receipt; validator independently reconstructs expected topology. Capture commits a directory before updating the manifest. Offline commands never import carla.

Review focus: interrupted directory rename versus manifest update; changed world on resume; one corrupt file versus a validated frame; late callback after sensor movement; malformed manifests and path traversal. Each receives an offline regression test. Static geometry is approximate, and real CARLA capture remains a server gate.


## Verification record (2026-09-24)

- Local Windows Python 3.14; CARLA is not installed. Linux CARLA 0.10.0 / Python 3.10 remains the server acceptance environment.
- `python -m pytest -q -p no:cacheprovider`: 310 passed, including the existing AOD, scout and path_cost suite. For this machine only, PYTEST_ADDOPTS pointed basetemp to ignored out/pytest-delivery because the default OS temp directory was inaccessible. Required ignored empty data/out directories were created locally.
- `python -m ruff check --no-cache .`: passed.
- `git diff --check`: passed.
- CLI plan: 72 nodes / 408 directed edges; smoke: 18 nodes / 66 edges; no CARLA import or connection.
- Real archived region_geometry.json contains 56,042 AABBs. Offline preliminary screening of the proposed Pilot leaves 72 valid nodes in one connected graph; this is archived geometry, not a fresh server capture.
- Meaningful RED→GREEN tests preceded each module. Final self-review regressions cover actual FOV and intrinsics, native weather objects without copying support, actual pose drift into an obstacle, geometry fingerprint tampering, reporter failures, corrupt receipts and quarantined temporary-file checksums.
- Final review performed inline by the author because the user explicitly prohibited subagents. No independent reviewer was used. No known deferred implementation defects; real rendering, point-cloud direction/scale and exposure suitability remain server checks, not claimed successes.

## Implementation decisions

- Exact slab intersection replaces sampled edge checks; edge_step_m remains recorded for compatibility and future mesh checks. This avoids thin-obstacle gaps.
- The smoke config uses one yaw (3×3×2×1), while the Pilot exercises all ±90-degree rotations.
- Added frame receipt.json, summary.json and diagnostic/recovery so completed directory commits can survive a stale node manifest without overwriting verified frames.
- No target vehicles are spawned in v1: the static map is shared by all nodes. Existing dynamic actors are refused. Controlled targets would require a separate explicit schema extension.
- Shared sync settings and callback setup were extracted without changing existing AOD/scout/path_cost commands. MAGICIAN access enforcement, coverage/Oracle computation and model evaluation remain out of scope.
