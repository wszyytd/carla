# Terminal camera scout

Start CARLA first, then run from the repository root:

```bash
conda activate carla10
python -m src.scout
```

Each movement saves an RGB PNG and a JSON record containing actual camera pose,
frame, simulation timestamp, map, image dimensions, FOV, and the command.
Files go to `out/scout/<timestamp>/`. Open images with VS Code Remote SSH.

Commands (press Enter after each):

| Command | Action |
| --- | --- |
| `w 10` / `s 10` | Horizontal forward / backward 10 m |
| `a 10` / `d 10` | Horizontal left / right 10 m |
| `up 5` / `down 5` | Vertical movement 5 m |
| `yaw 30` | Relative right turn 30 degrees |
| `pitch -45` | Absolute downward pitch 45 degrees |
| `home` | Return to initial spectator pose |
| `spawns` | List road spawn locations |
| `spawn 0 40` | Move 40 m above road spawn 0 |
| `load out/scout/<timestamp>/0001.json` | Restore a captured pose |
| `save` | Capture current pose again |
| `pose` / `help` / `quit` | Inspect / help / exit |

Distances default to 5 m when omitted. Movement is relative to camera yaw,
with horizontal movement independent of pitch. This is a camera scouting tool:
it does not simulate a drone or check collisions. World settings are unchanged.
Do not run alongside another controller moving the same spectator.

If the world is already synchronous, stop other ticking clients and run
`python -m src.scout --tick`. Do not use `--tick` in an asynchronous world.
The sensor is destroyed and spectator restored on normal exit or Ctrl+C.
No-rendering mode must be disabled for RGB capture.

Local unit tests cover displacement and pose matching. Actual CARLA rendering
must be verified on the server.
