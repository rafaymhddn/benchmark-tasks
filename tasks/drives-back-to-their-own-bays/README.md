# A1 — Return every drive to the bay it came from

**Memory demand:** identical items through a reshuffle. Chance 1/6. Category: Impact on AI timelines (data-centre work).

| | |
|---|---|
| Embodiment | `yam_memory` (bimanual I2RT YAM, Isaac Lab-Arena 0.3.1, Dream station rig, 3 cameras) |
| Instruction | "Put each drive back into the bay it came from; the storage array only rebuilds if every drive is in its own bay." |
| Observe phase | 0–6.8 s: arms held; sleds moved one at a time to cart slots (1, 3, 5 s) |
| Hidden state | sled → cart-slot arrangement (one of 24, cycling with the seed number) |
| Milestones (1/3 each) | each sled seated in its own bay |
| Success | all three, held 1 s |
| End | `sled_in_wrong_bay` · `sled_on_floor` |
| Horizon | 60 s |
| Oracle | `rta_oracle` (privileged): one sled at a time; left bay by the left arm, right bay by the right arm, centre bay by the arm on the slot's side; 3 cm finger opening between 5 cm slots |

```bash
inspect-robots run --task rta/drive_bays -T num_scenes=6 --embodiment yam_memory --policy rta_oracle --store-frames --log-dir outputs/a01/oracle
```


## Reproduce

Isaac Lab-Arena 0.3.1 (Isaac Sim 6.1.0) with the `yam-arena` embodiment package and Inspect Robots; `code/Dockerfile`
builds the exact environment (`uv sync --frozen` on Arena's lockfile). Install this task's code into that environment:

```bash
uv pip install --python $ARENA_DIR/.venv/bin/python --no-deps -e code
inspect-robots run --task rta/drive_bays -T num_scenes=3 --embodiment yam_memory --policy rta_oracle --store-frames --log-dir out/demo   # scripted full-score demo
inspect-robots run --task rta/drive_bays -T num_scenes=3 --embodiment yam_memory -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream --store-frames --log-dir out/dream   # Dream-1
inspect-robots view out/demo
```

Scores per trial: `q_score` (partial credit = latched milestones / milestones, 1.0 on success), `rta_success` (0/1) and one
`milestone/<name>` column per milestone, all computed from simulator state. `-E trace_dir=<dir>` writes a per-step JSONL event
log (phase, milestones, object poses, gripper positions) for auditing a trial. Scenes are seeded (`seed-<i>`): the hidden
choice cycles with the seed number so any full cycle of seeds is balanced. Runtime per trial: about 30 s of simulated time
at 10 Hz control; roughly 3 minutes of wall time on an RTX 4090 after a one-time 2-minute simulator start.
