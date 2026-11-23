# A3 — Scrap the gear the inspector rejected and refill the pocket it came from

**Memory demand:** one item among identical visible ones, then the pocket it left. Chance ≈ 1/14. Category: Impact on AI timelines (robot building).

| | |
|---|---|
| Embodiment | `yam_memory` (bimanual I2RT YAM, Isaac Lab-Arena 0.3.1, Dream station rig, 3 cameras) |
| Instruction | "The inspector put a defective gear back into the tray. Move it to the scrap bin, then put the spare gear in the pocket it came out of." |
| Observe phase | 0–6 s: arms held; one gear lifted 25 cm (1–1.8 s), held 2 s, set in the empty pocket |
| Hidden state | the rejected gear (1/7, cycling with the seed number) and its origin pocket; the initially empty pocket is random |
| Milestones (0.25, 0.25, 0.5) | rejected gear lifted · in the scrap bin · spare seated in the origin pocket |
| Success | in bin + spare seated, held 1 s |
| End | `good_gear_lifted` · `spare_in_wrong_pocket` · `gear_off_table` |
| Horizon | 60 s |
| Oracle | `rta_oracle` (privileged): the arm on the rejected gear's side bins it, then the arm on the origin pocket's side fetches the spare; 11 cm carry |

```bash
inspect-robots run --task rta/rejected_gear -T num_scenes=7 --embodiment yam_memory --policy rta_oracle --store-frames --log-dir outputs/a03/oracle
```


## Reproduce

Isaac Lab-Arena 0.3.1 (Isaac Sim 6.1.0) with the `yam-arena` embodiment package and Inspect Robots; `code/Dockerfile`
builds the exact environment (`uv sync --frozen` on Arena's lockfile). Install this task's code into that environment:

```bash
uv pip install --python $ARENA_DIR/.venv/bin/python --no-deps -e code
inspect-robots run --task rta/rejected_gear -T num_scenes=3 --embodiment yam_memory --policy rta_oracle --store-frames --log-dir out/demo   # scripted full-score demo
inspect-robots run --task rta/rejected_gear -T num_scenes=3 --embodiment yam_memory -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream --store-frames --log-dir out/dream   # Dream-1
inspect-robots view out/demo
```

Scores per trial: `q_score` (partial credit = latched milestones / milestones, 1.0 on success), `rta_success` (0/1) and one
`milestone/<name>` column per milestone, all computed from simulator state. `-E trace_dir=<dir>` writes a per-step JSONL event
log (phase, milestones, object poses, gripper positions) for auditing a trial. Scenes are seeded (`seed-<i>`): the hidden
choice cycles with the seed number so any full cycle of seeds is balanced. Runtime per trial: about 30 s of simulated time
at 10 Hz control; roughly 3 minutes of wall time on an RTX 4090 after a one-time 2-minute simulator start.
