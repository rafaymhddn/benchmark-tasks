# A4 — Replace the memory module whose indicator lit during boot

**Memory demand:** a brief location signal. Chance 1/4. Category: Impact on AI timelines (data-centre work).

| | |
|---|---|
| Embodiment | `yam_memory` (bimanual I2RT YAM, Isaac Lab-Arena 0.3.1, Dream station rig, 3 cameras) |
| Instruction | "During boot the light next to the faulty memory module came on. Pull that module, put it in the RMA tray, and seat the spare in its slot." |
| Observe phase | 0–4.5 s: arms held; red cube shown at one slot's far end 1–3.5 s |
| Hidden state | the flagged slot (one of 4, cycling with the seed number) |
| Milestones (0.25, 0.25, 0.5) | flagged module lifted 5 cm · in the RMA tray · spare seated in the flagged slot |
| Success | in tray + spare seated, held 1 s |
| End | `good_module_lifted` (an unflagged module up 2 cm) · `module_off_table` |
| Horizon | 60 s |
| Oracle | `rta_oracle` (privileged): the arm on the slot's side pulls the module to the tray, then seats the spare (3 cm finger opening between 5 cm slots) |

```bash
inspect-robots run --task rta/memory_module -T num_scenes=4 --embodiment yam_memory --policy rta_oracle --store-frames --log-dir outputs/a04/oracle
```


## Reproduce

Isaac Lab-Arena 0.3.1 (Isaac Sim 6.1.0) with the `yam-arena` embodiment package and Inspect Robots; `code/Dockerfile`
builds the exact environment (`uv sync --frozen` on Arena's lockfile). Install this task's code into that environment:

```bash
uv pip install --python $ARENA_DIR/.venv/bin/python --no-deps -e code
inspect-robots run --task rta/memory_module -T num_scenes=3 --embodiment yam_memory --policy rta_oracle --store-frames --log-dir out/demo   # scripted full-score demo
inspect-robots run --task rta/memory_module -T num_scenes=3 --embodiment yam_memory -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream --store-frames --log-dir out/dream   # Dream-1
inspect-robots view out/demo
```

Scores per trial: `q_score` (partial credit = latched milestones / milestones, 1.0 on success), `rta_success` (0/1) and one
`milestone/<name>` column per milestone, all computed from simulator state. `-E trace_dir=<dir>` writes a per-step JSONL event
log (phase, milestones, object poses, gripper positions) for auditing a trial. Scenes are seeded (`seed-<i>`): the hidden
choice cycles with the seed number so any full cycle of seeds is balanced. Runtime per trial: about 30 s of simulated time
at 10 Hz control; roughly 3 minutes of wall time on an RTX 4090 after a one-time 2-minute simulator start.
