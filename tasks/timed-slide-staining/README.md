# J2 — Stain the slide 8 to 12 s, rinse 4 to 6 s, then rack it

**Memory demand:** the robot's own elapsed time through a two-stage procedure. Chance ≈ 0. Category: Impact on jobs (histotechnicians).

| | |
|---|---|
| Embodiment | `yam_memory` (bimanual I2RT YAM, Isaac Lab-Arena 0.3.1, Dream station rig, 3 cameras) |
| Instruction | "Stain the slide: dip the carrier into the purple stain for 8 to 12 seconds, then into the blue rinse for 4 to 6 seconds, then stand it on the drying rack." |
| Observe phase | none (arms free from 1 s) |
| Hidden state | none: the timing is the task |
| Milestones (0.2, 0.2, 0.15, 0.15, 0.3) | stain dip 6–16 s · 8–12 s · rinse dip 3–8 s · 4–6 s · on the rack after both |
| Success | stain 6–16 s, rinse 3–8 s, carrier in the rack for 1 s |
| End | `second_stain_dip` · `rinse_before_stain` · `carrier_off_table` |
| Horizon | 45 s |
| Oracle | `rta_oracle` (privileged): left arm, 9.3 s + 4.3 s holds 4 cm below the rims (≈ 10 s and 5 s dips) |

```bash
inspect-robots run --task rta/slide_staining -T num_scenes=3 --embodiment yam_memory --policy rta_oracle --store-frames --log-dir outputs/j02/oracle
```


## Reproduce

Isaac Lab-Arena 0.3.1 (Isaac Sim 6.1.0) with the `yam-arena` embodiment package and Inspect Robots; `code/Dockerfile`
builds the exact environment (`uv sync --frozen` on Arena's lockfile). Install this task's code into that environment:

```bash
uv pip install --python $ARENA_DIR/.venv/bin/python --no-deps -e code
inspect-robots run --task rta/slide_staining -T num_scenes=3 --embodiment yam_memory --policy rta_oracle --store-frames --log-dir out/demo   # scripted full-score demo
inspect-robots run --task rta/slide_staining -T num_scenes=3 --embodiment yam_memory -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream --store-frames --log-dir out/dream   # Dream-1
inspect-robots view out/demo
```

Scores per trial: `q_score` (partial credit = latched milestones / milestones, 1.0 on success), `rta_success` (0/1) and one
`milestone/<name>` column per milestone, all computed from simulator state. `-E trace_dir=<dir>` writes a per-step JSONL event
log (phase, milestones, object poses, gripper positions) for auditing a trial. Scenes are seeded (`seed-<i>`): the hidden
choice cycles with the seed number so any full cycle of seeds is balanced. Runtime per trial: about 30 s of simulated time
at 10 Hz control; roughly 3 minutes of wall time on an RTX 4090 after a one-time 2-minute simulator start.
