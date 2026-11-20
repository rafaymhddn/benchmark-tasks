# H06 — Pair a Bluetooth speaker with a phone

**Demand:** operate two devices that only talk through a light and a screen. Hold a sprung button until the
light changes, act on the phone inside the pairing window, and pick this speaker by its printed name, not a
neighbour's device. Not a memory task: there is no observe phase. Category: Societal relatability.

| | |
|---|---|
| Embodiment | `yam_memory` (bimanual I2RT YAM, Isaac Lab-Arena 0.3.1, Dream station rig, 3 cameras) |
| Instruction | "Pair the SoundBox Mini Bluetooth speaker with the phone." |
| Speaker | fixed model, 10 x 20 x 7 cm at (0.22, +0.18), top panel printed SOUNDBOX MINI; sprung buttons (2.6 cm caps, 3 mm travel, 500 N/m) at x 0.21: power y 0.23, play/pause y 0.18, pairing y 0.13; status light at (0.21, 0.098), right of the pairing button |
| Phone | fixed model, 16.4 x 7.8 x 0.9 cm at (0.20, −0.17), 15.2 x 7.0 cm screen; touch zones 3.2 x 7.0 cm: bottom button x 0.14, rows x 0.236, 0.204, 0.172 |
| Light | steady white = on; pairing button held 3 s = pairing mode for 30 s (blinking blue); steady green = connected |
| Phone pages | Bluetooth settings → "Pair new device" → searching 1.5 s → "Available devices": Kitchen TV, Mara's Earbuds, and SoundBox Mini while the speaker is in pairing mode (from 2 s after it began) → "Connecting…" for 2 s → connected |
| Varies with the seed | the row order of the three devices (6 orders; a fixed row is right 1 time in 3) |
| Press / tap | cap 2 mm below rest / closed fingertips over a zone and within 5 mm of the glass; a tap counts once, when it begins |
| Milestones | pairing mode (0.25) · SoundBox Mini listed (0.25) · connected (0.5) |
| Success | connected (confirmed 2 s after the tap), then no button down and no fingertip on the screen for 2 s |
| Zero | `wrong_device`: Kitchen TV or Mara's Earbuds tapped |
| End, score kept | `speaker_switched_off`: the power button pressed |
| Horizon | 60 s |
| Oracle | `rta_oracle` (privileged): one step at a time: the right arm taps "Pair new device", the left arm holds the pairing button 4 s and lifts away, the speaker joins the list, the right arm taps its row, the phone confirms; about 24 s |

```bash
python3 tasks/h06_bluetooth_pairing/assets/build.py   # rebuild models, textures, layout.json (Pillow only)
scripts/rta check tasks/h06_bluetooth_pairing        # pure tests
scripts/rta sim h06 rta/bluetooth_pairing 6          # oracle in Isaac Sim, every row order once
```

## Models
`assets/build.py` writes everything in `assets/`: `speaker.usda`, `button_{pair,play,power}.usda`, `phone.usda`,
`textures/` and `layout.json` (the dimensions the task file reads). Geometry and artwork are original; the
speaker's name is made up.
- **Buttons** are separate free bodies, each on a prismatic joint to the world at its place on the speaker, with
  a spring drive. The task reads a press from the cap's height.
- **Light and screens** are parts of the speaker and phone models; `events()` switches them with
  `world.show(name, part, visible)`.

## Design notes
- **Either order works**, as on real devices: searching first lists only the other devices, and SoundBox Mini
  joins the list when the speaker enters pairing mode. Tapping a device before then can only pick a wrong one.
- **A short press does nothing.** The hold must be 3 s without a break; the blinking light is the only sign that
  it was long enough.
- **Guessing is punished, stopping is not.** Tapping another device scores 0, below stopping at 0.25 or 0.5.


## Reproduce

Isaac Lab-Arena 0.3.1 (Isaac Sim 6.1.0) with the `yam-arena` embodiment package and Inspect Robots; `code/Dockerfile`
builds the exact environment (`uv sync --frozen` on Arena's lockfile). Install this task's code into that environment:

```bash
uv pip install --python $ARENA_DIR/.venv/bin/python --no-deps -e code
inspect-robots run --task rta/bluetooth_pairing -T num_scenes=3 --embodiment yam_memory --policy rta_oracle --store-frames --log-dir out/demo   # scripted full-score demo
inspect-robots run --task rta/bluetooth_pairing -T num_scenes=3 --embodiment yam_memory -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy dream --store-frames --log-dir out/dream   # Dream-1
inspect-robots view out/demo
```

Scores per trial: `q_score` (partial credit = latched milestones / milestones, 1.0 on success), `rta_success` (0/1) and one
`milestone/<name>` column per milestone, all computed from simulator state. `-E trace_dir=<dir>` writes a per-step JSONL event
log (phase, milestones, object poses, gripper positions) for auditing a trial. Scenes are seeded (`seed-<i>`): the hidden
choice cycles with the seed number so any full cycle of seeds is balanced. Runtime per trial: up to 60 s of simulated time
(1,200 steps at 20 Hz control for Dream-1; the scripted demo runs at the default 10 Hz and stops at success). Wall time
on an RTX 4090 after a one-time 2-minute simulator start: roughly 3 minutes for a scripted trial, about 10 minutes for a
60 s Dream-1 trial.
