# Take an instant photo, then draw the print out of the held camera and lay it face down

A free instant camera stands aimed at an apple 50 cm away. A press home on its sprung shutter button takes the photo;
let go, the print comes out of a low slot under the film shield, which holds it. One hand lifts the camera, lens up;
after 5 s the other lifts the shield, draws the print out, lays it face down.

| | |
|---|---|
| Embodiment | `yam_memory` (two I2RT YAM arms, Isaac Lab-Arena 0.3.1, Dream station rig, 3 cameras), 20 Hz |
| Camera | Polaroid Go Generation 2, white (no logo): free, 274 g with film, centre (0.279, +0.055), lens toward +x; red shutter button on top; print slot low in front, film shield over it |
| Apple | red artificial apple, 7 cm, fixed at (0.842, +0.055), on the lens axis 50 cm from the lens |
| Print | Go film (66.6 x 53.9 mm; a 2.5 mm, 3.5 g card), inside the camera, picture side up |
| Run limit | 1,200 steps (60 s) |

## Scoring

| Score | Asked | Read on the real bench |
|---|---|---|
| 0.25 | photo taken: at the first press home (9/10 of the button's 2 mm) the camera level within 20 degrees, under 2 cm/s, the apple within 12 degrees of the lens axis, 45 cm to 2 m away | video; developed print |
| 0.5 | the print drawn free of the slot | video |
| 0.75 | at the end the print face down on the bench, still, hands at least 4 cm away | final photo |
| 1.0 | the shield lifted (past 15 degrees) at least 5 s after the print came out and before the print is drawn free; at the end the camera standing, still, hands off; held 3 s | video time stamps; final photo |

The first two count once reached, the last two while they hold. A first press home off the shot: 0
(`wasted_exposure`); `camera_knocked_over` (let go, over 60 degrees) and `dropped_off_bench` end the trial.

## The film shield

- Polaroid's guide to the Go Generation 2 (https://www.polaroid.com/blog/education/how-to-use-polaroid-go-gen-2):
  "The photo will eject underneath the film shield. Leave it there for 5 seconds. Then gently lift the film shield
  and let it roll back in. Remove the photo." "face down on a flat surface".
- Polaroid's Go manual (first generation: same body, field of view and fixed-focus lens; the 45 cm is its figure;
  https://parallaxphotographic.coop/wp-content/uploads/2021/09/Polaroid-Go-User-Manual-English.pdf): "eject from the
  slot [...] as soon as you let go of the shutter button"; "covered with a film shield to protect it from light";
  "pushed through two metal rollers".

Modelled: the shield is a part, a 1 mm flap (18 x 76 mm) hinged at the slot's upper lip. A rod under it, 4.5 mm from
the hinge, presses the print onto the slot's lower lip with the flap's spring: 0.26 N, 0.23 N as a pull starts it
sliding (the rod's friction eases the flap), 0.15 N while it slides. Friction (1.0 on top, 0.3 under) holds the print;
the reaction goes into the camera. The press ends only when the shield is up: lifted past 15 degrees, it rolls back to
45; a pull over 0.3 N draws the print from under it against 0.2 N (0.75 at most). In the 3 mm slot the print cannot
lift it that far. The rollers' nip closes behind the print: lens up, it rests on it.

From the runs' logs (hinge torque from its angle and speed; force = torque / lever):

| Step | Travel | Force |
|---|---|---|
| demo: shield down, print still | 0 | rod 0.25 to 0.26 N (5 to 95 % of steps) |
| demo, lens up: a fingertip lifts the shield 8 to 10 mm from its hinge; it rolls back | 0 to 15; to 45 degrees in 0.2 s | 1.2 to 4.3 mN m: 0.12 to 0.53 N |
| demo: print drawn out, shield up | 0 to 60 mm | its weight off the nip, 0.03 N |
| trial: pinched, drawn from under the shield (raised to 12 degrees) | 0 to 16 mm | rod 0.15 N sliding: friction 0.20 N |

## Key figures

| Figure | Value | Source | Constant |
|---|---|---|---|
| Camera size; mass | 105 x 84 x 62 mm; 239 g | maker (https://www.polaroid.com/products/go-polaroid-camera-generation-2): "105mm x 84mm x 62mm", "239 grams" "(without film pack)" | `build.py` |
| Film pack | 35 g | estimate: half of a double pack listed at "2.6 Ounces" (74 g; https://www.target.com/p/-/A-81961617) | `build.py` |
| Print | 66.6 x 53.9 mm; 3.5 g | maker (https://www.polaroid.com/products/color-go-instant-film): "Format: 66.6mm x 53.9mm"; estimate: pack less 7 g cassette, over 8 | `PRINT_MASS` |
| Slot over the bench; high | 5; 3 mm | estimate: the maker's product photo, scaled by the 62 mm height; the card and 0.5 mm | `FLOOR`, `GAP` |
| Out when let go; 5 s; shield lifted; face down; out in | 1.5 s | maker's manual and guide (above); estimate: 1.5 s | `SHIELD_S`, `EJECT_S` |
| Print left in the slot | 10 mm | estimate: just past the rollers | `INNER` |
| Shield: flap; rod; lower lip | 18 x 76 mm; 4.5 mm; 6 mm | estimate: over the print's first 18 mm, 11 mm past its sides | `build.py` |
| Shield lifted past; rolls back to | 15; 45 degrees | estimate: a gentle lift; clear of the lens | `LIFT_DEG` |
| Pull to start; moving | 0.3; 0.2 N | estimate: a light hold, 9 and 6 times the print's weight: the spring's 0.23 / 0.15 N x friction 1.3 | `HOLD_N` |
| Nearest subject; flash reach | 45 cm; 2 m | maker's manual: "at least 45cm", "flash reaches 2m" | `AIM_DIST` |
| Apple distance | 50 cm | the maker's 45 cm and 5 cm for placing | `APPLE_DIST` |
| Shutter travel; force | 2 mm; 4 N | estimate: a firm two-stage button | `SHUTTER_TRAVEL` |

The apple on the print (estimate): at 50 cm the maker's 48.1-degree field is 44.6 cm wide, so the 7 cm apple spans
about 7 mm of the 47 mm picture: the one red round shape on the plain bench, it reads as the apple.

## First minutes of a real trial

About 10 minutes, one sheet: camera, pack and print on a kitchen scale; the button's travel (calliper) and force
(kitchen scale under a finger); a phone video beside a ruler: how far the print comes out, how much the shield covers,
how far it lifts before it rolls back (protractor app); a luggage scale on a binder clip at the print's border, shield
down: the pull that starts it, keeps it moving, the travel until free.

## What stands in, what is set directly

The shield is a rigid flap, the real one a rolled film; the rollers' nip is a block that closes behind the print;
the print a rigid 2.5 mm card; no development, no flash; the apple fixed (putty). Set directly, nothing else: the
print's pose until it is out (5.7 cm out over 1.5 s; its weight carried, the reaction on the camera); the shield
spring's rest angle; the nip.

## Ways round

- Pressing with the camera carried, tilted, turned away, or nearer than 45 cm: wasted exposure, 0.
- A pinch, or the shield lifted under 15 degrees: the print stays. The shield lifted before 5 s or after the print
  is drawn free: 0.75; face up: 0.5.
- With the camera standing the slot is too low for fingers to close across the print.

## The hands and the bench

The camera is held across its 84 mm width, lens up; a fingertip lifts the shield beside the print; the print is
pinched, drawn up, stood on its end and let go: it falls face down. Real bench: camera and apple on marks (0.5 cm), a
new sheet per trial, reset 1 minute. `start_top.png`: the policy's 320 x 176 top view.

## How it differs

On this board a photo with a single-use camera ends at the press, as does DexJoCo's "Bimanual Photograph" (no
print); RoboCasa and LIBERO press buttons on fixed appliances. None lifts a flap to draw something out of a device
held in the other hand.

## Review TK-N289RM

Thank you for the review.

- "The simulated rollers release the print as soon as it is pinched, and the stated drag is applied to the camera
  but not the print." The maker puts the hold in the film shield, now a part: friction on the print, reaction on the
  camera; a pinch frees nothing; lifting the shield ends the hold.
- "The scripted demo does not demonstrate pulling the print through the specified 2.2 cm of roller engagement
  against resistance." The maker's rollers push the print through; the 2.2 cm is gone. The demo does the maker's
  handling (5 s, shield lifted, print drawn out); a trial draws it from under the shield (table).
- "Validate the print's ejection, film-shield handling and any roller retention or drag using the named camera and
  film." Not with the camera in hand: the maker's guide and manual stand in; the hold, the shield's size and angles
  and the print's place are estimates, for the first minutes to check.
- "Then provide a full-score demo with the corrected mechanism, and confirm that the apple is recognizable at the
  specified camera distance." Demo 1.0 (below). The apple stands at 50 cm (the maker: at least 45 cm), about 7 mm of
  the picture (estimate); a developed print in the first minutes confirms it.

## Models

The camera and the print: our geometry, plain colours. The apple is `apple_02` from Isaac Lab-Arena's library (NVIDIA's RoboLab assets): "CC BY-NC-SA 4.0 | ©
2026 NVIDIA Corporation" (https://github.com/NVlabs/RoboLab/blob/main/THIRD_PARTY_NOTICES.md): the loader is
Apache-2.0.

## Code and runs

In `code/tasks/h07_instant_photo/`: `task.py` (rules, score), `photo_geometry.py`, `photo_demo.py` (scripted run);
`code/tasks/h07_instant_photo/tests/test_rules.py`; `code/tasks/h07_instant_photo/assets/build.py` (the models).

```bash
uv pip install --python $ARENA_DIR/.venv/bin/python --no-deps -e code
(cd code && python -m pytest -q tasks/h07_instant_photo/tests)  # 35 passed
inspect-robots run --task rta/instant_photo -T num_scenes=1 --embodiment yam_memory -E control_hz=20 --policy rta_oracle --store-frames --log-dir out/demo
```

The model run: the same with `-E cam_width=320 -E cam_height=176 --policy dream`.

Scores: `q_score`, `rta_success`, `milestone/<name>`; `-E trace_dir=<dir>`: the per-step log. Every seed gives the one
fixed start; 7 to 8 minutes a trial (RTX 4090). Scripted run: 1.0 in 917 steps (45.85 s); photo 4.6 s, out 7.45 s,
shield lifted 23.9 s (right wrist view), drawn free 32.45 s, face down 42.05 s.
