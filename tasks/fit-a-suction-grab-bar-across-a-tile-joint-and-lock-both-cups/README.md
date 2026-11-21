# Re-mount a suction grab bar pressed on over the grout line: hold it, release both cups, move it onto tile, relock

Instruction: `Fix the suction grab bar on the tiled panel so that it holds.` Category: Safety and alignment.

## Why it matters and how it differs

Suction grab bars are a hand-hold for balance at the bath, basin or WC, not for a body's weight ([drive],
[sanismart]). They hold only on smooth, airtight surfaces ([wenko22]), both cups wholly on tile at least 1/2 in from
grout lines or joints, checked green before each use ([drive]): a cup over the grout lets air in and the bar comes off
when leaned on. Here the right cup was pressed on across the joint (red); the left holds (green). Releasing a cup
drops the bar and a lever's flip pushes on it, so one hand holds the bar while the other works the levers. Suction
lifters carry a pane; here the bar stays on the wall while its cups are released and set again.

## Setup (`parts.py`, `shapes.py`)

Station bench; m from the midpoint between the arm bases, `x` towards the wall, `y` left, `z` up. One fixed layout:
`--seed 0` under the harness gives trial seed `1415158438`, the same start for every seed.
- **Panel** (a test fixture, placed by us): two white glossy wall tiles 25 x 40 x 0.8 cm ([toom]) on an 18 mm multiplex
  board cut to 520 x 400 mm ([board]), a 3 mm open joint ([spacers]), on two steel angle brackets. Tile face
  at `x = 0.34`, joint upright at `y = 0`.
- **Bar:** Wenko Wandgriff Secura, 29 cm, white, no. 7820100 ([wenko]; on sale as "Wenko Wannengriff Secura 290 mm",
  [obi]; US twin Drive Medical RTL13083, [drive]): two 95 mm cups, each locked by a flip lever at its housing's outer
  end, a red/green tab on top. Start: level, its middle 70 mm over the bench (every grasp at 74 mm or lower), 60 mm
  left of the joint: the right cup across it, both levers locked. Arms at the ready pose.

**Mechanism.** Each lever turns on an upright pin at its housing's outer end, from `0` (released, swung out) to
`110 deg` (locked, along the housing's end, its tip to the wall), with a cam: loose up to `70 deg`, rising
to `22 N` at `32 mm` out (`0.70 N m`) at centre (`95 deg`), `4.4 N` holding it locked. A cup's seal is a weld to the
panel, made in the step its lever passes centre towards locked if the cup lies flat on the tile (`3 deg`, `1 mm`), was
pressed on with `12 N` or more within `0.2 s`, and its rim lies on one tile `12.7 mm` (1/2 in) or more from the joint
and `5 mm` from the tile's outer edges; cut when the lever comes back over centre. Start: left sealed, right not.

## Scoring (`task.py`)

Zero for the whole trial if the bar touches the bench. At the end, both grippers `30 mm` or more from the bar, bar and
levers at rest (the trial ends early once 1.0 has held 2 s): **1.0** both cups clear (rims `12.7 mm` from the joint,
`5 mm` from the tiles' outer edges), both sealed, the bar within `10 deg` of level; **0.5** both clear, one sealed;
**0** otherwise (the start scores 0). Shortcuts: leave the bar, press harder over the joint, drop it and set it
again: 0. On the bench: a ruler from each rim to joint and edges, the tabs, a level app on the grip; the bench touch
from the trial's video.

## Can it be seen

At 320 x 176 the top camera shows the red tab as 3 x 2 px (`start_top_policy.png`; `start_top.png`). The right wrist
camera, turned to the right cup from 196 mm by the demo's first move, shows it 15 x 12 px, the dark joint
running into the rim (`cue_red_tab_right_wrist_policy.png`; `cue_red_tab_right_wrist.png`).

## Key figures and their sources (`parts.py` `FIGURES`)

20 figures: 2 maker, 5 listing, 13 estimate with their derivations; none checked on a bought bar.

| Figure | Constants | Value | Source |
|---|---|---|---|
| Bar size | `BAR_L/W/H` | 290 x 115 x 85 mm | maker: "Maße (B x H x T): 29 x 11.5 x 8.5 cm" [wenko] |
| Bar mass, cups | `BAR_MASS`, `CUP_D` | 265 g, 95 mm | listing: "29 cm: 265 g", "Saugnäpfe 9,5 cm" [sanismart] |
| Joint margin | `JOINT_MARGIN` | 12.7 mm | maker (US twin): "at least 1/2” away from all grout lines or joints" [drive] |
| Tiles; joint; board | `TILE_*`, `JOINT_W`, `BOARD_T` | 250 x 400 x 8; 3; 18 mm | listing [toom], [spacers], [board]; DIN 18352: 2 to 8 mm joints [din] |
| Spacing; sections | `CUP_PITCH`; `CUP_T`, `HOUSING_*`, `GRIP_D` | 181; 8, 70 x 32, 38 mm | estimate: the levers at the 290 mm ends; the 85 mm depth |
| Lever, pivot, travel | `LEVER_L/WIDTH/T/ARM/PIVOT`, `LEVER_*DEG` | 45 x 30 x 5, 32 mm; 110, 95, 70 deg | estimate: an over-centre flip lever at each end (makers' photos) |
| Lever force, held | `LEVER_N`, `LEVER_HOLD` | 22 N, 4.4 N | estimate: a firm thumb's push ("must be clearly felt" [drive]) |
| Press a seal needs | `PRESS_N`, `PRESS_S` | 12 N in 0.2 s | estimate: a hand's press seals, a touch does not |
| Hold; across the joint | `HOLD_N`, `HOLD_JOINT_N` | over 40 N; under 3 N | estimate: under the listed 30 kg [sanismart]; air gets in |
| Edge margin; tabs | `EDGE_MARGIN`; `INDICATOR_*` | 5; 16 mm | estimate: the lip on glaze; "Ampelanzeige ... OK=Grün" [wenko22], on top |
| Masses; model values | `*_MASS`; `FACE_*`, frictions, `LEVER_CONTACT` | 10, 20 g; 3 deg, 1 mm | estimate: plastic, 2 mm rubber; tolerances |

**First minutes of a real trial** (about 25 min): build the panel, caliper on tiles and joint (10 min); scale and
caliper on bar, cups, levers (3 min); protractor app on a lever's stops, centre and cam, luggage scale 32 mm out on it
(3 min); panel on a kitchen scale, press the bar on until a lever seals (2 min); luggage scale on the grip: sealed to
40 N, across the joint, a cup moved to the joint in 2 mm steps until it no longer holds (5 min).

## Forces and their reactions

| Force | Code | Acts on | Reaction on | Check run `gb5d22` |
|---|---|---|---|---|
| Lever cam | hinge drive (`lever.py`) | lever | bar | up to 22 N, 32 mm out |
| Seal | `weld("seal_*")` | cup | panel | held 40 N, gave 2.6 mm (trial `gb5t8`) |
| Cup press | contact | cup | panel | 21.4 / 23.7 N at sealing |
| Releasing hand | side pinch | lever | bar, via the hinge | pulling 17 / 19 N at centre |
| Locking hand | fingertips on its outer face | lever | bar, via the hinge | pushing 16 / 17 N at centre |
| Holding hand | pinch on the grip | bar | the arm | 15 N in, 3 N up; 20 N in while a cup locks |

`task.py` pushes nothing; the demo moves the arms; trials push the bar or a lever.

## The demo (`demo.py`) and why the order is forced

Each arm works its own side's lever; a hand holds the bar whenever no cup does. The left cup holds at the start, so
the right lever goes first (after a look at the right cup); the right hand then holds the grip while the left lever is
released (unheld, the bar drops: trial `gb5t8`), moves the bar and lets go once the left cup holds; the left hand then
presses the grip while the right lever locks. A release pinches the lever from the side and pulls it along its
path, a step ahead (a slip: pinched again, 3 tries). A lock pushes the lever's outer face home with closed fingertips
while the other hand presses the bar on near that cup with `20 N`. Lever hands' wrist cameras stay `100 mm` or more off
the tiles (`checks.py`). Lever log, `gb5d22` (forces: most in the step):

| t s | step | lever deg (l / r) | hand | fingers N | press N (l / r) |
|---|---|---|---|---|---|
| 8.50 | right releases r | 110.0 / 95 | pull 19 N | 50.0 | 0 / 20.1 |
| 21.40 | left releases l, seal cut | 95 / 56.8 | pull 17 N | 50.0 | 31.8 / 6.6 |
| 29.05 | bar moved 58.4 mm | 55.3 / 58.6 | 20 N in | 24.1 | 12.9 / 7.2 |
| 36.40 | left locks l: sealed | 110.0 / 63.1 | push 16 N | 0.5 | 21.4 / 7.6 |
| 51.20 | right locks r: sealed | 110.0 / 110.0 | push 17 N | 1.5 | 2.3 / 23.8 |

End: bar `0.09 deg` off level, `2.0 mm` higher; rims `43.1` / `39.9 mm` from the joint, `24.6` / `24.3 mm` from the
tile's edge; hands at ready `0.35 s` before the end. `--quick` at worst: IK `0.09 mm`, joint margins
`5.8 deg`, pushes `0.56` of a joint's effort.

## Fidelity

Rigid parts at true size. The 3 mm open joint stands for a grout line: under a rim both let air in, and it shows at
once. The levers sit at the housings' outer ends and the indicator tabs on top, as the makers' photos show. A seal is
a weld, made only as the rule allows; rubber flex, air leakage and the tile's grip on a loaded cup are not modelled.

## Runs and commands

Checks `gb5d3`-`gb5d22`, trials `gb5t0`-`gb5t8`; run limit `2750` steps (`137.5 s`, 2.5 times the demo's `55 s`).
Third-party assets: none (kit primitives). From the repo root:
- `python -m pytest -q mech/tasks/grab_bar/test_grab_bar.py` (13 rule tests); `python mech/tasks/grab_bar/checks.py`
- `python mech/tasks/grab_bar/trials.py <out> rest,unheld,held,pull,twoseals`; `python mech/tasks/grab_bar/demo.py <out> --check`
- `inspect-robots run --embodiment yam_arena --policy <policy> -E task=mech/tasks/grab_bar/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<above>" --scorer task_score --max-steps 2750 --seed 0`

[wenko]: <https://web.archive.org/web/20191211170940/https://shop.wenko.de/bad/sicherheit-komfort/sicherheit-im-bad/wandgriff-secura-7820100>
[wenko22]: <https://web.archive.org/web/20220704091925/https://shop.wenko.de/bad/sicherheit-komfort/sicherheit-im-bad/wandgriff-secura>
[obi]: <https://www.obi.de/p/9808973/wenko-wannengriff-secura-290-mm>
[sanismart]: <https://www.sanismart.de/wandgriff-secura-mit-saugsystem--2821>
[drive]: <https://web.archive.org/web/20181221042355/http://cdn.drivemedical.com/media/files/3583/RTL13083_RTL13085_hangtag_Rev1_04_29_16%20(1).pdf>
[toom]: <https://toom.de/p/wandfliese-arktis-steingut-weiss-glaenzend-25-x-40-cm/2050669>
[spacers]: <https://www.obi.de/p/5412630/lux-tools-fliesenkreuze-3-mm-classic-250-stueck>
[board]: <https://www.obi.de/p/4560678/multiplex-platte-birke-250-cm-x-125-cm-x-1-8-cm-braun-zuschnitt>
[din]: <https://www.sanier.de/bodenbelag/fliesen/fliesen-fast-fugenlos-verlegen>
