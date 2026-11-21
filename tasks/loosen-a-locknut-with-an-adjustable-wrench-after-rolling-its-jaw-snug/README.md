# Loosen an M10 nylon-insert locknut with an adjustable wrench: roll the jaw snug onto the flats, then turn

**Instruction** (`task.py`): "Roll the adjustable wrench's thumbwheel until its jaw is snug on the nut's flats, loosen
the nut at least 60 degrees counterclockwise with it, and let go of the wrench, leaving it on the nut."

## The hard part: sizing the jaw to the nut

A Bahco 8071 adjustable wrench lies round an M10 nylon-insert locknut with its jaw open 23 mm, wider than the nut's
corners (18.9 mm): pulled as it is, it swings round the nut (0). First a fingertip rolls the knurled thumbwheel about
two turns (2.9 mm a turn), without pushing the wrench off the nut, until the jaw is on the flats. A loose jaw (over
17.5 mm) still turns this light nut after a free swing but bears on the corners, which rounds them or slips at real
loosening torques (CCOHS, key figures): 0.25. Then the turn, one flat (60 degrees) against the nylon insert; the
wrench is left on the loosened nut, as a person pausing would.

## Scene

Positions in m from the midpoint between the arm bases (+x away from the arms, +y left, z up from the bench); angles in
degrees counter-clockwise from +x seen from above. One fixed layout (`object_jitter=0`; nothing depends on the seed).
Both arms at ready. Start: `start_top.png`; `start_top_policy.png` as the policy sees it (320 x 176).

| Item | Product | At |
|---|---|---|
| Plate | steel, 330 x 340 x 8 mm, painted | centred at (0.250, -0.050) |
| Screw | ISO 4017 M10 x 25, steel 8.8, screwed up through a tapped hole in the plate (17 mm above it) | (0.280, 0.060) |
| Washer | DIN 9021 M10, 30 x 2.5 mm, round the screw | on the plate |
| Nut | a fresh DIN 985 M10 for every trial, steel 8, zinc plated, 17 mm across flats, run down onto the washer with a wrench, not tightened; a paint mark across its top and the washer | on the washer |
| Wrench | Bahco 8071, jaw set to 23 mm on its scale; round the nut with its fixed jaw flat on one flat and the nut against the throat, head on the washer, handle at 210 deg (back and to the right), its end on the plate | knurl at (0.26, 0.04) |

## Scoring (`score()` in `task.py`; the same numbers in `task.json`)

**Snug on the nut** (every level): the jaw at most 17.5 mm on its scale with the nut seated between the jaws: its axis
inside the jaw opening, each jaw face within 0.5 mm of the flat facing it, the head at the nut's height (the jaws on at
least 3 mm of the flats). Judge: the top photo and video show the wrench sitting on the nut, and a 0.5 mm feeler gauge
does not pass between either jaw and its flat. A jaw closed away from the nut is not snug.

Whole-trial 0: the nut is turned clockwise (tightened) past 10 degrees (paint mark, trial video). At the end:

- **1.0**: the nut loosened at least 60 degrees counterclockwise (one flat past its mark), the jaw snug on the nut and
  set at most 17.5 mm as the turn began, the wrench flat (within 15 degrees) on the plate, both grippers at least 30 mm
  from wrench and nut, wrench and nut still for 1.0 s (trial video; 1 mm and 0.5 degrees in the simulator).
- **0.5**: the nut loosened 60 degrees with the jaw snug on the nut and set at most 17.5 mm as the turn began, the rest
  not met (a hand still on the wrench, the handle held up).
- **0.25**: the jaw snug on the nut with the nut turned less than 60 degrees, or the nut loosened 60 degrees with a
  loose jaw (set over 17.5 mm as the turn began, or not snug on the nut at the end; the setting is read as the nut
  first passes 10 degrees, so a pull begun loose stays loose).
- **0**: otherwise (an open jaw swings round the nut; a jaw closed away from the nut).

The jaw is read on the wrench's own scale (the maker: "Scale for pre-setting jaw opening and for measuring bolt heads
and nuts"); a loose pull shows on the video as a free swing of the handle before the mark moves (12.4 degrees at
18.4 mm, 3.7 at 17.5; `checks.py`).

## Mechanics in the simulation

| Stated force | Where (`task.py`) | Acts on, reaction on | What ends it |
|---|---|---|---|
| Prevailing torque 1.5 N m | `step()` 1: hinge `nut_thread`, a stiff spring capped at 1.5 N m whose rest follows the nut (2 deg give) | the nut; the screw (world) | more than 1.5 N m turns it; it holds where left |
| Jaw clamp, at most 4 N | `step()` 2: slide `jaw`, a spring capped at 4 N, its rest set by the knurl's turns (2.9 mm a turn) | the movable jaw; the wrench | the knurl rolled back |
| No back-drive | `step()` 3: the jaw's opening stop at the tightest place it reached, plus 0.2 mm | the movable jaw; the wrench | the knurl rolled to open |
| Worm locked under load | `step()` 2: while the nut presses the jaw with over 20 N, knurl turns do not move it; a stiff spring (1e6 N/m, at most 500 N) holds it where the load found it | the movable jaw; the wrench | the load falls under 20 N |
| Knurl drag | `parts.py`: free hinge `knurl`, damping 0.002 N m s/rad (0.8 N at its rim at 20 mm/s) | the knurl; the wrench | (none) |
| Torque on the nut | the jaws' contacts on the nut only (sensors `movable_jaw`/`wrench` on `nut`) | the nut; the wrench | the wrench is let go |

No force is applied by a rule: the turn reaches the nut only through the jaws.

## Key figures and their sources

| Figure | Value | Source |
|---|---|---|
| Wrench length, jaw capacity, head thickness, jaw tip, weight | 205 mm, 27.0 mm, 12.9 mm, 6.0 mm, 255 g | maker's catalogue: Bahco ADJ-1602-ENG p. 11, dimensions A-D, https://docs.rs-online.com/ab1a/A700000014532817.pdf |
| Jaws angled | 15 deg | maker's catalogue, same document ("Jaws angled at 15°") |
| Nut across flats, across corners (min), height | 17.0 (16.73-17.00), 18.9, 9.64-10.0 mm | Böllhoff data sheet "DIN 985 similar to ISO 10511", https://eshop-hu.boellhoff.com/out/media/pdf/DIN_985_Stahl_08_gv_Feingewinde___en.pdf |
| A fresh nut for every trial | M10 x 1.5, 17 mm across flats, 100 a pack | listing: Würth 368.10, "M10-1.5 MC NYLON LOCKNUT CLASS 8.8 ZN DIN985", https://shop.wurth.ca/fasteners/nuts/nylon-insert-lock-nuts/metric-hexagon-nylon-lock-nuts/M10-1.5-MC-NYLON-LOCKNUT-CLASS--8.8-ZN-DIN985/368.10/; ISO M10 is 16 mm (Würth, "DIN-ISO changes to widths across flats") |
| Snug: jaw at most | 17.5 mm | CCOHS, Hand Tools - Wrenches: "Do not pull on an adjustable wrench that is loosely adjusted." https://www.ccohs.ca/oshanswers/safety_haz/hand_tools/wrenches.html; 17.5 = the 17.0 mm flats + the scale's half millimetre |
| On the nut: a jaw face to its flat | at most 0.5 mm | the 17.5 mm snug jaw less the 17.0 mm flats (a 0.5 mm feeler gauge); the solver's overlap (at most 0.18 mm in the runs) counts within it |
| On the nut: jaws on the flats | at least 3 of 8 mm | estimate: 5.9 mm with the head on the washer, so a wrench raised 3 mm is off |
| Prevailing torque, first removal | at least 1.5 N m (first installation at most 10.5) | standard: ISO 2320:2015 Tables 1, 3, 4 (M10) |
| Washer | 30 x 2.5 mm, 10.5 mm bore | standard: DIN 9021 M10 (listing: RS PRO 797-6254) |
| Knurl size and pitch | about 12 mm across, 11 mm long, 2.9 mm of jaw a turn | estimate: scaled from the catalogue's dimension drawing (A = 205 mm as the scale; 4 thread crests over 10.7 mm) |
| Jaw depth, jaw thickness at the nut, handle section | 23 mm, 9 mm, 21 x 8 mm | estimate: the same drawing (the jaws taper from D 12.9 to C 6.0 mm) |
| Snug clamp | 4 N | estimate: a fingertip's 1 N on the 7 mm knurl through a 2.9 mm worm at 30 % |
| Worm play | 0.2 mm | estimate |
| Wrench's middle of mass | from its shapes at one density | estimate |

## Fidelity: what the simulation stands in for

- The task stops at the loosened nut: the simulator cannot slide a snug jaw off the hex without dragging it (40 to
  60 N, gw11-gw14), so the wrench stays on the nut.
- Jaw friction on the nut is the kit's default (1.0): at dry steel's 0.2 the solver let the jaws roll round the nut
  (gw9); part of the torque goes by friction, where a real nut takes it on its flats.
- The knurl is 13.9 mm across, 0.5 mm proud of both faces, so the rigid fingertip reaches it; it collides with fingers
  only and turns the jaw only under a finger.
- The nut is carried at 0.25 kg on its hinge and the movable jaw at 80 g (of 255 g), with raised inertias: an 11 g nut
  let the jaws sink 0.8 mm into its corners (gw7), these 0.03 mm (gw8). The worm's lock is a stiff spring (the joint's
  stop gave 0.2 mm under 30 N, gw4).
- The nut rides 0.3 mm over the washer (no washer friction); one prevailing torque stands for breakaway and running;
  the nut does not rise on its thread.

## First minutes of a real trial (about 7 minutes)

- Knurl: caliper on its diameter; roll it 10 turns from 23 mm and read the jaw (pitch). 2 min.
- Prevailing torque: luggage scale in the wrench's hang hole, pulled square through 60 degrees on a fresh nut
  (F x r). 2 min.
- Snug clamp: kitchen scale between the jaws, the knurl rolled with a fingertip until it stops. 1 min.
- Reset: run the used nut off and a fresh one down with the wrench until it touches the washer, paint its mark in line
  with the washer's, set the jaw to 23 mm, lay the wrench round the nut, handle at 210 degrees. About 2 min.

## Runs

- **Reference run** (`--check` gw21, recording gw22): 1.0 at 49.5 s of the 90 s limit. The knurl rolled 2.4 turns in
  8 strokes (jaw 23.0 to 17.03 mm on the nut: 0.25 from 18.4 s); 12 to 14 N on the handle at 160 mm (about 2 N m
  against 1.5) turned the nut 77 deg, 90 N on each jaw.
- **Levels** (`--check`): open jaw 0 (`gw4`); open wrench slid off the nut, jaw then closed to 17.2 mm, 0 (`gw21`);
  loose jaw 18.6 mm, nut 62 deg, 0.25 (`gw10`); hand left on, 0.5 (`gw21`); clockwise past -10 deg, 0 (`gw16`).
  Known torque (`gw3`): holds 1.4 N m, slips at 1.6.

## Files and commands

`task.py` rules, `score()`; `parts.py` figures, shapes; `demo.py` runs; `checks.py` IK. Tests:
`scripts/rta check mech/tasks/adjustable_wrench`; runs: `scripts/rta demo <id> mech/tasks/adjustable_wrench full`.
