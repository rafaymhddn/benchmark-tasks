# Loosen an M10 nylon-insert locknut with an adjustable wrench: roll the jaw snug onto the flats, then turn

**Instruction** (`task.py`): "Roll the adjustable wrench's thumbwheel until its jaw is snug on the nut's flats, loosen
the nut at least 60 degrees counterclockwise with it, and let go of the wrench."

## The hard part: sizing the jaw to the nut

A Bahco 8071 adjustable wrench lies round an M10 nylon-insert locknut with its jaw open 23 mm, wider than the nut's
corners (18.9 mm): pulled as it is, it swings round the nut and scores 0. Before it can be used, the jaw has to be
sized to the nut with the knurled thumbwheel: about 12 mm across, sunk in a window in the 12.9 mm head, rolled by a
fingertip pressed on it, about two turns at 2.9 mm of jaw a turn, without pushing the wrench off the nut, and stopped
when the jaw is on the flats. A jaw left loose (over 17.5 mm) still turns this light nut after a free swing, but it
bears on the corners, which rounds them or slips at real loosening torques (CCOHS: "Do not pull on an adjustable
wrench that is loosely adjusted"): that scores 0.25. Only then the turn, at least one flat (60 degrees) against the
nylon insert's prevailing torque. The task ends at the loosened nut with both hands off, as a person pausing before
the next turn would.

## Scene

Positions in m from the midpoint between the arm bases (x forward, y left, z up from the bench); one fixed layout
(`object_jitter=0`; nothing depends on the seed). Both arms start at ready. Start: `start_top.png`, and
`start_top_policy.png` as the policy sees it (320 x 176).

| Item | Product | At |
|---|---|---|
| Plate | steel, 330 x 340 x 8 mm, painted | centred at (0.250, -0.050) |
| Screw | ISO 4017 M10 x 25, steel 8.8, screwed up through a tapped hole in the plate (17 mm above it) | (0.280, 0.060) |
| Washer | DIN 9021 M10, 30 x 2.5 mm, round the screw | on the plate |
| Nut | DIN 985 M10, steel 8, zinc plated (17 mm across flats), run down onto the washer with a wrench, not tightened; a paint mark across its top and the washer | on the washer |
| Wrench | Bahco 8071, jaw set to 23 mm on its scale; round the nut with its fixed jaw flat on one flat and the nut against the throat, head on the washer, handle pointing back and right (150 deg from +x), its end on the plate | knurl at (0.26, 0.04) |

## Scoring (`score()` in `task.py`; the same numbers in `task.json`)

Whole-trial 0: the nut is turned clockwise (tightened) past 10 degrees (paint mark, trial video). At the end:

- **1.0**: the nut loosened at least 60 degrees counterclockwise (one flat past its mark), the jaw at most 17.5 mm on
  its scale (a 0.5 mm feeler leaf beside the scale must not pass between a jaw and its flat), the wrench flat (within
  15 degrees) on the plate, both grippers at least 30 mm from wrench and nut, wrench and nut still for 1.0 s (trial
  video; 1 mm and 0.5 degrees in the simulator).
- **0.5**: nut loosened 60 degrees with the jaw at most 17.5 mm, the rest not met (a hand still on the wrench, the
  wrench held up or put off the plate).
- **0.25**: the jaw at most 17.5 mm with the nut turned less than 60 degrees, or the nut loosened 60 degrees with the
  jaw over 17.5 mm at the end or as the turn began (its setting is read as the nut first passes 10 degrees, so a pull
  begun loose stays loose, whatever is rolled after).
- **0**: otherwise (an open jaw swings round the nut).

**Why "snug" is scored.** Any jaw narrower than the nut's corners turns this nut, after a free swing (3.7 degrees at
17.5 mm, 12.4 at 18.4 mm, 22 at 18.9 mm; `checks.py`). A jaw left loose bears on the corners and rounds them at real
loosening torques (CCOHS, key figures); a snug jaw bears on the flats. The judge reads the jaw on the wrench's own
scale (the maker: "Scale for pre-setting jaw opening and for measuring bolt heads and nuts"), the nut's turn on its
paint mark; a loose pull shows on the video, with the head covered, as a free swing of the handle before the mark
moves (12.4 degrees at 18.4 mm against 3.7 at 17.5 mm).

## Mechanics in the simulation

| Stated force | Where (`task.py`) | Acts on, reaction on | What ends it |
|---|---|---|---|
| Prevailing torque 1.5 N m | `step()` 1: hinge `nut_thread`, a stiff spring capped at 1.5 N m whose rest follows the nut (2 deg give) | the nut; the screw (world) | the nut turns while more than 1.5 N m acts; it holds where left |
| Jaw clamp, at most 4 N | `step()` 2: slide `jaw`, a spring capped at 4 N, its rest set by the knurl's turns (2.9 mm a turn) | the movable jaw; the wrench's body | the knurl rolled back |
| No back-drive | `step()` 3: the jaw's opening stop at the tightest place it reached, plus 0.2 mm | the movable jaw; the wrench's body | the knurl rolled to open |
| Worm locked under load | `step()` 2: while the nut presses the jaw with more than 20 N, knurl turns do not move the jaw and a stiff spring (1e6 N/m, at most 500 N) holds it where the load found it | the movable jaw; the wrench's body | the load falls under 20 N |
| Knurl drag | `parts.py`: free hinge `knurl`, damping 0.002 N m s/rad (0.8 N at its rim at 20 mm/s) | the knurl; the wrench's body | (none) |
| Torque on the nut | contacts of the jaws on the nut only (contact sensors `movable_jaw`/`wrench` on `nut`) | the nut; the wrench | the wrench is let go |

No force is applied by a rule: the turn reaches the nut only through the jaws.

## Key figures and their sources

| Figure | Value | Source |
|---|---|---|
| Wrench length, jaw capacity, head thickness, jaw tip, weight | 205 mm, 27.0 mm, 12.9 mm, 6.0 mm, 255 g | maker's catalogue: Bahco ADJ-1602-ENG p. 11, dimensions A-D, https://docs.rs-online.com/ab1a/A700000014532817.pdf |
| Jaws angled | 15 deg | maker's catalogue, same document ("Jaws angled at 15°") |
| Nut across flats, across corners (min), height | 17.0 (16.73-17.00), 18.9, 9.64-10.0 mm | Böllhoff data sheet "DIN 985 similar to ISO 10511", https://eshop-hu.boellhoff.com/out/media/pdf/DIN_985_Stahl_08_gv_Feingewinde___en.pdf |
| Snug: jaw at most | 17.5 mm | CCOHS, Hand Tools - Wrenches: "Do not pull on an adjustable wrench that is loosely adjusted." https://www.ccohs.ca/oshanswers/safety_haz/hand_tools/wrenches.html; 17.5 = the 17.0 mm flats + the half millimetre the scale is read to |
| Prevailing torque, first removal | at least 1.5 N m (first installation at most 10.5) | standard: ISO 2320:2015 Tables 1, 3, 4 (M10) |
| Washer | 30 x 2.5 mm, 10.5 mm bore | standard: DIN 9021 M10 (listing: RS PRO 797-6254) |
| Knurl size and pitch | about 12 mm across, 11 mm long, 2.9 mm of jaw a turn | estimate: scaled from the catalogue's dimension drawing (A = 205 mm as the scale; 4 thread crests over 10.7 mm) |
| Jaw depth, jaw thickness at the nut, handle section | 23 mm, 9 mm, 21 x 8 mm | estimate: the same drawing (the jaws taper from D 12.9 to C 6.0 mm) |
| Snug clamp | 4 N | estimate: a fingertip's 1 N on the 7 mm knurl through a 2.9 mm worm at 30 % |
| Worm play | 0.2 mm | estimate |
| Wrench's middle of mass | from its shapes at one density | estimate |

## Fidelity: what the simulation leaves out or stands in for

- The task stops at the loosened nut: the simulator cannot slide a snug jaw off the hex without dragging it (the
  jaws dragged it at 40 to 60 N, runs gw11-gw14), so the wrench stays on the
  nut, as a person pausing before the next turn would leave it.
- The jaws' friction on the nut is the kit's default (1.2 static, 1.0 dynamic): with dry steel's 0.2 the jaws needed
  about 200 N on the flats' ends and the solver let them roll round the nut (gw9), so in the simulation part of the torque goes by friction, where a real nut takes it on its flats.
- The simulated knurl is 13.9 mm across, 0.5 mm proud of both faces, so the rigid fingertip can reach it (a real pad
  presses into the window); it collides with fingers only, and its turns move the jaw only while a finger touches it
  (an untouched knurl crept while the wrench turned).
- The nut is carried at 0.25 kg on its hinge and the movable jaw at 80 g of the wrench's 255 g, with raised turning
  inertias: with an 11 g nut the jaws sank 0.8 mm into its corners (gw7), with these 0.03 mm
  (gw8). The worm's lock is a stiff spring (the joint's stop alone gave 0.2 mm under 30 N, gw4).
- The nut rides 0.3 mm over the washer on its hinge (no washer friction adds to its torque); one prevailing torque
  stands for breakaway and running; the nut does not rise on its thread (0.25 mm in 60 degrees).

## First minutes of a real trial (about 6 minutes)

- Knurl: caliper on its diameter and the window; roll it 10 turns from 23 mm and read the jaw (pitch). 2 min.
- Prevailing torque: luggage scale in the wrench's hang hole, pulled square at a measured radius through 60 degrees,
  on the nut as set (F x r). 2 min.
- Snug clamp: kitchen scale between the jaws, the knurl rolled with a fingertip until it stops. 1 min.
- Reset: back the nut onto the washer with the wrench until it touches, line up its mark, set the jaw to 23 mm,
  lay the wrench at 150 degrees from +x as in the start photo. About 40 s.

## Runs

- **Reference run** (`--check` and recording): 1.0 by success at 49.5 s of the 90 s limit (the rest is room for a
  learned policy). The knurl rolled 2.3 turns in 8 strokes (jaw 23.0 to 17.05 mm); the hand pushed the handle round
  with 12 to 14 N at 160 mm (about 2 N m against 1.5): the nut turned 77 deg, 89 N on each jaw.
- **Levels** (`--check`): open jaw 0 (`gw4`); loose jaw 18.6 mm, nut 62 deg, 0.25 (`gw10`); hand left on, 0.5
  (`gw18`); clockwise past -10 deg, 0 (`gw16`). Known torque on the nut (`gw3`): holds 1.4 N m, slips at 1.6.

## Files and commands

- `task.py` rules, `score()`; `parts.py` figures, shapes, joints; `demo.py` the runs;
  `checks.py` IK and geometry; `test_adjustable_wrench.py` rule tests.
- `scripts/rta check mech/tasks/adjustable_wrench`; `checks.py` (in the container);
  `scripts/rta demo <id> mech/tasks/adjustable_wrench full [--check]`.
