# Set a specimen pan across a toilet bowl's opening, both wings on the rim, under the raised seat

Instruction: `Set the specimen pan upright across the toilet bowl opening with both wings on the rim, leaving the raised seat up, then let go and move both hands clear.`

Files: `task.json`; `task.py` (rules, `score()`); `scene.py` (bodies, hinge); `parts.py` (figures, sources); `checks.py`, `trials.py`, `demo.py`; `test_specimen_hat.py`; the kit (`mechkit.py`, `demo_kit.py`, `kit_extra.py`); `start_top.png`, `start_top_policy.png` (320 x 176), `start_left_wrist.png`, `start_right_wrist.png`.

![top camera at the start](start_top.png)

## What the task tests

- **Across, not lengthwise.** The pan is 222.3 x 336.6 mm over its wings, the bowl's opening about 264 x 379 mm. Set down lengthwise it drops into the bowl; turned 90 degrees its wings bridge the opening with 36 mm on the rim each side. The 100 mm deep pan is carried over the 16 cm rim and lowered straight in.
- **Leave the seat up.** The Bemis seat stays up by itself on its check hinges 11 degrees past upright (maker), pressed onto its stop with 0.47 N m. About 1 N at its front, kept on for 7 degrees, tips it past its balance at 94.4 deg and it falls shut: 0, or at most 0.5 if it lands on the pan. The toilet faces the arms with the seat and the wall away from them, so both hands are free for the pan.
- **Why the seat starts raised.** The seat's hinge stands 54 cm in front of the arm bases: in our offline reach checks a hand raised the closed seat's front only 15-20 degrees (12-16 cm), too little for the 10 cm pan and a hand under it. So the seat starts as a caregiver leaves it before setting the pan, and stays so.
- **Impact on jobs.** A specimen pan under the toilet seat is how urine or stool is collected from a patient who can use the toilet. O*NET 31-1131.00 Nursing Assistants (1,505,900 employed, 2025) lists "Collect specimens, such as urine, feces, or sputum." ([O*NET](https://www.onetonline.org/link/summary/31-1131.00)).

## Scene

Positions in cm from the midpoint between the arm bases, x forward, y left. The toilet is a Kohler K-4302 Highcrest elongated bowl's geometry over the rim at the maker's sizes, facing the arms: the rim 36.8 cm across (y -22.4 to +14.4) and 66.0 cm back to front (x 7.0 to 73.0) round the 26.4 x 37.9 cm opening (x 12.2 to 50.2), the inside 15.2 cm deep to the water surface, the wall at x 77.2 (27-5/8 in. from the bowl's front, the 10 in. rough-in). A stand puts the rim 16.0 cm over the bench, room for the 15.2 cm depth: it replaces the bowl's pedestal and the floor only (a real rim is 41.9 cm over the floor). The Bemis 1955CT seat is bolted on the bowl (bolts 5-1/2 in. apart at x 54.1), raised on its check hinges. The Medline pan stands upright on the bench at (28.0, -40.0), lengthwise. The layout is fixed: no seed draws any of it (runs used seed 0). Full text: `task.json` setup.

## Rules and score

0.0 for good if at any step the pan falls into the bowl (its lowest point within 8 mm of the water surface, its middle under the opening) or the seat falls from its stop (under 90 deg with no hand within 25 mm of it and not resting on the pan). Otherwise, at the end of the trial:

- **1.0**: the pan across the opening: upright (tilt <= 8 deg), its wings within 20 deg of square across the opening, its middle inside the opening, both wing tips over the side rims with their undersides at most 5 mm over the rim top; the seat up on its stop (at least 99 deg); every part of both hands at least 30 mm from the seat, the pan and the bowl; all held so for 1.0 s (20 steps) with the pan moving at most 1.5 mm and the seat at most 0.5 deg.
- **0.5**: the pan across the opening as above, any 1.0 condition not met.
- **0.0**: otherwise (a pan on the bench, lengthwise, tilted, off the rim).

On the bench: the pan falling in and the seat falling shut from the video (with times); tilt by phone inclinometer on the pan's rim; wings on the rim by eye and ruler; the seat on its stop by eye; hands and the 1.0 s from the video.

## Key figures and their sources

| Figure | Value | Source |
|---|---|---|
| Seat length, hinge bolts to front | 471.5 mm | maker, [spec sheet](https://www.toiletseats.com/assets/0B7012484_1955CT_SSCT-Bemis-Spec.pdf) "18-9/16"", [page](https://www.toiletseats.com/p/bemis-elongated-open-front-less-cover-plastic-toilet-seat-in-white-that-stays-tight-2) "Depth: 18.563 in" |
| Seat width; opening width; front gap | 368.3; 201.6; 88.9 mm | maker: "14-1/2"" ("Width: 14.5 in"); "7-15/16""; "3-1/2"" |
| Ring thickness; with bumper | 20.6; 28.6 mm | maker: "13/16""; "1-1/8"", "Ring Bumpers: Four" |
| Seat mass; its stop | 1.912 kg; 101 deg | maker: "Weight: 4.216 lbs"; "Check hinges prevent the seat from going 11° beyond vertical" |
| Bowl width; back to front; wall to front | 368; 660; 702 mm | maker, Kohler K-4302 [roughing-in sheet](https://resources.kohler.com/onlinecatalog/pdf/116148_1.pdf): "14-1/2" (368 mm)", "26" (660 mm)", "27-5/8" (702 mm)" |
| Water surface under the rim; water area | 152 mm; 289 x 264 mm | maker: "Water depth from rim 6″ (152 mm)", "Water area 11-3/8″ (289 mm) x 10-3/8″ (264 mm)" |
| Seat posts apart; rim over the floor | 139.7; 419 mm | maker: "Seat post hole centers 5-1/2″ (140 mm)" (Bemis "Bolt Spread: 5.5 in"); "16-1/2" (419 mm)" |
| Opening across; rim width | 263.5; 52.4 mm | estimate: the 10-3/8 in. water area across, the least the opening can be; (14-1/2 - 10-3/8 in.) / 2, taken the same at the front (the sheet's 9-3/4 x 14-11/16 in. are the footprint on the floor) |
| Seat posts to bowl front; opening's back edge in front of them | 471.5; 40 mm (opening 379 mm long) | estimate: the seat's front over the bowl's front; room for the hinge bases |
| Hinge pin over the rim | 35 mm | estimate: about 25 mm under the top of the 2-3/8 in. hinge end |
| Seat outline (hinge end, opening's back, horns) | 240, 60, 400, 160 mm | estimate: scaled from the maker's drawing |
| Bumper rows; offset from the middle | 140, 340; 142 mm | estimate: scaled from the maker's side view |
| Pan across wings; front to back; height | 336.6; 222.3; 100.0 mm | estimate from the [listing](https://www.odpbusiness.com/a/products/458238/Medline-Specimen-Collector-Pans-30-Oz/) "13-1/4 in.", "8-3/4 in.", "3-15/16 in." |
| Pan bowl width; wing depth; wing thickness; mass | 186; 160; 4 mm; 80 g | estimate: the wings 75 mm a side; about 3/4 of 222.3 mm; a 1 mm sheet on a 4 mm rib; 0.147 m2 of 0.6 mm polypropylene |
| Rim over the bench (the stand) | 160 mm | our fixture |

`parts.py` holds 41 figures: 17 maker, 16 estimate, 8 our fixture (the scoring limits among them).

## First minutes of a real trial

About 10 minutes with the bought seat and pan and a K-4302 bowl: the bowl's opening across and along and its rim width with a tape (2 min); the pan's three outside sizes, its bowl width and wing depth with a caliper and ruler (3 min); pan and seat on a kitchen scale (1 min); the seat's pin height and bumper positions with a ruler (2 min); the raised seat's angle on its stop with a phone protractor app and a luggage scale pushing its front over (1 min). For each figure marked estimate, a different reading replaces the estimate.

## Forces

| Force | On | Reaction on | Value |
|---|---|---|---|
| Seat's weight about the hinge | seat | its stop, then the hinge pins | 0.47 N m onto its stop at 101 deg, 0 at 94.4 deg, 4.08 N m closed (centre of mass 218 mm from the pin) |
| Push that tips it over | seat's front | the pusher | about 1.0 N (0.47 N m over its 0.47 m length), kept on for 7 deg |
| Pinch on the 4 mm front wall | pan | right fingers | 10 N a finger sent (2 mm x 5,000 N/m); demo peak 9.8 N |
| Pan on the rim | bowl | pan | 0.78 N (80 g) |

The grip is under 40 N a finger and inside the hand's 94 mm opening. The hand closes 8.8 cm over the bench and carries the pan with its grasp 26.0 cm up (the pan 12 mm over the rim).

## Checks run so far

- `checks.py` (offline): 14 waypoints within 0.073 mm and 0.013 deg, joint margins 29.3-58.0 deg; hand hulls (forearm to fingertips) at least 15.7 mm from bowl, wall and seat (fingers on the held pan excepted); the carried pan at least 12.0 mm over the rim, 25.1 mm from the raised seat and 10.8 mm from the opening's edges going in.
- Rule tests: 11/11 (`test_specimen_hat.py`).
- Demo `--check` (run srvd1): 1.0 (0.5 at 10.45 s, 1.0 at 13.5 s; run limit 45 s); the seat at 101.0 deg throughout, untouched; pan tilt 0.0 deg, wings 0.0 mm over the rim, 0.31 deg off square; hands 111 mm clear.
- Trials (PhysX, `trials.py`, run srvt1): pan put lengthwise into the opening: falls to the water surface, 0 (0.65 s); seat put at 92 deg: falls shut, 0 (0.75 s); seat put at 97 deg: back on its stop by itself, no zero; the hand keeps its pinch on the seated pan: 0.5.
- Start (run srvf2): 2.0 s idle, the pan moves 0.0 mm and the seat 0.0 deg, score 0; the four start pictures in this folder.

## How it differs

Nearest tasks we know of: taking a used commode pail out from under its seat; putting a quick-release toilet seat back on its hinge posts; fitting a front-lock raised toilet seat. Here a winged pan that drops into the bowl if set lengthwise is turned and bridged across a full-size bowl's opening under a raised seat that falls shut if pushed.

## Assets and sources

No third-party assets: every shape is a box or cylinder in `scene.py`; no picture, mesh or font from elsewhere. Product sources are linked in the table above (Bemis page and spec sheet and Kohler sheet opened by us on 2026-11-22; the pan listing not opened by us).

## How to run

```
scripts/rta check mech/tasks/specimen_hat                     # rule tests
scripts/rta simpy <id> mech/tasks/specimen_hat/checks.py [--frame0 /workspace/robocurve/outputs/<id>]
scripts/rta simpy <id> mech/tasks/specimen_hat/trials.py /workspace/robocurve/outputs/<id>
scripts/rta demo <id> mech/tasks/specimen_hat --check          # 1.0, no video
```
