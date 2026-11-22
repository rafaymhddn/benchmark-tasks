# Thread a 273 mm CryoSleeve onto a vial-loaded aluminum CryoCane up to its top tab before transfer-racking

Instruction given to the policy: `Slide the ivory cryo-sleeve from the prep mat onto the aluminum cryo-cane up to its bent top tab without spilling either vial, lay the sleeved cane flat inside the stainless transfer tray, and let go.`

Files: `task.json`; `task.py` (rules, score); `scene.py` (bodies); `parts.py` (figures, sources); `checks.py`, `trials.py`, `demo.py` (reach, PhysX trials, reference run); `test_cryocane_sleeve.py`; the kit (`mechkit.py`, `demo_kit.py`, `kit_extra.py`); `start_top.png`, `start_top_policy.png` (320 x 176), `start_left_wrist.png`, `start_right_wrist.png`; `end_top.png`, `end_top_policy.png`.

![the top camera's first picture](start_top.png)

## What the task tests

- **Level 180-degree yaw with loose tubes.** Two 1.8 mL round-bottom Nunc CryoTubes (`377267`, 12.5 x 49.0 mm) sit loose by gravity in open C-bays of a 290 mm 5-place stepped Al Nalgene CryoCane (`5015-0001`), whose 26 mm top tab starts facing the sleeve across a 13 mm gap. Yawing 180 degrees level keeps both tubes seated; rolling > ~60 deg or tipping steeply before sleeving spills them (`0.0`).
- **One-way coaxial 265 mm two-hand pass.** Pushing the 273 mm Nalgene CryoSleeve (`5016-0001`) onto the near end hits the 26 mm top tab at 0 mm. Only after the 180-degree yaw can both hands telescope the sleeve over the straight foot and drive a 265 mm pass against 5.0 N drag (35.0 N if cocked > 0.35 N m) to within 10 mm of the top tab, enclosing vials and foot.
- **Shortcuts scored 0.** Putting an unsleeved or half-sleeved cane in the tray latches `0.0`; wrong-end entry stops at the 26 mm tab; rolling spills the vials (`0.0`).
- **What stands in for what, and states set directly.** Rigid bodies in PhysX (`task.py`, `scene.py`, `demo.py` call zero `write_pose`, `set_joint` or `mk.push`; `trials.py` sets poses only for isolated rules). Because the path tracer draws fractional opacity opaque, the clear PVC sleeve is drawn opaque frosted ivory (`sRGB 0.92, 0.93, 0.88`). Its 8-stave bore is 21.6 mm flat-to-flat (23.6 mm outer) to clear the 13.2 mm stack; telescoped 10..60 mm coaxially over the foot (`<= 2.8 mm` mouth, `<= 2.0 deg`), `w_sleeve` joins in-place (zero snap) onto the 265 mm `slv_s` slide (`5.0 N` wipe, `35.0 N` if > `0.35 N m`), and direct geometric enclosure (`<= 6 mm`, `<= 5 deg`, `30..273 mm`) is also scored; solver masses 35 / 12 / 35 g (real 16 / 3.5 / 18 g); no PVC flex or slosh.
- **Grasps and visibility (`320 x 176`).** Left hand pinches the 26 mm top tab (`<= 11.5 cm` up, `29.6 N`/finger); right hand pinches the 23.6 mm sleeve tail (`<= 10.4 cm`, `33.0 N`/finger); TCPs `>= 262 mm` apart (`hulls >= 32 mm`). At `320 x 176` (`start_top_policy.png`, `end_top_policy.png`), blue tab, magenta vials, ivory sleeve, blue mat and steel tray stand out.

## Scene

- **Prep mat:** 130 x 600 x 2.0 mm blue silicone strip [hs-mat], taped at `(210, 0) mm` with white ticks.
- **CryoCane (`cane`):** Thermo Scientific Nalgene 5-place stepped Al CryoCane (`5015-0001`, 290 mm) [cane], trough-up along `x = 210 mm` from `y = +278 mm` (foot) to `-12 mm` (26 mm blue-labelled top tab).
- **Cryovials (`vial_a`, `vial_b`):** Thermo Scientific Nunc 1.8 mL round-bottom PP CryoTubes (`377267`, 12.5 x 49.0 mm) [vial], white caps to `-y`, sealed with 1.5 mL magenta water, loose in bays 3 and 2.
- **CryoSleeve (`sleeve`):** Thermo Scientific Nalgene PVC CryoSleeve (`5016-0001`, 273 mm) [sleeve], along `x = 210 mm` from `y = -298` to `-25 mm` (mouth 13 mm from top tab).
- **Transfer tray (`tray`):** our fixture, open 18/10 stainless tray 350 x 150 x 35 mm (any open tray >= 320 x 140 mm; cf. [bochem]), 420 g, at `(365, +35) mm`.

## Rules and score

- **0 for good** (from the video on a bench, end photo confirming) if either vial leaves its bay (`> 14 mm` off bore axis, `> 15 mm` off bay centre, or touching mat/tray/bench), cane or vial enters inside the tray (below its 35 mm rim) before the sleeve seats to the top tab, or any object drops off the bench.
- **0.25:** sleeve coaxially enclosing cane from the foot (`w_sleeve` or `<= 6 mm, <= 5 deg, 30..273 mm`) with leading mouth `>= 120 mm` onto the cane, vials retained.
- **0.5:** sleeve leading mouth `<= 10 mm` from the 90-degree top tab shoulder (`>= 258 mm` travel) and bottom foot wholly inside the 273 mm sleeve (`0 mm` bare), vials retained.
- **1.0:** `0.5` met, sleeved cane flat in tray (`344 x 144 mm` walls below 35 mm rim, tilt `<= 8 deg`), both hands' hulls `>= 30 mm` clear, held so for `1.0 s` (`20 steps`) with cane and sleeve moving `3 mm` at most. Limit `1,800 steps` (`90 s`); `SUCCESS_HOLD_S = 3.5 s`.

## Key figures and their sources

| Figure | Value | Source |
|---|---|---|
| CryoCane (`5015-0001`) | 290 mm (`29 cm`), Al, 5-place | maker: `'5015-0001 ... Aluminium ... Unterteilt ... 29 cm (11,31 in.)'` [cane]; listing: 5-place |
| Cryovial (`377267`) | 1.8 mL, 12.5 x 49.0 mm, round PP | maker: `'377267 ... 1.8 ml ... 12.5 mm ... 49 mm ... Polypropylen ... Rund'` [vial] |
| CryoSleeve (`5016-0001` / `HS23210`) | 273 mm PVC (`14.5 x 275 mm`) | maker: `'5016-0001 ... Nalgene CryoSleeve, 273 mm, PVC'` [sleeve]; `'14.5 mm diameter x 275 mm length'` [hs-slv] |
| Silicone mat | 350 x 600 x 2.0 mm | maker: `'14 x 23.5 x 0.1 in (35 x 60 x 0.2 cm)'` [hs-mat] |
| Transfer tray | 350 x 150 x 35 mm, 420 g | our fixture: generic open tray >= 320 x 140 mm (cf. [bochem]); mass estimate (0.6 mm steel) |
| Pitch, tab stop, tab width | 51.3, 268, 26.0 mm | estimate: scaled from `5015-0001` photo vs 290 mm; 26 mm tab blocks 21.6 mm bore |
| Spine, rail lip, step | 0.7, 2.8, 2.2 mm | estimate: 13.2 mm stack; capsule roll/pitch spill at ~57/50 deg |
| Bore, wipe & cocked bind | 21.6 mm flat; 5.0 N, 35.0 N (>0.35 N m) | estimate: clears 13.2 mm stack; PVC wipe over Al rails, 7x cocked wedge |
| Real masses | 16 g, 3.5 g, 18 g | estimate: 0.7 mm Al, 2 g PP + 1.5 mL water, 0.45 mm PVC |

`parts.FIGURES`: 8 maker, 1 listing, 9 our fixture, 15 estimate.

## First minutes of a real trial

On bought `5015-0001`, `5016-0001`, `377267`, mat and tray (~5 min; a different reading replaces the estimate): caliper top tab width, bay pitch, lip height and sleeve bore (1.5 min); weigh items on a scale (1 min); roll/tilt an unsleeved cane with two `377267` vials against a phone inclinometer for spill angles (1 min); pull a seated sleeve off with a luggage scale for wipe force and tab gap (1.5 min).

## Forces

| Force or stop | How made (`scene.py`) | Acts on | Reaction on | Ends / triggers |
|---|---|---|---|---|
| Open C-trough seating | Gravity + 2.8 mm rails, 2.2 mm steps | `vial_a`, `vial_b` | `cane` (equal & opposite) | Spills if unsleeved cane rolls > ~60 deg or pitches steeply |
| Top-tab one-way stop | Rigid 26.0 mm Al tab vs 21.6 mm bore | `sleeve` mouth | `cane` tab (equal & opposite) | Blocks wrong-end entry at 0 mm; needs 180 deg yaw |
| Coaxial wipe & bind | `slide("slv_s")` capped spring | `sleeve` | `cane` (equal & opposite) | 5.0 N drag over 265 mm (35 N if > 0.35 N m); unmates on > 5 N pull to 0 mm |

## Checks run so far

- Tests (`test_cryocane_sleeve.py`): 7 pass (zero `write_pose`/`set_joint`/`mk.push` in `task.py`, `scene.py`, `demo.py`).
- Reach (`checks.py`, `gcryo_g1`): 15 poses (`IK <= 0.09 mm`, margin `>= 9.7 deg`, hulls `>= 32.1 mm` apart, TCPs `>= 262.2 mm`); 2 s idle `0.0`.
- Trials (`trials.py`, `gcryo_g2f` + `gcryo_g2j`): 6/6 pass — wrong-end `-5.3 mm` (`0.0`), 75-deg roll spills vials (`0.0`), unsleeved in tray (`0.0`), 150 mm (`0.25`), 265.5 mm inverted `165 deg` (`0.5`), 1.74-deg angled slide (`0.25` -> `0.5`).
- Demo (`demo.py`, `gcryo_rec1`): `1.0` in `33.05 s` (`0.25` at `16.95 s`, `0.5` at `19.20 s`, `1.0` at `29.60 s`; `264.7 mm` slide vs `5.0 N` wipe, `4.1 mm` tab gap, roll/pitch `<= 2.2/1.5 deg`, `29.6/33.0 N` pinch, arms clear `218.9 mm`).

![the reference run's last picture](end_top.png)

## How it differs

Other tasks hang a canister hook in a Dewar neck, drop a short ring in a bench block, or shrink a collar onto a clamped wire. Here nothing is fixed: the robot lifts an untethered 290 mm open-trough cane holding two loose tubes, yaws 180 degrees level in mid-air without spilling them, and drives a 265 mm (`19:1`) two-hand coaxial pass to the top tab before racking.

## Assets and sources

No third-party meshes, scans, textures or sounds; only `mechkit.py` primitives.
- [cane] https://www.thermofisher.com/order/catalog/product/5015-0001
- [sleeve] https://www.thermofisher.com/order/catalog/product/5016-0001
- [vial] https://www.thermofisher.com/order/catalog/product/377267
- [hs-slv] https://www.heathrowscientific.com/pvc-cryogenic-cane-sleeves/
- [hs-mat] https://www.heathrowscientific.com/silicone-lab-mats/
- [bochem] https://bochem.de/produkt/instrumentenablage-18-10-stahl/

## How to run

Tests: `scripts/rta check cryocane_sleeve`. Reach/start: `scripts/rta simpy <id> mech/tasks/cryocane_sleeve/checks.py --frame0 /workspace/robocurve/outputs/<id>`. Trials: `scripts/rta simpy <id> mech/tasks/cryocane_sleeve/trials.py /workspace/robocurve/outputs/<id>`. Demo: `scripts/rta demo <id> mech/tasks/cryocane_sleeve` (`--no-video`, `--quick`).

A trial lasts at most 90 s (1,800 steps at 20 Hz); the reference run takes ~36 s. Fixed layout, nothing drawn from the seed: `--seed 0` under the harness gives trial seed 1415158438 and the same start.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=mech/tasks/cryocane_sleeve/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Slide the ivory cryo-sleeve from the prep mat onto the aluminum cryo-cane up to its bent top tab without spilling either vial, lay the sleeved cane flat inside the stainless transfer tray, and let go." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir <log dir>
```
