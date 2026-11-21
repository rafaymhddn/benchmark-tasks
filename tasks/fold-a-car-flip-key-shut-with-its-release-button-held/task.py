"""A car's folding key lies in a ceramic key dish on the bench, its steel blade folded out, the buttons up. The blade
turns on a pin at one end of the fob, half a turn, from straight out to folded into a channel along the fob's edge; a
torsion spring pushes it out. A pawl in the fob holds it at either end while the release button on the fob's face is
out: pushed without the button the blade does not move. With the button pressed in, the pawl is lifted and the blade
turns freely; let the button out within 3 degrees of folded and the pawl drops into the blade's notch, anywhere else
it rides on the blade's round boss until a notch comes under it, and the spring flicks the blade open again.

How it is built. The fob is one free body. The blade turns on a hinge to it with end stops at open and folded and
the spring. The release button slides into the fob on a sprung slide with stops at out and fully in. The pawl is
the blade hinge's pair of stops, moved once per control step: the button pressed past PRESSED_AT, both stops at the
blade's ends (it turns freely); the button out, the stops close round the blade at whichever end it is within CATCH
of. The dish is fixed: a floor and a wall of boxes round it.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, cylinder, hinge, slide  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Fold the car key shut while holding it clear of the tray and the bench, then put it back in the tray."
RUN_LIMIT_STEPS = 1900  # 95 s at 20 Hz: about three times the scripted full run (620 steps, 31.0 s, run w9e)
TOP = TABLE_TOP[SCENE]
GRAVITY = 9.81

BLACK, RUBBER, GREY, STEEL, GLAZE = (0.06, 0.06, 0.07), (0.20, 0.20, 0.22), (0.38, 0.38, 0.40), (0.72, 0.74, 0.77), (0.30, 0.42, 0.58)

# --- the key and its mechanism. (a): assumed, to be measured on the bought key (README, owner's bench test) ---
# | part            | value                                                     | source                                   |
# | fob             | 70 x 34 x 16 mm, 35 g                                     | (a) Kia folding key, 4 buttons (SK3)     |
# | blade           | 60 x 9 x 2.5 mm steel, 9 g; pin 5 mm from its root end:   | (a)                                      |
# |                 | 55 mm from the pin to the tip                             |                                          |
# | blade hinge     | free 0 to 180 deg, end stops at both ends                 | geometry                                 |
# | blade spring    | torsion, 0.02 N m open, 0.05 N m folded, towards open     | (a); bench test 2                        |
# | pawl            | holds the blade at 0 and at 180 deg while the button is   | (a); bench test 2 (5 N at the tip, no    |
# |                 | out; holds 0.6 N m before anything gives                  | motion); the scene's stops hold any load |
# | release button  | 10 mm round, slide 1.2 mm, 2 N out, 4 N fully in          | (a); bench test 1 (pressed on the scale) |
# | pawl lifted     | button pressed 0.9 mm or more of its 1.2 mm               | (a)                                      |
# | pawl catches    | button out, blade within 3 deg of an end                  | geometry                                 |
FOB_L, FOB_W, FOB_T, FOB_MASS = 0.070, 0.034, 0.016, 0.035
BLADE_L, BLADE_W, BLADE_T, BLADE_REAL = 0.060, 0.009, 0.0025, 0.009
BLADE_ROOT = 0.005  # m from the blade's root end to the pin
REACH = BLADE_L - BLADE_ROOT  # m from the pin to the tip
PIN = np.array([0.026, 0.0128, 0.0])  # in the fob's frame: x along it to the pin end, y across to the channel's edge, z out of the buttons' face
FOLDED = math.pi  # rad of fold from open
FOLD_BUILT = math.pi / 2  # the blade is built standing out sideways: its hinge's stops at -90 and +90 deg of travel
# (a stop or a drive's rest half a turn from the built pose wraps in PhysX: run w9b, the key thrown off)
SPRING_OPEN, SPRING_FOLDED = 0.02, 0.05  # N m towards open
SPRING_K = (SPRING_FOLDED - SPRING_OPEN) / FOLDED  # N m/rad
SPRING_BACK = SPRING_OPEN / SPRING_K  # rad: the spring's rest lies this far beyond open
BUTTON_AT = np.array([0.024, 0.0])  # the button's middle on the fob's face (a: its place is taken from the bought key)
BUTTON_R, BUTTON_H, BUTTON_TRAVEL, BUTTON_REAL = 0.005, 0.003, 0.0012, 0.001
BUTTON_OUT_N, BUTTON_IN_N = 2.0, 4.0
BUTTON_K = (BUTTON_IN_N - BUTTON_OUT_N) / BUTTON_TRAVEL  # N/m
BUTTON_BACK = BUTTON_OUT_N / BUTTON_K  # m: the spring's rest lies this far out beyond the out stop
PRESSED_AT = 0.0009  # m of the button's travel from which the pawl is lifted
CATCH = math.radians(3.0)  # the pawl drops into a notch within this of the blade's end
PLAY = math.radians(0.5)  # the blade's play in a notch
# Stand-ins for the solver: a light part pressed between a hand and a joint gives way in this simulator (kit lesson),
# so the blade and the button move with more inertia than the real parts; their weight over the real one is carried
# by an upward push (the bar clamp's gauge pad and trigger, same way).
BLADE_MASS, BUTTON_MASS = 0.050, 0.020
CARRIED = {"blade": BLADE_MASS - BLADE_REAL, "button": BUTTON_MASS - BUTTON_REAL}
BLADE_FRICTION = (0.4, 0.3)  # steel: assumed
BLADE_DAMPING = 6e-4  # N m s/rad on the hinge: about half of critical for the spring and the stand-in blade (a: the real blade stops at its stop, it does not ring)

# --- the dish: a round ceramic key dish (a: maker and size to be named), floor 150 mm across, rim 180 mm, 20 mm high ---
TRAY_AT = np.array([0.28, 0.0])
TRAY_FLOOR_R, TRAY_RIM_R, TRAY_H, TRAY_FLOOR_T, TRAY_SIDES = 0.075, 0.090, 0.020, 0.005, 24
TRAY_FRICTION = (0.4, 0.3)  # glazed ceramic: assumed
FLOOR_TOP = TOP + TRAY_FLOOR_T

# --- where the key starts: in the dish, blade open, buttons up, the open key's middle on the dish's middle, turned by seed ---
KEY_MIDDLE = 0.5 * (-FOB_L / 2 + PIN[0] + REACH)  # m along the fob from its middle to the open key's middle
FOB_AT = np.array([TRAY_AT[0] - KEY_MIDDLE, TRAY_AT[1], FLOOR_TOP + FOB_T / 2])


YAW_MID, YAW_HALF = math.radians(157.5), math.radians(42.5)  # the key turned 115 to 200 deg: what the scripted plan reaches (dry plans: 104.7 deg leaves 2.5 deg on the right wrist, 95 deg fails)


def start_yaw(seed: int | None) -> float:
    """The key's turn about the vertical through the dish's middle (rad, the fob's long axis from +x): by seed within
    YAW_HALF of YAW_MID (what the scripted plan reaches, README "Setup"); YAW_MID without a seed."""
    return YAW_MID if seed is None else YAW_MID + float(np.random.default_rng(seed).uniform(-YAW_HALF, YAW_HALF))


def _tray_shapes() -> list:
    shapes = [cylinder(TRAY_FLOOR_R + 0.001, TRAY_FLOOR_T, pos=(0.0, 0.0, TRAY_FLOOR_T / 2), color=GLAZE)]
    mid, thick = 0.5 * (TRAY_FLOOR_R + TRAY_RIM_R), TRAY_RIM_R - TRAY_FLOOR_R
    chord = 2 * TRAY_RIM_R * math.tan(math.pi / TRAY_SIDES)
    for i in range(TRAY_SIDES):
        a = 2 * math.pi * i / TRAY_SIDES
        shapes.append(box((thick, chord, TRAY_H), pos=(mid * math.cos(a), mid * math.sin(a), TRAY_H / 2), yaw=a, color=GLAZE))
    return shapes


body("tray", _tray_shapes(), pos=(TRAY_AT[0], TRAY_AT[1], TOP), color=GLAZE, friction=TRAY_FRICTION)

_remote = [cylinder(0.0045, 0.0008, pos=(x, y, FOB_T / 2 + 0.0004), color=RUBBER, solid=False) for x, y in ((-0.004, -0.007), (-0.004, 0.007), (-0.020, -0.007), (-0.020, 0.007))]
body("fob", [box((FOB_L, FOB_W, FOB_T), color=BLACK), *_remote], pos=tuple(FOB_AT), mass=FOB_MASS - BUTTON_REAL, color=BLACK)
_pin_world = FOB_AT + PIN
body("blade", [box((BLADE_L, BLADE_W, BLADE_T), pos=(BLADE_L / 2 - BLADE_ROOT, 0.0, 0.0), color=STEEL)], pos=tuple(_pin_world), mass=BLADE_MASS,
     yaw=FOLD_BUILT, color=STEEL, friction=BLADE_FRICTION)  # fmt: skip
_button_world = FOB_AT + np.array([BUTTON_AT[0], BUTTON_AT[1], FOB_T / 2 + BUTTON_TRAVEL - BUTTON_H / 2])
body("button", [cylinder(BUTTON_R, BUTTON_H, color=GREY)], pos=tuple(_button_world), mass=BUTTON_MASS, color=GREY)

# The blade's spring is a torque computed every control step from the fold (``spring_torque``) and put on the blade and,
# opposite, on the fob: a drive spring whose rest lies 120 deg beyond open would wrap at half a turn in PhysX.
hinge("fold", "blade", "fob", anchor=tuple(_pin_world), axis=(0.0, 0.0, 1.0), limits=(-FOLD_BUILT, FOLDED - FOLD_BUILT), damping=BLADE_DAMPING)
slide("press", "button", "fob", axis=(0.0, 0.0, -1.0), limits=(0.0, BUTTON_TRAVEL), spring=BUTTON_K, rest=-BUTTON_BACK,
      damping=2.0 * math.sqrt(BUTTON_K * BUTTON_MASS))  # fmt: skip

OBJECTS = list(mk.MECH.bodies.values())
KEY = ("fob", "blade", "button")
SIDES = ("left", "right")

# --- the rules ---
ON_BENCH = 0.02  # m: a part this far under the bench top has left it
LIFTED = 0.02  # m the fob's middle is raised over where it started: picked up
SWUNG = math.radians(90.0)  # the blade turned this far towards folded at some moment: 0.25 with the key picked up
CLEAR, RESTING, SETTLE_S = 0.03, 0.02, 0.25  # m grippers from the key, m/s at rest, s it is read over
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    """A new trial's state (no simulator needed)."""
    STATE.clear()
    STATE.update(seed=seed, yaw=start_yaw(seed), pawl="open", extra={}, fell=False, lifted=False, most_fold=0.0, start_z=float(FOB_AT[2]), track=[], events=[],
                 home=False, home_in_hands=False)  # fmt: skip
    return STATE


def _turned(point, yaw: float) -> np.ndarray:
    """A point of the key as built, turned ``yaw`` about the vertical through the dish's middle."""
    c, s = math.cos(yaw), math.sin(yaw)
    d = np.asarray(point, dtype=float) - np.array([TRAY_AT[0], TRAY_AT[1], 0.0])
    return np.array([TRAY_AT[0] + c * d[0] - s * d[1], TRAY_AT[1] + s * d[0] + c * d[1], d[2]])


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if mk.MECH.environment is not None:
        set_window(0.0, PLAY)  # the pawl in the open notch
        yaw = STATE["yaw"]
        for name in KEY:  # the blade open: its frame turned as the fob's
            mk.write_pose(name, _turned(mk.MECH.bodies[name].pos, yaw), mk.yaw_quat(yaw))


def fold() -> float:
    """The blade's fold from open (rad): 0 open, FOLDED folded."""
    return mk.joint("fold") + FOLD_BUILT


def set_window(low: float, high: float) -> None:
    """The blade hinge's stops at folds ``low`` and ``high`` (rad from open); in the simulator only."""
    if mk.MECH.environment is not None:
        mk.set_limits("fold", lower=low - FOLD_BUILT, upper=high - FOLD_BUILT)


def spring_torque() -> float:
    """N m the blade's spring turns it towards open with, at its fold now."""
    return SPRING_OPEN + SPRING_K * min(max(fold(), 0.0), FOLDED)


def loads() -> None:
    """Every computed load, once per control step: the carried weight of the stand-ins, the blade's spring (on the
    blade towards open, the opposite on the fob) and whatever a trial puts in STATE["extra"] (name -> force, torque)."""
    axis = mk.pose("fob")[1] @ np.array([0.0, 0.0, 1.0])
    t = -spring_torque() * axis
    force = {name: np.array([0.0, 0.0, CARRIED.get(name, 0.0) * GRAVITY]) for name in KEY}
    torque = {"fob": -t, "blade": t, "button": np.zeros(3)}
    for name, (f, tq) in STATE.get("extra", {}).items():
        force[name] = force[name] + np.asarray(f, dtype=float)
        torque[name] = torque[name] + np.asarray(tq, dtype=float)
    for name in KEY:
        mk.push(name, force=tuple(force[name]), torque=tuple(torque[name]))


def pawl() -> None:
    """The pawl, once per control step: lifted while the button is pressed PRESSED_AT or more (both stops at the
    blade's ends), otherwise in a notch or riding on the blade's boss until it comes within CATCH of an end."""
    angle, pressed = fold(), mk.joint("press") >= PRESSED_AT
    if pressed:
        if STATE["pawl"] != "lifted":
            STATE["pawl"] = "lifted"
            STATE["events"].append((mk.now(), f"button pressed: pawl lifted at {math.degrees(angle):.1f} deg"))
            set_window(0.0, FOLDED)
        return
    if STATE["pawl"] == "lifted":
        STATE["pawl"] = "riding"
        STATE["events"].append((mk.now(), f"button out at {math.degrees(angle):.1f} deg"))
    if STATE["pawl"] == "riding":
        if angle >= FOLDED - CATCH:
            STATE["pawl"] = "folded"
            set_window(FOLDED - PLAY, FOLDED)
        elif angle <= CATCH:
            STATE["pawl"] = "open"
            set_window(0.0, PLAY)
        if STATE["pawl"] != "riding":
            STATE["events"].append((mk.now(), f"pawl in the {STATE['pawl']} notch at {math.degrees(angle):.1f} deg"))


def watch() -> None:
    """What the rules need from the whole trial: the key off the bench, picked up, the blade's most fold, and whether
    the blade was swung home in the hands: on each step on which it comes within CATCH of folded from further open,
    whether the fob's middle is then LIFTED or more over where it started; the last such step decides."""
    z = mk.pose("fob")[0][2]
    if min(mk.pose(name)[0][2] for name in KEY) < TOP - ON_BENCH:
        STATE["fell"] = True
    if z >= STATE["start_z"] + LIFTED:
        STATE["lifted"] = True
    angle = fold()
    STATE["most_fold"] = max(STATE["most_fold"], angle)
    home = angle >= FOLDED - CATCH
    if home and not STATE["home"]:
        STATE["home_in_hands"] = bool(z >= STATE["start_z"] + LIFTED)
        STATE["events"].append((mk.now(), f"blade home {'in the hands' if STATE['home_in_hands'] else 'with the key down'} (fob {1000 * (z - STATE['start_z']):.0f} mm over its start)"))
    STATE["home"] = home


@mk.on_step
def step() -> None:
    if mk.MECH.environment is not None:
        loads()
    pawl()
    watch()
    track()


# --- readings ---
def to_world(name: str, point) -> np.ndarray:
    position, rotation = mk.pose(name)
    return position + rotation @ np.asarray(point, dtype=float)


def tip() -> np.ndarray:
    return to_world("blade", (REACH, 0.0, 0.0))


def _marks() -> dict:
    return {"fob": mk.pose("fob")[0].copy(), "tip": tip()}


def track() -> None:
    t, kept = mk.now(), STATE.setdefault("track", [])
    kept.append((t, _marks()))
    while len(kept) > 2 and kept[1][0] <= t - SETTLE_S:
        kept.pop(0)


def speeds() -> dict:
    kept = STATE.get("track", [])
    if len(kept) < 2 or kept[-1][0] <= kept[0][0]:
        return {"fob": None, "tip": None}
    (t0, then), (t1, now) = kept[0], kept[-1]
    return {name: float(np.linalg.norm(now[name] - then[name])) / (t1 - t0) for name in now}


def at_rest() -> bool:
    return all(v is not None and v < RESTING for v in speeds().values())


def latched() -> bool:
    """The blade folded and the pawl in its notch: it stays in under a pull at its tip (the bench's tug)."""
    return STATE["pawl"] == "folded" and fold() >= FOLDED - CATCH


def in_tray() -> bool:
    """The fob's middle inside the dish: within the floor's edge and lower than the floor's top plus the dish's height."""
    p = mk.pose("fob")[0]
    return bool(math.hypot(p[0] - TRAY_AT[0], p[1] - TRAY_AT[1]) <= TRAY_FLOOR_R and FLOOR_TOP <= p[2] <= FLOOR_TOP + TRAY_H)


def distance_to_key(point) -> float:
    """m from a point to the nearest of the fob's and the blade's boxes."""
    best = math.inf
    for name, half, centre in (("fob", np.array([FOB_L, FOB_W, FOB_T]) / 2, np.zeros(3)),
                               ("blade", np.array([BLADE_L, BLADE_W, BLADE_T]) / 2, np.array([BLADE_L / 2 - BLADE_ROOT, 0.0, 0.0]))):  # fmt: skip
        position, rotation = mk.pose(name)
        local = rotation.T @ (np.asarray(point, dtype=float) - position) - centre
        best = min(best, float(np.linalg.norm(np.maximum(np.abs(local) - half, 0.0))))
    return best


def let_go() -> bool:
    return all(distance_to_key(mk.gripper(side)[0]) >= CLEAR for side in SIDES)


def score(objects) -> float:
    """0 if a part of the key has left the bench at any moment. Folded and latched but swung home with the key lying in
    the tray or on the bench (not lifted 2 cm when the blade last came within 3 degrees of folded): 0.25. Folded and
    latched, swung home in the hands, the key at rest and both grippers 3 cm or more from it: 1.0 with the key in the
    tray, 0.5 anywhere else on the bench. 0 with the blade open at the end (the pawl in the open notch). Otherwise
    0.25 if the key was picked up and the blade turned 90 degrees or more towards folded at some moment; else 0."""
    if STATE["fell"]:
        return 0.0
    if latched() and not STATE["home_in_hands"]:
        return 0.25
    if latched() and at_rest() and let_go():
        return 1.0 if in_tray() else 0.5
    if STATE["pawl"] == "open":
        return 0.0
    return 0.25 if STATE["lifted"] and STATE["most_fold"] >= SWUNG else 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out: dict = {"pawl": STATE.get("pawl"), "events": [f"{t:.2f} s {what}" for t, what in STATE.get("events", [])], "fell": STATE.get("fell"),
                 "lifted": STATE.get("lifted"), "most fold (deg)": round(math.degrees(STATE.get("most_fold", 0.0)), 1), "home in hands": STATE.get("home_in_hands")}  # fmt: skip
    if mk.MECH.poses:
        out.update({"fold (deg)": round(math.degrees(fold()), 2), "button in (mm)": round(1000 * mk.joint("press"), 3), "in the tray": in_tray(),
                    "speeds (mm/s)": {k: None if v is None else round(1000 * v, 2) for k, v in speeds().items()}})  # fmt: skip
        if mk.MECH.grippers:
            out.update({"let go": let_go(), "at rest": at_rest(), "score": score(None)})
    return out
