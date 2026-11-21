"""Instant photo with a Polaroid Go Generation 2 (as modelled: the maker's 105 x 84 x 62 mm, no name or logo), free on
the bench, its lens aimed away from the arms at a fixed red apple (7 cm) APPLE_DIST in front of the lens; the red
shutter button on its top, front right; the print slot low in its front face, 5 mm up.

The first press of the sprung button home (9/10 of its 2 mm travel) takes the one exposure: good only if the camera is
then level (within 20 degrees), still (under 2 cm/s) and the apple within 12 degrees of the lens axis and AIM_DIST from
the lens; otherwise it is wasted and the episode ends. Let go (the maker's manual), the button sends the print out over
1.5 s, picture side up, its last INNER left in the slot; the rollers' nip closes behind it. Over it lies the film
shield, a thin flap hinged at the slot's upper lip: its ridge presses the print onto the slot's lower lip with a force
the rules set through the flap's spring. That friction holds the print (its reaction on the camera through the hinge
and the lip). The press ends only when the shield is up: lifted past LIFT_DEG, its spring rolls it back; a pull over
HOLD_N draws the print from under it against DRAG_N (0.75 at most). A pinch, or a touch short of the angle, frees
nothing. The slot faces away from the arms: the camera has to be held, lifted, turned.

Done (the maker's guide): the print left 5 s under the shield, the shield lifted, the print drawn out and laid face down
on the bench, the camera standing on the bench, both hands off, for 3 s. Development is not simulated. Helpers:
photo_geometry.py (the models' figures, the camera's frame), photo_demo.py (the scripted run).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = str(Path(__file__).resolve().parent)
if HERE not in sys.path:
    sys.path.insert(0, HERE)  # the helpers beside this file

import photo_demo  # noqa: E402
from photo_geometry import (BOX_C, CAM_SIZE, FLOOR, FRONT, HIDDEN_X, LENS, PRINT, PRINT_Z,  # noqa: E402
                            SHUTTER_TRAVEL, C, L, P, T, aim, box_distance, picture_up, print_in_camera,
                            shield_angle, shield_gravity, shutter_in, tilt_deg, to_world)
from rta.spec import TABLE_TOP, Asset  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
REST = TOP + 0.001  # models are spawned 1 mm over the bench (yam_arena station.REST_GAP)
INSTRUCTION = ("Take a photo of the apple with the instant camera, leave the print in the slot for 5 seconds, lift the "
               "film shield, pull the print out, lay it face down on the bench, and leave the camera standing on the "
               "bench.")
ACT_AFTER_S = 1.0  # nothing to watch: the arms are free after a short settle
MEMORY = False  # nothing is shown during the wait: no memory-task rule for the policy

# --- the film shield, the rollers' nip and what holds the print once it is out (assets/build.py)
S, R = L["shield"], L["rollers"]
SHIELD_K = S["spring"]  # N m/rad: the flap's spring; the rules set its rest
SHIELD_MU = sum(S["friction"])  # the ridge on the print's top and the lip under it: friction = SHIELD_MU x the press
# its lever at the slip: the rod's distance from the hinge + the hinge's height over the print x the rod's friction
PRESS_LEVER = S["ridge"] + S["ridge_rise"] * S["friction"][0]
SHIELD_UP = S["travel"][1]  # rad: its upper stop, rolled back (lifted-up positive)
HOLD_N, DRAG_N = 0.3, 0.2  # N: the pull that starts the print under the shield; the resistance while it slides (estimates)
LIFT_DEG = 15.0  # a finger lifts the shield's free edge past this and its spring rolls it back (estimate)
LIFT_RAD = math.radians(LIFT_DEG)
CLEAR_RAD = math.radians(2.0)  # while the rules place the print the shield stands this far up
STOW_STEP = math.radians(40.0)  # rolling back: the rest at most this far past the flap's angle (a drive pulls in
# proportion within 45 degrees)
INNER = P["inner"]  # m: the print's inner end stays this far inside the front face once it is out (estimate)
EJECTED_X = FRONT - INNER + PRINT[0] / 2  # the print's centre once out
PROTRUDE = EJECTED_X + PRINT[0] / 2 - FRONT  # how far it then stands out of the slot
NIP_OPEN, NIP_SHUT = R["travel"][1] + 0.001, R["travel"][0] - 0.001  # m: the nip spring's rest, 1 mm past a stop
SLIDE_V = 0.002  # m/s: the print moving along the slot faster than this slides (drag), slower it is still (hold)
SHIELD_RAMP_S = 0.2  # s: once the print is out, the shield's press rises to the hold over this
GRAVITY = 9.81

CAM_XY = (0.279, 0.055)  # the camera's origin
APPLE_DIST = 0.50  # m from the lens's front to the apple's centre: the maker's nearest distance, 45 cm, and 5 cm more
APPLE_XY, APPLE_EXT = (CAM_XY[0] + LENS[0] + APPLE_DIST, CAM_XY[1]), 0.07
A = L["assets"]
OBJECTS = [
    Asset("camera", asset=A["camera"], xy=(CAM_XY[0] + BOX_C[0], CAM_XY[1] + BOX_C[1]), extent=L["extent"]["camera"], mass=C["mass"]),
    Asset("print", asset=A["print"], xy=(CAM_XY[0] + HIDDEN_X, CAM_XY[1]), extent=L["extent"]["print"], mass=P["mass"]),
    Asset("apple", asset="apple_02_objaverse_robolab", xy=APPLE_XY, extent=APPLE_EXT),
]
MOUNTED = {("print", "camera")}  # the print starts inside the camera: not a spawn overlap
# the camera's sprung joints (assets/build.py): the shutter button, the film shield (a hinge), the rollers' nip
JOINTS = {"camera": {"shutter": (C["shutter_spring"], C["shutter_damping"]), "shield": (S["spring"], S["damping"]),
                     "rollers": (R["spring"], R["damping"])}}

# --- rules
PRESSED = 0.9 * SHUTTER_TRAVEL  # the shot: the button pressed home
RELEASED = 0.25 * SHUTTER_TRAVEL  # let go: the button back within a quarter of its travel of rest
LEVEL_DEG, AIM_DEG = 20.0, 12.0
AIM_DIST = (0.45, 2.0)  # m, lens to apple at the shot: the maker's "at least 45cm" and its flash's reach, 2 m
STEADY = 0.02  # m/s (its box centre's speed; the simulator's spin reading is noisy for a body held in a grip)
EJECT_S = 1.5  # the print takes this long to come out, once the button is let go (estimate)
SHIELD_S = 5.0  # the maker: "Leave your photo under the film shield for about 5 seconds" before the shield is lifted
OUT = T["lip"] + 0.002  # the print is out when its inner end is this far in front of the slot: past its lower lip
FACE_DOWN_DEG = 15.0
ON_BENCH = 0.006  # lying / standing on the bench: the lowest point within this of it
CLEAR = 0.04  # hands off: every tool centre point at least this far from the object's box
KNOCKED_DEG = 60.0


def setup(rng, trial) -> dict:
    return {"shot_t": None, "release_t": None, "out_t": None, "freed_t": None, "drawn_t": None, "fail": None,
            "came_out": False, "shield_n": 0.0, "shield_rest": 0.0, "sliding": False, "_depth": None, "_depth_t": None}


def placements(state) -> dict:
    return {"print": (CAM_XY[0] + HIDDEN_X, CAM_XY[1], REST + PRINT_Z)}


def good_shot(objects) -> bool:
    cam = objects["camera"]
    angle, dist = aim(objects)
    return bool(tilt_deg(cam) <= LEVEL_DEG and angle <= AIM_DEG and AIM_DIST[0] <= dist <= AIM_DIST[1]
                and cam.resting(max_speed=STEADY, max_spin=math.inf))


def print_out(objects) -> bool:
    """No part of the print is in the camera or on the slot's lower lip any more."""
    q = print_in_camera(objects)
    return bool(q[0] - PRINT[0] / 2 >= FRONT + OUT or abs(q[1]) >= CAM_SIZE[1] / 2 + PRINT[1] / 2 or q[2] >= FLOOR + T["gap"] + 0.01
                or q[2] <= -0.01 or q[0] <= -CAM_SIZE[0] / 2 - PRINT[0] / 2)


def hands_off(objects, name: str) -> bool:
    return all(box_distance(objects[name], tcp) >= CLEAR for tcp in objects.tcp.values())


def print_face_down(objects) -> bool:
    """Out of the camera, flat on the bench with the picture side down (the maker's instruction), still, let go."""
    pr = objects["print"]
    return bool(print_out(objects) and picture_up(pr) <= -math.cos(math.radians(FACE_DOWN_DEG)) and pr.low[2] <= TOP + ON_BENCH
                and pr.resting(max_speed=0.01, max_spin=math.inf) and hands_off(objects, "print"))


def camera_standing(objects) -> bool:
    cam = objects["camera"]
    return bool(tilt_deg(cam) <= LEVEL_DEG and cam.low[2] <= TOP + ON_BENCH and cam.resting(max_speed=0.01, max_spin=math.inf)
                and hands_off(objects, "camera"))


# --- what the camera does
def waited(state: dict) -> bool:
    """The maker's order: the shield lifted SHIELD_S or more after the print came out, before it is drawn free."""
    f, drawn = state["freed_t"], state["drawn_t"]
    return f is not None and state["out_t"] is not None and f - state["out_t"] >= SHIELD_S and (drawn is None or f <= drawn)


def shield_press(t: float, state: dict) -> float:
    """How hard (N) the shield presses the print once it is out: the friction it makes (SHIELD_MU x this) holds the
    print up to HOLD_N while it is still and resists with DRAG_N while it slides; the press rises from 0 over
    SHIELD_RAMP_S as the shield comes down; nothing once the shield has rolled back (``freed_t`` set)."""
    if not state["came_out"] or state["freed_t"] is not None:
        return 0.0
    rise = min(1.0, max(0.0, (t - state["out_t"]) / SHIELD_RAMP_S))
    return rise * (DRAG_N if state["sliding"] else HOLD_N) / SHIELD_MU


def shield_rest(t: float, state: dict, objects) -> float:
    """The shield spring's rest angle (rad, lifted-up positive). While the print is placed: CLEAR_RAD, its weight taken
    off. Once out: under the flap's angle by what the spring must add to the flap's own weight for the rod to press
    with ``shield_press`` (torque = press x PRESS_LEVER). Lifted past LIFT_RAD: STOW_STEP past its angle, at most 5
    degrees past its upper stop: it rolls back and stays there."""
    a, weight = shield_angle(objects), shield_gravity(objects)
    if state["freed_t"] is not None:
        return min(a + STOW_STEP, SHIELD_UP + math.radians(5.0))
    if not state["came_out"]:
        return CLEAR_RAD + weight / SHIELD_K
    return a - (shield_press(t, state) * PRESS_LEVER - weight) / SHIELD_K


def events(t: float, state: dict, world) -> None:
    """The camera's mechanism, once per control step. Until the button is let go after the shot the print waits inside;
    then the rollers push it out over EJECT_S: it is placed in the tunnel each step, the shield standing clear and its
    weight carried (``forces``). Once out nothing places it: the nip closes behind it and it lies where physics has it,
    held by the ridge's friction, the spring set to press with ``shield_press``. Lifted past LIFT_RAD (``freed_t``),
    the shield rolls back: the press is gone."""
    objects = world.objects
    if objects is None:
        return
    cam = objects["camera"]
    if not state["came_out"]:
        rel = state["release_t"]
        if rel is not None and t >= rel + EJECT_S:
            state["came_out"], state["out_t"], depth = True, rel + EJECT_S, EJECTED_X
        elif rel is not None:
            s = (t - rel) / EJECT_S
            depth = HIDDEN_X + s * s * (3 - 2 * s) * (EJECTED_X - HIDDEN_X)
        else:
            depth = HIDDEN_X
        world.set_pose("print", to_world(cam, (depth, 0.0, PRINT_Z)), quat=tuple(float(v) for v in cam.quat_xyzw))
        state["sliding"] = False
    else:
        depth = float(print_in_camera(objects)[0])
        if state["freed_t"] is None and shield_angle(objects) > LIFT_RAD:
            state["freed_t"] = t
        if state["drawn_t"] is None and print_out(objects):
            state["drawn_t"] = t  # first drawn free: a lift after this is out of the maker's order
        prev, prev_t = state["_depth"], state["_depth_t"]
        state["sliding"] = bool(prev is not None and t > prev_t and abs(depth - prev) > SLIDE_V * (t - prev_t))
    state["_depth"], state["_depth_t"] = depth, t
    state["shield_n"] = shield_press(t, state)
    state["shield_rest"] = shield_rest(t, state, objects)  # logged, lifted-up positive: the flap's torque is K (angle - this)
    world.set_joint_target("camera", "shield", -state["shield_rest"])  # the joint's own positive is the free edge down
    world.set_joint_target("camera", "rollers", NIP_SHUT if state["came_out"] else NIP_OPEN)


def forces(objects, state) -> dict:
    """While the rules place the print (inside, and coming out) its weight is carried, with the reaction on the camera,
    so that it neither drops between two placings nor presses on the floor; once it is out, no force at all: the
    shield's friction holds it. A force once pushed stays until the next push, so both are named every step."""
    w = 0.0 if state["came_out"] else P["mass"] * GRAVITY
    return {"print": (0.0, 0.0, w), "camera": (0.0, 0.0, -w)}


# --- scoring (privileged state)
MILESTONES = ("photo_taken", "print_out", "print_face_down", "camera_standing_hands_off")
WEIGHTS = (0.25, 0.25, 0.25, 0.25)
# the photo and the print drawn free count once reached; the end states only while they hold, the last (success)
# after it has held SUCCESS_HOLD_S (rta.spec)
LATCHED = (True, True, False, False)
SUCCESS_HOLD_S = 3.0
ZERO_ON: set = set()


def _track(objects, state) -> None:
    """Once per step: the shutter fires on the first press home; the print comes out when the button is let go."""
    if state.get("_t") == objects.t:
        return
    state["_t"] = objects.t
    pressed_in = shutter_in(objects)
    if state["shot_t"] is None and state["fail"] is None and pressed_in >= PRESSED:
        if good_shot(objects):
            state["shot_t"] = objects.t
        else:
            state["fail"] = "wasted_exposure"
    if state["shot_t"] is not None and state["release_t"] is None and pressed_in <= RELEASED:
        state["release_t"] = objects.t


def milestones(objects, state) -> list[bool]:
    _track(objects, state)
    taken = state["shot_t"] is not None
    out = taken and state["came_out"] and print_out(objects)
    down = out and print_face_down(objects)
    return [taken, out, down, down and camera_standing(objects) and waited(state)]


def success(objects, state) -> bool:
    return milestones(objects, state)[-1]


def failure(objects, state) -> str | None:
    _track(objects, state)
    if state["fail"]:
        return state["fail"]
    cam = objects["camera"]
    if tilt_deg(cam) > KNOCKED_DEG and hands_off(objects, "camera") and cam.low[2] <= TOP + ON_BENCH:
        return "camera_knocked_over"  # lying on the bench, let go (a held camera may be turned any way)
    if any(objects[n].high[2] < TOP - 0.05 for n in ("camera", "print")):
        return "dropped_off_bench"
    return None


def demo(state) -> dict:
    """The scripted full-score run (photo_demo.py): the shot with the camera standing, then the camera held lens up
    while the other hand waits out the 5 s, lifts the shield's free edge so it rolls back, draws the print out and
    lays it face down."""
    return photo_demo.plan(CAM_XY, TOP, PROTRUDE, SUCCESS_HOLD_S)
