"""Bluetooth pairing. On the bench, in front of the two arms: on the left a Bluetooth speaker (fixed, 10 x 20 x
7 cm, fabric body, 22 cm out, 18 cm left of the centre line). Its rubber top panel is printed SOUNDBOX MINI and
carries three round sprung buttons, 2.6 cm across, in a row 5 cm apart: power (red symbol) on the left, play /
pause in the middle, pairing (blue Bluetooth symbol) on the right, and a round status light right of the pairing
button, at the speaker's right end. On the right a phone lying flat (fixed, 16.4 x 7.8 x 0.9 cm, 20 cm out, 17 cm right of the centre line,
its top end pointing away from the arms) showing its Bluetooth settings page with a "Pair new device" button at
the bottom of the screen.

The speaker's light is steady white (on, not pairing). Holding the pairing button down for 3 s puts the speaker
in pairing mode for 30 s: the light blinks blue. Tapping "Pair new device" makes the phone search for 1.5 s and
then list the devices it can see, one per row: "Kitchen TV" and "Mara's Earbuds" (other people's devices)
always, and "SoundBox Mini" only while the speaker is in pairing mode, from 2 s after that mode began. Which row each device takes changes
between trials. Tapping "SoundBox Mini" starts the connection: the phone shows "Connecting…" for 2 s, then shows the speaker
as connected, and the speaker's light turns steady green. Tapping one of the other two devices pairs the phone with somebody else's device and sets the
score to zero. Pressing the power button switches the speaker off and ends the episode. A button is pressed by
pushing its cap down 2 mm; the screen is tapped with a closed fingertip. Finish with both grippers off the
buttons and the screen.
"""

from __future__ import annotations

import json
from pathlib import Path

from yam_arena.constants import FINGER_CLOSED

from rta.plans import press, wait
from rta.spec import TABLE_TOP, Asset

SCENE = "station"
TOP = TABLE_TOP[SCENE]
REST = TOP + 0.001  # models rest 1 mm over the bench (yam_arena station.REST_GAP)
L = json.loads((Path(__file__).resolve().parent / "assets" / "layout.json").read_text())  # written by assets/build.py
SPEAKER_NAME = L["speaker_name"]
INSTRUCTION = f"Pair the {SPEAKER_NAME} Bluetooth speaker with the phone."
ACT_AFTER_S = 1.0  # nothing to watch: the arms are free after a short settle

# --- speaker (left) ----------------------------------------------------------------------------------------------
SPK_XY = tuple(L["speaker"]["xy"])
BUTTONS = {b: tuple(xy) for b, xy in L["buttons"].items()}  # pair, play, power
CAP_TOP = REST + L["button"]["top"]  # a cap's top at rest
TRAVEL = L["button"]["travel"]
PRESSED = 0.002  # a button is down when its cap is this far below its rest height

# --- phone (right) -----------------------------------------------------------------------------------------------
PHONE_XY = tuple(L["phone"]["xy"])
GLASS = REST + L["phone"]["top"]
ZONE = tuple(v / 2 for v in L["screen"]["zone"])  # half-size of a touch zone: 1.6 cm along the phone, 3.5 cm across
SCAN_XY = tuple(L["scan_button"])  # "Pair new device" / "Scan again", at the bottom of the screen
ROW_XY = tuple(tuple(xy) for xy in L["rows"])  # row 0 is the top of the list
DEVICES = tuple(L["devices"])  # "speaker" is this speaker; the others belong to somebody else
ORDERS = [list(o) for o in L["orders"]]  # 6 row orders
TIP = 0.013  # the closed fingertips reach this far below the tool centre point
TOUCH = 0.005  # a tap: the fingertips no more than this above the glass

# --- timing ------------------------------------------------------------------------------------------------------
HOLD_S = 3.0  # the pairing button must stay down this long
WINDOW_S = 30.0  # pairing mode lasts this long
SCAN_S = 1.5  # the list appears this long after the search starts
DISCOVER_S = 2.0  # the phone finds the speaker this long after pairing mode began
CONNECT_S = 2.0  # the phone shows "Connecting…" this long before it confirms
BLINK_S = 0.4  # blue light: on 0.4 s, off 0.4 s

A = L["assets"]
OBJECTS = [
    Asset("speaker", asset=A["speaker"], xy=SPK_XY, extent=L["extent"]["speaker"]),
    *(Asset(f"button_{b}", asset=A[f"button_{b}"], xy=xy, extent=L["extent"]["button"], mass=0.01) for b, xy in BUTTONS.items()),
    Asset("phone", asset=A["phone"], xy=PHONE_XY, extent=L["extent"]["phone"]),
]
MOUNTED = {(f"button_{b}", "speaker") for b in BUTTONS}  # built into each other (tests/test_layouts.py)
LEDS = ("led_white", "led_blue", "led_green")
SCREENS = ("screen_home", "screen_scan", *(f"screen_list_{k}_{s}" for k in range(len(ORDERS)) for s in (0, 1)), "screen_connecting", "screen_connected")


def setup(rng, trial) -> dict:
    """The row order of the three devices cycles through its 6 permutations with the seed number, so tapping a
    fixed row connects the right device one time in three."""
    k = trial.balanced(len(ORDERS), rng)
    return {
        "order": k,
        "rows": list(ORDERS[k]),
        "mode": "standby",  # the speaker: standby | pairing | connecting | connected | off
        "pair_t": None,  # when pairing mode began
        "hold_t0": None,  # when the current press of the pairing button began
        "page": "home",  # the phone: home | scan | list | connecting | connected
        "connect_t": None,  # when the connection is confirmed
        "list_t": None,  # when the search ends
        "down": [],  # buttons down and screen zones touched at the last step
        "fail": None,
        "was_pairing": False,
    }


# --- what the devices show ---------------------------------------------------------------------------------------
def discoverable(t: float, state: dict) -> bool:
    """The phone can see the speaker: in pairing mode for at least DISCOVER_S."""
    return state["mode"] == "pairing" and t - state["pair_t"] >= DISCOVER_S


def listed(t: float, state: dict) -> list[str]:
    """Devices in the phone's list at time t."""
    if state["page"] != "list":
        return []
    return [d for d in state["rows"] if d != "speaker" or discoverable(t, state)]


def led(t: float, state: dict) -> str | None:
    """Which light is lit at time t (None: dark)."""
    mode = state["mode"]
    if mode == "standby":
        return "led_white"
    if mode == "connected":
        return "led_green"
    if mode in ("pairing", "connecting"):
        return "led_blue" if int((t - state["pair_t"]) / BLINK_S) % 2 == 0 else None
    return None


def screen(t: float, state: dict) -> str:
    """Which of the phone's screens is up at time t."""
    if state["page"] == "list":
        return f"screen_list_{state['order']}_{int(discoverable(t, state))}"
    return f"screen_{state['page']}"


def events(t: float, state: dict, world) -> None:
    """Switch the speaker's light and the phone's screen to what the state says (only on a change)."""
    shown = state.setdefault("_shown", {})
    lit, up = led(t, state), screen(t, state)
    for name, parts, on in (("speaker", LEDS, lit), ("phone", SCREENS, up)):
        for part in parts:
            want = part == on
            if shown.get(part) != want:
                world.show(name, part, want)
                shown[part] = want


# --- scoring (privileged state) ----------------------------------------------------------------------------------
MILESTONES = ("pairing_mode", "speaker_listed", "connected")
WEIGHTS = (0.25, 0.25, 0.5)
SUCCESS_HOLD_S = 2.0  # the confirmation stays up, hands off, before the episode ends
ZERO_ON = {"wrong_device"}
ZONES = {"scan": SCAN_XY, **{f"row{i}": xy for i, xy in enumerate(ROW_XY)}}


def down(objects, state) -> list[str]:
    """Buttons whose cap is pushed down, and screen zones with a fingertip on them, this step."""
    rest = state.setdefault("_rest", {b: float(objects[f"button_{b}"].high[2]) for b in BUTTONS})
    out = [b for b in BUTTONS if objects[f"button_{b}"].high[2] <= rest[b] - PRESSED]
    for zone, (x, y) in ZONES.items():
        if any(abs(tcp[0] - x) <= ZONE[0] and abs(tcp[1] - y) <= ZONE[1] and tcp[2] - TIP <= GLASS + TOUCH for tcp in objects.tcp.values()):
            out.append(zone)
    return out


def _track(objects, state) -> None:
    """Advance the speaker and the phone by one control step (once per time stamp)."""
    t = objects.t
    if state.get("_t") == t:
        return
    state["_t"] = t
    now = down(objects, state)
    new = [c for c in now if c not in state["down"]]
    state["down"] = now
    if state["mode"] in ("connected", "off"):
        return
    if state["mode"] == "connecting":  # nothing else responds until the phone confirms
        if t >= state["connect_t"]:
            state["mode"] = state["page"] = "connected"
        return
    # the speaker
    if "power" in new:
        state["mode"], state["fail"] = "off", "speaker_switched_off"
        return
    if state["mode"] == "pairing" and t - state["pair_t"] >= WINDOW_S:
        state["mode"] = "standby"
    if "pair" in now:
        if state["hold_t0"] is None:
            state["hold_t0"] = t
        if state["mode"] == "standby" and t - state["hold_t0"] >= HOLD_S:
            state["mode"], state["pair_t"], state["was_pairing"] = "pairing", t, True
            state["hold_t0"] = None  # one hold starts pairing once; a new 3 s hold restarts it after the window
    else:
        state["hold_t0"] = None
    # the phone: a tap is judged when it begins, against the screen that was up then
    visible = listed(t, state)
    if state["page"] == "scan" and t >= state["list_t"]:
        state["page"] = "list"
    if "scan" in new and state["page"] in ("home", "list"):
        state["page"], state["list_t"] = "scan", t + SCAN_S
        return
    for i in range(len(ROW_XY)):
        if f"row{i}" in new and state["rows"][i] in visible:
            if state["rows"][i] == "speaker":
                state["mode"], state["page"], state["connect_t"] = "connecting", "connecting", t + CONNECT_S
            else:
                state["fail"] = "wrong_device"
            return


def milestones(objects, state) -> list[bool]:
    _track(objects, state)
    connected = state["mode"] == "connected"
    return [state["was_pairing"], connected or "speaker" in listed(objects.t, state), connected]


def success(objects, state) -> bool:
    _track(objects, state)
    return state["mode"] == "connected" and not state["down"]


def failure(objects, state) -> str | None:
    _track(objects, state)
    return state["fail"]


# --- privileged oracle -------------------------------------------------------------------------------------------
TRAVEL_H = 0.14
BTN_PRESS_Z = CAP_TOP - TRAVEL - 0.001 + TIP  # fingertips 1 mm past the cap's end stop
TAP_Z = GLASS + TIP + 0.0005  # fingertips half a millimetre above the glass
LIST_UP_S = 2.0  # after the search tap: the search ends and the list (without the speaker) is on the screen
APPEAR_S = 1.0  # after the left arm is up: the speaker has joined the list and stays in view before the tap
END_S = 4.5  # the phone confirms 2 s after the tap; then 2 s with the hands off


def demo(state) -> dict:
    """One step at a time, each arm clear of its device before the other moves: the right arm taps "Pair new
    device" and the phone lists the other two devices; the left arm holds the pairing button until the light
    blinks, then lifts away; the speaker joins the list; the right arm taps its row."""
    search = press((*SCAN_XY, TAP_Z), travel_z=TOP + TRAVEL_H)
    hold = press((*BUTTONS["pair"], BTN_PRESS_Z), travel_z=REST + L["speaker"]["top"] + TRAVEL_H, press_s=HOLD_S + 1.0)
    entry = press((*ROW_XY[state["rows"].index("speaker")], TAP_Z), travel_z=TOP + TRAVEL_H)
    d_search, d_hold, d_entry = (sum(w.duration for w in p) for p in (search, hold, entry))
    left_start = d_search + LIST_UP_S
    right_idle = LIST_UP_S + d_hold + APPEAR_S
    return {
        "left": (0.0, [wait("left", left_start, finger=FINGER_CLOSED), *hold, wait("left", APPEAR_S + d_entry + END_S, finger=FINGER_CLOSED)]),
        "right": (0.0, [*search, wait("right", right_idle, finger=FINGER_CLOSED), *entry, wait("right", END_S, finger=FINGER_CLOSED)]),
    }
