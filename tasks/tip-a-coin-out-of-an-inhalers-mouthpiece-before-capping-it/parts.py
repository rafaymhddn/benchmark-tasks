"""The inhaler task's product figures and shapes, one source each: the Alvesco 160 Mikrogramm metered-dose inhaler
(Zentiva; its boot with the canister), its mouthpiece cap, a euro 1 cent coin wedged edge first in the round
mouthpiece, and the leather tray.

Sources: maker = Zentiva's product page and package leaflet (Gebrauchsinformation); study = de Vries et al., Respir Med
2009;103:1167-1173, Table 1 (doi 10.1016/j.rmed.2009.02.010); standard = the European Central Bank's coin figures;
listing = the tray's retailer page; estimate = derived as stated beside it.

The inhaler's frame I: origin where the mouthpiece's axis meets the barrel's axis; x along the mouthpiece, out of the
mouth; z along the barrel, towards the canister's end; y = z x x (up when the inhaler lies on its side).
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os

import numpy as np
from mechkit import box, cylinder, matrix_to_quat


@dataclass(frozen=True)
class CoinSpec:
    """A coin's published figures (SI: m, kg) and its rendered sRGB colour."""

    label: str
    diameter: float
    thickness: float
    mass: float
    color: tuple[float, float, float]
    source: str


COPPER = (0.72, 0.43, 0.29)  # copper-covered steel, drawn without sheen
DIME_SILVER = (0.45, 0.47, 0.50)
COIN_PRESETS: dict[str, CoinSpec] = {
    "eur_1c": CoinSpec("euro 1 cent", 0.01625, 0.00167, 0.00230, COPPER, "ECB, Common sides: Diameter (mm): 16.25, Thickness (mm): 1.67, Weight (g): 2.30"),
    "us_dime": CoinSpec("US dime", 0.01791, 0.00135, 0.002268, DIME_SILVER, "US Mint coin specifications: 17.91 mm, 1.35 mm, 2.268 g"),
}
COIN_PRESET = os.environ.get("INHALER_COIN_PRESET", "eur_1c")  # standard: the euro 1 cent; the dime only for the dry check
_COIN = COIN_PRESETS[COIN_PRESET]
COIN_D, COIN_T, COIN_MASS, COIN_COLOR = _COIN.diameter, _COIN.thickness, _COIN.mass, _COIN.color
COIN_R = 0.5 * COIN_D
LODGED_R, LODGED_T, LODGED_MASS, LODGED_COLOR = COIN_R, COIN_T, COIN_MASS, COIN_COLOR  # the coin in the mouthpiece is one of the four

COIN_FRICTION = 0.3  # estimate: the coin on plastic and on the bench (static = dynamic); min-combined, so also on the pads' rubber
THIN = 0.0005  # m contact offset of the coin, cap and boot (the kit's 2 mm exceeds the coin's thickness)
GRAVITY = 9.81  # m/s^2

# --- the inhaler: Alvesco 160 Mikrogramm Druckgasinhalation, Losung (Zentiva) ---
BORE_D = 0.016  # study, Table 1: the ciclesonide 160 inhaler's mouthpiece "Circular", 16 mm diameter, taken as the bore (our reading)
BORE_R = 0.5 * BORE_D
BORE_DEPTH = 0.033  # study, Table 1: nozzle to mouthpiece end 33 mm
WALL = 0.0012  # estimate: the mouthpiece's moulded wall
MOUTH_R_OUT = BORE_R + WALL  # estimate: 18.4 mm outside (16 mm bore + 2 walls)
BARREL_D = 0.030  # estimate: the boot's barrel round a 22 mm canister
BARREL_R = 0.5 * BARREL_D
BOOT_LOW = -0.015  # estimate: m, the boot's bottom along z, below the mouthpiece's axis
BOOT_H = 0.060  # estimate: the barrel from its bottom to its top rim
BOOT_HIGH = BOOT_LOW + BOOT_H
NOZZLE_X = 0.003  # estimate: the spray nozzle 3 mm in front of the barrel's axis
MOUTH_X = NOZZLE_X + BORE_DEPTH  # the mouth's end in I: 36 mm from the barrel's axis
MOUTH_OUT = MOUTH_X - BARREL_R  # the mouthpiece stands 21 mm out of the barrel (the boot is drawn solid behind it)
CAN_D = 0.022  # estimate: the canister
CAN_R = 0.5 * CAN_D
INHALER_H = 0.080  # estimate: from the boot's bottom to the canister's top end
CAN_TOP = BOOT_LOW + INHALER_H
CAN_LOW = BOOT_HIGH - 0.010  # the canister reaches 10 mm into the barrel
STAVES = 16  # flat staves round the bore and in the cap, their inner faces tangent to the stated circles
MOUTH_FRICTION = 0.3  # estimate: plastic on plastic; the mouthpiece's staves, min-combined (the cap slides on them, the coin's edge sits in them)
INHALER_MASS = 0.030  # estimate: kg, boot with its filled canister
BOOT_RED, CAN_GREY = (0.50, 0.07, 0.09), (0.78, 0.79, 0.81)  # look only

# --- the coin wedged edge first in the round mouthpiece (geometry) and its hold (estimate) ---
COIN_IN_I = np.column_stack([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])  # the coin's axis along I's y: its faces level as the inhaler lies


def middle_out(radius: float, thick: float, bore_r: float = BORE_R) -> float | None:
    """m by which an edge-first coin's middle stays outside the mouth's plane: its cross-section there (a chord of
    the disc by its thickness) fits the bore when r^2 - h^2 + (t/2)^2 = R^2. None: it is narrow enough to pass in."""
    h2 = radius**2 + (0.5 * thick) ** 2 - bore_r**2
    return math.sqrt(h2) if h2 > 0.0 else None


LODGE_MID = middle_out(COIN_R, COIN_T)  # 1.647 mm
if LODGE_MID is None:
    raise ValueError(f"the {_COIN.label} passes into a {1000 * BORE_D:.1f} mm bore: nothing to wedge")
LODGE_OUT = COIN_R + LODGE_MID  # 9.772 mm: how far it stands out of the mouth (a ruler checks the start)
LODGE_IN = COIN_R - LODGE_MID  # 6.478 mm: how deep it reaches into the bore
RIM_DEG = math.degrees(math.asin(LODGE_MID / COIN_R))  # 11.7 deg: the lip meets the coin's rim this far off square
LIP_FRICTION = COIN_FRICTION  # estimate: the coin's edge on the lip's plastic
SEAT_PUSH = 5.0  # estimate: N, the thumb push that seats the coin at a reset


def wedge_hold(push: float = SEAT_PUSH, mu: float = LIP_FRICTION, rim_deg: float = RIM_DEG) -> float:
    """N to draw the coin out after ``push`` seated it: the lip presses along the rim's normal, tilted ``rim_deg`` off
    square, so the push loads it with N = push / (mu cos a + sin a) and the draw needs N (mu cos a - sin a); a coin
    with mu < tan a springs out by itself (returns 0)."""
    a = math.radians(rim_deg)
    return max(0.0, push * (mu * math.cos(a) - math.sin(a)) / (mu * math.cos(a) + math.sin(a)))


HOLD = wedge_hold()  # 0.92 N: the coin's hold in the lip (estimate)
DRAW = 0.001  # m: the draw against HOLD after which the simulator lets the coin go (a wedge gives within its first millimetre)

# lying on its side, mouthpiece towards +y: I's x to the world's +y, y up, z (the barrel) to the world's +x
LYING = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # columns: I's x, y, z in the world
LYING_QUAT = matrix_to_quat(LYING)

# --- the cap: a round cup over the mouthpiece. Frame C: x out of its opening; origin at the middle of the cup ---
CAP_IN_R = 0.0095  # estimate: 19.0 mm inside, 0.3 mm play a side round the 18.4 mm mouthpiece
CAP_PLAY = CAP_IN_R - MOUTH_R_OUT  # 0.3 mm a side
CAP_WALL = 0.0015  # estimate: a moulded wall (thinner walls let the pads' pinch pass through them in the simulator)
CAP_OUT_R = CAP_IN_R + CAP_WALL  # 22.0 mm outside
CAP_DEPTH = 0.016  # estimate: outside depth along x
CAP_END = 0.003  # estimate: its closed end (solid, where the pads take it)
CAP_HEADSPACE = 0.0005  # estimate: m, the cap's closed end clear of the mouth when its rim is at its stop
CAP_SLIDE = CAP_DEPTH - CAP_END - CAP_HEADSPACE  # 12.5 mm from rim-at-mouth until its rim meets the mouthpiece's shoulder (the seat)
COLLAR = 0.002  # estimate: m, the shoulder round the mouthpiece where the cap's rim stops (its length along the mouthpiece)
LEAD, LEAD_LEN = 0.001, 0.003  # estimate: the rim's lead-in flare and its length
CAP_CLICKS = True  # maker's leaflet: "Stecken Sie die Kappe mit festem Druck auf bis sie einrastet."
CAP_CLICK_FORCE = 5.0  # estimate: N, the firm push to click it home (the seat weld waits for it: capfit.py)
CAP_HOLD_FORCE = 5.0  # estimate: N of pull-off, stood in for by the seat weld
SEAT_GAP = 0.001  # m: seated when within 1 mm of the seat
SEAT_AXIS_DEG = 10.0  # deg: cap axis within 10 deg of the mouthpiece's
SEAT_OFF = 0.001  # m: cap middle within 1 mm of the mouthpiece's axis
CAP_MASS = 0.0022  # estimate: kg, its polypropylene (0.9 g/cm3) volume
CAP_COLOR = (0.90, 0.90, 0.88)  # look only
STOP_SHORT = LODGE_OUT - CAP_HEADSPACE  # 9.27 mm: the cap's closed end meets the standing coin's edge this far short of its seat


def cap_headspace() -> float:
    """m between the mouth's end and the inside of the cap's closed end when the cap is seated."""
    return CAP_HEADSPACE


def cap_x_at_gap(gap: float = 0.0) -> float:
    """I's x of the cap's middle when it stands ``gap`` short of its seat along the mouthpiece."""
    return MOUTH_X + 0.5 * CAP_DEPTH - CAP_SLIDE + gap


def capped_middle_in_i() -> np.ndarray:
    """Middle of the seated inhaler-and-cap outline in I (for centring in the tray)."""
    front_x = cap_x_at_gap(0.0) + 0.5 * CAP_DEPTH
    return np.array([0.5 * (-BARREL_R + front_x), 0.0, 0.5 * (BOOT_LOW + CAN_TOP)])


def ring(r_flat: float, n: int = STAVES) -> np.ndarray:
    """Corners in I's (y, z) of the regular n-gon whose sides touch the circle ``r_flat`` at angles 2 pi k / n from
    I's y (so a side's middle lies on I's +-z, where the edge-first coin's corners are), counter-clockwise."""
    phi = 2.0 * math.pi * (np.arange(n) + 0.5) / n
    return (r_flat / math.cos(math.pi / n)) * np.c_[np.cos(phi), np.sin(phi)]


def coin_lodged(out_m: float | None = None) -> np.ndarray:
    """I's coordinates of the wedged coin's middle: on the mouthpiece's axis, ``out_m`` (default LODGE_OUT) of it
    standing out of the mouth."""
    out = LODGE_OUT if out_m is None else out_m
    return np.array([MOUTH_X - LODGED_R + out, 0.0, 0.0])


def check_coin_lodge(coin: CoinSpec | str, bore_d: float = BORE_D) -> dict:
    """Dry check: how ``coin`` wedges edge first in a round bore of ``bore_d``, and whether the wedge holds (mm, deg, N)."""
    spec = COIN_PRESETS[coin] if isinstance(coin, str) else coin
    r, t = 0.5 * spec.diameter, spec.thickness
    h = middle_out(r, t, 0.5 * bore_d)
    out = {"label": spec.label, "size_mm": [round(1000 * spec.diameter, 2), round(1000 * t, 2)], "mass_g": round(1000 * spec.mass, 3), "bore_mm": round(1000 * bore_d, 2)}
    if h is None:
        return {**out, "wedges": False, "reason": f"passes into the bore (needs {2000 * math.hypot(r, 0.5 * t):.2f} mm)"}
    rim = math.degrees(math.asin(h / r))
    return {**out, "wedges": True, "out_mm": round(1000 * (r + h), 2), "in_mm": round(1000 * (r - h), 2), "rim_deg": round(rim, 1),
            "holds_by_itself": LIP_FRICTION > math.tan(math.radians(rim)), "hold_N": round(wedge_hold(rim_deg=rim), 3)}  # fmt: skip


def check_all_coins() -> dict[str, dict]:
    return {key: check_coin_lodge(spec) for key, spec in COIN_PRESETS.items()}


def staves(r_flat: float, x0: float, x1: float, thick: float, color, n: int = STAVES, passes=(), flare: float = 0.0, seam: float = 0.0004,
           friction=None) -> list:  # fmt: skip
    """A tube along I's x from ``x0`` to ``x1`` of ``n`` flat staves whose inner faces touch the circle ``r_flat``
    (corners as ``ring``); with ``flare`` the staves lean out so that the inner wall is ``flare`` further out at ``x1``.
    Each stave is ``seam`` longer than its outer face, so that neighbours overlap (0: their outer corners meet)."""
    pts = ring(r_flat, n)
    overlap = 2.0 * thick * math.tan(math.pi / n) + seam  # closes the wedge between neighbours' outer corners
    lean = math.atan2(flare, x1 - x0)
    out = []
    for p, q in zip(pts, np.roll(pts, -1, axis=0)):
        t = (q - p) / np.linalg.norm(q - p)
        nrm = np.array([t[1], -t[0]])  # outwards (the polygon runs counter-clockwise)
        mid = 0.5 * (p + q) + (0.5 * thick + 0.5 * flare) * nrm
        along = np.array([math.cos(lean), math.sin(lean) * nrm[0], math.sin(lean) * nrm[1]])
        out_n = np.array([-math.sin(lean), math.cos(lean) * nrm[0], math.cos(lean) * nrm[1]])
        rot = np.column_stack([along, out_n, [0.0, t[0], t[1]]])
        length = (x1 - x0) / math.cos(lean)
        out.append(box((length, thick, float(np.linalg.norm(q - p)) + overlap), pos=(0.5 * (x0 + x1), mid[0], mid[1]), quat=matrix_to_quat(rot),
                       color=color, passes=passes, friction=None if friction is None else (friction, friction),
                       friction_combine=None if friction is None else "min"))  # fmt: skip
    return out




def inhaler_shapes() -> list:
    """Boot (solid barrel), canister, the mouthpiece's 16 staves from inside the barrel to the mouth, and the shoulder
    round the mouthpiece where the cap's rim stops."""
    shapes = [cylinder(BARREL_R, BOOT_HIGH - BOOT_LOW, pos=(0.0, 0.0, 0.5 * (BOOT_LOW + BOOT_HIGH)), axis="z", color=BOOT_RED),
              cylinder(CAN_R, CAN_TOP - CAN_LOW, pos=(0.0, 0.0, 0.5 * (CAN_LOW + CAN_TOP)), axis="z", color=CAN_GREY)]  # fmt: skip
    shapes += staves(BORE_R, BARREL_R - 0.004, MOUTH_X, WALL, BOOT_RED, seam=0.0, friction=MOUTH_FRICTION)  # outside exactly the 16-gon round 18.4 mm
    stop = MOUTH_X - CAP_SLIDE  # the shoulder's face, where the cap's rim stops
    rim_out = CAP_OUT_R + LEAD  # the rim's outside, flared
    shapes += staves(MOUTH_R_OUT, stop - COLLAR, stop, rim_out + 0.0005 - MOUTH_R_OUT, BOOT_RED, seam=0.0)
    return shapes


def coin_shapes() -> list:
    return [cylinder(COIN_R, COIN_T, axis="z", color=COIN_COLOR)]


def lodged_shapes() -> list:
    return [cylinder(LODGED_R, LODGED_T, axis="z", color=LODGED_COLOR)]


def cap_shapes(passes=()) -> list:
    """16 staves round the opening with the lead-in at the rim, and a disc closing the far end."""
    back, rim = -0.5 * CAP_DEPTH, 0.5 * CAP_DEPTH
    shapes = staves(CAP_IN_R, back + CAP_END, rim - LEAD_LEN, CAP_WALL, CAP_COLOR, passes=passes)
    shapes += staves(CAP_IN_R, rim - LEAD_LEN, rim, CAP_WALL, CAP_COLOR, passes=passes, flare=LEAD)
    shapes += [cylinder(CAP_OUT_R, CAP_END, pos=(back + 0.5 * CAP_END, 0.0, 0.0), axis="x", color=CAP_COLOR)]
    return shapes


# --- the tray: Room & Board "Brando" felt-and-leather tray: 7 x 5.5 x 1.5 in outside (178 x 140 x 38 mm), 5 x 3 x 1 in
# inside (127 x 76 x 25 mm) by the retailer's figures. Frame at the middle of its underside. ---
TRAY_L, TRAY_W, TRAY_H = 0.178, 0.140, 0.038  # listing: outside
TRAY_IN_L, TRAY_IN_W, TRAY_IN_H = 0.127, 0.076, 0.025  # listing: inside
TRAY_FLOOR = TRAY_H - TRAY_IN_H  # 13 mm
TRAY_MASS = 0.250  # estimate: kg
TRAY_LEATHER, TRAY_FELT = (0.38, 0.23, 0.14), (0.33, 0.34, 0.36)  # look only


def tray_shapes() -> list:
    """A floor TRAY_FLOOR thick; two side walls along x and two end walls between them, round the inside."""
    side, end, h = 0.5 * (TRAY_W - TRAY_IN_W), 0.5 * (TRAY_L - TRAY_IN_L), TRAY_IN_H
    return [box((TRAY_L, TRAY_W, TRAY_FLOOR), pos=(0.0, 0.0, 0.5 * TRAY_FLOOR), color=TRAY_FELT),
            *[box((TRAY_L, side, h), pos=(0.0, s * 0.5 * (TRAY_W - side), TRAY_FLOOR + 0.5 * h), color=TRAY_LEATHER) for s in (-1.0, 1.0)],
            *[box((end, TRAY_IN_W, h), pos=(s * 0.5 * (TRAY_L - end), 0.0, TRAY_FLOOR + 0.5 * h), color=TRAY_LEATHER) for s in (-1.0, 1.0)]]  # fmt: skip


if __name__ == "__main__":
    print(json.dumps({"bore_mm": round(1000 * BORE_D, 2), "out_mm": round(1000 * LODGE_OUT, 3), "in_mm": round(1000 * LODGE_IN, 3), "rim_deg": round(RIM_DEG, 2),
                      "hold_N": round(HOLD, 3), "cap_stops_short_mm": round(1000 * STOP_SHORT, 2), "coins": check_all_coins()}, indent=1))  # fmt: skip
