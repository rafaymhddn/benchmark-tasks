"""Every figure the MRI tray task rests on, in one place, each with its source; and the shapes built from them.

Source kinds (``FIGURES`` holds every figure with its kind and its source in words; the README's table is made
from it):
- ``maker``: the maker's own published figure (product page or pack specification, opened and quoted).
- ``listing``: a retailer's figure for the bought product (opened and quoted).
- ``our fixture``: a printed mark or part this task specifies to the stated size.
- ``estimate``: our figure, with how it was derived in one clause.

World frame: metres from the midpoint between the arm bases, x forward, y left, z up from the bench. The bench
stands in for the top of the wheeled patient cart, parked at the MRI room door; the polypropylene tray on it is
what the arms work over.
"""

from __future__ import annotations

import lettering

MAKER, LISTING, FIXTURE, ESTIMATE = "maker", "listing", "our fixture", "estimate"
FIGURES: dict[str, tuple[float, str, str, str]] = {}  # name -> (value, unit, source kind, note)

CAMBRO_PAGE = "https://www.cambro.com/Products/trays/restaurant-and-cafeteria-self-serve-trays/fast-food-trays/"
PONY_PAGE = "https://www.ponyjorgensen.com/product/classic-pony-light-duty-c-clamp/"
ABUS_SEARCH = "https://www.abus.com/fr/Securite-a-la-maison/Cadenas/Par-series/41-Eterna/41-30-bleu"  # 403 to fetcher
SHEARS_LISTING = "https://www.fatquartershop.com/gingher-8-inch-knife-edge-dressmaker-shears"
EVIAN_PAGE = "https://www.evian.com/en_us/natural-spring-water/bottled-water/330ml"
DYNAREX_PAGE = ("https://dynarex.com/products/disposable-medical-supplies/patient-care/"
                "4224-emesis-basin-9-500cc-capacity-graduated-in-100cc-increments")
ALLMRI_CLIP = "https://www.allmri.com/MRT-Klemmbrett/MRI-8034"
BIN_SEARCH = "https://www.truevalue.com"  # STANLEY STST55400 (item 253908) search snippet; page not opened by fetcher


def figure(name: str, value: float, unit: str, source: str, note: str) -> float:
    if source not in (MAKER, LISTING, FIXTURE, ESTIMATE):
        raise ValueError(f"{name}: unknown source kind {source!r}")
    FIGURES[name] = (float(value), unit, source, note)
    return float(value)


# --- the tray (Cambro 1418FF polypropylene fast-food tray; PP is MR-safe) -------------------------------------------------
TRAY_W = figure("tray width (y)", 0.450, "m", MAKER, f"Cambro 1418FF '45 x 35 cm' ('17 3/4\" x 13 13/16\"', {CAMBRO_PAGE})")
TRAY_D = figure("tray depth (x)", 0.350, "m", MAKER, "the 35 cm side of Cambro 1418FF on the same maker page")
TRAY_RIM = figure("tray rim height", 0.015, "m", ESTIMATE, "shallow fast-food tray lip scaled against the 45 cm width in Cambro's photo")
TRAY_WALL = 0.002  # m: the tray's wall thickness (estimate)
TRAY_MASS = figure("tray mass", 0.300, "kg", ESTIMATE, "0.32 m2 PP sheet at ~1 mm mean thickness and 910 kg/m3 (Cambro case 7.2 kg includes carton)")
TRAY_AT = (0.305, 0.0)  # m: the tray frame's origin (centre of its base's underside): x 13 to 48 cm, y centred
TRAY_FLOOR = TRAY_WALL  # m over the bench: where the items on the tray rest
TRAY_FRICTION = figure("PP items on the PP tray, friction", 0.40, "-", ESTIMATE, "dry plastic on plastic, the middle of its 0.25-0.5 range")

# --- the marked bin and its zone mat ---------------------------------------------------------------------------------------
BIN_L = figure("bin length (x)", 0.337, "m", ESTIMATE, f"13-1/4 in of STANLEY STST55400 #4 yellow PP bin (listing snippet at {BIN_SEARCH}, not opened)")
BIN_W = figure("bin width (y)", 0.229, "m", ESTIMATE, "9 in width from the same STANLEY STST55400 listing snippet (not opened)")
BIN_H = figure("bin height", 0.149, "m", ESTIMATE, "5-7/8 in height from the same STANLEY STST55400 listing snippet (not opened)")
BIN_WALL = 0.002  # m: the bin's wall thickness (estimate)
BIN_MASS = figure("bin mass", 0.450, "kg", ESTIMATE, "a 34 x 23 x 15 cm PP moulding at about 2 mm mean thickness")
BIN_AT = (0.2715, -0.350)  # m: the bin frame's origin (centre of its base's underside): its left edge 1.05 cm clear of the tray
LABEL_L = figure("bin holding-mat banner, length (y)", 0.400, "m", FIXTURE, "front safe-side banner of the bin's yellow holding mat")
LABEL_W = figure("bin holding-mat banner, depth (x)", 0.168, "m", FIXTURE, "three lines of block letters plus green border on the near side")
LABEL_AT = (0.032, 0.000)  # m: near-side banner centred between the arm bases, connected continuously under the bin
LABEL_LETTER_CELL = figure("holding-mat letter cell", 0.0042, "m", FIXTURE, "30 mm block letters (~7 pixel rows at 320 x 176 on the near side)")
LABEL_LETTER = "MR UNSAFE ITEMS"
LABEL_LINE2 = "HOLD HERE"
LABEL_LINE3 = "OUTSIDE ZONE IV"
BIN_HOLD_TEXT, BIN_HOLD_CELL, BIN_HOLD_X = "HOLD", 0.0070, 0.110  # 49 mm flat text on the bin's floor, centred in right_cam

# --- the doorway sign (our fixture: on the cart's far side at the doorway it is about to go through) ----------------------
SIGN_L = figure("sign mat, long side", 0.360, "m", FIXTURE, "the ZONE IV line of letters with a margin at each end")
SIGN_W = figure("sign mat, short side", 0.190, "m", FIXTURE, "both lines of letters plus the red frame border")
SIGN_AT = (0.755, 0.140)  # m: on the cart's far side at the doorway ahead (+x, +y), opposite the near-right holding bin
DOOR_LINE_X = 0.625  # m: full-bench red doorway threshold line across y = +-0.645 m, separating Zone IV from the safe side
BENCH_X = (-0.190, 0.910)  # m: front and back edges of the 1.10 x 1.30 m station bench centred at (0.360, 0.0)
BENCH_HALF_Y = 0.650  # m: half-width of the station bench
SIGN_LINE1, SIGN_CELL1 = "MRI", 0.0137  # the big line: 96 mm letters, about 21 pixel rows in the policy picture
SIGN_LINE2, SIGN_CELL2 = "ZONE IV", 0.0079  # 55 mm letters, about 12 pixel rows
SIGN_FRAME = 0.007  # m: the red border drawn inside the sign mat's edge

# --- the steel items (the ferromagnetic ones; each named, its material from its maker or listing) --------------------------
CLAMP_CAPACITY = figure("clamp capacity", 0.0508, "m", MAKER, f"Pony 2620 ('2\" | 400 lb. | 1\"'), ductile iron ({PONY_PAGE})")
CLAMP_STAND = figure("clamp standing height", 0.120, "m", ESTIMATE, "maker's photo scaled against the 2 in opening: foot, C-frame, screw and T-handle")
CLAMP_FRAME_T = figure("clamp frame thickness", 0.014, "m", ESTIMATE, "same photo: the C's flat back is about a quarter of the 2 in opening")
CLAMP_MASS = figure("clamp mass", 0.290, "kg", ESTIMATE, "about 40 cm3 of ductile iron at 7.3 g/cm3, from the scaled photo")
CLAMP_AT = (0.285, -0.150)  # m: the clamp's frame origin (centre of its foot's underside), standing on the tray, T-handle up

LOCK_W = figure("padlock width", 0.035, "m", ESTIMATE, f"width a = 35 mm of ABUS 41/30 laminated steel padlock ({ABUS_SEARCH}, 403 to fetcher)")
LOCK_D = figure("padlock depth", 0.022, "m", ESTIMATE, "depth e = 22 mm of ABUS 41/30 from the same search snippet (not opened)")
LOCK_MASS = figure("padlock mass", 0.129, "kg", ESTIMATE, "129 g of ABUS 41/30 from the same search snippet (~17 cm3 laminated steel)")
LOCK_BODY_H = figure("padlock body height", 0.030, "m", ESTIMATE, "five steel laminations under a 54 mm overall height")
LOCK_SHACKLE_R = figure("padlock shackle radius", 0.0025, "m", ESTIMATE, "5 mm diameter hardened-steel shackle of a 30 mm-class padlock")
LOCK_AT = (0.175, -0.120)  # m: the lock's frame origin (centre of its body's broad face's underside), lying on the tray
LOCK_TURN = 90.0  # deg: the lock lies turned so its 22 mm depth runs along x, across the closing fingers

SHEARS_L = figure("shears length", 0.203, "m", LISTING, f"Gingher art. 220520-1102 'Size: 8\"', chrome-over-nickel steel ({SHEARS_LISTING})")
SHEARS_MASS = figure("shears mass", 0.172, "kg", ESTIMATE, "forged-steel 8 in shears (~22 cm3 steel at 7.8 g/cm3; 0.38 lb in retail snippets)")
SHEARS_W = figure("shears width, closed", 0.065, "m", ESTIMATE, "listing's photo scaled against the 8 in (203 mm) length")
SHEARS_RING_LIFT = figure("shears finger rings over the blade plane", 0.030, "m", ESTIMATE, "'Angled handle' lifting the rings ~30 mm off the table plane, scaled from the 203 mm photo")
SHEARS_AT = (0.330, 0.055)  # m: the shears' frame origin (centre of the closed blade stack's underside), blades to +y, rings to -y

# --- the plastic items (the safe ones) -------------------------------------------------------------------------------------
BOTTLE_VOL = figure("bottle volume", 330.0, "mL", MAKER, f"'evian® Water 330 mL Bottle', '100% recycled plastic' ({EVIAN_PAGE})")
BOTTLE_H = figure("bottle height", 0.165, "m", ESTIMATE, "330 mL rPET cylinder with tapered shoulder and cap (matches 16.5 cm 24-pack tray height)")
BOTTLE_D = figure("bottle diameter", 0.061, "m", ESTIMATE, "61 mm body diameter enclosing 330 cm3 below the shoulder")
BOTTLE_MASS = figure("bottle mass, full", 0.354, "kg", ESTIMATE, "330 g water from the maker's 330 mL volume plus ~24 g rPET bottle and cap")
BOTTLE_AT = (0.433, 0.105)  # m: the bottle's frame origin (centre of its base), standing on the tray

DISH_L = figure("kidney dish length", 0.229, "m", MAKER, f"Dynarex Code 4224 'Emesis Basin 9\" 500cc Capacity' ({DYNAREX_PAGE})")
DISH_W = figure("kidney dish width", 0.095, "m", ESTIMATE, "Dynarex 4224 photo scaled against its 9 in (229 mm) length")
DISH_H = figure("kidney dish height", 0.045, "m", ESTIMATE, "45 mm depth holding 500 cc inside the 229 x 95 mm footprint")
DISH_MASS = figure("kidney dish mass", 0.019, "kg", MAKER, "calculated from Dynarex 4224 case spec: 'weight': '10.45' lb / '250/cs' = 19.0 g")
DISH_AT = (0.428, -0.090)  # m: the dish's frame origin (centre of its base's underside), long axis along y

CLIP_L = figure("clipboard length", 0.228, "m", ESTIMATE, f"A5 all-plastic clipboard class (ISO 216 A5 210x148 mm + margin; DIN A4 MRI ref {ALLMRI_CLIP})")
CLIP_W = figure("clipboard width", 0.150, "m", ESTIMATE, "150 mm width for ISO 216 A5 sheet with side alignment rule")
CLIP_MASS = figure("clipboard mass", 0.075, "kg", ESTIMATE, "228 x 150 x 2 mm plastic board (~68 cm3 at 1.05 g/cm3) plus plastic-spring clip")
CLIP_AT = (0.211, 0.096)  # m: the clipboard's frame origin (centre of its board's underside), its 228 mm side along y

STEEL = ("clamp", "padlock", "shears")
SAFE = ("bottle", "dish", "clipboard")
BODIES = ("tray", "bin", *STEEL, *SAFE)  # every free body in the scene, by name

# --- where the hands work (checks.py solves these; demo.py drives them) ----------------------------------------------------
GRASP_CLAMP_T = (CLAMP_AT[0] + 0.017, CLAMP_AT[1], 0.115)  # centred on the screw and T-handle neck at x_local = +17 mm
GRASP_LOCK = (LOCK_AT[0], LOCK_AT[1], LOCK_BODY_H / 2.0 + TRAY_FLOOR)  # across the 22 mm depth, which lies along x
GRASP_SHEARS = (0.330, -0.023, 0.026)  # both raised rings pinched together (pads 13 mm over the bench, rings at 28.5..35.5 mm)
GRASPS = {"clamp": GRASP_CLAMP_T, "padlock": GRASP_LOCK, "shears": GRASP_SHEARS}
PARK = (0.215, -0.200, 0.220)  # m: where the working hand passes between items (x, y, height over the bench)
RELEASE_Z = 0.175  # m over the bench: level shears release inside the bin mouth (rim 149 mm)
RELEASES = {"padlock": (0.165, -0.415), "clamp": (0.235, -0.395), "shears": (0.228, -0.310)}  # non-overlapping lanes inside bin
OVER = 0.001  # m: free bodies are built this far over their support so they settle and are drawn from the first picture

# --- the score's limits (task.py, task.json, README: the same numbers) ------------------------------------------------------
SAFE_MOVE = 0.030  # m: a safe item this far from its start is moved
SAFE_LIFT = 0.040  # m: a safe item's lowest point this high over the tray floor has been lifted clear
BIN_SINK = 0.005  # m: an item in the bin reaches at least this far under its rim
BENCH_REST_Z = 0.010  # m: a cleared item on the bench has its lowest corner at most this far over the bench top
HANDS_CLEAR = 0.030  # m: both hands this far from the tray, the bin and every item for every positive score level
HOLD_STEPS = 20  # 1.0 s at 20 Hz
HOLD_DRIFT = 0.003  # m: every body moves at most this during the hold
REST_SPEED = 0.010  # m/s: a body slower than this counts as at rest (read as drift over the hold, not a velocity)


def inside(x: float, y: float, centre, dx: float, dy: float, inset: float = 0.0) -> bool:
    """Is (x, y) inside the level rectangle centred at ``centre`` with sides ``dx`` by ``dy``, inset by ``inset``?"""
    return abs(x - centre[0]) <= dx / 2.0 - inset and abs(y - centre[1]) <= dy / 2.0 - inset


def on_parts_table() -> bool:
    """Every item lies inside the tray's inside rectangle and clear of the bin's footprint (a packing self-check)."""
    spots = {
        "clamp": (CLAMP_AT, CLAMP_FRAME_T + 0.02, 0.045),
        "padlock": (LOCK_AT, LOCK_D, LOCK_W),
        "shears": (SHEARS_AT, SHEARS_W, SHEARS_L),
        "bottle": (BOTTLE_AT, BOTTLE_D, BOTTLE_D),
        "dish": (DISH_AT, DISH_W, DISH_L),
        "clipboard": (CLIP_AT, CLIP_W, CLIP_L),
    }
    for (x, y), dx, dy in spots.values():
        if not inside(x, y, TRAY_AT, TRAY_D - dx, TRAY_W - dy, inset=TRAY_WALL):
            return False
    # no two items' footprints overlap (their half-extents, with a 3 mm gap)
    names = sorted(spots)
    for i, a in enumerate(names):
        (ax, ay), adx, ady = spots[a]
        for b in names[i + 1:]:
            (bx, by), bdx, bdy = spots[b]
            if abs(ax - bx) <= (adx + bdx) / 2.0 + 0.003 and abs(ay - by) <= (ady + bdy) / 2.0 + 0.003:
                return False
    return True


def summary() -> dict:
    return {"figures": {n: {"value": v, "unit": u, "source": s, "note": t} for n, (v, u, s, t) in FIGURES.items()},
            "layout on the tray checks": on_parts_table(),
            "bodies": {n: dict(at=dict(zip("xy", {"tray": TRAY_AT, "bin": BIN_AT, "clamp": CLAMP_AT, "padlock": LOCK_AT,
                                                        "shears": SHEARS_AT, "bottle": BOTTLE_AT, "dish": DISH_AT, "clipboard": CLIP_AT}[n])))
                       for n in BODIES},
            "grasps (m)": {k: [round(v, 4) for v in g] for k, g in GRASPS.items()},
            "releases (m)": {k: [round(v, 4) for v in r] for k, r in RELEASES.items()},
            "lettering": {"sign line 1 cells": lettering.cells(SIGN_LINE1), "sign line 2 cells": lettering.cells(SIGN_LINE2),
                          "label letters width (m)": round(lettering.width(LABEL_LETTER, LABEL_LETTER_CELL), 4),
                          "sign line widths (m)": [round(lettering.width(SIGN_LINE1, SIGN_CELL1), 4), round(lettering.width(SIGN_LINE2, SIGN_CELL2), 4)]}}
