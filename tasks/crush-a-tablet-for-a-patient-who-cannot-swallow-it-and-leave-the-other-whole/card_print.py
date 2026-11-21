"""The black print of the yellow cue card (and of cup 3's auxiliary label), as rectangles the task file draws.

The simulator draws boxes only, so the print is rasterised here at MM_PER_PX and cut into black rectangles: rows of
ink merged into runs, runs of equal extent merged down the rows. Run it once where Pillow and the font are installed:

    python3 card_print.py            # writes card_print.json beside it; the task file reads only that file

Frame of each print: millimetres from the card's (label's) centre, u to the right as one reads it, v up the page.
The words are set in a condensed bold sans (PT Sans Narrow Bold): the face of the printed card is the builder's
choice, the size and wording are the spec's (capitals at least 5 mm tall, one line; a crossed-tablet symbol at least
12 mm across).
"""

from __future__ import annotations

import json
import pathlib

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = pathlib.Path(__file__).resolve().parent
FONT = "/usr/share/fonts/truetype/paratype/PTN77F.ttf"  # PT Sans Narrow Bold
MM_PER_PX = 0.2
WORDS = "DO NOT CRUSH"

# The cards. The extended-release cup stands on the card's FRONT half (towards the arms, its middle at v = -depth/4);
# the print is on the BACK half, beyond the cup: the wrist camera sits 7 cm beyond the fingers and looks back towards
# the arms, so a print in front of the cup is hidden by the cup (run w12p1). It reads from the arms' side.
# "card": the spec's card (60 x 40 mm, capitals at least 5 mm); "card_large": the spec's fallback (80 x 50 mm, 8 mm).
CARDS = {
    "card": {"size": (60.0, 40.0), "cap": 5.5, "symbol": 12.5, "words_v": 14.0, "symbol_v": 12.5},
    "card_large": {"size": (80.0, 50.0), "cap": 8.0, "symbol": 14.0, "words_v": 16.0, "symbol_v": 15.0},
}
MARGIN_MM = 1.5  # from the card's left and right edges

# Cup 3's auxiliary label (a stocked 'DO NOT CRUSH' label: 1-9/16 x 3/8 in), wrapped on the cup's side by the task file.
LABEL = (39.7, 9.5)
LABEL_CAP_MM = 3.6


def _text_mask(cap_mm: float) -> np.ndarray:
    """The words' ink as a boolean raster (rows down the page), cropped to the ink, capitals cap_mm tall."""
    cap_px = cap_mm / MM_PER_PX
    size = 200
    font = ImageFont.truetype(FONT, size)
    h_box = font.getbbox("H")
    size = int(round(size * cap_px / (h_box[3] - h_box[1])))
    font = ImageFont.truetype(FONT, size)
    left, top, right, bottom = font.getbbox(WORDS)
    img = Image.new("L", (right - left + 8, bottom - top + 8), 0)
    ImageDraw.Draw(img).text((4 - left, 4 - top), WORDS, font=font, fill=255)
    mask = np.asarray(img) >= 128
    rows, cols = np.nonzero(mask)
    mask = mask[rows.min() : rows.max() + 1, cols.min() : cols.max() + 1]
    h_cap = font.getbbox("H")
    measured = (h_cap[3] - h_cap[1]) * MM_PER_PX
    assert abs(measured - cap_mm) <= 2 * MM_PER_PX, (measured, cap_mm)
    return mask


def _symbol_mask(diameter_mm: float) -> np.ndarray:
    """A prohibition ring with its bar over a caplet: the crossed-tablet symbol, diameter_mm across."""
    n = int(round(diameter_mm / MM_PER_PX))
    scale = 8  # drawn large, then reduced: smooth edges
    big = n * scale
    img = Image.new("L", (big, big), 0)
    d = ImageDraw.Draw(img)
    stroke = 1.3 / diameter_mm * big
    d.ellipse((0, 0, big - 1, big - 1), fill=255)
    d.ellipse((stroke, stroke, big - 1 - stroke, big - 1 - stroke), fill=0)
    # the caplet: 8 x 4 mm, a rounded rectangle across the middle
    w, h = 8.0 / diameter_mm * big, 4.0 / diameter_mm * big
    c = big / 2
    d.rounded_rectangle((c - w / 2, c - h / 2, c + w / 2, c + h / 2), radius=h / 2, fill=255)
    # the bar, upper left to lower right, with a 0.5 mm gap of card on each side where it crosses the caplet
    r_in = big / 2 - stroke
    a = np.array([c - r_in * 0.7071, c - r_in * 0.7071])
    b = np.array([c + r_in * 0.7071, c + r_in * 0.7071])
    d.line((*a, *b), fill=0, width=int(round((1.3 + 2 * 0.5) / diameter_mm * big)))
    d.line((*a, *b), fill=255, width=int(round(1.3 / diameter_mm * big)))
    img = img.resize((n, n), Image.Resampling.BOX)
    return np.asarray(img) >= 128


def _rects(mask: np.ndarray, u_left: float, v_top: float) -> list[list[float]]:
    """Black rectangles [u0, v0, u1, v1] (mm) covering mask, its top-left pixel corner at (u_left, v_top)."""
    open_runs: dict[tuple[int, int], int] = {}  # (c0, c1) -> first row
    out: list[tuple[int, int, int, int]] = []
    for r in range(mask.shape[0] + 1):
        runs = set()
        if r < mask.shape[0]:
            row = np.concatenate([[False], mask[r], [False]])
            edges = np.flatnonzero(row[1:] != row[:-1])
            runs = {(int(edges[i]), int(edges[i + 1])) for i in range(0, len(edges), 2)}
        for run in list(open_runs):
            if run not in runs:
                out.append((open_runs.pop(run), r, *run))
        for run in runs:
            open_runs.setdefault(run, r)
    p = MM_PER_PX
    return [[round(u_left + c0 * p, 3), round(v_top - r1 * p, 3), round(u_left + c1 * p, 3), round(v_top - r0 * p, 3)] for r0, r1, c0, c1 in out]


def card(size, cap, symbol, words_v, symbol_v) -> dict:
    words, sym = _text_mask(cap), _symbol_mask(symbol)
    words_w, words_h = words.shape[1] * MM_PER_PX, words.shape[0] * MM_PER_PX
    symbol_w = sym.shape[1] * MM_PER_PX
    u_symbol = -size[0] / 2 + MARGIN_MM
    u_words = size[0] / 2 - MARGIN_MM - words_w
    gap = u_words - (u_symbol + symbol_w)
    assert gap >= 1.0, f"the words ({words_w:.1f} mm) and the symbol do not fit on one line of the card: gap {gap:.1f} mm"
    rects = _rects(sym, u_symbol, symbol_v + symbol_w / 2) + _rects(words, u_words, words_v + words_h / 2)
    return {"size_mm": list(size), "cap_mm": cap, "words_mm": [round(words_w, 2), round(words_h, 2)], "symbol_mm": symbol,
            "words_at": [round(u_words, 2), words_v], "symbol_at": [round(u_symbol + symbol_w / 2, 2), symbol_v], "gap_mm": round(gap, 2), "rects": rects}  # fmt: skip


def label() -> dict:
    words = _text_mask(LABEL_CAP_MM)
    w, h = words.shape[1] * MM_PER_PX, words.shape[0] * MM_PER_PX
    assert w <= LABEL[0] - 2.0, (w, LABEL)
    return {"size_mm": list(LABEL), "cap_mm": LABEL_CAP_MM, "words_mm": [round(w, 2), round(h, 2)], "rects": _rects(words, -w / 2, h / 2)}


if __name__ == "__main__":
    data = {"font": "PT Sans Narrow Bold", "mm_per_px": MM_PER_PX, "words": WORDS, **{k: card(**v) for k, v in CARDS.items()}, "label": label()}
    (HERE / "card_print.json").write_text(json.dumps(data, separators=(",", ":")) + "\n")
    for k in CARDS:
        c = data[k]
        print(f"{k}: words {c['words_mm']} mm, gap {c['gap_mm']} mm, {len(c['rects'])} rectangles")
    print(f"label: words {data['label']['words_mm']} mm, {len(data['label']['rects'])} rectangles")
