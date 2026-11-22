"""Block lettering for the task's printed marks: a 5 x 7 dot-matrix font laid flat on the bench, so the top
camera reads it. Each stroke is a thin drawn-only box (``solid=False``); the text frame starts at the origin at
the left end of the first letter's cell and advances along +x, rows running towards -y (row 0 is the letter's
top). ``width()`` says how long a string comes out; callers centre a mat under it in scene.py.
"""

from __future__ import annotations

import mechkit as mk

GLYPHS: dict[str, tuple[str, ...]] = {
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "B": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
    "C": ("01111", "10000", "10000", "10000", "10000", "10000", "01111"),
    "D": ("11110", "10001", "10001", "10001", "10001", "10001", "11110"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "F": ("11111", "10000", "10000", "11110", "10000", "10000", "10000"),
    "G": ("01111", "10000", "10000", "10111", "10001", "10001", "01111"),
    "H": ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    "I": ("11111", "00100", "00100", "00100", "00100", "00100", "11111"),
    "K": ("10001", "10010", "10100", "11000", "10100", "10010", "10001"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "P": ("11110", "10001", "10001", "11110", "10000", "10000", "10000"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "U": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    "V": ("10001", "10001", "10001", "10001", "10001", "01010", "00100"),
    "W": ("10001", "10001", "10001", "10101", "10101", "11011", "10001"),
    "Z": ("11111", "00001", "00010", "00100", "01000", "10000", "11111"),
    "0": ("01110", "10001", "10011", "10101", "11001", "10001", "01110"),
    "1": ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    "4": ("00010", "00110", "01010", "10010", "11111", "00010", "00010"),
    "-": ("00000", "00000", "00000", "11111", "00000", "00000", "00000"),
    " ": ("00000", "00000", "00000", "00000", "00000", "00000", "00000"),
}
COLS, ROWS, GAP = 5, 7, 1.0  # cells per letter, rows per letter, gap cells between letters


def cells(text: str) -> int:
    """How many letter cells wide ``text`` is, gaps included."""
    return max(0, len(text) * COLS + (len(text) - 1) * int(GAP)) if text else 0


def width(text: str, cell: float) -> float:
    """m along +x that ``text`` takes at letter size ``cell`` (the letter's full height is 7 x cell)."""
    return cells(text) * cell


def height(cell: float) -> float:
    return ROWS * cell


def shapes(text: str, cell: float, color, thickness: float = 0.0012, at=(0.0, 0.0, 0.0)) -> list:
    """The letter boxes of ``text`` in the text frame (drawn only, no colliders), rows towards -y, the text frame's
    origin placed at ``at``."""
    out = []
    x = 0.0
    for ch in text:
        glyph = GLYPHS.get(ch.upper())
        if glyph is None:
            raise ValueError(f"no glyph for {ch!r}")
        for row, bits in enumerate(glyph):
            col = 0
            while col < COLS:
                if bits[col] == "1":
                    end = col + 1
                    while end < COLS and bits[end] == "1":
                        end += 1
                    span = end - col
                    out.append(mk.box((span * cell, cell, thickness),
                                      pos=(at[0] + x + (col + span / 2.0) * cell, at[1] - (row + 0.5) * cell, at[2]),
                                      color=color, solid=False))
                    col = end
                else:
                    col += 1
        x += COLS * cell + GAP * cell
    return out
