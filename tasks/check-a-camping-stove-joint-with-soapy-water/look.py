"""The stove check's pixel test: is there froth on the cartridge?

It works on the top camera's picture as a policy gets it (320 x 176; demo_kit.policy_picture) and counts the pixels
that lie on the cartridge's top and are near-white: red, green and blue all at least WHITE_FLOOR of 255. The
cartridge's top is where the camera sees a level disc 90 mm across at the shoulder's height about the place the
cartridge stands at the start (from the layout's constants and the camera's place, not from the simulator). The
shoulder is red, the froth white; the stove's bright metal, which stands in front of part of that disc, is there
with froth and without, so a count means something only against another: the same view of a sound joint, or the
same view before the joint was wetted.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))  # demo_kit

WHITE_FLOOR = 150  # of 255
FROTH_MORE = 15  # this many near-white pixels more than before the joint was wetted: froth


def cartridge_top(task_module, camera, shape: tuple[int, int] = (176, 320)) -> np.ndarray:
    """The pixels (rows x columns of ``shape``) on which ``camera`` sees the top of the cartridge where it stands at
    the start."""
    import demo_kit as kit

    m = task_module
    return kit.disc_mask(camera, shape, (*m.CART_XY, m.TOP + m.CART_H - m.CUP_H), m.CART_R)


def near_white(picture: np.ndarray, floor: int = WHITE_FLOOR) -> np.ndarray:
    return picture.min(axis=-1) >= floor


def white_on_cartridge(task_module, camera, frame: np.ndarray, floor: int = WHITE_FLOOR) -> int:
    """The count: near-white pixels on the cartridge's top in ``frame`` (a top camera picture of any size) seen at
    the policy's size."""
    import demo_kit as kit

    small = kit.policy_picture(frame)
    return int((near_white(small, floor) & cartridge_top(task_module, camera, small.shape[:2])).sum())
