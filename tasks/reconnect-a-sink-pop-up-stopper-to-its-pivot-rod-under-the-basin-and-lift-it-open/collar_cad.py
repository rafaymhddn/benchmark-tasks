#!/usr/bin/env python3
"""Printable closed 2-manifold 3D-CAD generator (ASCII STL) for the 4-slot PETG snap-collar on Danco 80767.

    python3 mech/tasks/sink_stopper/collar_cad.py [out.stl]

Replaces the kit's hand-turned captive nut on the Danco 80767 rear pivot boss:
- Front 10.0 mm skirt (26.4 mm OD, 19.2 mm pilot bore tapped to the estimated 20.0 x 1.5 mm / 0.78 in Danco 80767 rear
  port external thread; a 1-min thread-gauge check on the bought 80767 sets the tap) compresses the 88532 cone washer.
- Rear 9.0 mm 4-slot flexure crown (four 2.5 x 8.0 mm axial slots at 90 deg with closed radial side walls, 19.6 mm ball
  pocket, 15.6 mm ID snap ridge 5.5 mm behind the seated ball centre with 30 deg chamfer) clicks the 18.3 mm ball in.
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from pathlib import Path

import parts as P

N_SEG = 48
SLOT_W = 0.0025
SLOT_DEPTH = 0.0080


def collar_specs() -> dict[str, float]:
    return {
        "od_mm": round(1000.0 * P.COLLAR_OD, 2),
        "length_mm": round(1000.0 * P.COLLAR_LEN, 2),
        "thread_skirt_mm": round(1000.0 * P.COLLAR_THREAD_LEN, 2),
        "estimated_port_thread_major_mm": round(1000.0 * P.PORT_THREAD_MAJOR, 2),
        "estimated_port_thread_pitch_mm": 1.5,
        "pilot_bore_id_mm": 19.2,
        "pocket_id_mm": 19.6,
        "snap_ridge_id_mm": round(2000.0 * P.LIP_R, 2),
        "slot_width_mm": round(1000.0 * SLOT_W, 2),
        "slot_count": 4,
    }


def _in_slot(ang: float, z: float) -> bool:
    if z < P.COLLAR_LEN - SLOT_DEPTH:
        return False
    r_mid = 0.5 * (P.LIP_R + P.COLLAR_OD / 2.0)
    half_ang = 0.5 * SLOT_W / r_mid
    for q in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
        diff = abs((ang - q + math.pi) % (2.0 * math.pi) - math.pi)
        if diff <= half_ang:
            return True
    return False


def _inner_r(z: float) -> float:
    """Axial bore profile z in [0, COLLAR_LEN] (m from front skirt face)."""
    if z <= P.COLLAR_THREAD_LEN:
        return 0.0096  # 19.2 mm minor pilot bore for tapping to the 80767 port thread (estimated 20.0 x 1.5 mm)
    if z <= 0.0135:
        return 0.0098  # 19.6 mm ball clearance pocket behind cone washer
    if z <= 0.0160:
        return P.LIP_R  # 15.6 mm ID rounded snap ridge (1.35 mm interference on 18.3 mm ball)
    t = (z - 0.0160) / (P.COLLAR_LEN - 0.0160)
    return P.LIP_R + t * (0.0098 - P.LIP_R)  # 30-deg rear lead-in chamfer


def _tri(a, b, c) -> str:
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    ln = math.hypot(nx, math.hypot(ny, nz)) or 1.0
    return (
        f"  facet normal {nx / ln:.6f} {ny / ln:.6f} {nz / ln:.6f}\n"
        f"    outer loop\n"
        f"      vertex {1e3 * a[0]:.4f} {1e3 * a[1]:.4f} {1e3 * a[2]:.4f}\n"
        f"      vertex {1e3 * b[0]:.4f} {1e3 * b[1]:.4f} {1e3 * b[2]:.4f}\n"
        f"      vertex {1e3 * c[0]:.4f} {1e3 * c[1]:.4f} {1e3 * c[2]:.4f}\n"
        f"    endloop\n"
        f"  endfacet\n"
    )


def build_ascii_stl() -> str:
    zs = [0.0, P.COLLAR_THREAD_LEN, P.COLLAR_LEN - SLOT_DEPTH, 0.0135, 0.0160, P.COLLAR_LEN]
    nz = len(zs) - 1
    r_out = P.COLLAR_OD / 2.0
    angs = [2.0 * math.pi * k / N_SEG for k in range(N_SEG)]
    verts = {
        (k, i, outer): (
            (r_out if outer else _inner_r(zs[i])) * math.cos(angs[k]),
            (r_out if outer else _inner_r(zs[i])) * math.sin(angs[k]),
            zs[i],
        )
        for k in range(N_SEG)
        for i in range(len(zs))
        for outer in (0, 1)
    }
    present = {
        (k, i): not _in_slot(2.0 * math.pi * (k + 0.5) / N_SEG, 0.5 * (zs[i] + zs[i + 1]))
        for k in range(N_SEG)
        for i in range(nz)
    }

    out = ["solid danco_80767_4slot_snap_collar\n"]
    for k in range(N_SEG):
        k1 = (k + 1) % N_SEG
        km = (k - 1) % N_SEG
        for i in range(nz):
            if not present[(k, i)]:
                continue
            i00, o00 = verts[(k, i, 0)], verts[(k, i, 1)]
            i10, o10 = verts[(k1, i, 0)], verts[(k1, i, 1)]
            i01, o01 = verts[(k, i + 1, 0)], verts[(k, i + 1, 1)]
            i11, o11 = verts[(k1, i + 1, 0)], verts[(k1, i + 1, 1)]
            # Outer (+r) and inner (-r) cylindrical/conical walls
            out += [_tri(o00, o10, o11), _tri(o00, o11, o01)]
            out += [_tri(i00, i11, i10), _tri(i00, i01, i11)]
            # Bottom (-z) and top (+z) annular faces where exposed
            if i == 0 or not present[(k, i - 1)]:
                out += [_tri(i00, i10, o10), _tri(i00, o10, o00)]
            if i == nz - 1 or not present[(k, i + 1)]:
                out += [_tri(o01, o11, i11), _tri(o01, i11, i01)]
            # Radial slot side walls (-theta at k, +theta at k1) where adjacent sector is open slot
            if not present[(km, i)]:
                out += [_tri(i00, o00, o01), _tri(i00, o01, i01)]
            if not present[(k1, i)]:
                out += [_tri(o10, i10, i11), _tri(o10, i11, o11)]
    out.append("endsolid danco_80767_4slot_snap_collar\n")
    return "".join(out)


def verify_watertight(stl_text: str) -> dict[str, int]:
    """Check every undirected edge belongs to 2 triangles and every directed half-edge has 1 opposite half-edge."""
    verts = [tuple(line.split()[1:4]) for line in stl_text.splitlines() if line.strip().startswith("vertex ")]
    tris = [verts[j : j + 3] for j in range(0, len(verts), 3)]
    undirected: Counter = Counter()
    directed: Counter = Counter()
    for a, b, c in tris:
        for u, v in ((a, b), (b, c), (c, a)):
            undirected[tuple(sorted((u, v)))] += 1
            directed[(u, v)] += 1
    boundary = sum(1 for n in undirected.values() if n == 1)
    non_manifold = sum(1 for n in undirected.values() if n != 2)
    unpaired_dir = sum(1 for (u, v), n in directed.items() if n != 1 or directed[(v, u)] != 1)
    return {
        "triangles": len(tris),
        "edges": len(undirected),
        "boundary_edges": boundary,
        "non_manifold_edges": non_manifold,
        "unpaired_directed_edges": unpaired_dir,
    }


if __name__ == "__main__":
    stl = build_ascii_stl()
    if len(sys.argv) > 1:
        Path(sys.argv[1]).write_text(stl)
    else:
        sys.stdout.write(stl)
