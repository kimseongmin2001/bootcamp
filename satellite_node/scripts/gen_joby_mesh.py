#!/usr/bin/env python3
"""
Joby S4 eVTOL simplified 3D mesh generator → binary STL

Coordinate frame (body):
  +X = nose (forward)
  +Y = left wing
  +Z = up

Approximate scale: 1 unit = 1 metre
  Fuselage  : 6.4 m long
  Wingspan  : 9.0 m total
  Rotor dia : 2.4 m each (6 rotors, 3 per side)
"""

import math
import struct
import os
import sys

tris = []   # list of (v0, v1, v2)

# ── Primitives ────────────────────────────────────────────────────────────────

def tri(v0, v1, v2):
    tris.append((v0, v1, v2))

def quad(v0, v1, v2, v3):
    tri(v0, v1, v2)
    tri(v0, v2, v3)

def box(x0, y0, z0, x1, y1, z1):
    v = [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
         (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)]
    quad(v[3],v[2],v[1],v[0])   # -Z
    quad(v[4],v[5],v[6],v[7])   # +Z
    quad(v[0],v[1],v[5],v[4])   # -Y
    quad(v[2],v[3],v[7],v[6])   # +Y
    quad(v[0],v[4],v[7],v[3])   # -X
    quad(v[1],v[2],v[6],v[5])   # +X

def cylinder(cx, cy, cz, r, h, n=24):
    """Vertical cylinder along Z-axis centred at (cx,cy,cz)."""
    z0, z1 = cz - h/2, cz + h/2
    c = [(cx + r*math.cos(2*math.pi*i/n),
          cy + r*math.sin(2*math.pi*i/n)) for i in range(n)]
    for i in range(n):
        j = (i+1) % n
        b0 = (c[i][0], c[i][1], z0)
        b1 = (c[j][0], c[j][1], z0)
        t0 = (c[i][0], c[i][1], z1)
        t1 = (c[j][0], c[j][1], z1)
        tri((cx,cy,z1), t0, t1)          # top cap
        tri((cx,cy,z0), b1, b0)          # bottom cap
        quad(b0, t0, t1, b1)             # side

def ellipse_ring(x, ry, rz, zc, n=20):
    """Return n points on ellipse at station x, semi-axes ry(Y) rz(Z)."""
    return [(x,
             ry * math.cos(2*math.pi*i/n),
             zc + rz * math.sin(2*math.pi*i/n)) for i in range(n)]

def loft(stations, n=20):
    """
    stations: list of (x, ry, rz, zc)
    Loft a surface between adjacent cross-sections.
    ry=0 means a point (nose/tail tip).
    """
    rings = []
    for s in stations:
        x, ry, rz, zc = s
        rings.append(None if ry < 0.001 else ellipse_ring(x, ry, rz, zc, n))

    for si in range(len(stations)-1):
        r0, r1 = rings[si], rings[si+1]
        x0, _, _, zc0 = stations[si]
        x1, _, _, zc1 = stations[si+1]

        if r0 is None:                          # point → ring (fan forward)
            tip = (x0, 0, zc0)
            for i in range(n):
                j = (i+1) % n
                tri(tip, r1[i], r1[j])
        elif r1 is None:                        # ring → point (fan backward)
            tip = (x1, 0, zc1)
            for i in range(n):
                j = (i+1) % n
                tri(tip, r0[j], r0[i])
        else:                                   # ring → ring
            for i in range(n):
                j = (i+1) % n
                quad(r0[i], r0[j], r1[j], r1[i])

    # close open ends
    for idx, (s, ring) in enumerate(zip(stations, rings)):
        if ring is None:
            continue
        x, _, _, zc = s
        c = (x, 0.0, zc)
        is_last  = (idx == len(stations)-1)
        is_first = (idx == 0)
        if is_first:                            # tail cap (normal –X)
            for i in range(n):
                j = (i+1) % n
                tri(c, ring[j], ring[i])
        if is_last:                             # nose cap (normal +X)
            for i in range(n):
                j = (i+1) % n
                tri(c, ring[i], ring[j])

# ── Joby S4 geometry ─────────────────────────────────────────────────────────

# 1. Fuselage  (stations: x, ry, rz, z_centre)
#    Cockpit pod + slender tail boom
loft([
    (-3.30, 0.001, 0.001,  0.02),   # tail tip
    (-2.90, 0.08,  0.09,   0.04),
    (-2.20, 0.22,  0.24,   0.08),
    (-1.00, 0.36,  0.34,   0.12),
    ( 0.30, 0.44,  0.42,   0.14),   # widest (cabin)
    ( 1.40, 0.46,  0.44,   0.16),
    ( 2.20, 0.42,  0.40,   0.18),
    ( 2.80, 0.26,  0.28,   0.18),
    ( 3.20, 0.001, 0.001,  0.18),   # nose tip
], n=18)

# 2. Main wing  (thin flat slab, right side then mirrored left)
#    chord x: -0.55 … +0.80, thickness ±0.055
#    span  y: 0.45 (fuselage edge) … 4.60
def wing_side(y0, y1):
    # root
    box(-0.55,  y0, -0.055, 0.80,  y0+0.25, 0.055)
    # main span
    box(-0.55,  y0+0.25, -0.055, 0.80,  y1-0.15, 0.055)
    # tip taper (trailing edge sweeps back slightly)
    box(-0.45,  y1-0.15, -0.050, 0.72,  y1,      0.050)

wing_side( 0.45,  4.60)   # right (+Y)
wing_side(-4.60, -0.45)   # left  (-Y)  (mirrored)

# Wing-body fairing
box(-0.40, -0.45, -0.045, 0.70, 0.45, 0.045)

# 3. Horizontal stabiliser
box(-3.10, -2.10, -0.030, -2.10,  2.10, 0.030)

# 4. Vertical fin
box(-3.10, -0.07,  0.25, -2.15,  0.07, 1.20)

# 5. Six tilting-rotor nacelles + discs
#    Positions along wing (y): ±1.35, ±2.75, ±4.35
#    x = 0.95 (just ahead of wing LE), z = 0.30 (hover position — disc horizontal)
ROTOR_Y = [1.35, 2.75, 4.35]
DISC_R   = 1.10
PYLON_R  = 0.085

for ry in ROTOR_Y:
    for sy in (ry, -ry):
        # rotor disc (very thin horizontal cylinder)
        cylinder(0.95, sy, 0.30, DISC_R, 0.055, n=28)
        # pylon (connects wing top surface to nacelle)
        cylinder(0.95, sy, 0.14, PYLON_R, 0.27, n=10)
        # nacelle housing (small cylinder below disc)
        cylinder(0.95, sy, 0.28, 0.18, 0.12, n=12)

# 6. Simple cockpit canopy bump (optional visual)
loft([
    ( 1.40, 0.001, 0.001, 0.54),
    ( 1.80, 0.38,  0.18,  0.60),
    ( 2.30, 0.30,  0.12,  0.52),
    ( 2.75, 0.001, 0.001, 0.44),
], n=14)

# 7. Fixed tricycle landing gear
for (gx, gy_pair, gh) in [
    (2.20, [0.38, -0.38], 0.38),    # nose gear
    (-0.60, [0.72, -0.72], 0.42),   # main gear
]:
    for gy in gy_pair:
        cylinder(gx, gy, -0.42, 0.065, gh, n=8)

# ── Write binary STL ─────────────────────────────────────────────────────────

def normal3(v0, v1, v2):
    ax,ay,az = v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]
    bx,by,bz = v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]
    nx = ay*bz - az*by
    ny = az*bx - ax*bz
    nz = ax*by - ay*bx
    m  = math.sqrt(nx*nx + ny*ny + nz*nz)
    return (nx/m, ny/m, nz/m) if m > 1e-12 else (0.0, 0.0, 1.0)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'meshes', 'joby_s4.stl')

with open(out, 'wb') as f:
    header = b'Joby S4 eVTOL - auto-generated mesh for RViz' + b'\x00'*36
    f.write(header[:80])
    f.write(struct.pack('<I', len(tris)))
    for t in tris:
        n = normal3(*t)
        f.write(struct.pack('<fff', *n))
        for v in t:
            f.write(struct.pack('<fff', *v))
        f.write(struct.pack('<H', 0))

print(f"[gen_joby_mesh] {len(tris):,} triangles  →  {os.path.abspath(out)}")
