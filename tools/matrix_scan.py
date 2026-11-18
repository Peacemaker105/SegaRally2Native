"""Scans the dense RAM dumps for 3x3 rotation matrices: float triples whose
rows stay unit length and mutually orthogonal across every frame, and that
change smoothly (steering sweeps make them rotate). Prints every hit with its
position column, so the car matrix can be told from the camera's and the
wheels'.

Usage: python tools/matrix_scan.py
"""
import glob
import struct
import numpy as np

files = sorted(glob.glob("out/runC/ram_*.bin"))
dumps = [np.fromfile(f, dtype=">u4") for f in files]
n = len(dumps[0])
print(f"{len(dumps)} dumps, {n} words each")

def fs(a, i):
    return struct.unpack(">f", struct.pack(">I", int(a[i]) & 0xFFFFFFFF))[0]

hits = []
for base in range(0, n - 12, 4):   # 16-byte stride: matrices usually align so
    ok = True
    norms = []
    for k in range(len(dumps)):
        d = dumps[k]
        m = np.array([[fs(d, base), fs(d, base + 1), fs(d, base + 2)],
                      [fs(d, base + 4), fs(d, base + 5), fs(d, base + 6)],
                      [fs(d, base + 8), fs(d, base + 9), fs(d, base + 10)]])
        if not np.isfinite(m).all():
            ok = False
            break
        norms.append(np.linalg.norm(m, axis=1))
        if k == 0:
            pass
    if not ok:
        continue
    norms = np.array(norms)
    if np.abs(norms - 1.0).max() > 0.02:
        continue
    ortho = []
    for k in range(len(dumps)):
        d = dumps[k]
        m = np.array([[fs(d, base), fs(d, base + 1), fs(d, base + 2)],
                      [fs(d, base + 4), fs(d, base + 5), fs(d, base + 6)],
                      [fs(d, base + 8), fs(d, base + 9), fs(d, base + 10)]])
        o = abs(np.dot(m[0], m[1])) + abs(np.dot(m[0], m[2])) + abs(np.dot(m[1], m[2]))
        ortho.append(o)
    if max(ortho) > 0.02:
        continue
    # smoothness: the 9 elements across frames must not jump
    m0 = np.array([fs(dumps[0][base + j]) for j in range(9)])
    mlast = np.array([fs(dumps[-1][base + j]) for j in range(9)])
    if np.abs(m0 - mlast).max() < 0.01:
        continue   # constant (identity rotations in code or data tables)
    hits.append(base)

print(f"{len(hits)} rotation-matrix candidates")
for base in hits:
    print(f"\n--- matrix at {base*4:06X} ---")
    for k, f in enumerate(files):
        d = dumps[k]
        tx = fs(d, base + 12)
        ty = fs(d, base + 13)
        tz = fs(d, base + 14)
        frame = f.split("ram_")[1].split(".")[0]
        rows = [[fs(d, base + r * 4 + cc) for cc in range(3)] for r in range(3)]
        print(f"  {frame}: rows [{rows[0][0]:+.3f} {rows[0][1]:+.3f} {rows[0][2]:+.3f}]"
              f" [{rows[1][0]:+.3f} {rows[1][1]:+.3f} {rows[1][2]:+.3f}]"
              f" [{rows[2][0]:+.3f} {rows[2][1]:+.3f} {rows[2][2]:+.3f}]  t=({tx:.1f},{ty:.1f},{tz:.1f})")
