"""Compares the RAM dumps of the straight and the steering run to find the
car's position and orientation in the game's memory.

Usage: python tools/car_scan.py
"""
import struct
import numpy as np

FRAMES = [3200, 3600, 3800, 4400, 4750]

def load(run, frame):
    return np.fromfile(f"out/run{run}/ram_{frame}.bin", dtype=">u4")

A = {f: load("A", f) for f in FRAMES}
B = {f: load("B", f) for f in FRAMES}
N = len(A[3200])

def f32(word):
    return struct.unpack(">f", struct.pack(">I", word & 0xFFFFFFFF))[0]

print("anchor speed 0x22F3A8 (m/s):")
for fr in FRAMES:
    print(f"  frame {fr}: A={f32(A[fr][0x22F3A8>>2]):7.2f}  B={f32(B[fr][0x22F3A8>>2]):7.2f}")

# Steering-coupled float fields: same in both runs before the steering event
# (3500), different after, finite, not denormal junk.
print("\nsteering-coupled float fields (|A-B| > 0.02 at any frame after 3500, ~equal at 3200):")
hits = []
for i in range(N):
    a = [f32(A[f][i]) for f in FRAMES]
    b = [f32(B[f][i]) for f in FRAMES]
    if not all(np.isfinite(a)) or not all(np.isfinite(b)):
        continue
    pre = abs(a[0] - b[0]) + abs(a[1] - b[1])
    post = sum(abs(x - y) for x, y in zip(a[2:], b[2:]))
    if pre < 0.01 and post > 0.05:
        mag = max(max(abs(x) for x in a), max(abs(x) for x in b))
        if mag < 1e6 and mag > 1e-4:
            hits.append((i * 4, a, b))
print(f"{len(hits)} candidate fields")
for off, a, b in hits[:60]:
    print(f"  {off:06X}: A={['%8.3f' % x for x in a]}")
    print(f"          B={['%8.3f' % x for x in b]}")
