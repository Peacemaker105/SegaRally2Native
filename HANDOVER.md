# SegaRally2Native — agent handover

Read this, then `docs/TELEMETRY_HANDOVER.md` (the packet spec),
`docs/TELEMETRY.md` (user-facing), `docs/game_ram.md` (the RE knowledge
base) and `docs/ARCHITECTURE.md` (how the recomp works).

## What this is

An unofficial static recompilation of Sega Rally 2 (Sega Model 3 Step 2.1,
PowerPC → C++) by snicolast (github.com/snicolast/SegaRally2Native), cloned
and extended locally with **UDP telemetry for motion rigs/dashboards**. The
owner's own ROM (`srally2.zip`, MAME srally2 set) builds the game; nothing
ROM-derived may be shared (MIT+terms) — personal use only, no binary
distribution, no upstream PRs without the author's blessing (feature request
#111 there is the telemetry ask).

**Status: fully working.** Builds clean (VS2022 BuildTools + CMake + Ninja),
all self-tests pass, plays start to finish, telemetry validated end-to-end
against the dashboard (VN Motion Dashboard, Forza source on UDP 5300).

## Day-to-day

| Task | How |
|---|---|
| Build | `SR2_NO_PAUSE=1 cmd //c "set SR2_NO_PAUSE=1&& build.bat"` from the repo root (after an upstream `update.bat`, or a clean tree). Full build ≈ 5-8 min; needs Python 3.10+, VS2022 Build Tools (they live under `Program Files (x86)`), CMake ≥ 3.27 |
| Run | `SegaRally2.exe` (menu) — needs a Vulkan 1.3 GPU |
| Run with telemetry | `Play with telemetry.bat` — sets `SR2_TELEMETRY=127.0.0.1:5300@forza` (VNMD's Forza source). Edit the host/port in the bat; formats: `@json`, `@forza`, `@both` |
| Test telemetry | `python tools/telemetry_listen_test.py <port> <max> <base>` → writes `<base>.ndjson` + `<base>.forza` (index-aligned) |
| Scripted headless race | `SegaRally2.exe --headless --no-menu --frames 6000 --auto coin@900,start@960,gas@1500,gas@1800,...(every ~300: menus confirm by pedal)...,gas+@4000,aim165/128@3500` — `aimX/Y@F` sets the steering raw ADC from frame F |

## The local changes (vs upstream)

Exactly these, so they can be re-applied after `update.bat` (which overwrites
`runtime.cpp` and `CMakeLists.txt`, backing up edits to `out\update_backup\`):

1. `src/runtime/telemetry.cpp/.h` — the emitter (new files, update-safe).
2. `src/runtime/runtime.cpp` — `#include "telemetry.h"` near `stats.h`, and
   `telemetry_frame(g_drive.force(), g_drive.vibration());` immediately after
   `stats_frame();` in the vblank handler.
3. `CMakeLists.txt` — `src/runtime/telemetry.cpp` in the source list, `ws2_32`
   in `target_link_libraries`.
4. `Play with telemetry.bat`, `docs/TELEMETRY*.md`, `tools/telemetry_listen_test.py`,
   `tools/addr_xref.py`, `tools/car_scan.py`, `tools/matrix_scan.py` (new files).
5. `docs/game_ram.md` — added the "Player car state" section (RE findings).

## Telemetry: what the dashboard gets

UDP, one datagram per game frame (57.524 Hz), off unless `SR2_TELEMETRY` is
set. The `@forza` format is the standard 324-byte Forza Horizon 4/5 data-out
packet, **filled with the reader's conventions** (full byte table in
`docs/TELEMETRY_HANDOVER.md` §2):

- accel offsets 20/24/28 = X lateral (+ right) / Y vertical **with gravity
  removed** / Z longitudinal, **m/s²**
- VelocityZ (40) = signed forward speed (negative reversing); 32/36 = 0
- AngularVelocityY (48) = yaw rate (+ turning right), **slew-limited to
  ±0.436 rad/s of change per frame** (~1440°/s² — a limit, not a filter,
  per the dashboard agent); accelX is derived from the limited value
- yaw (56) / pitch (60) / roll (64) = real angles in radians: world heading;
  pitch **+ = nose down** in the packet (JSON `pitch` keeps the game's own
  + = nose up); roll + = lean left, identical to JSON
- suspension (68) = the game's real per-wheel values; RPM (16) = the game's
  own engine speed; silence while paused (dashboard watchdogs ~300 ms)

All channels are the arcade's real physics except vertical (modelled from the
FFB shake channel). `lat_g = v_road × yaw_rate` — **signed**, mirrors
correctly reversing (a parser bug report about positive-only lat_g was
investigated and disproven; see `docs/telemetry_sample/turn_sign_fixture.ndjson`
and the frames 4876-4906 negative-lat_g segment in `capture.ndjson`).

## The reverse-engineered car state (the valuable bit)

The player car's road-space physics lives in a 396-byte heap block, pointer
at **[0x2242B0]** (dereference every frame; the block address changes per
session). Full table in `docs/game_ram.md` "Player car state"; short form:

| Offset | Field |
|---|---|
| +0x00 | lateral offset, m, + right of centre |
| +0x04 | heading, rad, world |
| +0x08 | signed metres along the road |
| +0x0C | roll, rad, + lean left |
| +0x14 | road velocity ÷ 60.24, signed |
| +0x20 | game steering, ±0.637, − = right |
| +0x24 | 4× per-wheel suspension |
| +0x48 | track point (int) |
| +0xFC | pitch, rad, + nose up (brake dive; 0 under accel/at rest; impacts move it) |
| +0x10C | tire-model lateral force, + left |
| 0x1B0C58 | engine RPM normalized ×8800 |

Not mapped yet: world position, vertical velocity (heave is modelled),
the car's own orientation matrix (the game keeps the car in road space —
there is likely no chassis matrix in RAM at all; the render matrices found by
`tools/matrix_scan.py` are wheels/particles).

### The RE method (for extending the map)

Anchors first (HUD speed 0x22F3A8), then three RAM-dump experiments
(`SR2_RAM_DUMPS=frame,frame,...` writes 8 MB `out/ram_NNNN.bin`):

1. **Mirrored steering runs** (aim left vs right, dumps at identical frames):
   fields that flip sign exactly = lateral group (lat offset, roll, steer);
   identical fields = longitudinal (this proved SR2 has no steering-pitch).
2. **Straight full-brake run, dumps every ~10 frames through the brake
   bite**: pitch shows as a dive transient (−0.075 rad at −2.9 g); sparse
   dumps miss it. Launch-phase dumps prove squat is not modelled.
3. **Static xref**: `python tools/analyze.py` regenerates `out/listing.asm`,
   then `tools/addr_xref.py <lo> <hi> [w]` pairs lis/addi into absolute
   addresses and lists every access in a range (found the speed anchor's
   readers and the engine block).

`tools/car_scan.py` (A/B diff), `tools/matrix_scan.py` (orthonormal 3×3
scan). Sign conventions must be proven empirically (mirror test), never
assumed. Validate every candidate as finite and in range per frame — in
menus the car block is stale.

## Known pitfalls

- Building from Git Bash: plain `vcvars64.bat` calls fail — always go
  through `build.bat`. `printf` in Git Bash eats `\U` in paths; use Write +
  batch files for robocopy-style commands.
- `#define NOMINMAX` before windows.h in any new runtime file; `<algorithm>`
  for std::clamp; JSON line buffer ≥ 1024 bytes.
- The heap car pointer can go stale in menus — always range-check it.
- VRAM/GPU: no Vulkan SDK needed (volk loads vulkan-1.dll at runtime).
- `save/settings.ini` holds `experience = pc|arcade` — ARCADE ORIGINAL boots
  the cabinet attract (looks like "the original game's menu"); Esc → pause →
  back to PC EDITION, or edit the file while the game is closed.
- Upstream updates: `git pull upstream main` (this checkout's `upstream`
  remote is snicolast's repo, `origin` the fork). `update.bat` overwrites
  `runtime.cpp` and `CMakeLists.txt`, so after it the two-line hook (item 2
  above) has to be re-applied.

## Where things stand / next ideas

- VN Motion Dashboard ingests the Forza source on UDP 5300 successfully
  (pitch/roll/yaw displays live, angles in degrees). Its **custom UDP source
  on 20777 binds but does not display** — that's dashboard-side, the packets
  are proven correct (their agent's parser already validated the format
  offline and caught the one real bug, the reversing lat_g sign, now fixed).
- Possible next: world-position/vertical mapping (road spline height at
  track point), a JSON/Forza recorder + replay tool for rig tuning, upstream
  PR for the telemetry (author's call, issue #111).
- Times: every time sent (Forza 296-308, the JSON lap, race and HUD times)
  is seconds of the game's own clock, 1/60 s a frame, as the HUD shows them —
  4.1 % slower than real time. Until 2026-11-19 the lap timer counted real
  seconds and the JSON HUD times decoded 8 bits off (47.9 s read 0.479).
  Checked in a scripted practice race whose two laps the game's own progress
  code completed (`SR2_POKE=22432C=1@960,22CFBC=257@4600-4610,22CFBC=513@5600-5610`,
  game_ram.md "Progress"): the race time follows the stage clock to 0.003 s
  and the HUD's TOTAL TIME to a frame through both lap lines.
- A quirk of the game, not the emitter: lap 1's recorded time was 0.78 s
  less than the lap timer and the HUD showed at the line (18.94 against
  19.72), and from lap 2 on the HUD's own LAP TIME runs 0.77 s ahead of the
  lap timer — it is the stage time minus the recorded laps — while lap 2's
  record matched the lap timer (16.66 against 16.67). Seen with poked laps
  only (a poke 1 or 44 points past the line gives the same numbers); a lap
  driven over the line is still to compare.
- Best and last lap count the race's laps (+0x3C) since 2026-11-19. The
  lap count came from +0x38, the stages (1 in practice), so in practice they
  only ever looked at lap 1, and the JSON `laps` showed the stages.
- Related project: the same method and packet were carried over to Daytona
  USA (Model 2/i960) — Peacemaker105/daytona-arcade-recomp,
  `docs/telemetry.md`. Daytona USA 2 would be Model 3/PPC, like this game.
