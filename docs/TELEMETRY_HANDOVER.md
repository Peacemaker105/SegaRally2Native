# SegaRally2Native telemetry — handover for a motion-cueing dashboard source

This documents the UDP telemetry of SegaRally2Native, for implementing
a telemetry source in a dashboard/motion-cueing app. Everything here is
producer-guaranteed: the emitter is `src/runtime/telemetry.cpp`, one sample
per game frame, and the sample data in `docs/telemetry_sample/` was captured
from it.

Version 2, after mapping the game's own road-space car physics (see
`docs/game_ram.md` "Player car state"). The acceleration channels now come
from the arcade program's real state, in **the reader's axis convention**, and
the engine speed is the game's own RPM. Byte layout is unchanged and still the
standard 324-byte Forza Horizon 4/5 data-out packet (232-byte sled + 12-byte
gap + 80-byte dash block), so size-based dispatch is unaffected.

---

## 1. Transport

| | |
|---|---|
| Protocol | UDP unicast, datagrams from the game to a configured host:port |
| Enable | env var `SR2_TELEMETRY=host:port[@json\|@forza\|@both]`, set before starting `SegaRally2.exe`; default off |
| Handshake | none — first datagram arrives as soon as the game's first frame finishes |
| Rate | one datagram per video frame, **57.524 Hz** nominal (the Model 3's vblank, not 60) |
| Sender identity | ephemeral source port; one target per game instance; broadcast allowed (host ending `.255`) |
| Datagram sizes | JSON ~600-700 bytes, variable; Forza exactly 324 bytes |

**Difference from real Forza #1 — silence on pause.** Datagrams flow with
`is_race_on=0` in menus/attract/results, but when the game is **paused (Esc)
or closed the stream goes silent** (no frames execute). Watchdog ~250-300 ms
→ treat as paused/closed and home the rig.

**Difference #2 — game time vs wall time.** `timestamp_ms` counts game frames
(≈ +17.38 ms per packet). Windowed runs are real-time paced; a `--headless`
run executes as fast as the CPU allows, so recorded captures must be replayed
by packet timestamp, not arrival time. `timestamp_ms` resets to 0 when the
game relaunches.

## 2. Forza format, byte by byte (324 bytes, little-endian)

Acceleration channels use **the reader's convention, in m/s²** (not the Forza
game's own semantics): **X = lateral (+ right), Y = vertical with gravity
taken out (≈0 at rest), Z = longitudinal (+ accelerating)**. Legend:
**R** = real game data, **D** = computed from real game data, **S** =
synthetic placeholder, **0** = always zero.

### Sled (offsets 0-231)

| Offset | Type | Field | Value / meaning |
|---|---|---|---|
| 0 | i32 | is_race_on | **R** 1 in a race (clock counting or car GO), 0 in menus/attract/results. Gate motion cues on this |
| 4 | u32 | timestamp_ms | **R** game-frame counter × 17.384; resets on relaunch |
| 8 | f32 | engine_max_rpm | **S** constant 8800 |
| 12 | f32 | engine_idle_rpm | **S** constant 800 |
| 16 | f32 | current_engine_rpm | **R** the game's own engine speed (0x1B0C58 × 8800, clamped 800-8800); falls toward idle coasting, 0 in menus |
| 20 | f32 | accel X | **D** lateral, m/s², **+ right** = signed forward velocity × yaw rate (v_road × yaw_rate). Mirrors correctly when the car moves backwards — verified on 889 reversing frames |
| 24 | f32 | accel Y | **D** vertical, m/s², **gravity removed** (≈0 at rest): the FFB shake channel (kerbs, bumps, wall hits) |
| 28 | f32 | accel Z | **D** longitudinal, m/s², **+ accelerating**, from the signed road velocity. Wall hits show as real spikes |
| 32 | f32 | velocity X | **0** (sideways speed, reserved) |
| 36 | f32 | velocity Y | **0** |
| 40 | f32 | velocity Z | **R** signed forward speed along the road, m/s (negative when sliding backwards off a wall) |
| 44 | f32 | angular_velocity X | **0** (pitch rate) |
| 48 | f32 | angular_velocity Y | **D** yaw rate, rad/s, **+ turning right** (frame-to-frame derivative of the game's heading), **slew-limited**: the value may change by at most 0.436 rad/s per frame (25°/s per frame, ≈1440°/s²) — a limit, not a low-pass; wall-hit spikes are clamped |
| 52 | f32 | angular_velocity Z | **0** (roll rate) |
| 56 | f32 | yaw | **R** heading, radians, world frame (changes with road curvature + steering) |
| 60 | f32 | pitch | **R** body pitch, radians, **+ = nose down** (the game's own value negated: its brake dive of −0.075 ≈ −4.3° at −2.9 g arrives as +0.075), 0 at rest and under steady acceleration; wall hits and crests move it further |
| 64 | f32 | roll | **R** body roll, radians, **+ = leaning left** (outward in a right turn) — mirror-verified ±0.24 rad under hard steering; identical to the JSON `roll` |
| 68-83 | 4×f32 | susp travel norm FL,FR,RL,RR | **R** the game's per-wheel suspension, scaled ×4 to 0..1 (1 compressed); load transfer under braking/cornering is visible |
| 84-99 | 4×f32 | tire slip ratio FL,FR,RL,RR | **D** 0 except rear >0 on launch wheelspin (gas>0.7, speed<14) and brake lock |
| 100-115 | 4×f32 | wheel rotation speed RPM | **D** \|v_road\| × 30.8 (≈1.95 m rolling circumference) |
| 116-131 | 4×i32 | wheel on ground | **S** constant 1 |
| 132-147 | 4×f32 | puddle depth | **0** |
| 148-163 | 4×f32 | surface rumble | **R** the drive board's vibration 0..1 (the arcade wheel's shake) |
| 164-179 | 4×f32 | tire slip angle | **0** |
| 180-195 | 4×f32 | tire combined slip | **D** = the rear slip model |
| 196-211 | 4×f32 | susp travel meters | **D** mean of the four normalized values |
| 212 | i32 | car ordinal | **R** SELECT CAR model + 1 (1 Peugeot 306, 2 Corolla, 3 Escort, 4 Impreza, 5 Lancer, 6 Stratos, 7 Celica, 8 Delta) |
| 216 | i32 | car class | **S** 1 |
| 220 | i32 | performance index | **S** 999 |
| 224 | i32 | drivetrain | **R** 1 (RWD — true of every SR2 car) |
| 228 | i32 | cylinders | **S** 6 (12 for the Stratos) |

### Gap (232-243): 12 zero bytes — required by the FH4/FH5 layout.

### Dash block (offsets 244-323)

| Offset | Type | Field | Value / meaning |
|---|---|---|---|
| 244-255 | 3×f32 | position X,Y,Z | **0** — world position is not mapped (the game tracks the car in road space; see the JSON's road-space fields) |
| 256 | f32 | speed | **R** total speed m/s (always ≥ 0) |
| 260 | f32 | power | **D** W: 300 Nm × ω × throttle |
| 264 | f32 | torque | **D** Nm: 300 × throttle |
| 268-283 | 4×f32 | tire temp | **D** °F: 90 + speed·0.3 + slip·30 |
| 284 | f32 | boost | **S** 0 |
| 288 | f32 | fuel | **S** 0.5 |
| 292 | f32 | distance traveled | **R** signed metres along the road (decreases sliding backwards!) |
| 296 | f32 | best lap | **R** seconds, 0.0 until a lap completes |
| 300 | f32 | last lap | **R** seconds |
| 304 | f32 | current lap | **R** live lap timer (0 while the clock isn't counting) |
| 308 | f32 | current race time | **R** the stage clock: 0 at each stage start, laps summed, on the same clock as 304 (0 off the race) |
| 312 | u16 | lap number | **R** 1-based |
| 314 | u8 | race position | **R** 1-based, 1 = leader (stale on result screens — gate on is_race_on) |
| 315 | u8 | accel | **R** 0-255 |
| 316 | u8 | brake | **R** 0-255 |
| 317 | u8 | clutch | **S** 0 |
| 318 | u8 | handbrake | **R** 0 or 255 |
| 319 | u8 | gear | **R** 1-4 in a race, 0 otherwise; SR2 has 4 gears, no reverse reported |
| 320 | i8 | steer | **R** −127..127, **+ = right**, the cabinet ADC input |
| 321 | i8 | driving line | **S** −128 |
| 322 | i8 | AI brake difference | **S** −128 |
| 323 | u8 | (padding) | 0 |

## 3. Physics-grade notes for cueing

- **Longitudinal** comes from the game's own signed road velocity, so
  braking, lift-off, sand, wall hits and even reversing are all real. Hard
  wall contacts reach ±3 g for a few frames; brake dives ~−2.9 g sustained.
- **Lateral** is centripetal: **v_road × yaw_rate** (both signed, so it
  mirrors correctly reversing — verified across 889 reversing frames in the
  sample). It vanishes at walking pace (unlike the game's raw tire-force
  value, which pulls 1.4 g even rolling slowly — exported in JSON as
  `game_lat` if you want that arcade flavour instead). Extreme wall hits
  clamp at ±39.2 m/s² (4 g); two such frames exist in the sample.
- **Yaw rate** is the real frame-to-frame heading derivative. The heading
  follows the road spline, so a perfectly-driven corner still shows rotation —
  that is the car's true rotation and is what a sway/yaw platform wants.
- **Roll and pitch are real now** (offsets 64/60): roll flips mirrored with
  steering (±0.24 rad, + = lean left — outward, the felt direction); pitch is
  the game's brake dive, sent **nose-down positive** in the packet (braking
  → +0.075; the JSON `pitch` keeps the game's own sign, + = nose up), 0 at
  rest and under acceleration (the arcade models no squat). Yaw (56) is the
  world heading; the yaw rate (48) and everything derived from it are
  slew-limited to 0.436 rad/s of change per frame.
- **Vertical** is still modelled (FFB shake), 0 at rest by convention.
- **RPM** is the game's own engine speed; shift cues can key off the gear
  byte, RPM gradients are now trustworthy too.
- Road-space extras in JSON: `lat_off` (metres right of the centre line —
  lane position, useful for camber/kerb cues), `dist` (metres along the road),
  `track_point`, and `game_lat` (the tire model's own lateral force, + left).

## 4. JSON format (the other option, or "both")

One line per frame, `\n`-terminated, same sample moment as the Forza packet.
Fields beyond the Forza packet: `v_road` (signed road velocity m/s),
`game_steer` (the game's own steering, + = right, ±1), `yaw`, `yaw_rate`,
`roll` (radians, + = lean left), `lat_off`, `dist`, `track_point`, `game_lat`,
`susp[4]` (0..1 per wheel FL FR RL RR), plus the full race state (`pos`,
`cars`, `lap`, `laps`, `laps_done`, `prog`, `sect`, `chk`, `round`, `mode`,
`phase`, `seq`, `result`, `goal`, `clock`, `time_left`, `car`, `gears`,
`lap_time`, `best_lap`, `last_lap`, `race_time`, `total_bcd`, `lap_bcd`,
`ffb`, `vib`, `shake`, `hand`). `round`: championship Desert=3, Mountain=2,
Snowy=5, Riviera=4; practice = +6.

## 5. Sample data (in this repo)

`docs/telemetry_sample/` — one v2 capture, two files, **index-aligned**
(record i of the .forza file = line i of the .ndjson):

- `capture.ndjson` — 2914 JSON lines: 30 attract frames (`on:0`), then the
  race to the end of the run.
- `capture.forza` — 2914 × 324 bytes, raw concatenated packets.

The race: countdown → launch, steering right then left at speed, gear 1→2
shifts, wall contacts (long_g spikes, real yaw-rate oscillation), signed
distance going backwards when reversed off a wall, suspension load transfer.
Captured with `--headless` (compressed arrival times — replay by
`frame`/`timestamp_ms`). Captured before offset 308 carried the stage clock;
the run never completes a lap, so 308 equals the lap timer at 304
throughout, which is also what it reads now.

Reproduce a capture:

```
set SR2_TELEMETRY=127.0.0.1:20777@both
python tools\telemetry_listen_test.py 20777 20000 mycapture
SegaRally2.exe --headless --no-menu --frames 6000 --auto coin@900,start@960,gas@1500,gas@1800,gas@2100,gas@2400,gas@2700,gas@3000,gas@3300,gas@3600,gas+@4000,aim165/128@3500,aim95/128@4800,up@4300,up@4900
```

(menu confirms are repeated gas presses; `aimX/Y@F` sets the steering raw ADC
from frame F; drop `--headless` for a windowed run streaming at real 57.5 Hz
while you drive.)

## 6. Source implementation checklist

1. Bind/recv UDP; classify by datagram size (324 = Forza) or leading `{` (JSON).
2. Watchdog ~300 ms silence → paused/closed → home/zero the rig.
3. Gate cues on `is_race_on`; menus still stream (gear=0, speed=0, RPM=0).
4. Time cues by `timestamp_ms`/`frame` deltas; handle reset-to-0 on relaunch.
5. Axis audit: steer right while moving → accelX > 0 (m/s²), angularY > 0
   (rad/s), accelZ > 0 only while accelerating; at rest accelX ≈ accelY ≈
   accelZ ≈ 0 (vertical has gravity removed).
6. `is_race_on` 0 → cues fade; the packets keep flowing in menus.

## 7. Where the data comes from (for the curious)

The player car's road-space physics state is mapped at the pointer in
[0x2242B0]: lateral offset, heading, signed road distance, body roll,
road-relative velocity, per-wheel suspension, steering — confirmed by
mirrored-steering and braking RAM-dump experiments. The engine speed is at
0x1B0C58 (normalized, ×8800). The full table and the method are in
`docs/game_ram.md` "Player car state". Still unmapped: world position, pitch,
vertical velocity (heave is modelled from the FFB shake channel).
