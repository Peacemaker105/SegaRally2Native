# Telemetry (motion rigs and dashboards)

The port can send the game's live state as UDP while you race: speed, gear, the
pedals and steering you give it, the force the game asks of the wheel's servo,
acceleration, lap and race state, position on the grid. It is read the same way
as the statistics are (docs/game_ram.md), one sample per video frame at the
Model 3's 57.5 Hz, and nothing in the game is changed by it.

Off by default. Set the environment variable `SR2_TELEMETRY` before starting:

    SR2_TELEMETRY=127.0.0.1:20777            JSON, one line a frame
    SR2_TELEMETRY=127.0.0.1:20777@forza      Forza Horizon data-out (SimHub)
    SR2_TELEMETRY=127.0.0.1:20777@both       both to the same port

`host:port` can be another machine on the LAN (a broadcast address such as
`192.168.1.255` works too). On Windows: `set SR2_TELEMETRY=127.0.0.1:20777` in
the same prompt you start `SegaRally2.exe` from. The game prints
`[telemetry] sending ...` on the console when it is on.

## Formats

**json** — one line per frame, every field named (speed, signed forward
velocity, the game's RPM, gear, steer, gas, brake, ffb, long_g, lat_g, vert_g,
yaw, yaw_rate, roll, pitch, world position x/y/z, distance driven (odo),
per-wheel suspension,
plus the whole race state: progress, laps, times, round, rank, result). Good
for FlyPT Mover's generic UDP input, scripts, and logging.
`tools/telemetry_listen_test.py` is a small Python listener for testing:

    python tools/telemetry_listen_test.py 20777 5000 mycapture

It writes `mycapture.ndjson` (JSON lines) and `mycapture.forza` (raw packets)
so parsers get clean framing.

A worked example and the byte-for-byte layout of the Forza packet, written as
a handover for a dashboard's telemetry source, is in
[TELEMETRY_HANDOVER.md](TELEMETRY_HANDOVER.md), with a captured sample race
under `docs/telemetry_sample/`.

**forza** — the 324-byte Forza Horizon 4/5 data-out packet (232-byte sled +
12-byte gap + 80-byte dash block), which SimHub reads natively through its
Forza telemetry listener.

### SimHub setup

1. In SimHub, Settings > Raw datalog / Games: enable the **Forza Horizon 4/5**
   (or Forza Motorsport 7) data receiver and set its UDP port to the one you
   gave `SR2_TELEMETRY` (any free port works; there is no default to match).
2. Motion: SimHub's Forza profile works as-is — it cues from the acceleration
   channels. Tuning is easier with Sega Rally 2's own character in mind: the
   car slides more than it grips, so sway/roll cues come mostly from `lat_g`.
3. Dashboards: gear, speed (m/s), RPM (estimate), fuel, pedal and steering
   bytes all carry real values.

### What is real and what is derived

The game's own car state is mapped (docs/game_ram.md "Player car state"),
so most channels are the arcade program's real values:

- **Real**: speed, world position and velocity, heading, pitch and roll (world
  orientation: slopes, crests, banks and body motion), per-wheel suspension,
  the engine's RPM, gear, the wheel/pedals/shifter the port feeds the cabinet,
  the drive board's servo force and vibration, lap and race times, progress,
  grid position, race result.
- **Computed from real values**: the yaw rate (the heading's frame-to-frame
  change), longitudinal and lateral acceleration (the world velocity's change
  in the car's own axes, so braking, sand, wall hits and slides are all the
  game's own physics), forward velocity, distance driven, wheel rotation speeds.
- **Still modelled**: vertical acceleration (from the servo's shake channel,
  kerbs and bumps), slip ratios, tire temps, power/torque.

In the forza packet the acceleration channels follow the reader's convention,
in m/s^2: X lateral (+ right), Y vertical with gravity taken out (about 0 at
rest), Z longitudinal (+ accelerating); velocity Z is the signed forward
speed and angular velocity Y the yaw rate (+ turning right). A worked example
and the byte-for-byte layout of the packet, written as a handover for a
dashboard's telemetry source, is in
[TELEMETRY_HANDOVER.md](TELEMETRY_HANDOVER.md), with a captured sample race
under `docs/telemetry_sample/`.

## Heads-up

`update.bat` overwrites the files it updates (`runtime.cpp`, `CMakeLists.txt`
here) with the upstream ones, keeping your edited copies in
`out\update_backup\`: after an update, put the `telemetry_frame` call and the
`ws2_32` link back, or diff against the backup. New files (`telemetry.cpp`,
`telemetry.h`) are untouched.

This is a local modification for personal use (the license forbids sharing
builds); upstream, the same feature is requested in issue #111.
