// Telemetry for motion simulators and dashboards (telemetry.cpp).
//
// Off by default. Set SR2_TELEMETRY to turn it on:
//   SR2_TELEMETRY=127.0.0.1:20777             JSON lines, one per frame
//   SR2_TELEMETRY=127.0.0.1:20777@forza       Forza Horizon data-out packet
//                                             (324 bytes, what SimHub reads)
//   SR2_TELEMETRY=127.0.0.1:20777@both        both, JSON first
// The game sends one UDP datagram per video frame (57.5 Hz) to host:port.
// The drive board's force and vibration come in from the caller (ffb_frame
// has just read them off the servo). In the forza packet the acceleration
// channels are X lateral / Y vertical (gravity out) / Z longitudinal, in
// m/s^2, velocity Z is the signed forward speed and angular velocity Y the
// yaw rate (+ turning right) — the reader's convention, not the Forza game's.
void telemetry_frame(float ffb_force, float ffb_vibration);
