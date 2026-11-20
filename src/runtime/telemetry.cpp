// Telemetry for motion simulators and dashboards: one UDP datagram per video
// frame with the game's race state, read the same way as stats.cpp does at
// every vblank (docs/game_ram.md has each address and how it was found), plus
// what the runtime itself knows: the wheel, pedals and shifter it feeds the
// game (inputs.h) and the force the game asks of the drive board's servo
// (passed in by the caller). Nothing is hooked and nothing is written to the
// game's RAM.
//
// Off unless SR2_TELEMETRY is set (telemetry.h). Two formats:
// - json: one line per frame with every field named, for FlyPT Mover, a
//   script or anything else custom.
// - forza: the 324-byte Forza Horizon 4/5 data-out packet (232-byte sled +
//   12-byte gap + 80-byte dash block), what SimHub reads natively and most
//   motion software gets through it. Our acceleration values in it follow the
//   convention of the reader this was built for, not the Forza game's own:
//   X = lateral (+ right), Y = vertical with gravity taken out (~0 at rest),
//   Z = longitudinal (+ accelerating), all in m/s^2; velocity Z is the
//   forward speed, angular velocity Y the yaw rate (+ turning right).
//
// The player car's physics state is mapped (docs/game_ram.md "Player car
// state"): its world position and velocity, its heading, pitch and roll (all
// three checked against the car the renderer draws), the per-wheel suspension
// and the engine speed. From those the yaw rate and the acceleration in the
// car's own axes are computed per frame, so what a rig feels is the game's own
// physics; only the vertical channel is still modelled (from the force
// feedback's shake).
#include "telemetry.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>

#include "inputs.h"
#include "ppc_rt.h"

namespace {

constexpr double FRAME_HZ = 57.524;   // the Model 3's vblank
constexpr double GAME_CLOCK_FPS = 60;   // the game's own timekeeping: 1/60 s a frame (100 ticks), so its
                                        // seconds run 4.1 % slower than real ones. The HUD, the lap
                                        // records and every time sent below count this way
constexpr float G = 9.80665f;

// Game RAM
constexpr uint32_t SEQUENCE = 0x22D618, PHASE = 0x22D624;
constexpr uint32_t GAME = 0x22CF14;
constexpr uint32_t G_MODE = GAME + 0x00, G_ROUND = GAME + 0x34, G_LAPS = GAME + 0x3C;   // laps of this race (+0x38: the stages)
constexpr uint32_t G_CAR_STATUS = GAME + 0x6C, G_CLOCK = GAME + 0x70, G_LEFT = GAME + 0x74;
constexpr uint32_t G_GOAL = GAME + 0x7C, G_LAPS_DONE = GAME + 0x94, G_CHECKPOINTS = GAME + 0x98;
constexpr uint32_t G_SECTION = GAME + 0x9C, G_PROGRESS = GAME + 0xA8, G_RANK = GAME + 0x104;
constexpr uint32_t G_RACE_TIME = GAME + 0x16C, G_LAP_TIMES = GAME + 0x684, G_RESULT = GAME + 0x71C;
constexpr uint32_t HUD_POSITION = 0x13AB74, CARS = 0x225A18, HUD_GEAR = 0x13AB80;
constexpr uint32_t HUD_TOTAL_BCD = 0x13AB8C, HUD_LAP_BCD = 0x13AB90, HUD_LAP_NO = 0x13AB94;
constexpr uint32_t LAP_FRAMES = 0x1B0EB0;
constexpr uint32_t SPEED = 0x22F3A8;          // m/s, the player's car (trigger_frame reads it too)
constexpr uint32_t RPM_NORM = 0x1B0C58;       // engine speed 0..~1 (x RPM_MAX; falls toward idle coasting)
constexpr uint32_t CAR_PTR = 0x2242B0;        // pointer to the player car's state
// The player car state, at the pointer above (a heap address, read every
// frame). Found by RAM-dump diffing of mirrored steering runs and a straight
// braking run (docs/game_ram.md "Player car state" has the table and the
// method; tools/car_scan.py compares dumps).
// The block is the car's world-space rigid body (re-mapped 2026-11-20 against the
// renderer's own picture of the car, docs/game_ram.md "Player car state"):
constexpr uint32_t C_POS = 0x00;        // 3 floats: world X, Y (height, + up), Z, metres
constexpr uint32_t C_VEL = 0x0C;        // 3 floats: world velocity X, Y, Z, metres per game frame
constexpr uint32_t C_STEER = 0x20;      // the game's own steering, -0.637 (right) .. +0.637 (left)
constexpr uint32_t C_SUSP = 0x24;       // 4 wheels, suspension compression (x4 = about 0..1)
constexpr uint32_t C_TRACK_PT = 0x48;   // int: the track point under the car
constexpr uint32_t C_PITCH = 0x108;     // radians, + = nose up (world: slopes, crests, dive)
constexpr uint32_t C_HEADING = 0x10C;   // radians, world; minus it is the heading, which grows turning right
constexpr uint32_t C_ROLL = 0x110;      // radians, + = leaning right (world: banks, body roll)
constexpr float RPM_MAX = 8800.0f;      // the limiter every car hits; idle sits near 800
constexpr float STEER_MAX = 0.637f;     // the car state's full-lock steering
constexpr int MODE_CHAMPIONSHIP = 0, MODE_PRACTICE = 2;
constexpr int CLOCK_COUNTING = 2, CAR_GO = 1;
constexpr int MAX_LAPS = 8;
constexpr float RPM_IDLE = 800;
constexpr float YAW_RATE_STEP = 0.436f;   // the yaw rate may change by at most this per frame
                                          // (25°/s a frame, ~1440°/s² at 57.524 Hz): a wall hit
                                          // rotates the car violently, and the rig must not be
                                          // handed the spike. A limit, not a filter - the
                                          // dashboard asked for exactly this.

int32_t rdi(uint32_t a) { return int32_t(rd32(a)); }

float speed_of() {   // 0, and the HUD with it, while it is nonsense (menus)
    uint32_t bits = rd32(SPEED);
    float f;
    memcpy(&f, &bits, 4);
    if (!std::isfinite(f) || f < 0 || f > 150) return 0;
    return f;
}

float float_at(uint32_t a) {   // a game float, 0 when it is not finite
    uint32_t bits = rd32(a);
    float f;
    memcpy(&f, &bits, 4);
    return std::isfinite(f) ? f : 0;
}

bool valid_car(uint32_t car) {   // the pointer is a heap block in RAM, or stale
    return car >= 0x100000 && car < 0x800000 - 0x200;
}

// A HUD time: 0xMSSmmm, digits of minutes, seconds and thousandths (0x151466 is
// 1'51"466). 0 when the HUD shows none: -1 between stages, or any non-digit
int bcd_nib(uint32_t b, int shift) { return int(b >> shift) & 0xF; }
double bcd_time(uint32_t b) {
    if (b >> 24) return 0;
    for (int shift = 0; shift < 24; shift += 4)
        if (bcd_nib(b, shift) > 9) return 0;
    return bcd_nib(b, 20) * 60.0 + bcd_nib(b, 16) * 10.0 + bcd_nib(b, 12) + bcd_nib(b, 8) / 10.0 +
           bcd_nib(b, 4) / 100.0 + bcd_nib(b, 0) / 1000.0;
}

double seconds_of(int32_t ticks) { return double(ticks) / 6000.0; }

float wrap_pi(float a) {   // keep a yaw difference in -pi..pi
    while (a > 3.14159265f) a -= 6.28318531f;
    while (a < -3.14159265f) a += 6.28318531f;
    return a;
}

// ---- One game state sample, and what the two formats are built from ----
struct Sample {
    bool on = false;        // racing: what Forza's is_race_on and SimHub's active mean
    bool car_ok = false;    // the car state was read this frame (a heading of exactly 0 is a real heading)
    float speed = 0;        // m/s, the game's own total speed
    float v_road = 0;       // signed velocity along the road (negative reversing), m/s
    float rpm = 0;          // the game's engine speed, 800..8800
    float steer = 0;        // -1 left .. +1 right, what the port feeds the game
    float game_steer = 0;   // the car state's own steering, -1 right .. +1 left
    float gas = 0, brake = 0;
    bool hand = false;
    int gear = 1;           // the HUD's 1-4
    float ffb = 0, vib = 0, shake = 0;   // the servo's force and vibration
    float long_g = 0, lat_g = 0, vert_g = 1;   // car frame: + long accelerates, + lat right
    float yaw = 0, yaw_rate = 0, roll = 0, pitch = 0;   // radians; yaw rate + = turning right
    float pos[3] = {0, 0, 0};   // world X, Y (up), Z, metres
    float vel[3] = {0, 0, 0};   // world velocity, m/s
    double odo = 0;             // metres driven this race (the packet's distance)
    float susp[4] = {0.5f, 0.5f, 0.5f, 0.5f};  // 0..1, 1 compressed
    int track_point = -1;
    int position = 0, cars = 0;   // 0 = 1st, as the game keeps it
    int lap_no = 0, laps = 0;
    int progress = 0, section = 0, checkpoints = 0, laps_done = 0;
    int clock = 0, left = 0, goal = 0, result = 0, mode = -1, round = -1;
    int sequence = 0, phase = 0;
    int car = 0, gears = 0;
    double lap_time = 0, best_lap = 0, last_lap = 0, race_time = 0, total_bcd = 0, lap_bcd = 0;   // s, the game's clock
    uint64_t frame = 0;

    static Sample read(uint64_t frame, float ffb_force, float ffb_vibration) {
        Sample s;
        s.frame = frame;
        s.sequence = rdi(SEQUENCE), s.phase = rdi(PHASE);
        s.mode = rdi(G_MODE), s.round = rdi(G_ROUND), s.laps = rdi(G_LAPS);
        int car_status = rdi(G_CAR_STATUS);
        s.clock = rdi(G_CLOCK), s.left = rdi(G_LEFT);
        s.goal = rdi(G_GOAL), s.laps_done = rdi(G_LAPS_DONE);
        s.checkpoints = rdi(G_CHECKPOINTS), s.section = rdi(G_SECTION), s.progress = rdi(G_PROGRESS);
        s.result = rdi(G_RESULT);
        s.position = rdi(HUD_POSITION), s.cars = rdi(CARS), s.gear = rdi(HUD_GEAR), s.lap_no = rdi(HUD_LAP_NO);
        s.total_bcd = bcd_time(rd32(HUD_TOTAL_BCD)), s.lap_bcd = bcd_time(rd32(HUD_LAP_BCD));
        s.car = rdi(0x224324), s.gears = rdi(0x224334);
        s.on = s.sequence == 1 && s.phase == 1 && (s.mode == MODE_CHAMPIONSHIP || s.mode == MODE_PRACTICE) &&
               (s.clock == CLOCK_COUNTING || car_status == CAR_GO);
        s.speed = s.on ? speed_of() : 0;
        s.rpm = s.on ? std::clamp(float_at(RPM_NORM) * RPM_MAX, RPM_IDLE, RPM_MAX) : 0;
        uint32_t car = rd32(CAR_PTR);
        if (s.on && valid_car(car)) {
            s.car_ok = true;
            for (uint32_t i = 0; i < 3; i++) {
                s.pos[i] = float_at(car + C_POS + 4 * i);
                s.vel[i] = float_at(car + C_VEL + 4 * i) * float(FRAME_HZ);
            }
            s.yaw = wrap_pi(-float_at(car + C_HEADING));   // + = turned right
            s.pitch = float_at(car + C_PITCH);              // + = nose up
            s.roll = -float_at(car + C_ROLL);               // + = leaning left
            // forward velocity: the world velocity along the car's heading (signed: backwards < 0)
            s.v_road = s.vel[0] * std::sin(s.yaw) + s.vel[2] * std::cos(s.yaw);
            s.game_steer = std::clamp(-float_at(car + C_STEER) / STEER_MAX, -1.0f, 1.0f);   // + = right
            for (int i = 0; i < 4; i++)
                s.susp[i] = std::clamp(float_at(car + C_SUSP + 4 * uint32_t(i)) * 4.0f, 0.0f, 1.0f);
            s.track_point = rdi(car + C_TRACK_PT);
        }
        int steer = g_inputs.steer - 128;
        s.steer = std::clamp(float(steer) / 127.0f, -1.0f, 1.0f);
        s.gas = std::clamp(float(g_inputs.accel) / 255.0f, 0.0f, 1.0f);
        s.brake = std::clamp(float(g_inputs.brake) / 255.0f, 0.0f, 1.0f);
        s.hand = g_inputs.handbrake;
        s.ffb = ffb_force, s.vib = ffb_vibration;
        if (s.on && s.clock == CLOCK_COUNTING) s.lap_time = std::clamp(double(rdi(LAP_FRAMES)) / GAME_CLOCK_FPS, 0.0, 3600.0);
        for (int i = 0; i < std::min(std::min(s.laps_done, s.laps), MAX_LAPS); i++)
            s.last_lap = seconds_of(rdi(G_LAP_TIMES + 4 * uint32_t(i)));
        s.best_lap = s.last_lap;   // the game sorts nothing: keep the smallest of the lap times
        for (int i = 0; i < std::min(std::min(s.laps_done, s.laps), MAX_LAPS); i++)
            s.best_lap = std::min(s.best_lap, seconds_of(rdi(G_LAP_TIMES + 4 * uint32_t(i))));
        s.race_time = seconds_of(rdi(G_RACE_TIME));
        return s;
    }

    // Derived forces, from this and the last sample (called every frame).
    // The heading and the world velocity are the game's own values: the yaw
    // rate and the accelerations below are their frame-to-frame derivatives,
    // so a rig feels the real physics of the arcade program, walls and all.
    void derive(const Sample& was) {
        if (on && was.on && car_ok && was.car_ok) {
            // The heading derivative (+ = turning right), slew-limited against
            // was.yaw_rate (the previous frame's limited value)
            float raw = wrap_pi(yaw - was.yaw) * float(FRAME_HZ);
            yaw_rate = was.yaw_rate + std::clamp(raw - was.yaw_rate, -YAW_RATE_STEP, YAW_RATE_STEP);
            // The world acceleration in the car's own axes: forward (sin, cos)
            // of the heading, right (cos, -sin). + long accelerates, + lat is
            // to the right (a right turn), slides and wall hits included
            float ax = (vel[0] - was.vel[0]) * float(FRAME_HZ), az = (vel[2] - was.vel[2]) * float(FRAME_HZ);
            float sn = std::sin(yaw), cs = std::cos(yaw);
            lat_g = std::clamp((ax * cs - az * sn) / G, -4.0f, 4.0f);
            long_g = std::clamp((ax * sn + az * cs) / G, -3.0f, 3.0f);
        } else {
            yaw_rate = lat_g = long_g = 0;
        }
        if (on && was.on && car_ok && was.car_ok) {
            float h = std::sqrt(vel[0] * vel[0] + vel[2] * vel[2]);
            odo = was.odo + (h < 150.0f ? h / float(FRAME_HZ) : 0.0f);   // (a teleport to a restart adds nothing)
        } else if (!on) {
            odo = 0;
        }
        ffb_slow += 0.2f * (ffb - ffb_slow);
        shake = std::max(vib, std::fabs(ffb - ffb_slow));
        vert_g = 1.0f + shake * 0.6f;
        (void)prev_v;
    }
    float prev_v = 0;      // last frame's road velocity (kept for a future filter)
    static float ffb_slow;   // the FFB force's slow part, kept across frames
};

float Sample::ffb_slow = 0;

// ---- UDP ----
SOCKET sock = INVALID_SOCKET;
sockaddr_in dest{};
bool send_json = true, send_forza = false;
uint64_t frame_count = 0;

bool open_socket(const char* host, int port) {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa)) return false;
    memset(&dest, 0, sizeof dest);
    dest.sin_family = AF_INET;
    dest.sin_port = htons(uint16_t(port));
    // A dotted address as it is, else a name to look up (inet_pton returns 1
    // only for an address, and leaves sin_addr alone otherwise)
    if (inet_pton(AF_INET, host, &dest.sin_addr) != 1) {
        addrinfo hints{}, *ai = nullptr;
        hints.ai_family = AF_INET;   // dest is a sockaddr_in: an IPv4 answer (localhost is ::1 first)
        if (getaddrinfo(host, nullptr, &hints, &ai) || !ai) return false;
        dest.sin_addr = ((sockaddr_in*)ai->ai_addr)->sin_addr;
        freeaddrinfo(ai);
    }
    // The socket once the address is known: a name that does not resolve
    // leaves the telemetry off instead of sending into nothing
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) return false;
    // A broadcast address (a rig on another machine's subnet, x.x.x.255) is
    // allowed only with the socket option set
    if (ntohl(dest.sin_addr.s_addr) == 0xFFFFFFFF || (ntohl(dest.sin_addr.s_addr) & 0xFF) == 0xFF) {
        BOOL on = TRUE;
        setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (const char*)&on, sizeof on);
    }
    return true;
}

void put_f32(uint8_t* p, float v) { memcpy(p, &v, 4); }
void put_i32(uint8_t* p, int32_t v) { memcpy(p, &v, 4); }
void put_u32(uint8_t* p, uint32_t v) { memcpy(p, &v, 4); }

// The 324-byte Forza Horizon data-out packet: 232-byte sled, 12-byte gap, the
// 80-byte dash block. SimHub picks the format by the datagram's size. The
// acceleration channels follow the convention of the reader this was built
// for (see the header comment): X lateral / Y vertical / Z longitudinal, in
// m/s^2, vertical with gravity taken out.
size_t build_forza(const Sample& s, uint8_t* p) {
    memset(p, 0, 324);
    put_i32(p + 0, s.on ? 1 : 0);
    put_u32(p + 4, uint32_t(s.frame * 1000 / 57.524) & 0xFFFFFFFF);   // timestamp ms, wraps
    put_f32(p + 8, RPM_MAX), put_f32(p + 12, RPM_IDLE), put_f32(p + 16, s.rpm);
    put_f32(p + 20, s.lat_g * G);        // X: lateral, + right, m/s^2
    put_f32(p + 24, (s.vert_g - 1.0f) * G);   // Y: vertical, gravity out, ~0 at rest
    put_f32(p + 28, s.long_g * G);       // Z: longitudinal, + accelerating
    put_f32(p + 32, 0), put_f32(p + 36, 0), put_f32(p + 40, s.v_road);   // velocity: Z forward, signed
    put_f32(p + 44, 0), put_f32(p + 48, s.yaw_rate), put_f32(p + 52, 0);   // angular velocity: Y yaw, + right
    put_f32(p + 56, s.yaw), put_f32(p + 60, -s.pitch), put_f32(p + 64, s.roll);   // radians: heading (+ = turned right), pitch (reader's convention: + = nose down), roll (+ = lean left, as JSON)
    for (int i = 0; i < 4; i++) put_f32(p + 68 + 4 * uint32_t(i), s.susp[i]);   // FL FR RL RR, 1 = compressed
    // Slip at the driven (rear) wheels: spinning away, locking brakes
    float slip = 0;
    if (s.gas > 0.7f && s.speed < 14.0f) slip = s.gas * (1 - s.speed / 14.0f) * 0.8f;
    if (s.brake > 0.7f && s.speed > 5.0f && s.long_g < -0.5f) slip = std::max(slip, 0.3f * s.brake);
    put_f32(p + 84, 0), put_f32(p + 88, 0), put_f32(p + 92, slip), put_f32(p + 96, slip);
    float wheel_rpm = std::fabs(s.v_road) * 30.8f;   // a wheel of about 1.95 m a turn
    put_f32(p + 100, wheel_rpm), put_f32(p + 104, wheel_rpm), put_f32(p + 108, wheel_rpm), put_f32(p + 112, wheel_rpm);
    for (int i = 0; i < 4; i++) put_i32(p + 116 + 4 * uint32_t(i), 1);   // wheels on the ground
    float temp = 90 + s.speed * 0.3f + slip * 30.0f;   // Fahrenheit on the wire
    put_f32(p + 132, temp), put_f32(p + 136, temp), put_f32(p + 140, temp), put_f32(p + 144, temp);
    for (int i = 0; i < 4; i++) put_f32(p + 148 + 4 * uint32_t(i), s.vib);   // surface rumble
    for (int i = 0; i < 4; i++) put_f32(p + 164 + 4 * uint32_t(i), 0);   // slip angle
    for (int i = 0; i < 4; i++) put_f32(p + 180 + 4 * uint32_t(i), slip);   // combined slip
    float travel = std::clamp(0.25f * (s.susp[0] + s.susp[1] + s.susp[2] + s.susp[3]), 0.0f, 1.0f);
    put_f32(p + 196, travel), put_f32(p + 200, travel), put_f32(p + 204, travel), put_f32(p + 208, travel);
    put_i32(p + 212, s.car >= 0 && s.car < 8 ? s.car + 1 : 1);   // car ordinal: the SELECT CAR model
    put_i32(p + 216, 1), put_i32(p + 220, 999);   // class, performance index
    put_i32(p + 224, 1);   // drivetrain: RWD
    put_i32(p + 228, s.car == 5 ? 12 : 6);   // cylinders (the Stratos has a V12)
    // The dash block, after the 12-byte gap at 232
    uint8_t* d = p + 244;
    put_f32(d + 0, s.pos[0]), put_f32(d + 4, s.pos[1]), put_f32(d + 8, s.pos[2]);   // world position, metres (Y up)
    put_f32(d + 12, s.speed);   // m/s, total
    put_f32(d + 16, 300.0f * s.rpm * 6.28318f / 60.0f * s.gas);   // power W, a flat 300 Nm with the throttle
    put_f32(d + 20, 300.0f * s.gas);   // torque Nm
    for (int i = 0; i < 4; i++) put_f32(d + 24 + 4 * uint32_t(i), temp);
    put_f32(d + 40, 0);   // boost
    put_f32(d + 44, 0.5f);   // fuel
    put_f32(d + 48, float(s.odo));   // distance driven this race, metres
    // The times, seconds of the game's own clock as the HUD shows them: best, last and
    // current lap, then the race time - the stage clock, 0 at each stage start, laps summed
    put_f32(d + 52, float(s.best_lap)), put_f32(d + 56, float(s.last_lap)), put_f32(d + 60, float(s.lap_time));
    put_f32(d + 64, s.on ? float(std::clamp(s.race_time, 0.0, 3600.0)) : 0.0f);
    uint16_t lap_no = uint16_t(std::clamp(s.lap_no + 1, 1, 99));
    memcpy(d + 68, &lap_no, 2);
    d[70] = uint8_t(std::clamp(s.position + 1, 1, 255));   // race position, 1 = 1st
    d[71] = uint8_t(s.gas * 255), d[72] = uint8_t(s.brake * 255);
    d[73] = 0;   // clutch
    d[74] = s.hand ? 255 : 0;
    d[75] = uint8_t(std::clamp(s.gear, 0, 4));
    d[76] = int8_t(std::clamp(s.steer, -1.0f, 1.0f) * 127.0f);
    d[77] = 0x80, d[78] = 0x80;   // driving line, AI brake: none (read back as -1)
    d[79] = 0;   // the packet's last byte: padding
    return 324;
}

size_t build_json(const Sample& s, char* out, size_t cap) {
    snprintf(out, cap,
             "{\"frame\":%llu,\"on\":%d,\"speed\":%.2f,\"v_road\":%.2f,\"rpm\":%.0f,\"gear\":%d,"
             "\"steer\":%.3f,\"game_steer\":%.3f,\"gas\":%.3f,\"brake\":%.3f,\"hand\":%d,"
             "\"ffb\":%.3f,\"vib\":%.3f,\"shake\":%.3f,"
             "\"long_g\":%.3f,\"lat_g\":%.3f,\"vert_g\":%.3f,\"yaw\":%.4f,\"yaw_rate\":%.3f,\"roll\":%.4f,\"pitch\":%.4f,"
             "\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"odo\":%.1f,\"track_point\":%d,"
             "\"susp\":[%.3f,%.3f,%.3f,%.3f],"
             "\"pos\":%d,\"cars\":%d,\"lap\":%d,\"laps\":%d,\"laps_done\":%d,"
             "\"prog\":%d,\"sect\":%d,\"chk\":%d,\"round\":%d,\"mode\":%d,\"phase\":%d,\"seq\":%d,"
             "\"result\":%d,\"goal\":%d,\"clock\":%d,\"time_left\":%d,\"car\":%d,\"gears\":%d,"
             "\"lap_time\":%.2f,\"best_lap\":%.2f,\"last_lap\":%.2f,\"race_time\":%.2f,"
             "\"total_bcd\":%.3f,\"lap_bcd\":%.3f}\n",
             (unsigned long long)s.frame, s.on ? 1 : 0, s.speed, s.v_road, s.rpm, s.gear, s.steer,
             s.game_steer, s.gas, s.brake, s.hand ? 1 : 0, s.ffb, s.vib, s.shake, s.long_g, s.lat_g,
             s.vert_g, s.yaw, s.yaw_rate, s.roll, s.pitch, s.pos[0], s.pos[1], s.pos[2], s.odo, s.track_point, s.susp[0],
             s.susp[1], s.susp[2], s.susp[3], s.position + 1, s.cars, s.lap_no + 1, s.laps, s.laps_done,
             s.progress, s.section, s.checkpoints, s.round, s.mode, s.phase, s.sequence, s.result, s.goal,
             s.clock, s.left, s.car, s.gears, s.lap_time, s.best_lap, s.last_lap, s.race_time, s.total_bcd,
             s.lap_bcd);
    return strlen(out);
}

}  // namespace

void telemetry_frame(float ffb_force, float ffb_vibration) {
    static bool tried = false;
    if (!tried) {
        tried = true;
        const char* e = getenv("SR2_TELEMETRY");
        if (!e || !*e) return;
        char host[128];
        int port = 0;
        std::string spec(e);
        size_t colon = spec.find(':'), at = spec.find('@', colon == std::string::npos ? 0 : colon);
        std::string fmt = at == std::string::npos ? "json" : spec.substr(at + 1);
        if (at != std::string::npos) spec.resize(at);
        if (sscanf(spec.c_str(), "%127[^:]:%d", host, &port) != 2) {
            printf("[telemetry] SR2_TELEMETRY is \"%s\": want host:port[@json|@forza|@both]\n", e);
            return;
        }
        send_json = fmt == "json" || fmt == "both";
        send_forza = fmt == "forza" || fmt == "both";
        if (!send_json && !send_forza) {
            printf("[telemetry] unknown format \"%s\": json, forza or both\n", fmt.c_str());
            return;
        }
        if (!open_socket(host, port)) {
            printf("[telemetry] could not resolve %s or open a UDP socket (port %d)\n", host, port);
            return;
        }
        printf("[telemetry] sending %s to %s:%d at %.1f Hz\n", fmt.c_str(), host, port, FRAME_HZ);
    }
    if (sock == INVALID_SOCKET) return;
    static Sample was;
    Sample s = Sample::read(frame_count, ffb_force, ffb_vibration);
    s.derive(was);
    was = s;
    frame_count++;
    char buf[2048];
    if (send_json) {
        size_t n = build_json(s, buf, sizeof buf);
        sendto(sock, buf, int(n), 0, (sockaddr*)&dest, sizeof dest);
    }
    if (send_forza) {
        uint8_t pkt[324];
        build_forza(s, pkt);
        sendto(sock, (const char*)pkt, sizeof pkt, 0, (sockaddr*)&dest, sizeof dest);
    }
}
