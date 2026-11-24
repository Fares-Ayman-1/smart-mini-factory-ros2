#include "core.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const uint16_t REST[6] = REST_POSE;

uint8_t checksum(const char *s) { uint8_t c = 0; while (*s) c ^= (uint8_t)*s++; return c; }

void Core::begin(uint32_t ms, Send send) {
  memset(&out, 0, sizeof out); memset(&in_, 0, sizeof in_);
  send_ = send; state = IDLE; fault = F_NONE; len = 0; in_line = false;
  last_ms = tel_ms = ms; hb_ms = fault_ms = park_ms = oc_ms = bo_ms = 0;
  hb_seen = oc_on = bo_on = false; belt_cmd = 0; belt_milli = 0; tower = 0;
  ir[0] = ir[1] = raw[0] = raw[1] = false; raw_ms[0] = raw_ms[1] = ms;
  in_.estop_open = true;                 // unsafe until the first reading
}

void Core::emit(const char *body) {
  char b[80]; snprintf(b, sizeof b, "$%s*%02X\n", body, checksum(body)); send_(b);
}
void Core::reply(unsigned seq, const char *what, const char *why) {
  char b[32]; snprintf(b, sizeof b, why ? "%u,%s,%s" : "%u,%s", seq, what, why); emit(b);
}

// A fault always stops the belt at once. Servos hold their last position.
void Core::trip(Fault f, uint32_t ms, bool drop_relay) {
  state = FAULT; fault = f; fault_ms = ms; belt_cmd = 0; belt_milli = 0; out.belt = 0;
  if (drop_relay) out.relay = false;
}

void Core::rx(char c, uint32_t ms) {
  if (c == '$') { in_line = true; len = 0; return; }
  if (!in_line) return;
  if (c == '\n' || c == '\r') { in_line = false; buf[len] = 0; line(buf, ms); return; }
  if (len >= sizeof buf - 1) { in_line = false; reply(0, "NAK", "LEN"); return; }
  buf[len++] = c;
}

// Parse "seq,CMD,arg,arg" into n integers; returns false on anything odd.
static bool ints(char *s, long *v, uint8_t want) {
  uint8_t n = 0;
  while (s && *s) {
    char *e; long x = strtol(s, &e, 10);
    if (e == s || n >= want) return false;
    v[n++] = x;
    if (*e == 0) break;
    if (*e != ',' || e[1] == 0) return false;
    s = e + 1;
  }
  return n == want;
}

void Core::line(char *s, uint32_t ms) {
  char *star = strchr(s, '*');
  if (!star || strlen(star) != 3) { reply(0, "NAK", "CRC"); return; }
  *star = 0;
  char *e; long crc = strtol(star + 1, &e, 16);
  if (*e || crc != checksum(s)) { reply(0, "NAK", "CRC"); return; }
  long sq = strtol(s, &e, 10);
  if (e == s || *e != ',' || sq < 0 || sq > 65535L) { reply(0, "NAK", "SEQ"); return; }
  unsigned seq = (unsigned)sq;
  char *cmd = e + 1, *arg = strchr(cmd, ',');
  if (arg) *arg++ = 0;
  long v[6];

  if (!strcmp(cmd, "HB")) { hb_seen = true; hb_ms = ms; reply(seq, "ACK"); return; }
  if (!strcmp(cmd, "BELT")) {
    if (!ints(arg, v, 1) || v[0] < 0 || v[0] > BELT_MAX_DMM) { reply(seq, "NAK", "ARG"); return; }
    if (v[0] && state != RUN) { reply(seq, "NAK", "STATE"); return; }
    belt_cmd = (int16_t)v[0]; reply(seq, "ACK"); return;
  }
  if (!strcmp(cmd, "SRV")) {
    if (!ints(arg, v, 6)) { reply(seq, "NAK", "ARG"); return; }
    for (uint8_t i = 0; i < 6; i++)
      if (v[i] < SERVO_MIN_US || v[i] > SERVO_MAX_US) { reply(seq, "NAK", "ARG"); return; }
    if (state != IDLE && state != RUN) { reply(seq, "NAK", "STATE"); return; }
    for (uint8_t i = 0; i < 6; i++) out.srv[i] = (uint16_t)v[i];
    out.servos_on = true; reply(seq, "ACK"); return;
  }
  if (!strcmp(cmd, "RELAY")) {
    if (!ints(arg, v, 1) || v[0] < 0 || v[0] > 1) { reply(seq, "NAK", "ARG"); return; }
    if (!v[0]) {                          // asking for less power is always allowed
      out.relay = false; belt_cmd = 0; belt_milli = 0; out.belt = 0;
      if (state == RUN) state = IDLE;
      else if (state == PARK) trip(F_HEARTBEAT, ms, true);
      reply(seq, "ACK"); return;
    }
    if (state == RUN) { reply(seq, "ACK"); return; }
    if (state != IDLE) { reply(seq, "NAK", "STATE"); return; }
    if (in_.estop_open) { reply(seq, "NAK", "ESTOP"); return; }
    if (!fresh(ms)) { reply(seq, "NAK", "HB"); return; }
    if (!out.servos_on) { reply(seq, "NAK", "POSE"); return; }   // home pulses first
    if (in_.analog_ok && in_.mV < BROWNOUT_MV) { reply(seq, "NAK", "VOLT"); return; }
    out.relay = true; state = RUN; oc_on = bo_on = false; reply(seq, "ACK"); return;
  }
  if (!strcmp(cmd, "RST")) {
    if (arg) { reply(seq, "NAK", "ARG"); return; }
    if (state == FAULT) {
      if (in_.estop_open) { reply(seq, "NAK", "ESTOP"); return; }
      if (out.relay && !fresh(ms)) { reply(seq, "NAK", "HB"); return; }
      state = out.relay ? RUN : IDLE; fault = F_NONE; oc_on = bo_on = false;
    } else if (state == PARK) { reply(seq, "NAK", "STATE"); return; }
    reply(seq, "ACK"); return;
  }
  if (!strcmp(cmd, "LED")) {
    if (!ints(arg, v, 1) || v[0] < 0 || v[0] > 255) { reply(seq, "NAK", "ARG"); return; }
    out.led = (uint8_t)v[0]; reply(seq, "ACK"); return;
  }
  if (!strcmp(cmd, "TWR")) {              // bits: 1 green, 2 amber, 4 red, 8 buzzer
    if (!ints(arg, v, 1) || v[0] < 0 || v[0] > 15) { reply(seq, "NAK", "ARG"); return; }
    tower = (uint8_t)v[0]; reply(seq, "ACK"); return;
  }
  reply(seq, "NAK", "CMD");
}

void Core::tick(uint32_t ms, const Inputs &in) {
  uint32_t dt = ms - last_ms; last_ms = ms; in_ = in;

  // The E-stop wins over every other state. It also cuts the coil in hardware.
  if (in.estop_open && !(state == FAULT && fault == F_ESTOP)) trip(F_ESTOP, ms, true);

  if (state == RUN && !fresh(ms)) {
    state = PARK; park_ms = ms; belt_cmd = 0;
    for (uint8_t i = 0; i < 6; i++) park_from[i] = out.srv[i];
  }
  if (in.analog_ok && (state == RUN || state == PARK)) {
    if (in.mA > OVERCURRENT_MA) {
      if (!oc_on) { oc_on = true; oc_ms = ms; }
      else if (ms - oc_ms >= OVERCURRENT_MS) { out.srv[GRIPPER] = GRIPPER_OPEN_US; trip(F_OVERCURRENT, ms, false); }
    } else oc_on = false;
  }
  if (in.analog_ok && (state == RUN || state == PARK)) {
    if (in.mV < BROWNOUT_MV) {
      if (!bo_on) { bo_on = true; bo_ms = ms; }
      else if (ms - bo_ms >= BROWNOUT_MS) trip(F_BROWNOUT, ms, false);
    } else bo_on = false;
  }
  // Opening the gripper did not clear it (a short, say): cut the power.
  if (state == FAULT && fault == F_OVERCURRENT && out.relay && ms - fault_ms >= OVERCURRENT_CUT_MS
      && in.analog_ok && in.mA > OVERCURRENT_MA) out.relay = false;
  // Nobody is listening any more while a fault holds the relay closed.
  if (state == FAULT && out.relay && !fresh(ms)) out.relay = false;

  if (state == PARK) {
    uint32_t t = ms - park_ms;
    if (t >= PARK_MS) {
      for (uint8_t i = 0; i < 6; i++) out.srv[i] = REST[i];
      trip(F_HEARTBEAT, ms, true);
    } else
      for (uint8_t i = 0; i < 6; i++)
        out.srv[i] = (uint16_t)(park_from[i] + ((int32_t)REST[i] - (int32_t)park_from[i]) * (int32_t)t / PARK_MS);
  }

  if (state == RUN || state == PARK) {
    if (dt > 100) dt = 100;
    int32_t step = (int32_t)BELT_ACCEL_DMM_S * (int32_t)dt, target = (int32_t)belt_cmd * 1000;
    if (belt_milli < target) { belt_milli += step; if (belt_milli > target) belt_milli = target; }
    else if (belt_milli > target) { belt_milli -= step; if (belt_milli < target) belt_milli = target; }
  } else belt_milli = 0;
  out.belt = (int16_t)(belt_milli / 1000);

  const bool now[2] = {in.ir1, in.ir2};
  for (uint8_t i = 0; i < 2; i++) {
    if (now[i] != raw[i]) { raw[i] = now[i]; raw_ms[i] = ms; }
    else if (raw[i] != ir[i] && ms - raw_ms[i] >= DEBOUNCE_MS) {
      ir[i] = raw[i]; char b[12]; snprintf(b, sizeof b, "E,IR%u,%u", i + 1u, (unsigned)ir[i]); emit(b);
    }
  }

  if (state == FAULT || state == PARK) {
    out.r = fault == F_ESTOP && state == FAULT; out.y = !out.r; out.g = false; out.buzzer = false;
  } else { out.g = tower & 1; out.y = tower & 2; out.r = tower & 4; out.buzzer = tower & 8; }

  if (ms - tel_ms >= TELEMETRY_MS) {
    tel_ms = ms;
    unsigned fl = (ir[0] ? 1 : 0) | (ir[1] ? 2 : 0) | (in.estop_open ? 4 : 0) | (out.relay ? 8 : 0) |
                  (in.start ? 16 : 0) | (in.stop ? 32 : 0) | (out.servos_on ? 64 : 0) | (fresh(ms) ? 128 : 0);
    char b[48]; snprintf(b, sizeof b, "T,%u,%u,%u,%d,%u,%u", (unsigned)state, (unsigned)fault, fl,
                         (int)out.belt, (unsigned)in.mA, (unsigned)in.mV);
    emit(b);
  }
}
