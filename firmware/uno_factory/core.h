// Protocol and safety logic. No Arduino calls: the same code runs on the Uno
// and in the PC tests.
#pragma once
#include <stdint.h>
#include "config.h"
#define HB_LIMIT HEARTBEAT_MS
enum State : uint8_t { IDLE, RUN, PARK, FAULT };
enum Fault : uint8_t { F_NONE, F_ESTOP, F_HEARTBEAT, F_OVERCURRENT, F_BROWNOUT };

struct Inputs { bool estop_open, ir1, ir2, start, stop; uint16_t mA, mV; bool analog_ok; };
struct Outputs { bool relay, buzzer, g, y, r, servos_on; int16_t belt; uint16_t srv[6]; uint8_t led; };

class Core {
 public:
  typedef void (*Send)(const char *line);
  void begin(uint32_t ms, Send send);
  void rx(char c, uint32_t ms);
  void tick(uint32_t ms, const Inputs &in);
  Outputs out;
  State state;
  Fault fault;
 private:
  void line(char *s, uint32_t ms);
  void reply(unsigned seq, const char *what, const char *why = 0);
  void emit(const char *body);
  void trip(Fault f, uint32_t ms, bool drop_relay);
  bool fresh(uint32_t ms) const { return hb_seen && ms - hb_ms <= HB_LIMIT; }
  Send send_;
  char buf[72]; uint8_t len; bool in_line;
  uint32_t last_ms, hb_ms, fault_ms, park_ms, oc_ms, bo_ms, tel_ms;
  bool hb_seen, oc_on, bo_on;
  int16_t belt_cmd; int32_t belt_milli; uint16_t park_from[6]; uint8_t tower;
  Inputs in_; bool ir[2], raw[2]; uint32_t raw_ms[2];
};
uint8_t checksum(const char *s);
