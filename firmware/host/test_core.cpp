// PC tests of the same core.cpp that runs on the Uno.
#include "../uno_factory/core.h"
#include <cstdio>
#include <cstring>
#include <string>
static std::string tx; static void snd(const char *s) { tx += s; }
static int n = 0, bad = 0;
#define CHECK(x) do { n++; if (!(x)) { bad++; printf("FAIL line %d: %s\n", __LINE__, #x); } } while (0)
struct Rig {
  Core c; Inputs in{}; uint32_t t = 1000; unsigned seq = 1; bool hb = true;
  Rig() { tx.clear(); in.analog_ok = true; in.mV = 12000; c.begin(t, snd); run(20); }
  void run(uint32_t ms) { while (ms--) { t++; if (hb && t % 200 == 0) raw("HB"); c.tick(t, in); } }
  void raw(const char *b) { char a[90], l[100]; snprintf(a, 90, "%u,%s", seq++ % 65536, b); snprintf(l, 100, "$%s*%02X\n", a, checksum(a)); for (char *p = l; *p; p++) c.rx(*p, t); }
  bool cmd(const char *b, const char *expect = "ACK") { tx.clear(); unsigned s = seq; raw(b); char w[40]; snprintf(w, 40, "$%u,%s", s, expect); return tx.find(w) != std::string::npos; }
  void up() { run(250); CHECK(cmd("SRV,1500,1500,1500,1500,1500,1500")); CHECK(cmd("RELAY,1")); CHECK(c.state == RUN && c.out.relay); }
};
int main() {
  { Rig r; CHECK(!r.c.out.relay && !r.c.out.servos_on); }
  { Rig r; r.run(250); CHECK(r.cmd("RELAY,1", "NAK,POSE")); CHECK(!r.c.out.relay); }                 // home pulses first
  { Rig r; r.hb = false; r.run(250); r.cmd("SRV,1500,1500,1500,1500,1500,1500"); CHECK(r.cmd("RELAY,1", "NAK,HB")); }
  { Rig r; r.run(250); r.cmd("SRV,1500,1500,1500,1500,1500,1500"); r.in.mV = 9000; r.run(5); CHECK(r.cmd("RELAY,1", "NAK,VOLT")); }
  { Core c; Inputs i{}; tx.clear(); c.begin(0, snd); const char *b = "1,RELAY,1"; char l[40]; snprintf(l, 40, "$%s*%02X\n", b, checksum(b)); for (char *p = l; *p; p++) c.rx(*p, 1); CHECK(!c.out.relay); }
  { Rig r; CHECK(r.cmd("BELT,300", "NAK,STATE")); r.up(); CHECK(r.cmd("BELT,300")); r.run(50); CHECK(r.c.out.belt == 100); r.run(150); CHECK(r.c.out.belt == 300);
    CHECK(r.cmd("BELT,0")); r.run(160); CHECK(r.c.out.belt == 0); CHECK(r.cmd("BELT,601", "NAK,ARG")); CHECK(r.cmd("BELT,-1", "NAK,ARG")); }
  { Rig r; r.up(); r.cmd("BELT,300"); r.run(300); r.in.estop_open = true; r.run(1);                  // E-stop: same tick
    CHECK(!r.c.out.relay && r.c.out.belt == 0 && r.c.state == FAULT && r.c.fault == F_ESTOP && r.c.out.r && !r.c.out.g);
    CHECK(r.cmd("RELAY,1", "NAK,STATE")); CHECK(r.cmd("RST", "NAK,ESTOP")); r.in.estop_open = false; r.run(500);
    CHECK(r.c.state == FAULT && !r.c.out.relay); CHECK(r.cmd("RST")); r.run(50); CHECK(r.c.state == IDLE && !r.c.out.relay); CHECK(r.cmd("RELAY,1")); CHECK(r.c.out.relay); }
  { Rig r; r.up(); r.cmd("BELT,300"); r.run(300); r.hb = false; r.run(1700); CHECK(r.c.state == RUN); r.run(600); CHECK(r.c.state == PARK && r.c.out.relay && r.c.out.belt == 0 && r.c.out.y);
    CHECK(r.cmd("SRV,1000,1000,1000,1000,1000,1000", "NAK,STATE")); CHECK(r.cmd("RELAY,1", "NAK,STATE"));
    r.run(3200); CHECK(r.c.state == FAULT && r.c.fault == F_HEARTBEAT && !r.c.out.relay && r.c.out.srv[0] == 1352); }   // PDF: park, then relay off
  { Rig r; r.up(); r.hb = false; r.run(2600); CHECK(r.c.state == PARK); r.in.estop_open = true; r.run(1); CHECK(!r.c.out.relay && r.c.fault == F_ESTOP); }
  { Rig r; r.up(); r.hb = false; r.run(2600); r.hb = true; r.run(4000); CHECK(r.c.state == FAULT && !r.c.out.relay); CHECK(r.cmd("RST")); CHECK(r.cmd("RELAY,1")); }
  { Rig r; r.up(); r.in.mA = 7000; r.run(200); r.in.mA = 1000; r.run(50); CHECK(r.c.state == RUN); r.in.mA = 7000; r.run(310);
    CHECK(r.c.fault == F_OVERCURRENT && r.c.out.srv[5] == 2000 && r.c.out.relay && r.c.out.belt == 0); r.in.mA = 500; r.run(1500); CHECK(r.c.out.relay); CHECK(r.cmd("RST")); CHECK(r.c.state == RUN); }
  { Rig r; r.up(); r.in.mA = 9000; r.run(1400); CHECK(!r.c.out.relay); }                              // short: cut
  { Rig r; r.up(); r.in.mV = 10000; r.run(100); r.in.mV = 12000; r.run(20); CHECK(r.c.state == RUN); r.in.mV = 10000; r.run(210); CHECK(r.c.fault == F_BROWNOUT && r.c.out.relay); r.hb = false; r.run(2300); CHECK(!r.c.out.relay); }
  { Rig r; r.in.analog_ok = false; r.in.mV = 0; r.in.mA = 60000; r.up(); r.run(2000); CHECK(r.c.state == RUN); }   // bench mode
  { Rig r; r.up(); CHECK(r.cmd("RELAY,0")); CHECK(r.c.state == IDLE && !r.c.out.relay); }
  { Rig r; tx.clear(); const char *l = "$5,RELAY,1*00\n"; for (const char *p = l; *p; p++) r.c.rx(*p, r.t); CHECK(tx.find("0,NAK,CRC") != std::string::npos);
    tx.clear(); for (int i = 0; i < 500; i++) r.c.rx(i ? 'x' : '$', r.t); CHECK(tx.find("NAK,LEN") != std::string::npos);
    CHECK(r.cmd("HB")); CHECK(r.cmd("NOPE", "NAK,CMD")); CHECK(r.cmd("SRV,1500", "NAK,ARG")); CHECK(r.cmd("SRV,1,1,1,1,1,1", "NAK,ARG")); CHECK(r.cmd("SRV,1500,1500,1500,1500,1500,1500,1500", "NAK,ARG")); CHECK(r.cmd("LED,256", "NAK,ARG")); CHECK(r.cmd("RELAY,x", "NAK,ARG")); CHECK(r.cmd("BELT", "NAK,ARG")); }
  { Rig r; tx.clear(); r.in.ir1 = true; r.run(5); r.in.ir1 = false; r.run(30); CHECK(tx.find("E,IR1") == std::string::npos); r.in.ir1 = true; r.run(30); CHECK(tx.find("$E,IR1,1*") != std::string::npos);
    size_t f = tx.find("E,IR1,1"); CHECK(tx.find("E,IR1,1", f + 1) == std::string::npos); }
  { Rig r; tx.clear(); r.run(1000); int k = 0; for (size_t p = 0; (p = tx.find("$T,", p)) != std::string::npos; p++) k++; CHECK(k == 10); }
  { Core c; Inputs i{}; i.analog_ok = true; i.mV = 12000; c.begin(0xFFFFFF00u, snd); uint32_t t = 0xFFFFFF00u; for (int k = 0; k < 6000; k++) { t++; c.tick(t, i); } CHECK(c.state == IDLE); }   // millis rollover
  // random bytes and inputs: the relay must never be closed with the E-stop open
  { unsigned s = 1; auto rnd = [&] { s = s * 1103515245u + 12345u; return (s >> 16) & 0x7fff; }; Rig r; const char *cs[] = {"HB","RELAY,1","RELAY,0","RST","BELT,300","SRV,1500,1500,1500,1500,1500,1500"}; int viol = 0, closed = 0; r.hb = false;
    for (int k = 0; k < 300000; k++) { int q = rnd() % 40; if (q < 6) r.raw(cs[q]); else if (q == 6) r.c.rx((char)rnd(), r.t); if (rnd() % 400 == 0) r.in.estop_open ^= 1; if (rnd() % 900 == 0) r.in.mA = rnd() % 9000; if (rnd() % 900 == 0) r.in.mV = 9000 + rnd() % 4000; r.t += 1 + rnd() % 20; r.c.tick(r.t, r.in); if (r.in.estop_open && (r.c.out.relay || r.c.out.belt)) viol++; if (!r.c.out.relay && r.c.out.belt) viol++; closed += r.c.out.relay; tx.clear(); }
    CHECK(viol == 0); CHECK(closed > 1000); }
  printf("%d checks, %d failed\n", n, bad); return bad != 0;
}
