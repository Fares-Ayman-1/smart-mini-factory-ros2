// Simulated Uno: the real core.cpp behind a virtual serial port.
// Prints the port path, then takes commands on stdin: estop|ir1|ir2|start|stop 0/1,
// ma N, mv N, mute 0/1 (stop answering), quit
#include "../uno_factory/core.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
static int fd; static bool mute = false;
static void snd(const char *s) { if (!mute) { ssize_t r = write(fd, s, strlen(s)); (void)r; } }
static uint32_t ms() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint32_t)(t.tv_sec * 1000ull + t.tv_nsec / 1000000); }
int main() {
  fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK); grantpt(fd); unlockpt(fd);
  int keep = open(ptsname(fd), O_RDWR | O_NOCTTY); termios t; tcgetattr(keep, &t); cfmakeraw(&t); tcsetattr(keep, TCSANOW, &t);
  printf("%s\n", ptsname(fd)); fflush(stdout);
  Core c; Inputs in{}; in.analog_ok = true; in.mV = 12000; c.begin(ms(), snd); int last = -1;
  for (;;) {
    pollfd p[2] = {{fd, POLLIN, 0}, {0, POLLIN, 0}}; poll(p, 2, 1);
    char b[256]; ssize_t n;
    if (p[0].revents & POLLIN) { n = read(fd, b, sizeof b); for (ssize_t i = 0; i < n; i++) if (!mute) c.rx(b[i], ms()); }
    if (p[1].revents & (POLLIN | POLLHUP)) {
      n = read(0, b, sizeof b - 1); if (n <= 0) return 0; b[n] = 0;
      for (char *l = strtok(b, "\n"); l; l = strtok(0, "\n")) { char k[16]; int v = 0; sscanf(l, "%15s %d", k, &v);
        if (!strcmp(k, "estop")) in.estop_open = v; else if (!strcmp(k, "ir1")) in.ir1 = v; else if (!strcmp(k, "ir2")) in.ir2 = v;
        else if (!strcmp(k, "start")) in.start = v; else if (!strcmp(k, "stop")) in.stop = v; else if (!strcmp(k, "ma")) in.mA = v;
        else if (!strcmp(k, "mv")) in.mV = v; else if (!strcmp(k, "mute")) mute = v; else if (!strcmp(k, "quit")) return 0; }
    }
    c.tick(ms(), in);
    int st = c.state * 1000 + c.fault * 100 + c.out.relay * 10;
    if (st != last) { last = st; printf("state %d fault %d relay %d srv0 %u\n", c.state, c.fault, c.out.relay, c.out.srv[0]); fflush(stdout); }
  }
}
