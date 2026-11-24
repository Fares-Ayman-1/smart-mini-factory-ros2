"""Serial protocol shared with the Uno firmware (firmware/uno_factory/core.cpp).

Every line is  $<body>*<XX>\\n  where XX is the XOR of the characters of <body>
as two hex digits.

PC -> Uno   <seq>,HB                      heartbeat (every 200 ms)
            <seq>,BELT,<dmm>              belt speed in tenths of mm/s, 0..600
            <seq>,SRV,<us>x6              six servo pulse widths, microseconds
            <seq>,RELAY,<0|1>             motor power off / on
            <seq>,RST                     clear a fault (E-stop must be released)
            <seq>,LED,<0..255>            hood lighting
            <seq>,TWR,<0..15>             tower bits: 1 green, 2 amber, 4 red, 8 buzzer
Uno -> PC   <seq>,ACK  |  <seq>,NAK,<why>   one reply per command (seq 0 = bad line)
            E,IR1,<0|1>  E,IR2,<0|1>      sensor events
            T,<state>,<fault>,<flags>,<belt_dmm>,<mA>,<mV>   telemetry, 10 Hz
"""

from functools import reduce

STATES = ('IDLE', 'RUN', 'PARK', 'FAULT')
FAULTS = ('none', 'e-stop', 'heartbeat lost', 'servo overcurrent', '12 V brown-out')
FLAGS = ('ir1', 'ir2', 'estop_open', 'relay', 'start', 'stop', 'servos_on', 'hb_fresh')


def checksum(body: str) -> int:
    return reduce(lambda a, c: a ^ ord(c), body, 0)


def encode(seq: int, cmd: str, *args) -> bytes:
    body = ','.join([str(seq), cmd, *[str(a) for a in args]])
    return f'${body}*{checksum(body):02X}\n'.encode()


class Parser:
    """Collects bytes and yields complete, checksum-verified messages.

    Yields tuples:
      ('reply', seq, ok, why)        ('event', name, value)       ('telemetry', dict)
    Lines with a bad checksum or shape are counted in `bad` and dropped.
    """

    def __init__(self):
        self.buf = b''
        self.bad = 0

    def feed(self, data: bytes):
        self.buf += data
        if len(self.buf) > 4096:                  # no newline for ages: noise
            self.buf = self.buf[-256:]
            self.bad += 1
        out = []
        while b'\n' in self.buf:
            raw, self.buf = self.buf.split(b'\n', 1)
            msg = self._line(raw.decode('ascii', 'replace').strip('\r'))
            if msg:
                out.append(msg)
        return out

    def _line(self, s):
        if not s:
            return None
        i = s.rfind('$')                          # resynchronise on the last '$'
        star = s.rfind('*')
        if i < 0 or star < i or len(s) - star != 3:
            self.bad += 1
            return None
        body, crc = s[i + 1:star], s[star + 1:]
        try:
            ok = int(crc, 16) == checksum(body)
        except ValueError:
            ok = False
        if not ok:
            self.bad += 1
            return None
        f = body.split(',')
        try:
            if f[0] == 'T' and len(f) == 7:
                state, fault, flags, belt, ma, mv = (int(x) for x in f[1:])
                d = {'state': STATES[state], 'fault': FAULTS[fault], 'fault_code': fault,
                     'belt_dmm': belt, 'mA': ma, 'mV': mv}
                d.update({n: bool(flags >> k & 1) for k, n in enumerate(FLAGS)})
                return ('telemetry', d)
            if f[0] == 'E' and len(f) == 3:
                return ('event', f[1], int(f[2]))
            if len(f) >= 2 and f[1] in ('ACK', 'NAK'):
                return ('reply', int(f[0]), f[1] == 'ACK', f[2] if len(f) > 2 else '')
        except (ValueError, IndexError):
            pass
        self.bad += 1
        return None
