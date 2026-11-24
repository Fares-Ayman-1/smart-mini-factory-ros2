"""Unit tests for the serial protocol parser."""

from factory_control.uno_protocol import Parser, checksum, encode


def line(body):
    return f'${body}*{checksum(body):02X}\n'.encode()


def test_encode_and_checksum():
    assert encode(7, 'HB') == b'$7,HB*' + f'{checksum("7,HB"):02X}'.encode() + b'\n'
    assert encode(1, 'SRV', 1500, 1500) == line('1,SRV,1500,1500')
    assert checksum('A') == 0x41 and checksum('AB') == 0x03


def test_replies_events_telemetry():
    p = Parser()
    out = p.feed(line('5,ACK') + line('6,NAK,STATE') + line('E,IR2,1') + line('T,1,0,137,300,120,12100'))
    assert out[0] == ('reply', 5, True, '')
    assert out[1] == ('reply', 6, False, 'STATE')
    assert out[2] == ('event', 'IR2', 1)
    t = out[3][1]
    assert t['state'] == 'RUN' and t['belt_dmm'] == 300 and t['mV'] == 12100
    assert t['ir1'] and t['relay'] and t['hb_fresh'] and not t['estop_open'] and not t['ir2']
    assert p.bad == 0


def test_split_across_reads_and_garbage():
    p = Parser()
    data = b'noise\x00' + line('1,ACK') + b'$2,AC' + b'junk*00\n' + line('3,ACK')
    got = []
    for i in range(0, len(data), 3):
        got += p.feed(data[i:i + 3])
    assert [m[1] for m in got] == [1, 3]
    assert p.bad == 1          # the damaged "$2,AC...junk" line; noise before a valid line is just skipped


def test_bad_checksum_and_shapes_are_dropped():
    p = Parser()
    assert p.feed(b'$1,ACK*00\n') == []
    assert p.feed(b'$1,ACK*0\n') == []
    assert p.feed(b'$1,ACK\n') == []
    assert p.feed(line('T,9,0,0,0,0,0')) == []       # state index out of range
    assert p.feed(line('E,IR1')) == []
    assert p.bad == 5


def test_runaway_input_without_newline_is_bounded():
    p = Parser()
    p.feed(b'x' * 10000)
    assert len(p.buf) <= 256
    assert p.feed(line('1,ACK')) == [('reply', 1, True, '')]
