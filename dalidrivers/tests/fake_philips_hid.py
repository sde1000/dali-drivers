"""Fake `hid` module that models the Philips USB to DALI protocol seen in the captures.
One DALI control gear on short address 3 with a memory bank 0 UID.
Models the 1 kHz idle stream, the stale first poll answer and the two-poll latency."""
import collections
import time

UID = bytes.fromhex("5752136544FFFFFF")


def _decode(tail):
    s = "".join("{:08b}".format(b) for b in tail)
    bits = []
    for i in range(0, len(s), 2):
        pr = s[i:i + 2]
        if pr == "01":
            bits.append(1)
        elif pr == "10":
            bits.append(0)
        else:
            break
    assert bits[0] == 1 and len(bits) == 17, bits
    v = 0
    for b in bits[1:]:
        v = (v << 1) | b
    return v


class FakeHid:
    def __init__(self, gear_short=3):
        self.gear = gear_short
        self.q = collections.deque()
        self.last = bytes(64)
        self.dtr0 = self.dtr1 = 0
        self.pending = (2, 0)
        self.polls = 0
        self.frames = []        # (value, twice flag)
        self.writes = []        # raw 64-byte payloads
        self.opened = False

    # module API
    def enumerate(self, vid, pid):
        return [{"path": b"fake", "manufacturer_string": "Philips Lighting",
                 "product_string": "Philips USB to DALI",
                 "serial_number": "TEST"}] if (vid, pid) == (0x0471, 3) else []

    def device(self):
        return self._Dev(self)

    class _Dev:
        def __init__(self, o):
            self.o = o

        def open_path(self, p):
            self.o.opened = True

        def close(self):
            self.o.opened = False

        def error(self):
            return "fake"

        def write(self, buf):
            o = self.o
            b = bytes(buf)
            assert b[0] == 0 and len(b) == 65, "needs report id 0 + 64 bytes"
            p = b[1:]
            o.writes.append(p)
            if p[0] == 0xB2:
                v = _decode(p[15:20])
                a, c = v >> 8, v & 0xFF
                o.frames.append((v, p[6]))
                status, data = 2, 0
                if a == 0xA3:
                    o.dtr0 = c
                elif a == 0xC3:
                    o.dtr1 = c
                elif a & 1 and (a >> 1) == o.gear and a < 0x80:
                    if c == 0x91:
                        status, data = 1, 0xFF
                    elif c == 0xC5:
                        status = 1
                        data = (UID[o.dtr0 - 0x0B]
                                if o.dtr1 == 0 and 0x0B <= o.dtr0 <= 0x12 else 0)
                o.pending, o.polls = (status, data), 0
            elif p[0] == 0x02:
                o.polls += 1
                status, data = (0, o.last[5]) if o.polls == 1 else o.pending
                rep = bytes([p[1], 0x70, 0, 1, status, data]) + b"2.0\x00LI"
                o.q.append(rep.ljust(64, b"\x00"))
                o.last = rep
            elif p[0] == 0x14:
                body = b"1.2.0" if (p[3] == 0x00 and p[4] == 0x01) else b"USB2DALI"
                o.q.append((bytes([p[1], 0x10, 0, 0]) + body).ljust(64, b"\x00"))
            return 65

        def read(self, n, timeout_ms=0):
            o = self.o
            if o.q:
                return list(o.q.popleft())
            if timeout_ms == 0:
                return []
            time.sleep(0.001)
            return list(b"\xff" + o.last[1:])
