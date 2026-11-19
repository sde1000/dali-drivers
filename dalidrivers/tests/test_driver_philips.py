"""Offline tests for dalidrivers.misc.philips.

Includes report vectors from MultiOne captures and a fake HID device.
"""
import pytest

from dalidrivers.misc import philips as P
from dalidrivers.tests.fake_philips_hid import FakeHid

# (16-bit DALI frame, twice flag, sequence number, 5 Manchester bytes) from the capture
VECTORS = [
    (0x0191, 0, 0, "6aaa5a6a7f"), (0x01C2, 0, 5, "6aaa56a9bf"), (0xA300, 0, 0, "59a96aaabf"),
    (0xA500, 1, 7, "59a66aaabf"), (0xA700, 1, 9, "59a56aaabf"), (0xC332, 0, 8, "56a96969bf"),
    (0xFF23, 1, 4, "555569a97f"), (0xFF80, 1, 3, "55555aaabf"),
]


@pytest.mark.parametrize("value,twice,seq,tail", VECTORS)
def test_tx_report(value, twice, seq, tail):
    r = P.tx_report(seq, value, bool(twice))
    assert r[:7] == bytes([0xB2, seq, 0x70, 0x00, 0x00, 0x26, twice])
    assert r[7:15] == bytes.fromhex("983a000010270000")
    assert r[15:20].hex() == tail
    assert len(r) == 64


@pytest.mark.parametrize(
    "seq,on,h",
    [(7, 0, "810715"), (8, 1, "810815000001"),
     (9, 0, "810915"), (0, 1, "810015000001")],
)
def test_power_report(seq, on, h):
    assert P.power_report(seq, on).rstrip(b"\x00") == bytes.fromhex(h)


@pytest.fixture
def bus():
    fake = FakeHid()
    b = P.PhilipsUSB2DALI(bus_power=True, hid_module=fake)
    b.POWER_SETTLE = 0
    b.connect()
    b.fake = fake
    yield b
    b.close()


def test_startup(bus):
    assert (bus.name, bus.version) == ("USB2DALI", "1.2.0")


def test_scan(bus):
    assert bus.scan_present(range(8)) == [3]


def test_query(bus):
    assert bus.query(0x07, 0x91) == 0xFF
    assert bus.query(0x09, 0x91) is None


def test_twice_flag(bus):
    bus.send_forward(0xA500, twice=True)
    assert bus.fake.frames[-1] == (0xA500, 1)


def test_not_found():
    class Empty(FakeHid):
        def enumerate(self, v, p):
            return []
    with pytest.raises(P.NotFoundError):
        P.PhilipsUSB2DALI(hid_module=Empty()).connect()


def test_close_turns_off_owned_bus_power(bus):
    assert bus.fake.writes[-1][0] != 0x81 or bus.fake.writes[-1][5] == 1
    bus.close()
    assert bus.fake.writes[-1][:6] == bytes([0x81, bus.fake.writes[-1][1], 0x15, 0, 0, 0])


def test_requested_path_not_found():
    with pytest.raises(P.NotFoundError, match="path"):
        P.PhilipsUSB2DALI(path=b"missing", hid_module=FakeHid()).connect()


def test_send_without_connection():
    with pytest.raises(P.Usb2DaliError, match="not connected"):
        P.PhilipsUSB2DALI(hid_module=FakeHid()).send_forward(0x0191)
