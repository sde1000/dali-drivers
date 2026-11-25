"""Tests for the typed HID drivers' I/O backends (hidraw and hidapi)."""
import asyncio
import os
import queue
import sys
import threading

import pytest

from dali.gear.general import QueryActualLevel
from dali.address import GearShort

from dalidrivers.typed import hid as hidmod
from dalidrivers.typed.hid import tridonic, hasseb

TRIDONIC_PATH = b"\\\\?\\hid#vid_17b5&pid_0020#fake"


def _info(data: bytes) -> bytes:
    """A 64-byte Tridonic _MODE_INFO report with ``data`` after the mode byte."""
    return (bytes([tridonic._MODE_INFO]) + data).ljust(64, b"\0")


FIRMWARE_REPLY = _info(bytes([0x00, 0x00, 3, 7]))          # firmware 3.7
SERIAL_REPLY = _info(bytes([0x12, 0x34, 0x56, 0x78]))      # serial 12345678


class FakeTridonicHidapi:
    """Fake hidapi module modelling a Tridonic DALI-USB behind hidapi.

    Like real hidapi on Windows, write() requires a leading report ID byte
    (0 for these unnumbered reports) and read() returns the report without it.
    """

    def __init__(self, devices=(TRIDONIC_PATH,)):
        self.devices = list(devices)
        self.writes = []            # payloads as the device sees them
        self.reports = queue.Queue()
        self.opened = []
        self.closed = 0
        self.fail_open = False
        self.unplugged = threading.Event()
        self.read_threads = set()

    def enumerate(self, vid, pid):
        if (vid, pid) != (0x17B5, 0x0020):
            return []
        return [{"path": p, "serial_number": "12345678",
                 "product_string": "DALI-USB"} for p in self.devices]

    def device(self):
        return _FakeDev(self)

    def unplug(self):
        self.unplugged.set()


class _FakeDev:
    def __init__(self, mod):
        self.mod = mod
        self.open = False

    def open_path(self, path):
        if self.mod.fail_open or path not in self.mod.devices:
            raise OSError("open failed")
        self.open = True
        self.mod.opened.append(path)

    def close(self):
        assert threading.current_thread() not in self.mod.read_threads, \
            "closed from the reader thread"
        self.open = False
        self.mod.closed += 1

    def error(self):
        return "fake error"

    def write(self, buf):
        assert self.open
        if self.mod.unplugged.is_set():
            return -1
        assert buf[0] == 0, "missing report ID byte"
        payload = bytes(buf[1:])
        self.mod.writes.append(payload)
        if payload[0] == tridonic._CMD_INIT:
            self.mod.reports.put(
                FIRMWARE_REPLY if payload[1] == tridonic._CMD_INIT_READVERSION
                else SERIAL_REPLY)
        elif payload[0] == tridonic._CMD_SEND:
            seq = payload[1]
            # The interface echoes the forward frame, then reports the
            # backward frame 0x7f
            self.mod.reports.put(struct_response(
                seq, tridonic._RESPONSE_FRAME_DALI16, payload[4:8]))
            self.mod.reports.put(struct_response(
                seq, tridonic._RESPONSE_FRAME_DALI8, bytes([0, 0, 0, 0x7F])))
        return len(buf)

    def read(self, max_length, timeout_ms=0):
        self.mod.read_threads.add(threading.current_thread())
        if not self.open:
            raise ValueError("not open")
        if self.mod.unplugged.is_set():
            raise OSError("read error")
        try:
            data = self.mod.reports.get(timeout=timeout_ms / 1000)
        except queue.Empty:
            return []
        return list(data[:max_length])


def struct_response(seq, rtype, frame):
    return tridonic._resptmpl.pack(tridonic._MODE_RESPONSE, rtype, frame, 0, seq)


@pytest.mark.asyncio
async def test_hidapi_connect_handshake_and_query():
    fake = FakeTridonicHidapi()
    d = tridonic(backend="hidapi", hidapi_module=fake, reconnect_limit=0)
    try:
        await d.connect(timeout=2)
        assert d.is_connected
        assert fake.opened == [TRIDONIC_PATH]   # found by USB IDs
        assert d.firmware_version == "3.7"
        assert d.serial == "12345678"
        # The device receives the 64-byte command without the report ID
        assert [w[:2] for w in fake.writes] == [
            bytes([tridonic._CMD_INIT, tridonic._CMD_INIT_READVERSION]),
            bytes([tridonic._CMD_INIT, tridonic._CMD_INIT_READSERIAL])]
        assert all(len(w) == 64 for w in fake.writes)

        response = await d.send(QueryActualLevel(GearShort(1)))
        assert response.raw_value.as_integer == 0x7F
    finally:
        d.close()
    assert fake.closed == 1
    assert not d.is_connected


@pytest.mark.asyncio
async def test_hidapi_unplug_disconnects_and_reconnects():
    fake = FakeTridonicHidapi()
    d = tridonic(backend="hidapi", hidapi_module=fake,
                 reconnect_interval=0.01)
    status = []
    d.connection_status_callback.register(lambda _d, s: status.append(s))
    try:
        await d.connect(timeout=2)
        fake.unplug()
        for _ in range(200):
            if "disconnected" in status:
                break
            await asyncio.sleep(0.01)
        assert "disconnected" in status
        assert not d.is_connected
        assert fake.closed == 1

        # Plugging it back in lets the reconnect loop pick it up again
        fake.unplugged.clear()
        await asyncio.wait_for(d.connected.wait(), 2)
        assert len(fake.opened) == 2
    finally:
        d.close()


@pytest.mark.asyncio
async def test_hidapi_failed_write_reports_communication_error():
    from dali.exceptions import CommunicationError
    fake = FakeTridonicHidapi()
    d = tridonic(backend="hidapi", hidapi_module=fake, reconnect_limit=0)
    try:
        await d.connect(timeout=2)
        fake.unplugged.set()
        with pytest.raises(CommunicationError):
            await d.send(QueryActualLevel(GearShort(1)))
    finally:
        d.close()


@pytest.mark.asyncio
async def test_hidapi_explicit_path_and_glob():
    other = b"\\\\?\\hid#vid_17b5&pid_0020#other"
    fake = FakeTridonicHidapi(devices=(TRIDONIC_PATH, other))
    d = tridonic(other.decode(), backend="hidapi", hidapi_module=fake)
    try:
        await d.connect(timeout=2)
        assert fake.opened == [other]
    finally:
        d.close()

    fake.opened.clear()
    d = tridonic("*#other", glob=True, backend="hidapi", hidapi_module=fake)
    try:
        await d.connect(timeout=2)
        assert fake.opened == [other]
    finally:
        d.close()

    assert [x["path"] for x in tridonic.enumerate(fake)] == [
        TRIDONIC_PATH, other]


@pytest.mark.asyncio
async def test_hidapi_no_device_fails_fast():
    fake = FakeTridonicHidapi(devices=())
    d = tridonic(backend="hidapi", hidapi_module=fake)
    with pytest.raises(FileNotFoundError):
        await d.connect(timeout=2)
    assert d._reconnect_task is None

    fake = FakeTridonicHidapi()
    fake.fail_open = True
    d = tridonic(backend="hidapi", hidapi_module=fake)
    with pytest.raises(OSError, match="open failed"):
        await d.connect(timeout=2)


@pytest.mark.asyncio
async def test_hidapi_missing_package(monkeypatch):
    def missing():
        raise ImportError("no hidapi")
    monkeypatch.setattr(hidmod, "_import_hidapi", missing)
    d = tridonic(backend="hidapi")
    with pytest.raises(ImportError):
        await d.connect(timeout=2)


def test_backend_selection(monkeypatch):
    monkeypatch.setattr(sys, "platform", "linux")
    assert tridonic("/dev/hidraw0").backend == "hidraw"
    with pytest.raises(ValueError):
        tridonic()      # hidraw needs a path
    monkeypatch.setattr(sys, "platform", "win32")
    assert tridonic().backend == "hidapi"
    monkeypatch.setattr(sys, "platform", "darwin")
    assert tridonic().backend == "hidapi"
    assert tridonic("/dev/hidraw0", backend="hidraw").backend == "hidraw"
    with pytest.raises(ValueError):
        tridonic("x", backend="usb")   # type: ignore[arg-type]
    assert (hasseb.usb_vendor_id, hasseb.usb_product_id) == (0x04CC, 0x0802)


@pytest.mark.skipif(not hasattr(os, "openpty"), reason="needs a pty")
@pytest.mark.asyncio
async def test_hidraw_backend_over_pty():
    """The hidraw backend still reads/writes the device node directly."""
    import tty
    master, slave = os.openpty()
    tty.setraw(slave)
    path = os.ttyname(slave)
    d = tridonic(path, backend="hidraw", reconnect_limit=0)
    loop = asyncio.get_running_loop()
    received = bytearray()
    got_cmd = asyncio.Event()

    def on_master():
        received.extend(os.read(master, 4096))
        got_cmd.set()

    loop.add_reader(master, on_master)
    try:
        connect = asyncio.create_task(d.connect(timeout=2))
        for reply in (FIRMWARE_REPLY, SERIAL_REPLY):
            await asyncio.wait_for(got_cmd.wait(), 2)
            got_cmd.clear()
            os.write(master, reply)
        await connect
        assert d.firmware_version == "3.7" and d.serial == "12345678"
        # No report ID byte is added on the hidraw path
        assert received[0] == tridonic._CMD_INIT
    finally:
        loop.remove_reader(master)
        d.close()
        os.close(master)
        os.close(slave)
