"""Tests for the typed HID drivers' I/O backends (hidraw and hidapi).

Also covers the parts of the Tridonic DALI-USB driver that do not need a
device: frame mode selection, command packing, and the bus watch decode.
"""
import asyncio
import os
import queue
import sys
import threading
import unittest

import pytest

from dali import address
from dali import firmware
from dali import frame as dali_frame
from dali.exceptions import UnsupportedFrameTypeError
from dali.gear.general import DAPC, QueryActualLevel
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


def test_hidapi_reader_stops_when_loop_closed_without_disconnect():
    # A program that crashes (or forgets to disconnect) lets asyncio.run()
    # close the loop while the reader thread is still running.  The thread
    # must then stop quietly instead of raising "Event loop is closed".
    fake = FakeTridonicHidapi()
    d = tridonic(backend="hidapi", hidapi_module=fake, reconnect_limit=0)
    errors = []
    old_hook = threading.excepthook
    threading.excepthook = errors.append
    try:
        loop = asyncio.new_event_loop()
        loop.run_until_complete(d.connect(timeout=2))
        reader = d._transport._thread
        # Shut down the loop the way asyncio.run() does
        tasks = asyncio.all_tasks(loop)
        for task in tasks:
            task.cancel()
        loop.run_until_complete(
            asyncio.gather(*tasks, return_exceptions=True))
        loop.close()
        # Bus traffic arriving after the loop has gone
        fake.reports.put(struct_response(
            1, tridonic._RESPONSE_FRAME_DALI16, bytes(4)))
        reader.join(timeout=2)
        assert not reader.is_alive()
    finally:
        threading.excepthook = old_hook
    assert errors == []


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


# 32-bit frame support (IEC 62386-105 firmware transfer)


def _response(rtype, raw, *, origin=tridonic._MODE_OBSERVE, interval=0, seq=0):
    """Build a 64-byte report as the device would send it."""
    return tridonic._resptmpl.pack(origin, rtype, raw, interval, seq)


class TestCommandMode(unittest.TestCase):
    def test_modes(self):
        self.assertEqual(tridonic._command_mode(dali_frame.Frame(8)),
                         tridonic._SEND_MODE_DALI8)
        self.assertEqual(tridonic._command_mode(dali_frame.Frame(16)),
                         tridonic._SEND_MODE_DALI16)
        self.assertEqual(tridonic._command_mode(dali_frame.Frame(24)),
                         tridonic._SEND_MODE_DALI24)
        self.assertEqual(tridonic._command_mode(dali_frame.Frame(32)),
                         tridonic._SEND_MODE_DALI32)

    def test_dali32_mode_value(self):
        """the 32-bit send mode is the value Tridonic's driver uses"""
        self.assertEqual(tridonic._SEND_MODE_DALI32, 0x0E)

    def test_unsupported(self):
        for bits in (1, 17, 25, 33, 64):
            self.assertRaises(UnsupportedFrameTypeError,
                              tridonic._command_mode, dali_frame.Frame(bits))


class TestCommandPacking(unittest.TestCase):
    def test_dali32_frame_is_packed_big_endian(self):
        """a 32-bit frame fills all four frame bytes of the report"""
        cmd = firmware.StartFirmwareTransfer(address.FirmwareGearBroadcast())
        data = tridonic._cmd(
            tridonic._CMD_SEND, 0x42,
            mode=tridonic._command_mode(cmd.frame),
            frame=cmd.frame.pack_len(4))
        self.assertEqual(len(data), 64)
        self.assertEqual(data[0], tridonic._CMD_SEND)
        self.assertEqual(data[1], 0x42)
        self.assertEqual(data[3], 0x0E)
        self.assertEqual(data[4:8], bytes((0xFE, 0xFB, 0x00, 0x00)))

    def test_shorter_frames_are_right_aligned(self):
        """16- and 24-bit frames keep the padding the device expects"""
        f16 = DAPC(address.Broadcast(), 100).frame
        self.assertEqual(
            tridonic._cmd(tridonic._CMD_SEND, 1, frame=f16.pack_len(4))[4:8],
            bytes((0x00, 0x00, 0xFE, 0x64)))
        f32 = firmware.TransferBlockData(0xAABBCC).frame
        self.assertEqual(
            tridonic._cmd(tridonic._CMD_SEND, 1, frame=f32.pack_len(4))[4:8],
            bytes((0xBD, 0xAA, 0xBB, 0xCC)))


class TestBusWatch(unittest.IsolatedAsyncioTestCase):
    async def _observe(self, *messages):
        """Feed reports to a bus watch task and collect the callbacks."""
        dev = tridonic("/dev/null/does-not-exist")
        seen = []
        dev.bus_traffic.register(
            lambda driver, cmd, response, error: seen.append(
                (cmd, response, error)))
        task = asyncio.create_task(dev._bus_watch())
        try:
            for message in messages:
                dev._bus_watch_data.append(message)
            dev._bus_watch_data_available.set()
            # Long enough for the watch task to time out waiting for a
            # repeat or a response (the timeout in _bus_watch is 0.2s)
            await asyncio.sleep(0.4)
        finally:
            task.cancel()
        return seen

    async def test_dali32_forward_frame(self):
        """a 32-bit frame on the bus decodes to a firmware transfer command"""
        for rtype in tridonic._RESPONSE_FRAMES_DALI32:
            seen = await self._observe(
                _response(rtype, bytes((0xFE, 0xFB, 0x00, 0x00))))
            self.assertEqual(len(seen), 1, f"rtype {rtype:#04x} was not reported")
            cmd, response, error = seen[0]
            self.assertIsInstance(cmd, firmware.StartFirmwareTransfer)
            self.assertEqual(cmd.destination, address.FirmwareGearBroadcast())
            self.assertIsNone(response)
            self.assertFalse(error)

    async def test_dali32_response_type_values(self):
        """both candidate 32-bit response types are accepted"""
        self.assertIn(0xFE, tridonic._RESPONSE_FRAMES_DALI32)
        self.assertIn(0x7E, tridonic._RESPONSE_FRAMES_DALI32)

    async def test_dali32_query_with_response(self):
        """a backward frame following a 32-bit query is attached to it"""
        seen = await self._observe(
            _response(tridonic._RESPONSE_FRAME_DALI32,
                      bytes((0xFE, 0xFB, 0x07, 0x00))),
            _response(tridonic._RESPONSE_FRAME_DALI8,
                      bytes((0x00, 0x00, 0x00, 0xFF))),
        )
        self.assertEqual(len(seen), 1)
        cmd, response, error = seen[0]
        self.assertIsInstance(cmd, firmware.QueryFirmwareUpdateReceiverReady)
        self.assertTrue(response.value)
        self.assertFalse(error)

    async def test_dali32_query_without_response(self):
        """a 32-bit query with no backward frame reports 'No'"""
        seen = await self._observe(
            _response(tridonic._RESPONSE_FRAME_DALI32,
                      bytes((0xFE, 0xFB, 0x08, 0x00))))
        self.assertEqual(len(seen), 1)
        cmd, response, error = seen[0]
        self.assertIsInstance(cmd, firmware.QueryBlockIncompleteOrFault)
        self.assertFalse(response.value)
        self.assertFalse(error)

    async def test_dali24_still_decodes(self):
        """adding the 32-bit type did not disturb 24-bit decoding"""
        seen = await self._observe(
            _response(tridonic._RESPONSE_FRAME_DALI24,
                      bytes((0x00, 0xFF, 0xFE, 0x00))))
        self.assertEqual(len(seen), 1)
        self.assertEqual(len(seen[0][0].frame), 24)
