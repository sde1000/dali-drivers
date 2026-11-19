# philips.py - driver for the Philips/Signify "USB to DALI" interface (USB2DALI, LCN8600)
# Copyright (C) 2026 contributors to dali-drivers
#
# This file is part of dali-drivers.
#
# dali-drivers is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General
# Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# dali-drivers is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
# details.
#
# You should have received a copy of the GNU Lesser General Public License along with dali-drivers. If not, see
# <https://www.gnu.org/licenses/>.
"""Driver for the Philips/Signify "USB to DALI" interface (USB2DALI / LCN8600, USB VID 0471, PID 0003).

This is the interface used by the MultiOne software. There is no public protocol description; the protocol was
reverse engineered from USB captures of MultiOne (HID SET_REPORT of 64 bytes to the device, answers on the
interrupt IN endpoint). Only what the captures show is implemented: 16-bit forward frames, reading the answer,
the interface's own bus power (250 mA) and the start-up sequence.

Needs the `hidapi` package (module `hid`). The driver is synchronous: `send()` blocks until the frame has been
transmitted and the answer, if any, has been received.

    from dalidrivers.misc.philips import PhilipsUSB2DALI
    from dali.address import GearShort
    from dali.gear.general import QueryControlGearPresent

    with PhilipsUSB2DALI(bus_power=True) as bus:
        r = bus.send(QueryControlGearPresent(GearShort(0)))
        print(r.value)

Protocol summary (64-byte reports, no report ID):

  send forward frame   B2 <seq> 70 00 00 26 <twice> 98 3A 00 00 10 27 00 00 <5 bytes Manchester>
  poll result          02 <seq> 70 00 01      -> <seq> 70 00 01 <status> <data>    (0 busy, 1 answer, 2 none)
  bus power            81 <seq> 15 00 00 <1|0>
  start-up             84 <seq> 30|50|60 00 00 01, then 14 <seq> 10 (name) and 14 <seq> 10 00 01 (version)

`<seq>` counts 0..9. The interface streams the last answer all the time with 0xFF in place of `<seq>`.
"""
import logging
import threading
import time

log = logging.getLogger(__name__)


VID, PID = 0x0471, 0x0003
REPORT_SIZE = 64

# status byte in the answer to a poll
STATUS_BUSY = 0          # frame still being sent / waiting for an answer (data byte is stale)
STATUS_ANSWER = 1        # a backward frame (8 bit) was received, in Result.data
STATUS_NO_ANSWER = 2     # nothing was received


class Usb2DaliError(Exception):
    pass


class NotFoundError(Usb2DaliError):
    pass


class TimeoutError_(Usb2DaliError):
    pass


class Result:
    """Outcome of one forward frame. `status` is one of the STATUS_* values; `data` is the answer byte (only
    meaningful for STATUS_ANSWER)."""
    __slots__ = ("status", "data")

    def __init__(self, status, data):
        self.status, self.data = status, data

    @property
    def answered(self):
        return self.status == STATUS_ANSWER

    @property
    def value(self):
        """Answer byte, or None when there was no (valid) answer."""
        return self.data if self.status == STATUS_ANSWER else None

    def __repr__(self):
        return "Result(status=%d, data=0x%02X)" % (self.status, self.data)


# --- pure encoding functions (verified byte for byte against 402 captured frames) ----------------------------
_TX_MIDDLE = bytes([0x70, 0x00, 0x00, 0x26])
_TX_TIMING = bytes([0x98, 0x3A, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00])    # identical in every captured frame


def manchester_tail(value, bits=16):
    """Start bit + `bits` data bits (MSB first) as Manchester symbols ('01' = 1, '10' = 0), padded with idle
    ('11') to 5 bytes."""
    seq = [1] + [(value >> (bits - 1 - i)) & 1 for i in range(bits)]
    s = "".join("01" if b else "10" for b in seq).ljust(40, "1")
    if len(s) != 40:
        raise ValueError("frame too long for this interface (%d bits)" % bits)
    return int(s, 2).to_bytes(5, "big")


def encode_forward(value):
    """Alias of manchester_tail() for a 16-bit forward frame (address byte << 8 | command byte)."""
    return manchester_tail(value & 0xFFFF)


def tx_report(seq, value, twice=False):
    """64-byte report that transmits one 16-bit forward frame. `twice` makes the interface send it twice
    (needed for DALI configuration commands, e.g. RESET, STORE ..., INITIALISE, RANDOMISE)."""
    body = (bytes([0xB2, seq & 0xFF]) + _TX_MIDDLE + bytes([1 if twice else 0]) + _TX_TIMING
            + encode_forward(value))
    return body.ljust(REPORT_SIZE, b"\x00")


def poll_report(seq):
    """64-byte report that asks for the result of the last frame."""
    return bytes([0x02, seq & 0xFF, 0x70, 0x00, 0x01]).ljust(REPORT_SIZE, b"\x00")


def power_report(seq, on):
    """64-byte report that switches the interface's own bus supply (250 mA) on or off. No answer is sent."""
    return bytes([0x81, seq & 0xFF, 0x15, 0x00, 0x00, 1 if on else 0]).ljust(REPORT_SIZE, b"\x00")


def enumerate_interfaces(hid_module=None):
    """List connected interfaces as dicts from hid.enumerate (path, serial_number, product_string, ...)."""
    hid = hid_module or _import_hid()
    return list(hid.enumerate(VID, PID))


def _import_hid():
    try:
        import hid
    except Exception as e:      # pragma: no cover
        raise Usb2DaliError("The 'hid' module is missing (pip install hidapi): %s" % e)
    return hid


class PhilipsUSB2DALI:
    """
    Usage::

        with PhilipsUSB2DALI(bus_power=True) as bus:
            r = bus.send_forward(0x0191)          # QUERY CONTROL GEAR PRESENT, short address 0 (0x01<<8 | 0x91)
            print(r.value)                        # 255 (= YES) or None
            print(bus.query_present(0))

    Close MultiOne (or any program using the interface) first. The object is thread safe (one frame at a time).
    """
    ANSWER_TIMEOUT = 1.0      # max time to wait for a frame to finish
    POLL_INTERVAL = 0.003     # MultiOne polls every ~16 ms; polling faster only returns the result sooner
    POWER_SETTLE = 0.3        # wait after switching bus power before the first frame

    def __init__(self, path=None, serial=None, bus_power=False, hid_module=None):
        self.path, self.serial = path, serial
        self.bus_power = bool(bus_power)
        self._hid = hid_module
        self._dev = None
        self._seq = 1
        self._lock = threading.RLock()
        self._power_sent = False
        self.name = self.version = None

    # --- context manager ---------------------------------------------------------------------------------
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()

    # --- connection --------------------------------------------------------------------------------------
    def connect(self):
        with self._lock:
            hid = self._hid or _import_hid()
            self._hid = hid
            items = list(hid.enumerate(VID, PID))
            if self.serial:
                items = [i for i in items if i.get("serial_number") == self.serial]
            if not items:
                raise NotFoundError("No Philips USB to DALI interface (USB %04X:%04X) found. Is it connected, "
                                    "and is MultiOne closed?" % (VID, PID))
            item = items[0]
            if self.path is not None:
                matches = [it for it in items
                           if it.get("path") in (self.path, str(self.path).encode())]
                if not matches:
                    raise NotFoundError("No Philips USB to DALI interface at path %r" % self.path)
                item = matches[0]
            self._dev = hid.device()
            self._dev.open_path(item["path"])
            log.info("Opened %s %s (serial %s)", item.get("manufacturer_string"), item.get("product_string"),
                     item.get("serial_number"))
            self._start_up()

    def close(self):
        with self._lock:
            try:
                if self._dev is not None:
                    if self._power_sent and self.bus_power:
                        try:
                            self._write(power_report(self._next_seq(), False))     # do not leave the supply on
                        except Exception:
                            pass
                    self._dev.close()
            finally:
                self._dev = None
                self._power_sent = False

    @property
    def connected(self):
        return self._dev is not None

    def _start_up(self):
        """What MultiOne sends on connect: three property writes, then name and version of the interface."""
        for port in (0x30, 0x50, 0x60):
            self._write(bytes([0x84, self._next_seq(), port, 0x00, 0x00, 0x01]))
        rep = self._query(bytes([0x14]), bytes([0x10]), 0x10)
        self.name = bytes(rep[4:20]).split(b"\x00")[0].decode("ascii", "replace")
        rep = self._query(bytes([0x14]), bytes([0x10, 0x00, 0x01]), 0x10)
        self.version = bytes(rep[4:20]).split(b"\x00")[0].decode("ascii", "replace")
        log.info("Interface %s, version %s", self.name, self.version)
        if not self.name.startswith("USB2DALI"):
            log.warning("Unexpected interface name %r", self.name)
        self.set_bus_power(self.bus_power)

    # --- public API --------------------------------------------------------------------------------------
    def set_bus_power(self, on):
        """Switch the interface's own DALI bus supply (250 mA) on or off."""
        with self._lock:
            self._write(power_report(self._next_seq(), on))
            self.bus_power = bool(on)
            self._power_sent = True
            log.info("Bus power %s", "ON" if on else "OFF")
            time.sleep(self.POWER_SETTLE)

    def send_forward(self, value, twice=False):
        """Send one 16-bit forward frame (address byte << 8 | command byte) and wait for the result."""
        with self._lock:
            if self._dev is None:
                raise Usb2DaliError("not connected")
            self._drain()
            self._write(tx_report(self._next_seq(), value, twice))
            return self._wait_result()

    def query(self, address_byte, command):
        """Send address_byte/command and return the answer byte, or None if there was no valid answer."""
        return self.send_forward(((address_byte & 0xFF) << 8) | (command & 0xFF)).value

    def query_present(self, short, tries=3):
        """QUERY CONTROL GEAR PRESENT on a short address (0..63). True only after two valid YES (0xFF) answers,
        so a single garbled reply is not counted. Stops at once if the first try gets no answer."""
        hits = 0
        for _ in range(tries):
            if self.query((short << 1) | 1, 0x91) == 0xFF:
                hits += 1
                if hits >= 2:
                    return True
            elif hits == 0:
                return False
        return False

    def scan_present(self, shorts=range(64)):
        """Short addresses that answer QUERY CONTROL GEAR PRESENT."""
        return [s for s in shorts if self.query_present(s)]

    def send(self, cmd):
        """python-dali style: send a command object (needs python-dali) and return its response object."""
        frame = cmd.frame
        if len(frame) != 16:
            raise NotImplementedError("only 16-bit forward frames are supported (got %d bits)" % len(frame))
        try:
            value = frame.as_integer
        except AttributeError:
            value = int.from_bytes(bytes(frame.as_byte_sequence), "big")
        res = self.send_forward(value, bool(getattr(cmd, "sendtwice", False)))
        resp = getattr(cmd, "response", None)
        if not resp:
            return None
        if res.status == STATUS_ANSWER:
            from dali.frame import BackwardFrame
            return resp(BackwardFrame(res.data))
        return resp(None)

    # --- low level ---------------------------------------------------------------------------------------
    def _next_seq(self):
        s = self._seq
        self._seq = (self._seq + 1) % 10
        return s

    def _write(self, payload):
        if self._dev is None:
            raise Usb2DaliError("not connected")
        buf = bytes([0]) + bytes(payload).ljust(REPORT_SIZE, b"\x00")      # leading 0 = no report ID (hidapi)
        n = self._dev.write(list(buf))
        if n != REPORT_SIZE + 1:
            try:
                err = self._dev.error()
            except Exception:
                err = "unknown error"
            raise Usb2DaliError("Writing to the interface failed (wrote %r bytes): %s" % (n, err))

    def _drain(self):
        # the interface streams a report every ~1 ms (0xFF + the last answer); throw away what is queued
        for _ in range(300):
            if not self._dev.read(REPORT_SIZE, 0):
                return

    def _read_echo(self, seq, port, timeout=0.5):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rep = self._dev.read(REPORT_SIZE, 20)
            if rep and len(rep) >= 6 and rep[0] == seq and rep[1] == port:
                return rep
        return None

    def _query(self, head, tail, port):
        self._drain()
        seq = self._next_seq()
        self._write(head + bytes([seq]) + tail)
        rep = self._read_echo(seq, port)
        if rep is None:
            raise TimeoutError_("The interface did not answer (command %02X)" % head[0])
        return rep

    def _wait_result(self):
        end = time.monotonic() + self.ANSWER_TIMEOUT
        while time.monotonic() < end:
            self._drain()
            seq = self._next_seq()
            self._write(poll_report(seq))
            rep = self._read_echo(seq, 0x70)
            if rep is not None and rep[4] != STATUS_BUSY:
                if rep[4] not in (STATUS_ANSWER, STATUS_NO_ANSWER):
                    log.warning("Unknown status %d (data 0x%02X)", rep[4], rep[5])
                return Result(rep[4], rep[5])
            time.sleep(self.POLL_INTERVAL)
        raise TimeoutError_("The interface did not finish sending the frame in time")
