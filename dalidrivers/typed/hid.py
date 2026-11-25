# async device drivers for HID-based devices like Tridonic DALI-USB
# and hasseb DALI Master

# Two I/O backends are available:
#
# - "hidraw" (default on Linux): opens a /dev/hidrawN node directly and
#   needs no extra dependencies.  See examples/50-dali-hid.rules for udev
#   rules that will make suitably-named symlinks to the hidraw devices.
# - "hidapi" (default elsewhere, e.g. Windows and macOS): uses the hidapi
#   package (pip install dali-drivers[driver-hid]) with a reader thread.

from __future__ import annotations

import asyncio
import fnmatch
import os
import struct
import logging
import random
import glob
import sys
import threading
import warnings
from typing import Any, Callable, Iterator, Literal, Protocol, TYPE_CHECKING
from dali.device.helpers import DeviceInstanceTypeMapper
from dali.exceptions import UnsupportedFrameTypeError, CommunicationError
import dali.sequences
from dali.sequences import sleep as seq_sleep
from dali.sequences import progress as seq_progress
import dali.frame

# dali.command and dali.gear are required for the bus traffic callback
import dali.command
import dali.gear
from dali.gear.general import EnableDeviceType

from dalidrivers.typed._queue import DistributorQueue


def _hex(b: bytes) -> str:
    return ''.join("%02X" % x for x in b)


def _single_command_sequence(
    cmd: dali.command.Command,
) -> dali.sequences.DALISequence[Any]:
    """Wrap a single command as a trivial sequence that returns its response."""
    return (yield cmd)


class _callback:
    """Helper class for callback registration
    """
    def __init__(self, parent: hid) -> None:
        self._parent = parent
        self._callbacks: dict[
            _callback._callback_handle, Callable[..., object]
        ] = {}

    class _callback_handle:
        """Callback handle

        Call unregister() to remove this callback.
        """
        def __init__(self, callback: _callback) -> None:
            self._callback = callback

        def unregister(self) -> None:
            del self._callback._callbacks[self]

    def register(
        self, func: Callable[..., object]
    ) -> _callback._callback_handle:
        wrapper = self._callback_handle(self)
        self._callbacks[wrapper] = func
        return wrapper

    def _invoke(self, *args: object) -> None:
        for func in self._callbacks.values():
            asyncio.get_running_loop().call_soon(func, self._parent, *args)


HidBackend = Literal["auto", "hidraw", "hidapi"]


def _default_backend() -> Literal["hidraw", "hidapi"]:
    """hidraw on Linux (no dependencies, works with the udev rules), else hidapi."""
    return "hidraw" if sys.platform.startswith("linux") else "hidapi"


def _import_hidapi() -> Any:
    """Import the hidapi package's binding module.

    On Linux the hidapi wheel ships two modules: ``hidraw`` (kernel hidraw
    backend, same permissions as the hidraw backend of this driver) and
    ``hid`` (libusb backend, which needs USB device permissions and detaches
    the kernel driver).  Prefer ``hidraw`` there; elsewhere only ``hid``
    exists.
    """
    if sys.platform.startswith("linux"):
        try:
            import hidraw  # type: ignore[import-not-found]
            return hidraw
        except ImportError:
            pass
    try:
        import hid as hidapi  # type: ignore[import-not-found]
    except ImportError as e:
        raise ImportError(
            "The hidapi backend needs the 'hidapi' package "
            "(pip install dali-drivers[driver-hid])") from e
    return hidapi


class _Transport(Protocol):
    """An open HID device delivering input reports to the event loop."""

    def write(self, buf: bytes) -> None:
        """Send one output report (without report ID); raise OSError on failure."""

    def close(self) -> None:
        """Stop delivering reports and release the device."""


class _HidrawTransport:
    """Linux hidraw device node, read via the event loop's add_reader()."""

    def __init__(self, path: str, on_data: Callable[[bytes], None],
                 on_lost: Callable[[], None]) -> None:
        self._fd: int | None = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        self._on_data = on_data
        self._on_lost = on_lost
        self._loop = asyncio.get_running_loop()
        self._loop.add_reader(self._fd, self._reader)

    def _reader(self) -> None:
        # The reader is only registered while the device file is open.
        assert self._fd is not None
        try:
            # No need to retry on InterruptedError since 3.5
            data = os.read(self._fd, 64)
        except OSError:
            # Device has gone away
            data = b''
        if len(data) == 0:
            self._on_lost()
            return
        self._on_data(data)

    def write(self, buf: bytes) -> None:
        assert self._fd is not None
        os.write(self._fd, buf)

    def close(self) -> None:
        if self._fd is None:
            return
        self._loop.remove_reader(self._fd)
        os.close(self._fd)
        self._fd = None


class _HidapiTransport:
    """Cross-platform device opened through hidapi.

    hidapi only offers blocking reads, so a daemon thread reads input reports
    and hands them to the event loop with call_soon_threadsafe().  Writes are
    issued directly from the event loop: they are a single short interrupt
    transfer.
    """

    # How often the reader thread checks whether it should stop.
    _READ_TIMEOUT_MS = 100

    def __init__(self, hidapi: Any, path: bytes,
                 on_data: Callable[[bytes], None],
                 on_lost: Callable[[], None]) -> None:
        self._dev = hidapi.device()
        self._dev.open_path(path)
        self._on_data = on_data
        self._on_lost = on_lost
        self._loop = asyncio.get_running_loop()
        self._closed = False
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._read_loop, name=f"dali-hid-reader {path!r}",
            daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        try:
            while not self._stop.is_set():
                try:
                    data = self._dev.read(64, self._READ_TIMEOUT_MS)
                except (OSError, ValueError):
                    # Device has gone away (hidapi raises IOError/OSError on
                    # read errors, ValueError if the device was closed).
                    if not self._stop.is_set():
                        self._loop.call_soon_threadsafe(self._deliver_lost)
                    return
                if data:
                    self._loop.call_soon_threadsafe(self._deliver, bytes(data))
        except RuntimeError:
            # The event loop was closed without close() being called, e.g.
            # asyncio.run() finished after the program forgot to disconnect
            # or crashed.  Nothing can receive reports any more, so stop.
            return

    # These run on the event loop thread, as does close(), so checking
    # _closed here drops reports that were queued just before closing.
    def _deliver(self, data: bytes) -> None:
        if not self._closed:
            self._on_data(data)

    def _deliver_lost(self) -> None:
        if not self._closed:
            self._on_lost()

    def write(self, buf: bytes) -> None:
        # hidapi expects the report ID as the first byte; these devices use
        # unnumbered reports, i.e. report ID 0.  On Linux hidraw strips a
        # leading zero, so the device sees the same bytes as with the
        # hidraw transport; on Windows the ID byte is mandatory.
        written = self._dev.write(b"\x00" + buf)
        if written is not None and written < 0:
            raise OSError(f"hidapi write failed: {self._dev.error()}")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        # hidapi devices must not be closed while another thread is in
        # read(); the read timeout bounds how long this takes.
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=2 * self._READ_TIMEOUT_MS / 1000 + 1)
        self._dev.close()


class hid:
    """Shared code for drivers that work with HID devices

    :param path: Device to open.  With the hidraw backend this is a device
    node such as ``/dev/hidraw0`` (or a glob pattern if ``glob`` is true).
    With the hidapi backend it is a hidapi device path as returned by
    :meth:`enumerate` (matched with fnmatch if ``glob`` is true), or None to
    open the first device with this driver's USB vendor and product ID.
    :param backend: ``"hidraw"``, ``"hidapi"`` or ``"auto"`` (hidraw on
    Linux, hidapi on other platforms).
    """
    # USB IDs of the supported device, used by the hidapi backend to find
    # devices when no path is given.  Set by subclasses.
    usb_vendor_id: int | None = None
    usb_product_id: int | None = None

    def __init__(self, path: str | None = None, reconnect_interval: float = 1,
                 reconnect_limit: int | None = None,
                 glob: bool = False,
                 dev_inst_map: DeviceInstanceTypeMapper | None = None,
                 backend: HidBackend = "auto",
                 hidapi_module: Any = None) -> None:
        self._log = logging.getLogger()
        self._path = path
        if backend == "auto":
            backend = _default_backend()
        if backend not in ("hidraw", "hidapi"):
            raise ValueError(f"unknown HID backend {backend!r}")
        if backend == "hidraw" and path is None:
            raise ValueError("the hidraw backend needs a device path")
        self.backend: Literal["hidraw", "hidapi"] = backend
        self._hidapi = hidapi_module
        self._reconnect_interval = reconnect_interval
        self._reconnect_limit = reconnect_limit
        self._reconnect_count = 0
        self._reconnect_task: asyncio.Task[None] | None = None
        self._glob = glob
        if dev_inst_map is None:
            dev_inst_map = DeviceInstanceTypeMapper()
        self.dev_inst_map: DeviceInstanceTypeMapper = dev_inst_map
        self._transport: _Transport | None = None

        # The exception from the most recent _open() attempt, so an async
        # connect() can fail fast instead of waiting out the reconnect loop.
        self._open_error: Exception | None = None

        # Lazily created when new_dali_rx_queue() is first called: a parent
        # fan-out queue fed from the bus_traffic callback, and its handle.
        self._rx_queue_parent: DistributorQueue | None = None
        self._rx_traffic_handle: _callback._callback_handle | None = None

        # Should the send() method raise an exception if there is a
        # problem communicating with the underlying device, or should
        # it catch the exception and keep trying?  Set this attribute
        # as required.
        self.exceptions_on_send = True

        # Acquire this lock to perform a series of commands as a
        # transaction.  While you hold the lock, you must call send()
        # with keyword argument in_transaction=True
        self.transaction_lock = asyncio.Lock()

        # Register to be called back with "connected", "disconnected"
        # or "failed" as appropriate ("failed" means the reconnect
        # limit has been reached; no more connections will be
        # attempted unless you call connect() explicitly.)
        self.connection_status_callback = _callback(self)

        # Register to be called back with bus traffic; three arguments are passed:
        # command, response, config_command_error

        # config_command_error is true if the config command has a response, or
        # if the command was not sent twice within the required time limit
        self.bus_traffic = _callback(self)

        # This event will be set when we are connected to the device
        # and cleared when the connection is lost
        self.connected = asyncio.Event()

        # firmware_version and serial may be populated on some
        # devices, and will read as None on devices that don't support
        # reading them.  They are only valid after self.connected is
        # set.
        self.firmware_version: str | None = None
        self.serial: str | None = None

    async def connect(self, *, scan_dev_inst: bool = False,
                      timeout: float = 10.0) -> None:
        """Open the device and wait until it is ready for use.

        This mirrors the serial driver's ``connect()`` so both drivers expose
        one uniform async interface: open the device, block until the
        device-specific handshake has completed, and optionally scan the bus
        for control-device instance types.

        A missing or inaccessible device node fails fast with the underlying
        OSError; a device that opens but never completes its handshake raises
        TimeoutError after ``timeout`` seconds.

        :param scan_dev_inst: Scan the DALI bus for control devices and
        populate ``dev_inst_map`` with their instance types.
        :param timeout: Seconds to wait for the handshake to complete.
        """
        self._open_error = None
        if not self._open() and self._open_error is not None:
            # The device node is missing or not permitted. Cancel the
            # reconnection loop _open() scheduled and report the failure.
            if self._reconnect_task is not None:
                self._reconnect_task.cancel()
                self._reconnect_task = None
            raise self._open_error
        try:
            await asyncio.wait_for(self.connected.wait(), timeout)
        except TimeoutError:
            self.disconnect()
            raise
        if scan_dev_inst:
            await self.send(self.dev_inst_map.autodiscover())

    @classmethod
    def enumerate(cls, hidapi_module: Any = None) -> list[dict[str, Any]]:
        """List connected devices of this type using hidapi.

        Returns hidapi's device dicts (``path``, ``serial_number``,
        ``product_string``, ...); pass ``d["path"]`` as the ``path`` argument
        with the hidapi backend.  Works on all platforms hidapi supports.
        """
        hidapi = hidapi_module or _import_hidapi()
        if cls.usb_vendor_id is None or cls.usb_product_id is None:
            raise NotImplementedError(f"{cls.__name__} has no USB IDs")
        return list(hidapi.enumerate(cls.usb_vendor_id, cls.usb_product_id))

    def _candidate_paths(self) -> list[str | bytes]:
        """Resolve the configured path to concrete paths for the backend."""
        if self.backend == "hidraw":
            assert self._path is not None
            if self._glob:
                return list(glob.glob(self._path))
            return [self._path]
        if self._hidapi is None:
            self._hidapi = _import_hidapi()
        if self._path is not None and not self._glob:
            return [self._path.encode()]
        devices = self.enumerate(self._hidapi)
        found: list[bytes] = [d["path"] for d in devices]
        if self._path is not None:
            pattern = self._path
            found = [p for p in found
                     if fnmatch.fnmatchcase(p.decode(errors="replace"), pattern)]
        return list[str | bytes](found)

    def _open_transport(self, path: str | bytes) -> _Transport:
        if self.backend == "hidraw":
            assert isinstance(path, str)
            return _HidrawTransport(path, self._handle_read, self._device_lost)
        assert isinstance(path, bytes)
        return _HidapiTransport(
            self._hidapi, path, self._handle_read, self._device_lost)

    def _open(self) -> bool:
        """Attempt to open the device.

        If opening fails, schedules a reconnection attempt and records the
        exception in ``self._open_error``.

        Returns True if opening the device succeded immediately,
        False otherwise.  NB you must still await connected.wait()
        before using the device, because there may be further
        initialisation for the driver to perform.
        """
        if self._transport is not None:
            return True
        self._log.debug("trying to connect to %s via %s...",
                        self._path, self.backend)
        ex: Exception | None = None
        try:
            path = self._candidate_paths()
        except ImportError:
            # A missing dependency will not fix itself by retrying.
            raise
        except Exception as e:
            path = []
            ex = e
        if path:
            try:
                if self._glob or self._path is None:
                    self._log.debug("trying concrete path %r", path[0])
                self._transport = self._open_transport(path[0])
            except Exception as e:
                self._transport = None
                ex = e
        elif ex is None:
            self._log.debug("path %s not found", self._path)
            ex = FileNotFoundError(
                self._path if self._path is not None
                else f"no {type(self).__name__} HID device found")
        if self._transport is None:
            # It didn't work.  Schedule a reconnection attempt if we can.
            self._open_error = ex
            self._log.debug("hid failed to open %s (%s) - waiting to try again", self._path, ex)
            self._reconnect_task = asyncio.create_task(self._reconnect())
            return False
        self._open_error = None
        self._reconnect_count = 0
        self._log.debug("hid opened %r", path[0])
        try:
            self._initialise_device()
        except OSError as e:
            # The device vanished between opening and the first write;
            # treat it like a failed open so reconnection keeps trying.
            self._log.debug("hid failed to initialise %r (%s)", path[0], e)
            self._transport.close()
            self._transport = None
            self._open_error = e
            self._reconnect_task = asyncio.create_task(self._reconnect())
            return False
        self.connection_status_callback._invoke("connected")
        return True

    async def _reconnect(self) -> None:
        self._reconnect_count += 1
        if self._reconnect_limit is not None \
           and self._reconnect_count > self._reconnect_limit:
            # We have failed.
            self._log.debug("connection limit reached")
            self._reconnect_count = 0
            self._reconnect_task = None
            return
        await asyncio.sleep(self._reconnect_interval)
        self._reconnect_task = None
        self._open()

    def disconnect(self, reconnect: bool = False) -> None:
        self._log.debug("disconnecting")
        if self._reconnect_task:
            self._reconnect_task.cancel()
            self._reconnect_task = None
        if self._transport is not None:
            self._transport.close()
        self._shutdown_device()
        self._transport = None
        self.connected.clear()
        self.connection_status_callback._invoke("disconnected")
        if reconnect:
            self._reconnect_task = asyncio.ensure_future(self._reconnect())

    def close(self) -> None:
        """Close the device for good, mirroring the serial driver's close().

        Unlike ``disconnect()`` this does not schedule a reconnection and also
        tears down the received-traffic fan-out queue.
        """
        if self._rx_traffic_handle is not None:
            self._rx_traffic_handle.unregister()
            self._rx_traffic_handle = None
        self._rx_queue_parent = None
        self.disconnect()

    @property
    def is_connected(self) -> bool:
        """True while the device is open and its handshake has completed."""
        return self._transport is not None and self.connected.is_set()

    def new_dali_rx_queue(self) -> DistributorQueue:
        """Return a queue of DALI commands observed on the bus.

        Each call returns an independent child queue. The first call starts
        forwarding the ``bus_traffic`` callback's commands into the fan-out;
        responses to the driver's own queries are not included (``send()``
        returns those to the caller).
        """
        if self._rx_queue_parent is None:
            self._rx_queue_parent = DistributorQueue()
            self._rx_traffic_handle = self.bus_traffic.register(
                self._distribute_bus_traffic
            )
        return DistributorQueue(self._rx_queue_parent)

    def _distribute_bus_traffic(
        self,
        driver: hid,
        command: dali.command.Command,
        response: Any,
        config_command_error: bool,
    ) -> None:
        if self._rx_queue_parent is not None:
            self._rx_queue_parent.distribute(command)

    async def send(
        self,
        msg: dali.command.Command | dali.sequences.DALISequence[Any],
        *,
        priority: int = 5,
        progress: Callable[[str | dali.sequences.progress], None] | None = None,
        exceptions: bool | None = None,
    ) -> Any:
        """Send a command or run a sequence as a single transaction

        A bare Command is wrapped in a trivial sequence, so a single command
        and a multi-command sequence share the same execution path. The whole
        transaction is sent while holding the transaction_lock, so it cannot be
        interrupted by another sender. Returns the command's response for a
        single command, otherwise the sequence's return value.

        When a command has a non-zero device type, an EnableDeviceType frame is
        transmitted immediately before it.

        priority is accepted for a uniform driver API and validated against the
        IEC 62386-101 Table 22 range 2..5 (a value outside it raises
        ValueError), but HID dongles transmit at a fixed priority, so it has no
        effect on the wire.

        This call can raise dali.exceptions.CommunicationError if there is a
        problem sending a command to the device. If you prefer to wait for the
        device to become available again, pass exceptions=False or set the
        exceptions_on_send attribute to False.
        """
        if priority not in (2, 3, 4, 5):
            raise ValueError(
                f"priority must be in the range 2..5, not {priority!r}"
            )
        if exceptions is None:
            exceptions = self.exceptions_on_send

        seq = (
            _single_command_sequence(msg)
            if isinstance(msg, dali.command.Command)
            else msg
        )

        await self.transaction_lock.acquire()
        response = None
        try:
            while True:
                try:
                    cmd = seq.send(response)
                except StopIteration as r:
                    return r.value
                response = None
                if isinstance(cmd, seq_sleep):
                    await asyncio.sleep(cmd.delay)
                elif isinstance(cmd, seq_progress):
                    if progress:
                        progress(cmd)
                else:
                    command_sent = False
                    while not command_sent:
                        try:
                            if cmd.devicetype is not None:
                                await self._send_raw(
                                    EnableDeviceType(cmd.devicetype)
                                )
                            response = await self._send_raw(cmd)
                            command_sent = True
                        except CommunicationError:
                            if exceptions:
                                raise
        finally:
            self.transaction_lock.release()
            seq.close()

    async def power_supply(
        self, supply_on: bool, in_transaction: bool = False,
        exceptions: bool | None = None
    ) -> dali.command.Response[Any] | None:
        """
        TODO
        """
        if exceptions is None:
            exceptions = self.exceptions_on_send

        if not in_transaction:
            await self.transaction_lock.acquire()
        try:
            command_sent = False
            while not command_sent:
                try:
                    response = await self._power_supply(supply_on)
                    command_sent = True
                except CommunicationError:
                    if exceptions:
                        raise
            return response
        finally:
            if not in_transaction:
                self.transaction_lock.release()

    async def run_sequence(
        self,
        seq: dali.sequences.DALISequence[Any],
        progress: Callable[[str | dali.sequences.progress], None] | None = None,
    ) -> Any:
        """Deprecated alias for send(), which now accepts commands and sequences
        """
        warnings.warn(
            "run_sequence() is deprecated; use send() instead",
            DeprecationWarning,
            stacklevel=2,
        )
        return await self.send(seq, progress=progress)

    async def _send_raw(
        self, command: dali.command.Command
    ) -> dali.command.Response[Any] | None:
        """Transmit a single frame; implemented in a device-specific subclass.
        """
        raise NotImplementedError(
            "'_send_raw()' needs to be implemented in a subclass"
        )

    async def _power_supply(
        self, supply_on: bool
    ) -> dali.command.Response[Any] | None:
        """Switch the bus power supply; implemented in a subclass if supported.
        """
        raise NotImplementedError(
            "'_power_supply()' needs to be implemented in a subclass"
        )

    def _initialise_device(self) -> None:
        """Send any device-specific initialisation commands
        """
        # Some devices may need to send initialisation commands and await
        # responses.  Those devices should override this method, and make sure
        # they set self.connected once initialisation is complete
        self.connected.set()

    def _shutdown_device(self) -> None:
        """Shut down everything that is waiting for the device

        The device has gone away.  Everything that is waiting for the
        device needs to be shut down explicitly.
        """
        pass

    def _device_lost(self) -> None:
        """Called by the transport when the device has gone away."""
        self.disconnect(reconnect=True)

    def _raw_write(self, buf: bytes) -> None:
        """Send one output report; raises OSError if the device has failed."""
        if self._transport is None:
            raise OSError("HID device is not open")
        self._transport.write(buf)

    def _handle_read(self, data: bytes) -> None:
        pass


class tridonic(hid):
    usb_vendor_id = 0x17B5
    usb_product_id = 0x0020

    # Commands sent to the interface
    # cmd, seq, ctrl, mode, frame (4 bytes), dtr, prio, devtype
    _cmdtmpl = struct.Struct(">4B4s3B53x")
    # length of the "useful part", without the trailing padding
    _cmdtmpl_useful_size = _cmdtmpl.size - 53

    # _CMD send in byte 0
    _CMD_INIT = 0x01
    _CMD_BOOTLOADER = 0x02
    _CMD_SEND = 0x12
    _CMD_SEND_ANSWER = 0x15
    _CMD_SET_IOPINS = 0x20
    _CMD_READ_IOPINS = 0x21
    _CMD_IDENTIFY = 0x22
    _CMD_POWERSUPPLY = 0x40

    # For _CMD_INIT, send in byte 1:
    _CMD_INIT_READVERSION = 0x00
    _CMD_INIT_READSERIAL = 0x02

    # For _CMD_SEND, send a sequence number is byte 1 and you can set
    # the following flag bits in control byte 2:
    _SEND_CTRL_SET_ACTUAL_LEVEL_TO_DTR = 0x04
    _SEND_CTRL_SEND_DEVTYPE = 0x80
    _SEND_CTRL_SETDTR0 = 0x10
    _SEND_CTRL_SENDTWICE = 0x20
    _SEND_CTRL_IDENTIFY = 0x40  # response is a _RESPONSE_INFO

    # For _CMD_SEND, send in mode byte 3:
    _SEND_MODE_DALI8 = 2
    _SEND_MODE_DALI16 = 3
    _SEND_MODE_eDALI25 = 4
    _SEND_MODE_DSI = 5
    _SEND_MODE_DALI24 = 6
    _SEND_MODE_HELVAR17 = 8

    # For _CMD_POWERSUPPLY, send in byte 1;
    _POWER_SUPPLY_OFF = 0x00
    _POWER_SUPPLY_ON = 0x01
    _POWER_SUPPLY_TBD = 0xFF  # APPLY?

    # Responses received from the interface
    # Decodes to mode, response type, frame, interval, seq
    _resptmpl = struct.Struct(">BB4sHB55x")
    # length of the "useful part", without the trailing padding
    _resptmpl_useful_size = _resptmpl.size - 55
    _MODE_INFO = 0x01  # Response to an init command
    _MODE_OBSERVE = 0x11  # Other traffic observed on the bus
    _MODE_RESPONSE = 0x12  # Response to a send command

    # Response types
    _RESPONSE_NO_FRAME = 0x71
    _RESPONSE_FRAME_DALI8 = 0x72
    _RESPONSE_FRAME_DALI16 = 0x73
    _RESPONSE_FRAME_eDALI25 = 0x74
    _RESPONSE_FRAME_DSI = 0x75
    _RESPONSE_FRAME_DALI24 = 0x76
    _RESPONSE_INFO = 0x77  # bus or framing error - see byte frame[3]
    _RESPONSE_FRAME_HELVAR17 = 0x78

    # Bus status in byte frame[3]
    _BUS_STATUS_CHECKSUM = 1
    _BUS_STATUS_SHORTED = 2
    _BUS_STATUS_FRAMING_ERROR = 3
    _BUS_STATUS_OK = 4
    _BUS_STATUS_DSI_MODE = 5
    _BUS_STATUS_DALI_MODE = 6

    @staticmethod
    def _seqnum(i: int) -> Iterator[int]:
        """Sequence number generator
        """
        while True:
            yield i
            i += 1
            if i > 0xff:
                i = 1

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._log = self._log.getChild("tridonic")

        # Initialise the command sequence number to a random value,
        # avoiding zero
        self._cmd_seq = iter(self._seqnum(random.randint(0x01, 0xff)))

        # Outstanding command events and message queues indexed by
        # sequence number
        self._outstanding: dict[
            int, tuple[asyncio.Event, list[bytes | str]]
        ] = {}

        # Semaphore controlling number of outstanding commands
        self._command_semaphore = asyncio.BoundedSemaphore(2)

        # Bus watch task, event and message queue
        self._bus_watch_task: asyncio.Task[None] | None = None
        self._bus_watch_data_available = asyncio.Event()
        self._bus_watch_data: list[bytes] = []

    def _raw_write(self, buf: bytes) -> None:
        self._log.debug(f"_raw_write: {_hex(buf[0:self._cmdtmpl_useful_size])}")
        super()._raw_write(buf)

    def _initialise_device(self) -> None:
        # Read firmware version; pick up the reply in _handle_read
        self._raw_write(self._cmd(
            tridonic._CMD_INIT, tridonic._CMD_INIT_READVERSION))

    async def _power_supply(self, supply_on: bool) -> None:
        await self.connected.wait()
        async with self._command_semaphore:
            data = self._cmd(
                self._CMD_POWERSUPPLY,
                self._POWER_SUPPLY_ON if supply_on else self._POWER_SUPPLY_OFF)

            try:
                self._raw_write(data)
            except OSError:
                # The device has failed.  Disconnect, schedule a
                # reconnection, and report this command as failed.
                self._log.debug("fail on transmit, disconnecting")
                self.disconnect(reconnect=True)
                raise CommunicationError

    @staticmethod
    def _command_mode(frame: dali.frame.Frame) -> int:
        if len(frame) == 8:
            return tridonic._SEND_MODE_DALI8
        if len(frame) == 16:
            return tridonic._SEND_MODE_DALI16
        if len(frame) == 24:
            return tridonic._SEND_MODE_DALI24
        raise UnsupportedFrameTypeError

    async def _send_raw(
        self, command: dali.command.Command
    ) -> dali.command.Response[Any] | None:
        frame = command.frame
        if len(frame) not in (16, 24):
            raise UnsupportedFrameTypeError
        await self.connected.wait()
        async with self._command_semaphore:
            seq = next(self._cmd_seq)
            self._log.debug(f"Sending with {seq=:02x}")
            event = asyncio.Event()
            messages: list[bytes | str] = []
            # If seq is in self._outstanding this means we've wrapped
            # around the whole sequence number space with an event
            # still outstanding: clearly a bug!
            assert seq not in self._outstanding
            self._outstanding[seq] = (event, messages)
            data = self._cmd(
                self._CMD_SEND, seq,
                ctrl=self._SEND_CTRL_SENDTWICE if command.sendtwice else 0,
                mode=self._command_mode(frame),
                frame=frame.pack_len(4))
            try:
                self._raw_write(data)
            except OSError:
                # The device has failed.  Disconnect, schedule a
                # reconnection, and report this command as failed.
                self._log.debug("fail on transmit, disconnecting")
                self.disconnect(reconnect=True)
                raise CommunicationError

            outstanding_transmissions = 2 if command.sendtwice else 1
            response: dali.frame.BackwardFrame | str | None = None
            while outstanding_transmissions or response is None:
                self._log.debug(f"_send_raw waiting for {outstanding_transmissions=} "
                                f"{response=!s}")
                if len(messages) == 0:
                    try:
                        await asyncio.wait_for(event.wait(), timeout=0.2)
                    except TimeoutError:
                        self._log.error(f"_send_raw timed out {seq=:02x}, faking an error response")
                        response = dali.frame.BackwardFrameError(255)
                        break
                    finally:
                        event.clear()
                message = messages.pop(0)
                if message == "fail":
                    # The device has gone away, possibly in the middle
                    # of processing our command.
                    self._log.debug("processing queued fail on receive")
                    raise CommunicationError
                if TYPE_CHECKING:
                    # Only the "fail" sentinel is a str; every other queued
                    # message is a raw response packet.
                    assert isinstance(message, bytes)

                # The message mode is guaranteed to be _MODE_RESPONSE
                mode, rtype, frame, interval, seq = self._resptmpl.unpack(
                    message)
                self._log.debug(
                    f"_send_raw got message: {mode=:02x} {rtype=:02x} "
                    f"frame={_hex(frame)} {interval=:04x} {seq=:02x}")
                if rtype in (self._RESPONSE_FRAME_DALI16,
                             self._RESPONSE_FRAME_DALI24):
                    # XXX check the frame contents?
                    outstanding_transmissions -= 1
                    self._log.debug(
                        f"_send_raw rtype DALI16 or DALI24, new "
                        f"{outstanding_transmissions=}")
                elif rtype == self._RESPONSE_FRAME_DALI8:
                    response = dali.frame.BackwardFrame(frame)
                elif (rtype == self._RESPONSE_INFO
                      and frame[3] == self._BUS_STATUS_FRAMING_ERROR):
                    response = dali.frame.BackwardFrameError(255)
                elif rtype == self._RESPONSE_NO_FRAME:
                    response = "no"
                else:
                    self._log.debug(f"didn't understand {rtype=:02x}")
            self._log.debug(f"_send_raw finished {seq=:02x}")
            del self._outstanding[seq], event, messages
            if command.response:
                # Construct response and return it
                if response == "no":
                    return command.response(None)
                if TYPE_CHECKING:
                    # The only str value is the "no" sentinel handled above.
                    assert not isinstance(response, str)
                return command.response(response)
        # No response was expected, or the context manager swallowed an error.
        return None

    async def _bus_watch(self) -> None:
        # Why is this a task, and not just run from _handle_read()?
        # It's so that when we see a forward frame on the bus that
        # didn't originate with us, we can apply a timeout after which
        # if we don't see another related frame (either a repeated
        # config command forward frame, or a backward frame) we can
        # assume there wasn't one.

        # Command awaiting repeat or reply
        current_command: dali.command.Command | None = None
        devicetype: int | None = None

        while True:
            # Wait for data
            if len(self._bus_watch_data) == 0:
                if current_command:
                    self._log.debug("bus_watch waiting with timeout")
                    try:
                        await asyncio.wait_for(self._bus_watch_data_available.wait(), 0.2)
                    except asyncio.TimeoutError:
                        pass
                else:
                    self._log.debug("bus_watch waiting for data, no timeout")
                    await self._bus_watch_data_available.wait()
                self._bus_watch_data_available.clear()

            # Figure out why we've woken up
            if len(self._bus_watch_data) == 0:
                self._log.debug("bus_watch timeout")
                timeout = True
            else:
                timeout = False
                message = self._bus_watch_data.pop(0)
                self._log.debug("bus_watch message %s", _hex(message[0:self._resptmpl_useful_size]))
                origin, rtype, raw_frame, interval, seq = self._resptmpl.unpack(message)
                if origin not in (self._MODE_OBSERVE, self._MODE_RESPONSE):
                    self._log.warning("bus_watch: unexpected packet mode, ignoring")
                    continue
                if rtype == self._RESPONSE_FRAME_DALI16:
                    frame: dali.frame.Frame | str = dali.frame.ForwardFrame(
                        16, raw_frame)
                elif rtype == self._RESPONSE_FRAME_DALI24:
                    frame = dali.frame.ForwardFrame(24, raw_frame)
                elif rtype == self._RESPONSE_FRAME_DALI8:
                    frame = dali.frame.BackwardFrame(raw_frame)
                elif rtype == self._RESPONSE_NO_FRAME:
                    frame = "no"
                elif (rtype == self._RESPONSE_INFO
                      and message[5] == self._BUS_STATUS_FRAMING_ERROR):
                    frame = dali.frame.BackwardFrameError(255)
                else:
                    # Probably a bus status message other than framing error
                    self._log.debug("bus_watch: ignoring packet")
                    continue

            # Resolve the current_command before considering anything else.
            if current_command:
                # current_command will be a config command or a
                # command that expects a response.  It cannot be
                # EnableDeviceType()
                if current_command.sendtwice:
                    # We are waiting for a repeat of the command
                    if timeout:
                        # We didn't get it: report a failed command
                        self._log.warning("Failed sendtwice command: %s", current_command)
                        self.bus_traffic._invoke(current_command, None, True)
                        current_command = None
                        continue
                    elif isinstance(frame, dali.frame.ForwardFrame):
                        # If frame matches command, it's a valid config command
                        if current_command.frame == frame:
                            self._log.debug("Config command: %s", current_command)
                            self.bus_traffic._invoke(current_command, None, False)
                            current_command = None
                            continue
                        else:
                            self._log.warning(
                                "Failed config command (second frame didn't "
                                "match): %s", current_command)
                            self.bus_traffic._invoke(current_command, None, True)
                            current_command = None
                            # Fall through to continue processing frame
                    elif isinstance(frame, dali.frame.BackwardFrame):
                        # Error: config commands don't get backward frames.
                        self._log.error("Failed config command %s with backward frame",
                                        current_command)
                        self.bus_traffic._invoke(current_command, None, True)
                        current_command = None
                    elif frame == "no":
                        self._log.warning(
                            "No frame in response to a sendtwice command: %s",
                            current_command)
                        self.bus_traffic._invoke(current_command, None, True)
                        current_command = None
                        continue
                    else:
                        self._log.error(
                            "Unexpected response waiting for retransmit of "
                            "config command, frame = %s", frame)
                elif current_command.response:
                    # We are waiting for a response
                    response_cls = current_command.response
                    if timeout or frame == "no":
                        # The response is "No".
                        self._log.debug("Command %s response \'No\'", current_command)
                        self.bus_traffic._invoke(
                            current_command, response_cls(None), False)
                        current_command = None
                        continue
                    elif isinstance(frame, dali.frame.BackwardFrame):
                        # There's a response
                        self._log.debug("Command %s response %s",
                                        current_command, response_cls(frame))
                        self.bus_traffic._invoke(
                            current_command, response_cls(frame), False)
                        current_command = None
                        continue
                    else:
                        # The response is "No" and we have a new frame to deal with;
                        # fall through to process it
                        self._log.debug("Command %s response \'No\' (on new frame)",
                                        current_command)
                        self.bus_traffic._invoke(
                            current_command, response_cls(None), False)
                        current_command = None

            # If we reach here, there is no current_command and there
            # may still be a frame to process.
            assert current_command is None
            if timeout:
                pass
            elif isinstance(frame, dali.frame.ForwardFrame):
                command = dali.command.from_frame(
                    frame, devicetype=devicetype,
                    dev_inst_map=self.dev_inst_map)
                if TYPE_CHECKING:
                    # from_frame always returns a Command for a forward frame.
                    assert command is not None
                devicetype = None
                if command.sendtwice or command.response:
                    # We need more information.  Stash the command and wait.
                    current_command = command
                else:
                    # We're good.  Report it.
                    self._log.debug("Command %s, immediate", command)
                    self.bus_traffic._invoke(command, None, False)
                if isinstance(command, EnableDeviceType):
                    devicetype = command.param
                    self._log.debug("remembering device type %s", devicetype)
            elif isinstance(frame, dali.frame.BackwardFrame):
                self._log.debug("Unexpected backward frame %s", frame)
            # self._log.debug("End of loop")

    def _handle_read(self, data: bytes) -> None:
        self._log.debug("_handle_read %s", _hex(data[0:self._resptmpl_useful_size]))
        if data[0] == self._MODE_INFO:
            # Response to initialisation command
            if not self.firmware_version:
                self.firmware_version = f"{data[3]}.{data[4]}"
                # Now read the serial number
                self._raw_write(self._cmd(
                    tridonic._CMD_INIT, tridonic._CMD_INIT_READSERIAL))
            elif not self.serial:
                self.serial = _hex(data[1:5])
                self.connected.set()
                self._bus_watch_task = asyncio.create_task(self._bus_watch())
            else:
                self._log.debug("Unsolicited init command response")

        elif data[0] == self._MODE_OBSERVE:
            # Something happened that we didn't initiate with a command
            self._bus_watch_data.append(data)
            self._bus_watch_data_available.set()

        elif data[0] == self._MODE_RESPONSE:
            self._bus_watch_data.append(data)
            self._bus_watch_data_available.set()
            seq = data[8]
            if seq in self._outstanding:
                self._log.debug(f"_handle_read: signaling for {seq=:02x}")
                event, messages = self._outstanding[seq]
                messages.append(data)
                event.set()
                del event, messages
            else:
                # The Tridonic DALI USB has a firmware bug.  When it
                # observes a frame on the bus, not generated by itself,
                # that matches the most recent frame it transmitted, it
                # reports it as if it had transmitted it itself.  We
                # ignore this - the sequence number will have been removed
                # from self._outstanding.
                self._log.debug(f"_handle_read: nothing to do for {seq=:02x}")

        else:
            self._log.debug("Unknown response mode %x", data[0])

    def _shutdown_device(self) -> None:
        # All outstanding commands need to be woken up and told they've failed
        for event, messages in self._outstanding.values():
            messages.append("fail")
            event.set()
        self._outstanding = {}
        # Cancel the bus watch task
        if self._bus_watch_task is not None:
            self._bus_watch_task.cancel()
            self._bus_watch_task = None
        self._bus_watch_data_available.clear()
        self._bus_watch_data = []
        # Clear these so that we won't get confused when we reconnect
        self.firmware_version = None
        self.serial = None

    @staticmethod
    def _cmd(cmd: int, serial: int, ctrl: int = 0, mode: int = 0,
             frame: bytes = bytes(), dtr0: int = 0, prio: int = 0,
             devtype: int = 0) -> bytes:
        """Return 64 bytes for the specified command
        """
        return tridonic._cmdtmpl.pack(
            cmd, serial, ctrl, mode, frame, dtr0, prio, devtype)


class hasseb(hid):
    """hasseb DALI Master based on NXP LPC11xx_LPC13xx

    This device doesn't support listening to the DALI bus for frames
    transmitted by other masters.  We only report our own frames and
    responses on the bus_traffic callback.

    It contains an internal table of commands that produce responses.
    This table doesn't cover device types other than zero.  If you
    send a command it doesn't know that expects a response, the driver
    will deadlock.
    """
    usb_vendor_id = 0x04CC
    usb_product_id = 0x0802

    _cmdtmpl = struct.Struct("BB")
    _NO_DATA_AVAILABLE = 0
    _NO_ANSWER = 1
    _OK = 2
    _INVALID_ANSWER = 3

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._log = self._log.getChild("hasseb")
        self._command_lock = asyncio.Lock()
        self._response_available = asyncio.Event()
        self._response: bytes | str | None = None

    async def _send_raw(
        self, command: dali.command.Command
    ) -> dali.command.Response[Any] | None:
        frame = command.frame
        if len(frame) != 16:
            raise UnsupportedFrameTypeError
        await self.connected.wait()
        async with self._command_lock:
            times = 2 if command.sendtwice else 1
            for rep in range(times):
                self._raw_write(frame.pack_len(2))
            # Earlier commands may have left a response available that
            # we need to ignore.  We're only interested in responses
            # that become available in the future.
            self._response_available.clear()
            response: dali.command.Response[Any] | None = None
            if command.response:
                # The hasseb device appears to have an internal table
                # of which commands require responses.  If a command
                # does not require a response, it won't reply to us at
                # all.  Only wait for a reply if one is needed.
                #
                # NB there is NO WAY that this can be reliable: it
                # won't understand commands from IEC-62386 part 202,
                # for example.
                await self._response_available.wait()
                self._response_available.clear()
                if TYPE_CHECKING:
                    # A reply is always set before the event fires: either the
                    # two-byte packet from _handle_read or the "fail" sentinel.
                    assert self._response is not None
                if self._response == "fail":
                    raise CommunicationError
                elif self._response[0] == self._NO_ANSWER:
                    response = command.response(None)
                elif self._response[0] == self._OK:
                    if TYPE_CHECKING:
                        # A non-"fail" reply is the two-byte status packet.
                        assert isinstance(self._response, bytes)
                    response = command.response(dali.frame.BackwardFrame(self._response[1]))
                elif self._response[0] == self._INVALID_ANSWER:
                    if TYPE_CHECKING:
                        assert isinstance(self._response, bytes)
                    # Latent bug: 'command._response' is not a Command attribute
                    # (should be 'command.response'); raises AttributeError.
                    response = command._response(  # type: ignore[attr-defined]
                        dali.frame.BackwardFrameError(self._response[1]))
                else:
                    self._log.debug("Unknown response code %x", self._response[0])

            self.bus_traffic._invoke(command, response, False)
            return response

    def _handle_read(self, data: bytes) -> None:
        # Response should be two bytes.  First byte is status, second
        # byte is optional response data.  The hasseb appears to send
        # "NO DATA AVAILABLE" reports continuously when it is idle.
        if data[0] != self._NO_DATA_AVAILABLE:
            self._response = data
            self._response_available.set()

    def _shutdown_device(self) -> None:
        # If there's a command in progress, tell it it's failed
        self._response = "fail"
        self._response_available.set()
