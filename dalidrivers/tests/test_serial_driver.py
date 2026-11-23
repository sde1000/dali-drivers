import asyncio
from functools import reduce
import logging
from operator import xor
from unittest.mock import AsyncMock, MagicMock

import pytest

from dali import firmware
from dali.address import (
    DeviceShort,
    FirmwareBroadcast,
    GearShort,
    InstanceNumber,
)
from dali.command import NumericResponse
from dali.device.general import QueryInstanceType
from dali.exceptions import MissingResponse
from dalidrivers.typed.serial import (
    _DALI_FRAMING_ERROR,
    DistributorQueue,
    DriverLubaRs232,
)
from dali.gear.general import QueryActualLevel


@pytest.mark.asyncio
async def test_timed_out_query_reports_missing_value():
    """A query that times out must return the command's own response type.

    A bare Response(None) is reported as missing, so autodiscovery no longer
    leaks a None value that crashed on int(None).
    """
    driver = DriverLubaRs232("luba232:/dev/ttyACM0")
    driver._connected.set()
    protocol = MagicMock()
    protocol.reset_dali_response = MagicMock()
    protocol.send_dali_command = AsyncMock()
    protocol.wait_dali_raw_response = AsyncMock(side_effect=asyncio.TimeoutError)
    driver._protocol = protocol

    response = await driver.send(
        QueryInstanceType(device=DeviceShort(0), instance=InstanceNumber(0))
    )

    assert isinstance(response, NumericResponse)
    assert response.missing
    assert not response.valid
    with pytest.raises(MissingResponse):
        _ = response.value


@pytest.mark.asyncio
async def test_reset_dali_response_drains_all_without_critical(caplog):
    """Stale buffered responses must all be cleared and not logged as CRITICAL.

    A late backward frame that missed its query is routine on a busy bus; the
    old code discarded only one item and logged it at CRITICAL, spamming logs.
    """
    protocol = DriverLubaRs232.LubaProtocol()
    protocol._queue_rx_raw_dali.put_nowait(6)
    protocol._queue_rx_raw_dali.put_nowait(7)
    protocol._queue_rx_raw_dali.put_nowait(8)

    with caplog.at_level(logging.DEBUG, logger="dalidrivers.misc"):
        protocol.reset_dali_response()

    assert protocol._queue_rx_raw_dali.empty()
    assert not [r for r in caplog.records if r.levelno >= logging.CRITICAL]
    assert any("discarded 3 stale" in r.getMessage() for r in caplog.records)


def _connecting_driver(monkeypatch, info_side_effect):
    """Return a driver whose transport is faked and handshake is scripted."""
    driver = DriverLubaRs232("luba232:/dev/ttyACM0")
    protocol = MagicMock()
    connected = asyncio.Event()
    connected.set()
    protocol.connected = connected
    protocol.send_device_info_query = AsyncMock(side_effect=info_side_effect)
    protocol.send_device_settings = AsyncMock()
    protocol.reset_luba_response = MagicMock()
    monkeypatch.setattr(
        "dalidrivers.typed.serial.serialx.create_serial_connection",
        AsyncMock(return_value=(MagicMock(), protocol)),
    )
    return driver, protocol


@pytest.mark.asyncio
async def test_connect_retries_handshake_on_timeout(monkeypatch):
    """A busy bus can time out the first handshake frames; connect must retry.

    The interface is slow to acknowledge the info/settings exchange while a
    device streams events, so a single timeout should not fail the connect.
    """
    driver, protocol = _connecting_driver(
        monkeypatch, [asyncio.TimeoutError, asyncio.TimeoutError, None]
    )

    await driver.connect()

    assert driver.is_connected
    assert protocol.send_device_info_query.await_count == 3
    assert protocol.send_device_settings.await_count == 1
    assert protocol.reset_luba_response.call_count == 2


@pytest.mark.asyncio
async def test_connect_without_applying_settings_leaves_hardware_untouched(
    monkeypatch,
):
    """connect(apply_settings=False) must not write the device settings.

    A read-only probe of a discovered interface must not overwrite the
    hardware configuration, otherwise it could depower a bus the integrated
    supply is already driving.
    """
    driver, protocol = _connecting_driver(monkeypatch, [None])

    await driver.connect(apply_settings=False)

    assert driver.is_connected
    assert protocol.send_device_info_query.await_count == 1
    assert protocol.send_device_settings.await_count == 0


@pytest.mark.asyncio
async def test_connect_raises_after_exhausting_handshake_attempts(monkeypatch):
    """When every handshake attempt times out, connect must give up and raise."""
    driver, protocol = _connecting_driver(monkeypatch, asyncio.TimeoutError)

    with pytest.raises(asyncio.TimeoutError):
        await driver.connect()

    assert not driver.is_connected
    attempts = DriverLubaRs232.handshake_attempts
    assert protocol.send_device_info_query.await_count == attempts
    assert protocol.reset_luba_response.call_count == attempts - 1


@pytest.mark.asyncio
async def test_lost_transport_is_reported_as_disconnected(monkeypatch):
    """An interface that goes away must stop claiming to be connected.

    The driver's own event only records that the handshake once succeeded, so
    an unplugged USB interface still looked connected and every frame ran into
    its transmit timeout instead of failing fast.
    """
    driver, protocol = _connecting_driver(monkeypatch, [None])
    await driver.connect()
    assert driver.is_connected

    # What LubaProtocol.connection_lost() does when the port disappears
    protocol.connected.clear()

    assert not driver.is_connected


@pytest.mark.asyncio
async def test_connect_reopens_the_port_after_a_lost_transport(monkeypatch):
    """Once the interface is back, connect() must open the port again."""
    driver, protocol = _connecting_driver(monkeypatch, [None])
    await driver.connect()
    protocol.connected.clear()

    replacement = MagicMock()
    connected = asyncio.Event()
    connected.set()
    replacement.connected = connected
    replacement.send_device_info_query = AsyncMock()
    replacement.send_device_settings = AsyncMock()
    monkeypatch.setattr(
        "dalidrivers.typed.serial.serialx.create_serial_connection",
        AsyncMock(return_value=(MagicMock(), replacement)),
    )

    await driver.connect()

    assert driver.is_connected
    assert driver._protocol is replacement


@pytest.mark.asyncio
async def test_send_dali_command_resyncs_after_stale_confirmation(caplog):
    """A stale confirmation must not desync every following transmission.

    A weak bus can let a send's confirmation arrive after it timed out; the
    next send then finds that stale confirmation queued ahead of its own. It
    must be discarded and the matching one awaited, so the stream recovers
    instead of every later frame inheriting the one-off mismatch.
    """
    protocol = DriverLubaRs232.LubaProtocol()
    protocol.transport = MagicMock()
    protocol.rx_idle.set()

    command = QueryActualLevel(GearShort(0))
    stale = QueryActualLevel(GearShort(1))
    protocol._queue_tx_conf.put_nowait(
        protocol.LubaMsgTxConf(tx_id=1, message=stale)
    )
    protocol._queue_tx_conf.put_nowait(
        protocol.LubaMsgTxConf(tx_id=2, message=command)
    )

    with caplog.at_level(logging.WARNING, logger="dalidrivers.misc"):
        await protocol.send_dali_command(command)

    assert protocol._queue_tx_conf.empty()
    assert any("stale LUBA confirmation" in r.getMessage() for r in caplog.records)


@pytest.mark.asyncio
async def test_reset_luba_response_drains_all(caplog):
    """Stale LUBA command replies must be cleared before a handshake retry."""
    protocol = DriverLubaRs232.LubaProtocol()
    protocol._queue_rx_luba_cmd.put_nowait("stale-1")
    protocol._queue_rx_luba_cmd.put_nowait("stale-2")

    protocol.reset_luba_response()

    assert protocol._queue_rx_luba_cmd.empty()


def _luba_framing_error_event():
    """Build a LUBA 'DALI frame received' event reporting a framing error."""
    payload = [
        0x00, 0x01,          # Time tick
        0x00,                # DALI line
        (2 << 6) | 63,       # Event type 2, event info 63 (framing error)
    ]
    body = [
        DriverLubaRs232.LubaCmd.EVENT_MESSAGE.value,
        len(payload),
        *payload,
    ]
    return bytes([0x59, *body, reduce(xor, body)])


@pytest.mark.asyncio
async def test_framing_error_event_is_queued():
    """A framing-error event must be surfaced on the raw response queue."""
    protocol = DriverLubaRs232.LubaProtocol()

    protocol.data_received(_luba_framing_error_event())

    raw = await asyncio.wait_for(protocol._queue_rx_raw_dali.get(), timeout=1)
    assert raw is _DALI_FRAMING_ERROR


@pytest.mark.asyncio
async def test_framing_error_query_returns_error_response():
    """A query that hits a framing error must report it, not time out silently.

    Several gear answering at once (a short-address collision) garble the
    backward frame; the response must carry the error so callers can tell it
    apart from an address that did not answer.
    """
    driver = DriverLubaRs232("luba232:/dev/ttyACM0")
    driver._connected.set()
    protocol = MagicMock()
    protocol.reset_dali_response = MagicMock()
    protocol.send_dali_command = AsyncMock()
    protocol.wait_dali_raw_response = AsyncMock(return_value=_DALI_FRAMING_ERROR)
    driver._protocol = protocol

    response = await driver.send(
        QueryInstanceType(device=DeviceShort(0), instance=InstanceNumber(0))
    )

    assert response.raw_value is not None
    assert response.raw_value.error is True


async def _transmit(cmd):
    """Send a command through a faked transport and return the bytes written."""
    protocol = DriverLubaRs232.LubaProtocol()
    protocol.transport = MagicMock()
    protocol.rx_idle.set()
    protocol._queue_tx_conf.put_nowait(
        protocol.LubaMsgTxConf(tx_id=1, message=cmd)
    )
    await protocol.send_dali_command(cmd)
    protocol.transport.write.assert_called_once()
    return list(protocol.transport.write.call_args[0][0])


def _luba_event(dali_bytes):
    """Build a LUBA 'DALI frame received' event message."""
    payload = [
        0x00, 0x01,                        # Time tick
        0x00,                              # DALI line
        (2 << 6) | (8 * len(dali_bytes)),  # Event type 2, frame length in bits
        *dali_bytes,
    ]
    body = [
        DriverLubaRs232.LubaCmd.EVENT_MESSAGE.value,
        len(payload),
        *payload,
    ]
    return bytes([0x59, *body, reduce(xor, body)])


@pytest.mark.asyncio
async def test_send_32_bit_frame_fills_the_whole_data_field():
    """A firmware transfer frame uses all four LUBA data bytes."""
    written = await _transmit(
        firmware.StartFirmwareTransfer(FirmwareBroadcast()))

    assert written[0] == 0x59
    assert written[1] == DriverLubaRs232.LubaCmd.ADD_DALI_FRAME_TO_TX_CMD.value
    assert written[2] == 7  # Payload length is unchanged
    assert written[4] == 32  # Number of bits in the DALI frame
    assert written[6:10] == [0xFE, 0xFB, 0x00, 0x00]
    assert written[-1] == reduce(xor, written[1:-1])
    assert len(written) == 11


@pytest.mark.asyncio
async def test_send_32_bit_block_data():
    """Block payload bytes reach the wire unchanged."""
    written = await _transmit(firmware.TransferBlockData(0xAABBCC))

    assert written[4] == 32
    assert written[6:10] == [0xBD, 0xAA, 0xBB, 0xCC]


@pytest.mark.asyncio
async def test_send_16_bit_frame_is_still_padded():
    """Shorter frames stay left-aligned with the data field zero-padded."""
    written = await _transmit(QueryActualLevel(GearShort(0)))

    assert written[2] == 7
    assert written[4] == 16
    assert written[6:10] == [0x01, 0xA0, 0x00, 0x00]
    assert len(written) == 11


@pytest.mark.asyncio
async def test_send_rejects_frames_the_data_field_cannot_hold():
    """An 8-bit frame is not a forward frame the interface can transmit."""
    protocol = DriverLubaRs232.LubaProtocol()
    protocol.transport = MagicMock()
    protocol.rx_idle.set()
    cmd = MagicMock()
    cmd.frame.as_byte_sequence = [0xFF]

    with pytest.raises(ValueError, match="16, 24 or 32 bit"):
        await protocol.send_dali_command(cmd)

    protocol.transport.write.assert_not_called()


@pytest.mark.asyncio
async def test_received_32_bit_frame_decodes_to_a_firmware_command():
    """A 32-bit frame observed on the bus is decoded, not dropped."""
    protocol = DriverLubaRs232.LubaProtocol()
    rx_queue = DistributorQueue(protocol.queue_rx_dali)

    protocol.data_received(_luba_event([0x0A, 0xFB, 0x07, 0x00]))

    received = await asyncio.wait_for(rx_queue.get(), timeout=1)
    assert isinstance(received, firmware.QueryFirmwareUpdateReceiverReady)
    assert received.destination.address == 5


@pytest.mark.asyncio
async def test_received_24_bit_frame_still_decodes():
    """Accepting 32-bit frames did not disturb 24-bit decoding."""
    protocol = DriverLubaRs232.LubaProtocol()
    rx_queue = DistributorQueue(protocol.queue_rx_dali)

    protocol.data_received(_luba_event([0xFF, 0xFE, 0x00]))

    received = await asyncio.wait_for(rx_queue.get(), timeout=1)
    assert len(received.frame) == 24
