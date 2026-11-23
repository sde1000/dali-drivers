"""Tests for the 32-bit frame support in the Tridonic DALI-USB driver.

These exercise the parts of dalidrivers.misc.hid.tridonic that do not need a
device: frame mode selection, command packing, and the bus watch decode.
"""
import asyncio
import unittest

from dali import address
from dali import firmware
from dali import frame
from dalidrivers.misc.hid import tridonic
from dali.exceptions import UnsupportedFrameTypeError
from dali.gear.general import DAPC


def _response(rtype, raw, *, origin=tridonic._MODE_OBSERVE, interval=0, seq=0):
    """Build a 64-byte report as the device would send it."""
    return tridonic._resptmpl.pack(origin, rtype, raw, interval, seq)


class TestCommandMode(unittest.TestCase):
    def test_modes(self):
        self.assertEqual(tridonic._command_mode(frame.Frame(8)),
                         tridonic._SEND_MODE_DALI8)
        self.assertEqual(tridonic._command_mode(frame.Frame(16)),
                         tridonic._SEND_MODE_DALI16)
        self.assertEqual(tridonic._command_mode(frame.Frame(24)),
                         tridonic._SEND_MODE_DALI24)
        self.assertEqual(tridonic._command_mode(frame.Frame(32)),
                         tridonic._SEND_MODE_DALI32)

    def test_dali32_mode_value(self):
        """the 32-bit send mode is the value Tridonic's driver uses"""
        self.assertEqual(tridonic._SEND_MODE_DALI32, 0x0E)

    def test_unsupported(self):
        for bits in (1, 17, 25, 33, 64):
            self.assertRaises(UnsupportedFrameTypeError,
                              tridonic._command_mode, frame.Frame(bits))


class TestCommandPacking(unittest.TestCase):
    def test_dali32_frame_is_packed_big_endian(self):
        """a 32-bit frame fills all four frame bytes of the report"""
        cmd = firmware.StartFirmwareTransfer(address.FirmwareBroadcast())
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
            self.assertEqual(cmd.destination, address.FirmwareBroadcast())
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
        self.assertIsInstance(cmd, firmware.QueryBlockFault)
        self.assertFalse(response.value)
        self.assertFalse(error)

    async def test_dali24_still_decodes(self):
        """adding the 32-bit type did not disturb 24-bit decoding"""
        seen = await self._observe(
            _response(tridonic._RESPONSE_FRAME_DALI24,
                      bytes((0x00, 0xFF, 0xFE, 0x00))))
        self.assertEqual(len(seen), 1)
        self.assertEqual(len(seen[0][0].frame), 24)


if __name__ == '__main__':
    unittest.main()
