#!/usr/bin/env python3
"""async-firmware-update.py - transfer a firmware image over DALI

Firmware transfer (IEC 62386-105) uses 32-bit forward frames, which are
supported by the Tridonic DALI USB interface and by Lunatone LUBA RS232
interfaces.  The commands live in dali.firmware and the update
procedure in dali.firmware_sequences.

Bit 24 of a firmware transfer frame selects the address space, so a
destination says both which short address is meant and whether it is
in the control gear or the control device space: control gear A3 and
control device A3 are different units.  This example scans both spaces
by default and reports which one each target answered in.

One bus unit may hold both, though.  Part 105 section 4.2 says that
starting an update on a bus unit affects every logical unit inside it,
so a product that presents both control gear and a control device
appears twice in the scan but only needs updating once.

    # See what could be updated, and change nothing
    ./async-firmware-update.py --uri luba232:/dev/ttyUSB0 --scan

    # Update one control gear
    ./async-firmware-update.py --uri luba232:/dev/ttyUSB0 \
        --address 3 --image new-firmware.bin --block-size 48

    # Update one control device
    ./async-firmware-update.py --uri luba232:/dev/ttyUSB0 \
        --address 3 --control-device --image new-firmware.bin \
        --block-size 48

The block size is a property of the target product and comes from its
firmware update documentation; there is no sensible default.  The
image must already be in the part 105 Annex A update file format,
block 0 included.  Read the warnings in dali.firmware_sequences before
pointing this at hardware you care about: none of this has been run
against a unit that accepts a firmware image.

This file is part of python-dali.

python-dali is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this program. If not, see
<https://www.gnu.org/licenses/>.
"""
import argparse
import asyncio
import sys

from dali.address import (
    FirmwareDeviceBroadcast,
    FirmwareDeviceShort,
    FirmwareGearBroadcast,
    FirmwareGearShort,
)
from dalidrivers.typed.hid import tridonic
from dalidrivers.typed.serial import DriverLubaRs232
from dali.firmware_sequences import FindFirmwareUpdateTargets, FirmwareUpdate


async def connect(uri):
    """Open either a LUBA serial interface or a Tridonic DALI USB one"""
    if uri.startswith(f"{DriverLubaRs232.uri_scheme}:"):
        driver = DriverLubaRs232(uri=uri)
        await driver.connect()
        return driver

    driver = tridonic(uri, glob="*" in uri)
    driver.connect()
    print("Waiting for device...")
    await driver.connected.wait()
    print(f"Connected, firmware={driver.firmware_version}, "
          f"serial={driver.serial}")
    return driver


async def scan(driver):
    targets = await driver.send(FindFirmwareUpdateTargets(), progress=print)
    if not targets:
        print("\nNo bus unit answered the firmware transfer queries")
        return targets
    print()
    for target in targets:
        kind = "control device" if target.control_device else "control gear"
        print(f"A{target.address} ({kind}): firmware update version "
              f"{target.version}, features {target.features:#04x}")

    shared = {t.address for t in targets if t.control_device} & \
        {t.address for t in targets if not t.control_device}
    for a in sorted(shared):
        print(f"\nNote: short address {a} answered in both address spaces. "
              "If that is one bus unit holding both control gear and a "
              "control device, updating either one updates all of it; do "
              "not transfer the image twice.")
    return targets


async def update(driver, destination, image, block_size, restart):
    print(f"\nUpdating {destination} with {len(image)} bytes")
    blocks = await driver.send(
        FirmwareUpdate(destination, image, block_size=block_size,
                       restart=restart),
        progress=print,
    )
    print(f"Transferred {blocks} blocks")


async def main(args):
    driver = await connect(args.uri)
    try:
        if args.scan or args.image is None:
            await scan(driver)
            return

        with open(args.image, "rb") as f:
            image = f.read()

        if args.broadcast:
            # Every unit in the chosen address space gets the same image
            # and none of them can be asked whether it worked; only do
            # this when the bus holds nothing but the product being
            # updated.
            destination = (FirmwareDeviceBroadcast() if args.control_device
                           else FirmwareGearBroadcast())
        else:
            destination = (FirmwareDeviceShort(args.address)
                           if args.control_device
                           else FirmwareGearShort(args.address))

        await update(driver, destination, image, args.block_size,
                     not args.no_restart)
    finally:
        # Only the HID drivers have an explicit teardown
        if hasattr(driver, "disconnect"):
            driver.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument(
        "--uri", default="/dev/dali/daliusb-*",
        help="LUBA serial URI (luba232:/dev/ttyUSB0) or a Tridonic DALI "
        "USB device path, which may be a glob")
    parser.add_argument(
        "--scan", action="store_true",
        help="list the bus units that support firmware transfer and exit")
    parser.add_argument("--image", help="firmware image to transfer")
    parser.add_argument(
        "--address", type=int, help="short address of the unit to update")
    parser.add_argument(
        "--control-device", action="store_true",
        help="address the control device space rather than control gear")
    parser.add_argument(
        "--broadcast", action="store_true",
        help="update every unit in the address space at once, without "
        "verification")
    parser.add_argument(
        "--block-size", type=int,
        help="block size in bytes, from the product documentation")
    parser.add_argument(
        "--no-restart", action="store_true",
        help="leave the unit running its old firmware after the transfer")
    args = parser.parse_args()

    if args.image and not args.scan:
        if args.block_size is None:
            parser.error("--block-size is required to transfer an image")
        if args.address is None and not args.broadcast:
            parser.error("--address or --broadcast is required")

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        sys.exit(1)
