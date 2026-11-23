Changes
=======

1.0 (in preparation)
--------------------

- Work with python-dali release 1.0
- Add a synchronous driver for Philips/Signify USB2DALI (LCN8600, USB
  0471:0003), with 16-bit frames, backward-frame responses, optional bus
  power control, and offline HID protocol tests.
- Move drivers with type annotations to dalidrivers.misc.typed
- ``dalidrivers.misc.serial`` and ``dalidrivers.misc.trace_logging`` are
  back as deprecated aliases of their ``dalidrivers.typed`` counterparts,
  so code written against the old locations keeps working. They raise a
  DeprecationWarning.
- Send ``EnableDeviceType`` for a command whose device type is 0,
  fluorescent lamp control gear, which the drivers previously treated as
  "send this command on its own". Needs python-dali 1.0a3 or newer, where
  ``Command.devicetype`` uses ``None`` for that instead.
- Transmit and receive 32-bit frames in the LUBA serial driver and in the
  Tridonic DALI-USB driver, as IEC 62386-105 firmware transfer uses.

0.2
---

- Remove some remaining references to dali.drivers

0.1
---

- Initial release, forked from python-dali
