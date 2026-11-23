"""Deprecated location of the serial drivers.

The serial drivers moved to :mod:`dalidrivers.typed`, which carries a
``py.typed`` marker so downstream projects type-check against them.
Importing them from here still works and gives the same classes, but
raises a DeprecationWarning; import from ``dalidrivers.typed.serial``
instead.
"""

import warnings

from dalidrivers.typed.serial import (  # noqa: F401
    DistributorQueue,
    DriverLubaRs232,
    DriverSCIRS232,
    DriverSerialBase,
    drivers_map,
)

warnings.warn(
    "dalidrivers.misc.serial has moved to dalidrivers.typed.serial",
    DeprecationWarning, stacklevel=2)

__all__ = [
    "DistributorQueue",
    "DriverLubaRs232",
    "DriverSCIRS232",
    "DriverSerialBase",
    "drivers_map",
]
