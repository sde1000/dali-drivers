"""Deprecated location of the trace logging level.

It moved to :mod:`dalidrivers.typed.trace_logging` along with the
serial drivers that use it. Importing it from here still installs the
level and the Logger.trace() method, but raises a DeprecationWarning.
"""

import warnings

from dalidrivers.typed.trace_logging import *  # noqa: F401,F403

warnings.warn(
    "dalidrivers.misc.trace_logging has moved to "
    "dalidrivers.typed.trace_logging",
    DeprecationWarning, stacklevel=2)
