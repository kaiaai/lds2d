# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""lds2d — Python driver for 2D LiDARs (a Pythonic port of kaiaai/LDS)."""
from .core import (
    Lidar,
    LidarDriver,
    Scan,
    ScanPoint,
    LidarError,
    ChecksumError,
    UnknownModelError,
    NotSupportedError,
    available_models,
    register,
)
from . import drivers  # noqa: F401,E402  — register bundled drivers on import

__version__ = "0.7.0"

__all__ = [
    "Lidar",
    "LidarDriver",
    "Scan",
    "ScanPoint",
    "LidarError",
    "ChecksumError",
    "UnknownModelError",
    "NotSupportedError",
    "available_models",
    "register",
    "__version__",
]
