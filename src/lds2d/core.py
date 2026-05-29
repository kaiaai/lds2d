# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""Core types, the driver base class, and the model registry for lds2d.

A Pythonic reimagining of the kaiaai/LDS C++ library: instead of registering
Arduino-style callbacks, you iterate over points or full scans.

    from lds2d import Lidar
    with Lidar.open("LD14P", "/dev/serial0") as lidar:
        for scan in lidar.scans():
            for p in scan:
                print(p.angle_deg, p.dist_mm, p.quality)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterator, List, Optional, Type


class LidarError(Exception):
    """Base class for all lds2d errors."""


class ChecksumError(LidarError):
    """A packet failed its CRC check."""


class UnknownModelError(LidarError):
    """No driver is registered for the requested model name."""


class NotSupportedError(LidarError):
    """The driver does not implement this capability (e.g. motor control)."""


@dataclass(frozen=True)
class ScanPoint:
    """One LiDAR measurement."""
    angle_deg: float      # 0-360, increasing in the sensor's scan direction
    dist_mm: int          # distance in millimetres; 0 means "no return"
    quality: int          # signal intensity 0-255 (0 if the model has none)

    @property
    def valid(self) -> bool:
        return self.dist_mm > 0


@dataclass
class Scan:
    """One full ~360° rotation."""
    points: List[ScanPoint] = field(default_factory=list)
    scan_freq_hz: float = 0.0

    def __iter__(self) -> Iterator[ScanPoint]:
        return iter(self.points)

    def __len__(self) -> int:
        return len(self.points)

    @property
    def valid_points(self) -> List[ScanPoint]:
        return [p for p in self.points if p.valid]


# --- driver registry -------------------------------------------------------

_REGISTRY: Dict[str, Type["LidarDriver"]] = {}


def register(*names: str):
    """Class decorator: register a driver under one or more model names."""
    def deco(cls: Type["LidarDriver"]) -> Type["LidarDriver"]:
        for n in names:
            _REGISTRY[n.upper()] = cls
        return cls
    return deco


def available_models() -> List[str]:
    return sorted(_REGISTRY)


class LidarDriver:
    """Base class for all LiDAR drivers.

    Subclasses implement ``_packets()`` (and, optionally, motor control).
    ``points()`` and ``scans()`` are derived from it for free.
    """

    MODEL_NAME: str = "?"
    DEFAULT_BAUD: int = 230400

    def __init__(self, transport):
        self._t = transport

    # -- to implement by each driver --
    def _packets(self):
        """Yield ``(scan_freq_hz, [ScanPoint, ...])`` tuples, one per packet."""
        raise NotImplementedError

    # -- derived iteration --
    def points(self) -> Iterator[ScanPoint]:
        """A flat stream of measurements."""
        for _freq, pts in self._packets():
            yield from pts

    def scans(self) -> Iterator[Scan]:
        """Group the point stream into full rotations.

        A new scan starts whenever the angle wraps (drops by more than 180°).
        """
        current = Scan()
        prev_angle: Optional[float] = None
        for freq, pts in self._packets():
            current.scan_freq_hz = freq
            for p in pts:
                if prev_angle is not None and p.angle_deg + 180.0 < prev_angle:
                    yield current
                    current = Scan(scan_freq_hz=freq)
                prev_angle = p.angle_deg
                current.points.append(p)

    # -- optional motor control; command-driven models override these --
    def start(self) -> None:
        raise NotSupportedError(f"{self.MODEL_NAME} has no software motor start")

    def stop(self) -> None:
        raise NotSupportedError(f"{self.MODEL_NAME} has no software motor stop")

    def set_scan_freq(self, hz: float) -> None:
        raise NotSupportedError(f"{self.MODEL_NAME} has no settable scan rate")

    def get_scan_freq(self, listen_s: float = 1.0) -> Optional[float]:
        """Best-effort current scan rate in Hz (None if no data)."""
        raise NotSupportedError(f"{self.MODEL_NAME} cannot report its scan rate")

    # -- lifecycle --
    def close(self) -> None:
        self._t.close()

    def __enter__(self) -> "LidarDriver":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class Lidar:
    """Factory façade. ``Lidar.open("LD14P", "/dev/serial0")``."""

    @staticmethod
    def open(model: str, port: Optional[str] = None, baud: Optional[int] = None,
             transport=None, **serial_kwargs) -> LidarDriver:
        # Import drivers lazily so registering happens on first use.
        from . import drivers  # noqa: F401  (populates the registry)

        cls = _REGISTRY.get(model.upper())
        if cls is None:
            raise UnknownModelError(
                f"unknown model {model!r}; known: {', '.join(available_models())}")
        if transport is None:
            from .transport import SerialTransport
            if port is None:
                raise ValueError("either 'port' or 'transport' is required")
            transport = SerialTransport(port, baud or cls.DEFAULT_BAUD,
                                        **serial_kwargs)
        return cls(transport)
