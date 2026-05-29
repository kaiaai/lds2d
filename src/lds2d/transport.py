# Copyright 2026 KAIA.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""Serial transport. Drivers talk to any object with read/write/close, so they
can be unit-tested against recorded byte streams with no hardware attached."""
from __future__ import annotations


class SerialTransport:
    """Thin wrapper over pyserial."""

    def __init__(self, port: str, baud: int, timeout: float = 1.0):
        import serial  # imported lazily so the package installs without hardware
        self._ser = serial.Serial(port, baud, timeout=timeout)

    def read(self, size: int) -> bytes:
        return self._ser.read(size)

    def write(self, data: bytes) -> int:
        n = self._ser.write(data)
        self._ser.flush()
        return n

    def reset_input(self) -> None:
        self._ser.reset_input_buffer()

    def close(self) -> None:
        self._ser.close()
