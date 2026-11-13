#!/usr/bin/env python3
"""Genera un log binario di test e verifica il round-trip con bin2csv.py.

Uso: python3 tools/test_bin2csv.py
"""

from __future__ import annotations

import struct
import subprocess
import sys
import tempfile
from math import sin
from pathlib import Path

LOG_MAGIC = 0x47534C41
TYPE_HEADER = 0x0048
TYPE_SAMPLE = 0x0053

SAMPLE_COUNT = 1001


def make_record(rtype: int, body: bytes) -> bytes:
    """Frame header (10 byte) + body."""
    size = 10 + len(body)
    return struct.pack("<IHH", LOG_MAGIC, 0, rtype) + struct.pack("<H", size) + body


def build_log() -> tuple[bytes, list[list[float]]]:
    header_body = struct.pack("<BBII32s32s", 1, 0, 10_000, 0, b"antares-test", b"unit test")
    blob = bytearray(make_record(TYPE_HEADER, header_body))

    expected: list[list[float]] = []
    for i in range(SAMPLE_COUNT):
        values = [
            0.01 * i,  # accel x
            -0.02 * i,
            1.0,
            1.0 + sin(i / 10.0),
            0.5,
            -1.5,
            2.5,
            10.0 + i % 360,
            20.0,
            30.0,
            1013.25 + i * 0.001,
            120.5 + i * 0.01,
            i * 0.02,
            130.0 + i * 0.01,
            24.75,
        ]
        flags = 0x01 if i % 2 == 0 else 0x03
        body = (
            struct.pack("<IBBI", i * 10_000, i % 4, flags, 0)
            + struct.pack("<15f", *values)
            + bytes([flags])
            + b"\x00\x00\x00"
        )
        blob += make_record(TYPE_SAMPLE, body)
        expected.append(values)
    return bytes(blob), expected


def main() -> int:
    here = Path(__file__).resolve().parent
    blob, expected = build_log()
    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / "log0.bin"
        dst = Path(tmp) / "log0.csv"
        src.write_bytes(blob)

        result = subprocess.run(
            [sys.executable, str(here / "bin2csv.py"), str(src), str(dst)],
            capture_output=True,
            text=True,
        )
        print(result.stdout.strip() or result.stderr.strip())
        if result.returncode != 0:
            return 1

        lines = dst.read_text().splitlines()
        data = [line for line in lines if not line.startswith("#")]
        header = data[0].split(",")
        rows = [line.split(",") for line in data[1:]]
        assert len(rows) == SAMPLE_COUNT, f"righe: {len(rows)} != {SAMPLE_COUNT}"
        assert len(header) == 19, f"colonne: {len(header)}"

        for i, row in enumerate(rows):
            assert len(row) == 19, f"riga {i}: {len(row)} colonne"
            assert row[1] in ("Idle", "Flight", "Descent", "Landed"), row[1]
            assert row[1] == ("Idle", "Flight", "Descent", "Landed")[i % 4], (i, row[1])
            # flags = 0x01 (continuity) for even samples, 0x03 (continuity +
            # pyro) for odd ones.
            expected_flags = "1"
            assert row[17] == expected_flags, (i, row[17])
            assert row[18] == ("1" if i % 2 else "0")
            for col, value in enumerate(expected[i], start=2):
                assert abs(float(row[col]) - value) < 1e-4, (i, col, row[col], value)

        print(f"OK: {len(rows)} campioni, {len(header)} colonne, valori coerenti")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
