#!/usr/bin/env python3
"""Converte i log binari "Antares Micro R3" (Logs/logN.bin) in CSV leggibile.

Formato atteso (vedi src/LogFormat.h). Tutti i valori sono little endian.

Struttura di un record (identica per header e campioni):

    uint32 magic     = 0x47534C41 ("ALSG")
    uint16 reserved  = 0
    uint16 type      = 0x0048 ('H', header) | 0x0053 ('S', sample)
    uint16 size      = dimensione totale del record in byte

Record 'H' (uno solo, a inizio file), subito dopo il frame header:

    uint8  version
    uint8  reserved
    uint32 sample_interval_us
    uint8  reserved2
    char   build[32]
    char   note[32]

Record 'S' (un campione), subito dopo il frame header:

    uint32 timestamp_us   (micros() dall'avvio)
    uint8  status         (0=Idle, 1=Flight, 2=Descent, 3=Landed)
    uint8  flags          (bit0 = continuity, bit1 = pyro)
    uint32 crc32          (0 = non calcolato)
    uint8  payload[64]    (15 float, poi 1 byte di flags, poi 3 byte di padding)

Grazie al magic il convertitore si riallinea anche su file troncati o con
settori incompleti, saltando i record non validi.

Uso:
    python3 tools/bin2csv.py Logs/log0.bin                 # -> Logs/log0.csv
    python3 tools/bin2csv.py Logs/log0.bin out.csv
    python3 tools/bin2csv.py Logs/ --out-dir csv/          # intera cartella
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

LOG_MAGIC = 0x47534C41
# magic, reserved, type, size (size is AFTER the two reserved bytes)
FRAME = struct.Struct("<IHHH")
FRAME_HEADER_LEN = FRAME.size  # 10
TYPE_HEADER = 0x0048
TYPE_SAMPLE = 0x0053

# Contenuto dopo il frame header
HEADER_BODY = struct.Struct("<BBIB32s32s")  # version, reserved, interval, reserved2, build, note
# version(1) reserved(1) interval(4) reserved2(1) = 7 byte, poi build(32) e note(32)
SAMPLE_BODY = struct.Struct("<IBBI")  # timestamp_us, status, flags, crc32

PAYLOAD_LEN = 64
FIELD_COUNT = 15
FLAGS_OFFSET = FIELD_COUNT * 4

COLUMNS = [
    "Timestamp",
    "Status",
    "AccelX",
    "AccelY",
    "AccelZ",
    "AccelTOT",
    "GyroX",
    "GyroY",
    "GyroZ",
    "RotX",
    "RotY",
    "RotZ",
    "Pressure",
    "Altitude",
    "Relative Altitude",
    "Max Altitude",
    "Temperature",
    "Continuity",
    "Pyro",
]

STATUS_NAMES = {0: "Idle", 1: "Flight", 2: "Descent", 3: "Landed"}
FLAG_CONTINUITY = 0x01
FLAG_PYRO = 0x02


@dataclass
class Stats:
    samples: int = 0
    skipped: int = 0

    def merge(self, other: "Stats") -> None:
        self.samples += other.samples
        self.skipped += other.skipped

    def __str__(self) -> str:
        return f"{self.samples} campioni, {self.skipped} record saltati"


def find_record(data: bytes, start: int) -> tuple[int, int, int] | None:
    """Cerca il prossimo frame header valido: (offset, type, size)."""
    magic = LOG_MAGIC.to_bytes(4, "little")
    pos = start
    while True:
        pos = data.find(magic, pos)
        if pos < 0 or pos + FRAME_HEADER_LEN > len(data):
            return None
        _, _, rtype, size = FRAME.unpack_from(data, pos)
        if rtype in (TYPE_HEADER, TYPE_SAMPLE) and size >= FRAME_HEADER_LEN:
            return pos, rtype, size
        pos += 1


def _text(raw: bytes) -> str:
    return raw.rstrip(b"\x00").decode("utf-8", "replace")


def convert(src: Path, dst: Path) -> Stats:
    data = src.read_bytes()
    stats = Stats()
    rows: list[list[str]] = []
    header_info: dict[str, object] = {}

    pos = 0
    while True:
        found = find_record(data, pos)
        if found is None:
            break
        offset, rtype, size = found
        if offset > pos:
            stats.skipped += 1  # byte di rumore/riempimento prima del record
        body = offset + FRAME_HEADER_LEN
        if offset + size > len(data):
            stats.skipped += 1  # record troncato: fine del file
            break

        if rtype == TYPE_HEADER:
            version, _r1, interval, _r2, build, note = HEADER_BODY.unpack_from(data, body)
            header_info = {
                "version": version,
                "interval_us": interval,
                "build": _text(build),
                "note": _text(note),
            }
        else:
            ts, status, flags, _crc = SAMPLE_BODY.unpack_from(data, body)
            payload_at = body + SAMPLE_BODY.size
            # The payload also carries a (derived) flags byte, kept for
            # compatibility; the record's own "flags" field is authoritative.
            _payload_flags = data[payload_at + FLAGS_OFFSET]
            values = struct.unpack_from(f"<{FIELD_COUNT}f", data, payload_at)
            rows.append(
                [
                    f"{ts / 1_000_000.0:.6f}",
                    STATUS_NAMES.get(status, f"Status{status}"),
                    *(f"{v:.6f}" for v in values),
                    "1" if flags & FLAG_CONTINUITY else "0",
                    "1" if flags & FLAG_PYRO else "0",
                ]
            )
            stats.samples += 1

        pos = offset + size

    with dst.open("w", newline="", encoding="utf-8") as out:
        if header_info:
            out.write(
                f"# build={header_info['build']} note={header_info['note']} "
                f"version={header_info['version']} "
                f"sample_interval_us={header_info['interval_us']}\n"
            )
        out.write(",".join(COLUMNS) + "\n")
        for row in rows:
            out.write(",".join(row) + "\n")

    return stats


def output_path(src: Path, out_dir: Path | None) -> Path:
    return (out_dir / src.with_suffix(".csv").name) if out_dir else src.with_suffix(".csv")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("input", type=Path, help="file .bin oppure cartella Logs/")
    parser.add_argument("output", type=Path, nargs="?", help="CSV di uscita (solo input singolo)")
    parser.add_argument("--out-dir", type=Path, help="cartella di uscita per più file")
    args = parser.parse_args(argv)

    if args.input.is_dir():
        if args.output:
            parser.error("con una cartella usa --out-dir, non un file di uscita")
        out_dir = args.out_dir or args.input
        out_dir.mkdir(parents=True, exist_ok=True)
        files = sorted(args.input.glob("*.bin"))
        if not files:
            print(f"nessun file .bin in {args.input}", file=sys.stderr)
            return 1
        total = Stats()
        for src in files:
            dst = output_path(src, out_dir)
            stats = convert(src, dst)
            total.merge(stats)
            print(f"{src.name} -> {dst}: {stats}")
        print(f"Totale: {total}")
        return 0

    if not args.input.is_file():
        print(f"file non trovato: {args.input}", file=sys.stderr)
        return 1
    dst = args.output or output_path(args.input, args.out_dir)
    dst.parent.mkdir(parents=True, exist_ok=True)
    stats = convert(args.input, dst)
    print(f"{args.input} -> {dst}: {stats}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
