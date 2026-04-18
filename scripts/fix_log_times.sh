#!/usr/bin/python3
from __future__ import annotations

import os
import sys
from pathlib import Path

MAGIC = b"WPILOG"


def usage() -> None:
    print("Usage: fix_log_times.sh <input.wpilog> [output.wpilog]")
    print()
    print("Normalizes WPILOG record timestamps so the log starts at 0 us.")
    print("If no output path is given, writes to <input>_fixed.wpilog.")


def iter_records(raw: bytearray):
    total = len(raw)
    if total < 12:
        raise ValueError("File is too small to be a valid WPILOG")
    if raw[:6] != MAGIC:
        raise ValueError("Not a valid WPILOG file (bad magic bytes)")

    extra_header_len = int.from_bytes(raw[8:12], "little")
    records_start = 12 + extra_header_len
    if records_start > total:
        raise ValueError("WPILOG extra header extends past EOF")

    pos = records_start
    record_index = 0
    while pos < total:
        bitfield_offset = pos
        bitfield = raw[pos]
        pos += 1

        entry_id_len = (bitfield & 0b00000011) + 1
        payload_len_len = ((bitfield >> 2) & 0b00000011) + 1
        ts_len = ((bitfield >> 4) & 0b00000111) + 1

        header_remaining = entry_id_len + payload_len_len + ts_len
        if pos + header_remaining > total:
            raise ValueError(
                f"Truncated record header at offset {bitfield_offset}"
            )

        pos += entry_id_len

        payload_size = int.from_bytes(raw[pos:pos + payload_len_len], "little")
        pos += payload_len_len

        ts_offset = pos
        timestamp = int.from_bytes(raw[ts_offset:ts_offset + ts_len], "little")
        pos += ts_len

        payload_end = pos + payload_size
        if payload_end > total:
            raise ValueError(f"Payload extends past EOF at offset {pos}")

        yield {
            "index": record_index,
            "timestamp": timestamp,
            "ts_offset": ts_offset,
            "ts_len": ts_len,
        }

        pos = payload_end
        record_index += 1


def process_wpilog(input_path: str, output_path: str) -> None:
    with open(input_path, "rb") as file:
        raw = bytearray(file.read())

    records = list(iter_records(raw))
    if not records:
        raise ValueError("WPILOG contains no records")

    base_timestamp = next(
        (record["timestamp"] for record in records if record["timestamp"] != 0),
        0,
    )

    records_patched = 0
    records_earlier_than_base = 0

    for record in records:
        timestamp = record["timestamp"]
        if timestamp < base_timestamp:
            records_earlier_than_base += 1
            continue

        normalized_timestamp = timestamp - base_timestamp
        if normalized_timestamp == timestamp:
            continue

        ts_offset = record["ts_offset"]
        ts_len = record["ts_len"]
        max_value = (1 << (ts_len * 8)) - 1
        if normalized_timestamp > max_value:
            raise ValueError(
                "Normalized timestamp does not fit in original field width"
            )

        raw[ts_offset:ts_offset + ts_len] = normalized_timestamp.to_bytes(
            ts_len, "little"
        )
        records_patched += 1

    with open(output_path, "wb") as file:
        file.write(raw)

    print("Done.")
    print(f"  Base timestamp : {base_timestamp}")
    print(f"  Records visited: {len(records)}")
    print(f"  Records patched: {records_patched}")
    print(f"  Records earlier: {records_earlier_than_base}")
    print(f"  Output written : {output_path}")

if len(sys.argv) < 2:
    usage()

input_path = sys.argv[1]
if not os.path.isfile(input_path):
    print(f"Error: '{input_path}' not found.", file=sys.stderr)

if len(sys.argv) >= 3:
    output_path = sys.argv[2]
else:
    input_file = Path(input_path)
    output_path = str(input_file.with_name(f"{input_file.stem}_fixed{input_file.suffix}"))

process_wpilog(input_path, output_path)
