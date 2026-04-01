#!/usr/bin/env python3
"""
fix_wpilog_timestamps.py

Reads a WPILog binary file, finds every record whose timestamp is above
1,000,000 (microseconds), replaces that timestamp with 0, and writes the
result to a new file.

WPILog file header layout:
  Bytes 0-5  – magic "WPILOG"
  Bytes 6-7  – version (little-endian uint16)
  Bytes 8-11 – extra header length (little-endian uint32)
  Bytes 12.. – extra header data (extra header length bytes)
  Bytes 12+extra.. – records

WPILog record layout (all little-endian, no padding between records):
  Byte 0  – bitfield
              bits [1:0]  entry_id length    (0→1 B, 1→2 B, 2→3 B, 3→4 B)
              bits [3:2]  payload_size length (same mapping)
              bits [6:4]  timestamp length   (0→1 B … 7→8 B)
  Bytes 1..  entry_id     (variable width)
  Bytes ..   payload_size (variable width)
  Bytes ..   timestamp    (variable width)
  Bytes ..   payload data (payload_size bytes)
"""

import sys
import os
from pathlib import Path

MAGIC = b"WPILOG"
THRESHOLD = 1_000_000  # microseconds


def process_wpilog(input_path: str, output_path: str) -> None:
    with open(input_path, "rb") as f:
        raw = bytearray(f.read())

    total = len(raw)

    # Validate & parse file header
    # 6-byte magic | 2-byte version | 4-byte extra-header length | extra-header
    if raw[:6] != MAGIC:
        raise ValueError("Not a valid WPILog file (bad magic bytes)")

    extra_header_len = int.from_bytes(raw[8:12], "little")
    records_start = 12 + extra_header_len

    pos = records_start
    records_visited = 0
    records_patched = 0

    # Walk every record
    while pos < total:
        if pos + 1 > total:
            print(f"[WARN] Truncated record at offset {pos}, stopping.")
            break

        bitfield = raw[pos]
        pos += 1

        entry_id_len    = (bitfield & 0b00000011) + 1         # bits 1:0 → 1-4
        payload_len_len = ((bitfield >> 2) & 0b00000011) + 1  # bits 3:2 → 1-4
        ts_len          = ((bitfield >> 4) & 0b00000111) + 1  # bits 6:4 → 1-8

        header_remaining = entry_id_len + payload_len_len + ts_len
        if pos + header_remaining > total:
            print(f"[WARN] Truncated record header at offset {pos - 1}, stopping.")
            break

        # Skip entry_id
        pos += entry_id_len

        # Read payload size
        payload_size = int.from_bytes(raw[pos:pos + payload_len_len], "little")
        pos += payload_len_len

        # Timestamp — patch in-place if above threshold
        ts_offset = pos
        timestamp = int.from_bytes(raw[ts_offset:ts_offset + ts_len], "little")
        pos += ts_len

        records_visited += 1

        if timestamp > THRESHOLD:
            for i in range(ts_len):
                raw[ts_offset + i] = 0
            records_patched += 1

        # Skip payload data
        if pos + payload_size > total:
            print(f"[WARN] Payload extends past EOF at offset {pos}, stopping.")
            break
        pos += payload_size

    # Write output
    with open(output_path, "wb") as f:
        f.write(raw)

    print(f"Done.")
    print(f"  Records visited : {records_visited}")
    print(f"  Records patched : {records_patched}")
    print(f"  Output written  : {output_path}")


def main():
    if len(sys.argv) < 2:
        print("Usage: fix_wpilog_timestamps.py <input.wpilog> [output.wpilog]")
        print()
        print("  Replaces every timestamp > 1,000,000 µs with 0.")
        print("  If no output path is given, writes to <input>_fixed.wpilog")
        sys.exit(1)

    input_path = sys.argv[1]

    if len(sys.argv) >= 3:
        output_path = sys.argv[2]
    else:
        stem = Path(input_path).stem
        suffix = Path(input_path).suffix
        output_path = str(Path(input_path).with_name(stem + "_fixed" + suffix))

    if not os.path.isfile(input_path):
        print(f"Error: '{input_path}' not found.")
        sys.exit(1)

    process_wpilog(input_path, output_path)


if __name__ == "__main__":
    main()