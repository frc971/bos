#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
from pathlib import Path

SUPPORTED_EXTENSIONS = (".png", ".jpg", ".jpeg", ".bmp", ".gif", ".tiff")
CAMERA_DIRS = ("main_bot_left", "main_bot_right")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Copy timestamped images from main_bot_left/main_bot_right into a "
            "new folder, keeping only images inside an inclusive timestamp range."
        )
    )
    parser.add_argument("source", help="Source folder containing camera folders")
    parser.add_argument("destination", help="Destination folder to copy into")
    parser.add_argument("start_time", type=float, help="Minimum timestamp to copy")
    parser.add_argument("end_time", type=float, help="Maximum timestamp to copy")
    parser.add_argument(
        "--flat",
        action="store_true",
        help="Copy directly into destination instead of preserving camera folders",
    )
    return parser.parse_args()


def extract_timestamp(filename: str) -> float:
    match = re.search(r"\d+(\.\d+)?", filename)
    if not match:
        raise ValueError(f"No timestamp found in filename: {filename}")
    return float(match.group())


def iter_image_paths(folder: Path):
    for path in sorted(folder.iterdir()):
        if path.is_file() and path.name.lower().endswith(SUPPORTED_EXTENSIONS):
            yield path


def copy_camera_folder(
    source_dir: Path,
    destination_dir: Path,
    start_time: float,
    end_time: float,
) -> tuple[int, int]:
    destination_dir.mkdir(parents=True, exist_ok=True)
    copied = 0
    skipped = 0

    for image_path in iter_image_paths(source_dir):
        try:
            timestamp = extract_timestamp(image_path.name)
        except ValueError:
            skipped += 1
            continue

        if start_time <= timestamp <= end_time:
            shutil.copy2(image_path, destination_dir / image_path.name)
            copied += 1
        else:
            skipped += 1

    return copied, skipped


def main() -> int:
    args = parse_args()
    source = Path(args.source).expanduser()
    destination = Path(args.destination).expanduser()

    if args.start_time > args.end_time:
        print("Error: start_time must be <= end_time", file=sys.stderr)
        return 1

    if not source.is_dir():
        print(f"Error: source folder not found: {source}", file=sys.stderr)
        return 1

    camera_dirs = [source / camera_dir for camera_dir in CAMERA_DIRS]
    if not all(camera_dir.is_dir() for camera_dir in camera_dirs):
        missing = [str(camera_dir) for camera_dir in camera_dirs if not camera_dir.is_dir()]
        print(
            "Error: source must contain main_bot_left and main_bot_right. "
            f"Missing: {', '.join(missing)}",
            file=sys.stderr,
        )
        return 1

    total_copied = 0
    total_skipped = 0
    for camera_dir in camera_dirs:
        output_dir = destination if args.flat else destination / camera_dir.name
        copied, skipped = copy_camera_folder(
            camera_dir,
            output_dir,
            args.start_time,
            args.end_time,
        )
        total_copied += copied
        total_skipped += skipped
        print(f"{camera_dir.name}: copied {copied}, skipped {skipped}")

    print(f"Done. Copied {total_copied} images into {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
