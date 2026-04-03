#!/usr/bin/env python3
# Usage: ./viewer.py <user@host:/remote/path>
# Example: ./viewer.py nvidia@10.9.71.11:/bos/logs/log51/left

import sys
import subprocess
import tempfile
import os
import cv2

IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp'}
CHUNK_SIZE = 300

def is_image(filename):
    return os.path.splitext(filename)[1].lower() in IMAGE_EXTS

def list_remote_images(host, path):
    result = subprocess.run(
        ['ssh', host, f"ls '{path}'"],
        capture_output=True, text=True, check=True
    )
    files = result.stdout.strip().splitlines()
    images = [f for f in files if is_image(f)]
    # Sort numerically descending (latest timestamp first)
    images.sort(key=lambda f: int(os.path.splitext(f)[0]) if os.path.splitext(f)[0].isdigit() else f, reverse=True)
    return images

def fetch_image(host, remote_path, filename):
    with tempfile.NamedTemporaryFile(suffix=os.path.splitext(filename)[1], delete=False) as tmp:
        tmp_path = tmp.name
    subprocess.run(
        ['scp', '-q', f"{host}:{remote_path}/{filename}", tmp_path],
        check=True
    )
    img = cv2.imread(tmp_path)
    os.unlink(tmp_path)
    return img

def main():
    if len(sys.argv) not in (2, 3):
        print(f"Usage: {sys.argv[0]} <user@host:/remote/path>")
        sys.exit(1)

    host, remote_path = sys.argv[1].split(':', 1)

    print(f"→ Fetching image list from {host}:{remote_path}...")
    images = list_remote_images(host, remote_path)
    total = len(images)

    if total == 0:
        print("No image files found. Exiting.")
        sys.exit(0)

    print(f"  Found {total} images (newest first, chunk size {CHUNK_SIZE}).")
    print("  Press any key for next chunk, Q/Esc to quit.\n")

    cv2.namedWindow("viewer", cv2.WINDOW_NORMAL)

    offset = 0
    while offset < total:
        chunk = images[offset:offset + CHUNK_SIZE]

        # Pick the lowest timestamp in this chunk (last element, since sorted descending)
        frame = chunk[-1]

        print(f"  Chunk {offset+1}–{offset+len(chunk)} of {total} → showing oldest in chunk: {frame}")
        img = fetch_image(host, remote_path, frame)

        if img is None:
            print(f"  Warning: could not decode {frame}, skipping chunk.")
        else:
            cv2.setWindowTitle("viewer", f"{frame}  [chunk {offset//CHUNK_SIZE + 1}, frame {offset+len(chunk)}/{total}]")
            cv2.imshow("viewer", img)

        key = cv2.waitKey(0) & 0xFF
        if key == ord('q') or key == 27:
            print("Quit.")
            break

        offset += CHUNK_SIZE

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
