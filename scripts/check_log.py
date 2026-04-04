#!/usr/bin/env python3
# Usage: ./sync_preview.py <user@host:/remote/path> <local_dest>
# Example: ./sync_preview.py nvidia@10.9.71.11:/bos/logs/log51 ./local_logs

import sys
import subprocess
import tempfile
import os
import cv2

CHUNK_SIZE = 300

def get_remote_files(remote_host, remote_path, subfolder):
    result = subprocess.run(
        ["ssh", remote_host, f"ls '{remote_path}/{subfolder}'"],
        capture_output=True, text=True, check=True
    )
    files = []
    for filename in result.stdout.splitlines():
        seconds_part = filename.split(".")[0]
        if not seconds_part.isdigit():
            continue
        ext = filename.rsplit(".", 1)[-1].lower()
        if ext in {"jpg", "jpeg", "png", "gif", "bmp", "tiff", "webp", "heic"}:
            files.append(filename)
    # Sort descending by timestamp (latest first)
    files.sort(key=lambda f: float(f.rsplit(".", 1)[0]), reverse=True)
    return files

def scp_file(remote_host, remote_path, subfolder, filename, local_sub):
    dest = os.path.join(local_sub, filename)
    subprocess.run(
        ["scp", "-q", f"{remote_host}:{remote_path}/{subfolder}/{filename}", dest],
        check=True
    )
    return dest

def sync_subfolder(remote_host, remote_path, subfolder, local_dest):
    local_sub = os.path.join(local_dest, subfolder)
    os.makedirs(local_sub, exist_ok=True)

    print(f"\n{'━' * 38}")
    print(f"  {subfolder}/ — fetching file list...")
    print(f"{'━' * 38}")

    all_files = get_remote_files(remote_host, remote_path, subfolder)
    total = len(all_files)
    print(f"  Found {total} image files.")

    if total == 0:
        print(f"  No files found. Skipping {subfolder}/.")
        return

    offset = 0
    while offset < total:
        chunk = all_files[offset:offset + CHUNK_SIZE]
        end = offset + len(chunk)

        print(f"\n  Chunk {offset + 1}–{end} of {total}  [{subfolder}/]")

        # chunk[0] is the latest in this chunk (descending sort)
        latest = chunk[0]
        print(f"  → Copying latest in chunk: {latest}")

        local_path = scp_file(remote_host, remote_path, subfolder, latest, local_sub)

        img = cv2.imread(local_path)
        if img is not None:
            cv2.imshow(f"{subfolder} — {latest}  (any key: continue, ESC: stop)", img)
            key = cv2.waitKey(0) & 0xFF
            cv2.destroyAllWindows()
            if key == 27:  # ESC
                print(f"  Stopped at chunk ending at {end}/{total}.")
                return
        else:
            print(f"  (Could not read image: {local_path})")
            input("  Press Enter to continue...")

        offset = end

    print(f"\n  ✓ Done with {subfolder}/.")

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <user@host:/remote/path> <local_dest>")
        sys.exit(1)

    remote = sys.argv[1]
    local_dest = sys.argv[2] if len(sys.argv) > 2 else "./synced_files"

    remote_host, remote_path = remote.split(":", 1)

    print(f"→ Remote: {remote}")
    print(f"→ Local:  {local_dest}")

    sync_subfolder(remote_host, remote_path, "left", local_dest)
    sync_subfolder(remote_host, remote_path, "right", local_dest)

    print("\n✓ All done.")

if __name__ == "__main__":
    main()
