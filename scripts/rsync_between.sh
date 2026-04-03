#!/usr/bin/env bash
# Usage: ./rsync_between.sh <user@host:/remote/path> <local_dest> <start_seconds> <end_seconds>
# Filenames are expected in <seconds>.<microseconds>.jpg format (e.g. 100.091057.jpg)
# Example: ./rsync_between.sh nvidia@10.9.71.11:/bos/logs/log51/left ./local_logs 50 450

set -euo pipefail

REMOTE="${1:-}"
LOCAL_DEST="${2:-./synced_files}"
START_TIME="${3:-}"
END_TIME="${4:-}"

if [[ -z "$REMOTE" || -z "$START_TIME" || -z "$END_TIME" ]]; then
  echo "Usage: $0 <user@host:/remote/path> <local_dest> <start_seconds> <end_seconds>"
  exit 1
fi

# Parse user@host and remote path
REMOTE_HOST="${REMOTE%%:*}"
REMOTE_PATH="${REMOTE#*:}"

mkdir -p "$LOCAL_DEST"

echo "→ Fetching file list from $REMOTE..."
echo "  Filtering for timestamps between $START_TIME and $END_TIME (seconds)"

MATCHED_FILES=()
while IFS= read -r filename; do
  # Filenames are <seconds>.<microseconds>.<ext>, e.g. 100.091057.jpg
  # Extract just the seconds portion (first field before the first dot)
  seconds_part="${filename%%.*}"

  # Skip if not numeric
  if ! [[ "$seconds_part" =~ ^[0-9]+$ ]]; then
    continue
  fi

  # Compare seconds directly against start/end
  if [[ $seconds_part -ge $START_TIME && $seconds_part -le $END_TIME ]]; then
    ext="${filename##*.}"
    ext_lower=$(echo "$ext" | tr '[:upper:]' '[:lower:]')
    case "$ext_lower" in
    jpg | jpeg | png | gif | bmp | tiff | webp | heic)
      MATCHED_FILES+=("$filename")
      ;;
    esac
  fi
done < <(ssh "$REMOTE_HOST" "ls '$REMOTE_PATH'" | sort -t. -k1,1 -n)

TOTAL=${#MATCHED_FILES[@]}
echo "  Found $TOTAL matching image files in range."

if [[ $TOTAL -eq 0 ]]; then
  echo "No files matched the time range. Exiting."
  exit 0
fi

echo ""
echo "→ Rsyncing $TOTAL image files to $LOCAL_DEST..."

include_args=()
for f in "${MATCHED_FILES[@]}"; do
  include_args+=(--include="$f")
done

rsync -avz --progress \
  "${include_args[@]}" \
  --exclude='*' \
  "${REMOTE}/" "$LOCAL_DEST/"

echo ""
echo "✓ Done. $TOTAL images synced to $LOCAL_DEST"
echo "  Range: $START_TIME → $END_TIME seconds"
