#!/usr/bin/env bash
# Usage: ./rsync_between.sh <user@host:/remote/path> <local_dest> <start_seconds> <end_seconds>
# Remote path should contain left/ and right/ subdirectories
# Filenames are expected in <seconds>.<microseconds>.jpg format (e.g. 100.091057.jpg)
# Example: ./rsync_between.sh nvidia@10.9.71.11:/bos/logs/log51 ./local_logs 50 450

set -euo pipefail

REMOTE="${1:-}"
LOCAL_DEST="${2:-./synced_files}"
START_TIME="${3:-}"
END_TIME="${4:-}"

if [[ -z "$REMOTE" || -z "$START_TIME" || -z "$END_TIME" ]]; then
  echo "Usage: $0 <user@host:/remote/path> <local_dest> <start_seconds> <end_seconds>"
  exit 1
fi

REMOTE_HOST="${REMOTE%%:*}"
REMOTE_PATH="${REMOTE#*:}"

sync_subfolder() {
  local subfolder="$1"
  local remote_sub="${REMOTE_HOST}:${REMOTE_PATH}/${subfolder}"
  local local_sub="${LOCAL_DEST}/${subfolder}"

  mkdir -p "$local_sub"

  echo ""
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo "  Syncing $subfolder/"
  echo "  Filtering for timestamps between $START_TIME and $END_TIME (seconds)"
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

  local matched=()
  while IFS= read -r filename; do
    local seconds_part="${filename%%.*}"
    if ! [[ "$seconds_part" =~ ^[0-9]+$ ]]; then
      continue
    fi
    if [[ $seconds_part -ge $START_TIME && $seconds_part -le $END_TIME ]]; then
      local ext_lower
      ext_lower=$(echo "${filename##*.}" | tr '[:upper:]' '[:lower:]')
      case "$ext_lower" in
      jpg | jpeg | png | gif | bmp | tiff | webp | heic)
        matched+=("$filename")
        ;;
      esac
    fi
  done < <(ssh "$REMOTE_HOST" "ls '${REMOTE_PATH}/${subfolder}'" | sort -t. -k1,1 -n)

  local total=${#matched[@]}
  echo "  Found $total matching image files."

  if [[ $total -eq 0 ]]; then
    echo "  No files matched. Skipping $subfolder/."
    return
  fi

  local include_args=()
  for f in "${matched[@]}"; do
    include_args+=(--include="$f")
  done

  rsync -avz --progress \
    "${include_args[@]}" \
    --exclude='*' \
    "${remote_sub}/" "$local_sub/"

  echo "  ✓ $total images synced to $local_sub"
}

echo "→ Remote: $REMOTE"
echo "→ Local:  $LOCAL_DEST"
echo "→ Range:  $START_TIME → $END_TIME seconds"

sync_subfolder "second_bot_left"
sync_subfolder "second_bot_right"

echo ""
echo "✓ All done. Both folders synced to $LOCAL_DEST"
