#!/usr/bin/env bash
# Usage: ./sync_chunks.sh <user@host:/remote/path> <local_dest>
# Example: ./sync_chunks.sh nvidia@10.9.71.11:/bos/logs/log44 ./local_logs

set -euo pipefail

REMOTE="${1:-nvidia@10.9.71.11:/bos/logs/log44}"
LOCAL_DEST="${2:-./synced_files}"
CHUNK_SIZE=300

# Parse user@host and remote path
REMOTE_HOST="${REMOTE%%:*}"
REMOTE_PATH="${REMOTE#*:}"

mkdir -p "$LOCAL_DEST"

echo "→ Fetching file list from $REMOTE..."
ALL_FILES=()
while IFS= read -r line; do
  ALL_FILES+=("$line")
done < <(ssh "$REMOTE_HOST" "ls '$REMOTE_PATH'" | sort -t. -k1,1 -rn)

TOTAL=${#ALL_FILES[@]}
echo "  Found $TOTAL files."

if [[ $TOTAL -eq 0 ]]; then
  echo "No files found. Exiting."
  exit 0
fi

offset=0

while [[ $offset -lt $TOTAL ]]; do
  # Slice the next chunk
  chunk=("${ALL_FILES[@]:$offset:$CHUNK_SIZE}")
  end=$(( offset + ${#chunk[@]} ))

  echo ""
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo "  Syncing files $((offset+1))–$end of $TOTAL"
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

  # Build rsync --include list for this chunk
  include_args=()
  for f in "${chunk[@]}"; do
    include_args+=(--include="$f")
  done

  rsync -avz --progress \
    "${include_args[@]}" \
    --exclude='*' \
    "${REMOTE}/" "$LOCAL_DEST/"

  # Show the last downloaded image (lowest timestamp = last in reverse-sorted chunk)
  latest_image=""
  for f in "${chunk[@]}"; do
    candidate="$LOCAL_DEST/$f"
    if [[ -f "$candidate" ]]; then
      ext=$(echo "${f##*.}" | tr '[:upper:]' '[:lower:]')
      case "$ext" in
        jpg|jpeg|png|gif|bmp|tiff|webp|heic)
          latest_image="$candidate"
          ;;
      esac
    fi
  done

  if [[ -n "$latest_image" ]]; then
    echo "  Opening latest image: $latest_image"
    open "$latest_image"
  else
    echo "  (No image files in this chunk to preview)"
  fi

  offset=$end

  if [[ $offset -ge $TOTAL ]]; then
    echo ""
    echo "✓ All $TOTAL files synced."
    break
  fi

  echo ""
  read -rp "Continue syncing next chunk? [Y/n] " answer
  case "$(echo "$answer" | tr '[:upper:]' '[:lower:]')" in
    n|no)
      echo "Stopped at $offset/$TOTAL files."
      exit 0
      ;;
  esac
done
