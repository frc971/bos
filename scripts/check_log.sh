#!/usr/bin/env bash
# Usage: ./rsync_between.sh <user@host:/remote/path> <local_dest>
# Example: ./rsync_between.sh nvidia@10.9.71.11:/bos/logs/log51 ./local_logs

set -euo pipefail

REMOTE="${1:-}"
LOCAL_DEST="${2:-./synced_files}"
CHUNK_SIZE=300

if [[ -z "$REMOTE" ]]; then
  echo "Usage: $0 <user@host:/remote/path> <local_dest>"
  exit 1
fi

REMOTE_HOST="${REMOTE%%:*}"
REMOTE_PATH="${REMOTE#*:}"

sync_subfolder() {
  local subfolder="$1"
  local local_sub="${LOCAL_DEST}/${subfolder}"

  mkdir -p "$local_sub"

  echo ""
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo "  $subfolder/ — fetching file list..."
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

  local all_files=()
  while IFS= read -r filename; do
    local seconds_part="${filename%%.*}"
    if ! [[ "$seconds_part" =~ ^[0-9]+$ ]]; then
      continue
    fi
    local ext_lower
    ext_lower=$(echo "${filename##*.}" | tr '[:upper:]' '[:lower:]')
    case "$ext_lower" in
    jpg | jpeg | png | gif | bmp | tiff | webp | heic)
      all_files+=("$filename")
      ;;
    esac
  done < <(ssh "$REMOTE_HOST" "ls '${REMOTE_PATH}/${subfolder}'" | sort -t. -k1,1 -rn)

  local total=${#all_files[@]}
  echo "  Found $total image files."

  if [[ $total -eq 0 ]]; then
    echo "  No files found. Skipping $subfolder/."
    return
  fi

  local offset=0
  while [[ $offset -lt $total ]]; do
    local chunk=("${all_files[@]:$offset:$CHUNK_SIZE}")
    local end=$((offset + ${#chunk[@]}))

    echo ""
    echo "  Chunk $((offset + 1))–$end of $total  [$subfolder/]"

    # List is sorted descending, so chunk[0] is the latest in this chunk
    local latest="${chunk[0]}"
    echo "  → Copying latest in chunk: $latest"

    scp -q "${REMOTE_HOST}:${REMOTE_PATH}/${subfolder}/${latest}" "${local_sub}/${latest}"
    open "${local_sub}/${latest}"

    offset=$end

    if [[ $offset -ge $total ]]; then
      echo ""
      echo "  ✓ Done with $subfolder/."
      break
    fi

    echo ""
    read -rp "  Continue to next chunk of $subfolder/? [Y/n] " answer
    case "$(echo "$answer" | tr '[:upper:]' '[:lower:]')" in
    n | no)
      echo "  Stopped at chunk ending at $end/$total."
      return
      ;;
    esac
  done
}

echo "→ Remote: $REMOTE"
echo "→ Local:  $LOCAL_DEST"

sync_subfolder "left"
sync_subfolder "right"

echo ""
echo "✓ All done."
